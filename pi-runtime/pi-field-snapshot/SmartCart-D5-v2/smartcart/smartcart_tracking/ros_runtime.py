"""전용 localhost ROS 문맥에서 읽기 전용 추적·상태만 발행한다."""

from copy import deepcopy
import json
import os
import re
import time

from smartcart_sensing.ros import scan_from_message, validate_topic
from smartcart_uwb.config import _reject_constant, _unique_object
from smartcart_uwb.estimator import ESTIMATE_SCHEMA
from .clusters import CLUSTERS_SCHEMA
from .runtime import LiveTrackingRuntime
from .tracker import TRACK_SCHEMA

MAX_JSON_BYTES = 65536
LIVE_TOPICS = {name: "/smartcart/tracking/" + name for name in ("clusters", "target", "status")}
REPLAY_TOPICS = {name: "/smartcart/replay/tracking/" + name for name in ("clusters", "target", "status")}


def decode_status(text):
    """외부 ROS String의 크기·중복 키·비유한 상수를 먼저 거부한다."""
    if type(text) is not str or len(text) > MAX_JSON_BYTES:
        raise ValueError("ROS 상태 JSON 크기 또는 형식이 잘못되었습니다")
    try:
        if len(text.encode("utf-8")) > MAX_JSON_BYTES:
            raise ValueError("ROS 상태 JSON 크기 상한을 넘었습니다")
        value = json.loads(text, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except (ValueError, RecursionError, UnicodeError) as error:
        raise ValueError("올바른 유한 JSON 상태가 필요합니다") from error
    if type(value) is not dict:
        raise ValueError("전체 UWB 상태 객체가 필요합니다")
    return value


def _json(value):
    encoded = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    if len(encoded.encode("utf-8")) > MAX_JSON_BYTES:
        raise ValueError("추적 ROS 출력 크기 상한을 넘었습니다")
    return encoded


def _result(value, mode):
    expected = {"uwb_estimate", "lidar_clusters", "track", "source_mode", "drive_authorized"}
    if type(value) is not dict or set(value) != expected or value["source_mode"] != mode or value["drive_authorized"] is not False:
        raise ValueError("발행 출처 또는 추적 결과가 잘못되었습니다")
    for name, schema in (("uwb_estimate", ESTIMATE_SCHEMA), ("lidar_clusters", CLUSTERS_SCHEMA), ("track", TRACK_SCHEMA)):
        item = value[name]
        if (type(item) is not dict or len(item) > 64 or item.get("schema") not in (schema, "smartcart.single_uwb_estimate.v1" if name == "uwb_estimate" else schema)
                or item.get("source_mode") != mode or item.get("drive_authorized") is not False):
            raise ValueError("다른 출처의 관측을 추적 토픽으로 발행할 수 없습니다")
    _json(value)
    return deepcopy(value)


def _safe_error(store, reason):
    try:
        store.set_error(reason)
    except Exception:
        pass


def _cleanup(executor, node, context):
    if executor is not None:
        try:
            executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass
    if context is not None:
        try:
            if context.ok():
                context.shutdown()
        except Exception:
            pass


def _environment():
    # 현재 프로세스와 그 자식의 ROS 검색 범위만 제한하며 시스템 설정을 바꾸지 않는다.
    os.environ["ROS_LOCALHOST_ONLY"] = "1"


def _ros_types():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    return (rclpy, Context, SingleThreadedExecutor, Node, QoSProfile, ReliabilityPolicy,
            HistoryPolicy, DurabilityPolicy, SignalHandlerOptions, String)


def tracking_ros_loop(lidar_store, tracking_store, mounting, uwb_settings, tracking_settings, *,
                      scan_topic="/scan", status_topic="/smartcart/uwb/dual/status", ros_args=None,
                      dual_uwb_store=None, single_left=False):
    """전체 UWB 상태와 스캔만 구독하며 센서 드라이버·USB·주행 명령을 열지 않는다."""
    context = node = executor = None
    try:
        validate_topic(scan_topic)
        validate_topic(status_topic)
        reserved = set(LIVE_TOPICS.values()) | set(REPLAY_TOPICS.values())
        if scan_topic == status_topic or scan_topic in reserved or status_topic in reserved:
            raise ValueError("입력과 추적 출력 토픽은 달라야 합니다")
        _environment()
        (rclpy, Context, Executor, Node, QoS, Reliability, History, Durability, Signals, String) = _ros_types()
        from sensor_msgs.msg import LaserScan
        runtime = LiveTrackingRuntime(mounting, uwb_settings, tracking_settings, single_left=single_left)
        context = Context()
        rclpy.init(args=[] if ros_args is None else ros_args, context=context, signal_handler_options=Signals.NO)
        # 절대 출력 이름을 전역 remap으로 실측·재생·명령 토픽에 바꾸지 못하게 한다.
        node = Node("smartcart_tracking_" + str(os.getpid()), context=context, use_global_arguments=False,
                    enable_rosout=False, start_parameter_services=False)
        qos = QoS(history=History.KEEP_LAST, depth=5, reliability=Reliability.BEST_EFFORT, durability=Durability.VOLATILE)
        publishers = {name: node.create_publisher(String, topic, qos) for name, topic in LIVE_TOPICS.items()}

        def publish(result, now):
            result = _result(result, "live")
            tracking_store.publish(result, now_ns=now)
            for name, value in (("clusters", result["lidar_clusters"]), ("target", result["track"]), ("status", result)):
                message = String()
                message.data = _json(value)
                publishers[name].publish(message)

        def status_callback(message):
            now = time.monotonic_ns()
            try:
                status = decode_status(message.data)
                result = runtime.consume_status(status, now)
                if dual_uwb_store is not None:
                    dual_uwb_store.consume_status(status, now)
            except Exception:
                runtime.invalidate_status("invalid_status_json")
                if dual_uwb_store is not None:
                    dual_uwb_store.set_error()
                result = runtime.snapshot(now)
            publish(result, now)

        def scan_callback(message):
            now = time.monotonic_ns()
            try:
                if (message.header.frame_id != mounting.lidar.frame_id or
                        not 1 <= len(message.ranges) <= tracking_settings.max_rays or
                        len(message.intensities) > tracking_settings.max_rays):
                    raise ValueError("LiDAR 프레임 또는 배열 크기가 잘못되었습니다")
                scan = {key: getattr(message, key) for key in (
                    "angle_min", "angle_max", "angle_increment", "range_min", "range_max", "scan_time", "time_increment")}
                scan["ranges"] = list(message.ranges)
                result = runtime.consume_scan(scan, now)
                if result["lidar_clusters"]["valid"]:
                    lidar_store.publish(scan_from_message(message, scan_topic))
                else:
                    lidar_store.set_error("잘못된 ROS 스캔을 거부했습니다", rejected=True)
            except Exception:
                runtime.invalidate_scan("invalid_ros_scan")
                lidar_store.set_error("잘못된 ROS 스캔을 거부했습니다", rejected=True)
                result = runtime.snapshot(now)
            publish(result, now)

        def tick():
            now = time.monotonic_ns()
            publish(runtime.snapshot(now), now)

        node.create_subscription(LaserScan, scan_topic, scan_callback, qos)
        node.create_subscription(String, status_topic, status_callback, qos)
        node.create_timer(.05, tick)
        executor = Executor(context=context)
        executor.add_node(node)
        while not tracking_store.stop.is_set() and not lidar_store.stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception:
        if dual_uwb_store is not None:
            dual_uwb_store.set_error('dual_ros_error')
        _safe_error(tracking_store, "tracking_ros_error")
        _safe_error(lidar_store, "추적 ROS 연결 오류")
    finally:
        _cleanup(executor, node, context)


def tracking_replay_ros_loop(tracking_store, *, ros_args=None):
    """선택한 재생 뷰만 전용 이름으로 발행하고 실측 토픽을 만들지 않는다."""
    context = node = executor = None
    try:
        _environment()
        (rclpy, Context, Executor, Node, QoS, Reliability, History, Durability, Signals, String) = _ros_types()
        context = Context()
        rclpy.init(args=[] if ros_args is None else ros_args, context=context, signal_handler_options=Signals.NO)
        node = Node("smartcart_tracking_replay_" + str(os.getpid()), context=context, use_global_arguments=False,
                    enable_rosout=False, start_parameter_services=False)
        qos = QoS(history=History.KEEP_LAST, depth=5, reliability=Reliability.BEST_EFFORT, durability=Durability.VOLATILE)
        publishers = {name: node.create_publisher(String, topic, qos) for name, topic in REPLAY_TOPICS.items()}

        def snapshot():
            envelope = tracking_store.snapshot()
            if (type(envelope) is not dict or envelope.get("schema") != "smartcart.tracking_view.v1"
                    or envelope.get("mode") != "replay" or envelope.get("source_mode") != "replay"
                    or envelope.get("drive_authorized") is not False):
                raise ValueError("ROS 재생 발행에는 재생 뷰만 허용합니다")
            if envelope.get("result") is None:
                state, error = envelope.get("state"), envelope.get("error")
                if (state not in ("waiting", "stale", "ended", "error") or envelope.get("valid") is not False
                        or (error is not None and (type(error) is not str
                            or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", error)))):
                    raise ValueError("좌표 없는 재생 상태의 필드가 잘못되었습니다")
                # 결과가 없어도 대기·오류 상태는 전한다. 위치·군집·장착 좌표는 복사하지 않는다.
                status = {"schema": "smartcart.tracking_view.v1", "mode": "replay", "source_mode": "replay",
                          "state": state, "frame_id": "base_link", "valid": False, "provisional": True,
                          "drive_authorized": False, "result": None, "error": error}
                return None, status
            payload = envelope["result"]
            if type(payload) is not dict or set(payload) != {"uwb_estimate", "lidar_clusters", "track"}:
                raise ValueError("재생 표시의 정제된 최신 결과가 필요합니다")
            result = _result({**payload, "source_mode": "replay", "drive_authorized": False}, "replay")
            if envelope.get("state") in ("stale", "ended", "error", "off") and result["track"].get("valid"):
                raise ValueError("만료되거나 끝난 뷰의 유효 위치를 발행하지 않습니다")
            return result, result

        def emit(names):
            try:
                result, status = snapshot()
                for name in names:
                    if name != "status" and result is None:
                        continue
                    value = status if name == "status" else result["lidar_clusters" if name == "clusters" else "track"]
                    message = String()
                    message.data = _json(value)
                    publishers[name].publish(message)
            except Exception:
                _safe_error(tracking_store, "tracking_replay_ros_error")

        node.create_timer(.1, lambda: emit(("target",)))
        node.create_timer(.5, lambda: emit(("clusters", "status")))
        executor = Executor(context=context)
        executor.add_node(node)
        while not tracking_store.stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception:
        _safe_error(tracking_store, "tracking_replay_ros_error")
    finally:
        _cleanup(executor, node, context)
