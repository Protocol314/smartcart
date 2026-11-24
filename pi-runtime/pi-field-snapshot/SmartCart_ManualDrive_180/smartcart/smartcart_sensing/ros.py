"""ROS 2 수신과 읽기 전용 UWB 진단 발행. 모터·잠금·센서 설정 명령은 없다."""
from __future__ import annotations

import json
import math
import os
import re

from smartcart_radar.core import normalise_scan
from .uwb import MAX_ROS_JSON_BYTES, SCHEMA, UwbStore

SENSING_DIAGNOSTICS_TOPIC = "/smartcart/sensing/diagnostics"


def validate_topic(topic: str) -> str:
    """명시적인 절대 ROS 토픽 이름만 사용한다."""
    if (not isinstance(topic, str) or not 2 <= len(topic) <= 200
            or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z_0-9]*/)*[A-Za-z_][A-Za-z_0-9]*", topic)):
        raise ValueError("유효한 절대 ROS 토픽 이름이 필요합니다")
    return topic


def scan_from_message(message, topic: str = "/scan") -> dict:
    """LaserScan을 원래 각도 인덱스를 유지한 JSON 자료로 검증한다."""
    if len(message.ranges) > 100000 or len(message.intensities) > 100000:
        raise ValueError("과도하게 큰 LaserScan 배열입니다")
    return normalise_scan({
        "header": {"frame_id": message.header.frame_id,
                   "stamp": {"sec": message.header.stamp.sec,
                             "nanosec": message.header.stamp.nanosec}},
        "angle_min": message.angle_min, "angle_max": message.angle_max,
        "angle_increment": message.angle_increment, "scan_time": message.scan_time,
        "time_increment": message.time_increment, "range_min": message.range_min,
        "range_max": message.range_max, "ranges": list(message.ranges),
        "intensities": list(message.intensities),
    }, kind="ros", label=topic)


def encode_uwb_snapshot(snapshot: dict) -> str:
    """ROS String에 들어갈 유한 JSON을 제한된 길이로 직렬화한다."""
    if not isinstance(snapshot, dict) or snapshot.get("schema") != SCHEMA:
        raise ValueError("지원하지 않는 UWB 스냅샷입니다")
    encoded = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    if len(encoded.encode("utf-8")) > MAX_ROS_JSON_BYTES:
        raise ValueError("UWB ROS 메시지가 허용 길이를 초과했습니다")
    return encoded


def consume_uwb_json(store: UwbStore, data: str) -> bool:
    """외부 ROS String도 크기와 JSON 타입을 확인한 뒤 UWB 원시 진단을 재검증한다."""
    try:
        if not isinstance(data, str) or len(data) > MAX_ROS_JSON_BYTES:
            raise ValueError("UWB ROS 문자열이 허용 길이를 초과했습니다")
        if len(data.encode("utf-8")) > MAX_ROS_JSON_BYTES:
            raise ValueError("UWB ROS 메시지가 허용 바이트 수를 초과했습니다")
        def reject_constant(value):
            raise ValueError("비유한 JSON 값입니다")
        payload = json.loads(data, parse_constant=reject_constant)
        return store.publish_payload(payload)
    except (ValueError, TypeError, RecursionError, OverflowError):
        store.set_error("잘못된 UWB ROS JSON을 거부했습니다", rejected=True)
        return False


def position_data(snapshot: dict) -> dict | None:
    """신선하고 유효한 실측 위치만 ROS PointStamped 발행 대상으로 반환한다."""
    if snapshot.get("state") != "live" or snapshot.get("synthetic") is not False:
        return None
    measurement = snapshot.get("measurement")
    if (not isinstance(measurement, dict) or measurement.get("valid") is not True
            or measurement.get("authenticated") is not True):
        return None
    position = measurement.get("position")
    if not isinstance(position, dict) or position.get("frame_id") != "base_link":
        return None
    for key in ("x", "y"):
        value = position.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value) > 60:
            return None
    age = snapshot.get("age_ms")
    limit = snapshot.get("stale_after_ms")
    if (isinstance(age, bool) or isinstance(limit, bool) or not isinstance(age, (int, float))
            or not isinstance(limit, (int, float)) or not math.isfinite(age)
            or not math.isfinite(limit) or age < 0 or age > min(limit, 500)):
        return None
    return {"frame_id": "base_link", "x": float(position["x"]),
            "y": float(position["y"]), "z": 0.0, "age_ms": float(age)}


def diagnostic_data(snapshot: dict) -> dict:
    """DiagnosticStatus 수준을 ROS 설치 없이도 검증할 수 있게 결정한다."""
    state = snapshot.get("state", "error")
    measurement = snapshot.get("measurement") or {}
    valid = position_data(snapshot) is not None
    if state in {"waiting", "stale", "off"}:
        level, message = 3, "UWB 신선한 실측 없음"
    elif state == "error":
        level, message = 2, "UWB 수신 또는 입력 검증 오류"
    elif state == "synthetic":
        level, message = 1, "합성 데모: 실측 데이터 아님"
    elif valid:
        level, message = 0, "UWB 실측 수신 중"
    else:
        level, message = 2, "UWB 위치 무효 또는 ID 불일치"
    return {"level": level, "name": "smartcart/uwb", "message": message,
            "hardware_id": "esp32_bu04_bu03",
            "values": {"state": str(state), "valid": str(valid).lower(),
                       "synthetic": str(bool(snapshot.get("synthetic"))).lower(),
                       "age_ms": str(snapshot.get("age_ms")),
                       "firmware_state": str(measurement.get("firmware_state", "")),
                       "rejected_frames": str(snapshot.get("rejected_frames", 0))}}


def lidar_diagnostic_data(snapshot: dict) -> dict:
    """최신 LiDAR 입력과 만료 상태를 구분한다. 합성·재생 입력은 정상 실측으로 표시하지 않는다."""
    state = snapshot.get("state", "error")
    scan = snapshot.get("scan") or {}
    source = scan.get("source") or {}
    synthetic = (state == "synthetic" or snapshot.get("source_mode") == "demo"
                 or source.get("kind") == "synthetic" or source.get("original_kind") == "synthetic")
    age, limit = snapshot.get("age_ms"), snapshot.get("stale_after_ms")
    fresh = (isinstance(age, (int, float)) and not isinstance(age, bool)
             and isinstance(limit, (int, float)) and not isinstance(limit, bool)
             and math.isfinite(age) and math.isfinite(limit) and 0 <= age <= limit)
    valid = (state == "live" and fresh and not synthetic and source.get("kind") == "ros"
             and (scan.get("stats") or {}).get("valid_sensor_rays", 0) > 0)
    if state == "error":
        level, message = 2, "LiDAR 수신 또는 입력 검증 오류"
    elif state == "waiting":
        level, message = 3, "LiDAR 첫 스캔 대기"
    elif state == "stale" or (state == "live" and not fresh):
        level, message = 3, "LiDAR 입력 시간 초과"
    elif synthetic:
        level, message = 1, "합성 LiDAR 데모: 실측 데이터 아님"
    elif state in {"replay", "ended"}:
        level, message = 1, "LiDAR 파일 재생: 현재 실측 데이터 아님"
    elif valid:
        level, message = 0, "LiDAR 실측 스캔 수신 중"
    else:
        level, message = 2, "LiDAR 유효 반사점 없음"
    return {"level": level, "name": "smartcart/lidar", "message": message,
            "hardware_id": "rplidar_c1_ros_scan",
            "values": {"state": str(state), "valid": str(valid).lower(),
                       "synthetic": str(synthetic).lower(), "age_ms": str(age),
                       "receive_hz": str(snapshot.get("receive_hz", 0)),
                       "rejected_scans": str(snapshot.get("rejected_scans", 0))}}


def ros_loop(lidar_store, uwb_store: UwbStore, topic: str = "/scan",
             uwb_topic: str = "/smartcart/uwb", publish_uwb: bool = True,
             ros_args: list[str] | None = None, *, subscribe_scan: bool = True,
             subscribe_uwb: bool = True) -> None:
    """개별 ROS context와 단일 executor를 사용하며 ESP32 포트는 열지 않는다.

    serial 입력은 별도 단일 worker가 담당한다. ROS 수신 모드에서는 자기 토픽으로
    재발행하지 않는다. 위치 상실 때 PointStamped는 중단하고 진단만 계속 발행한다.
    """
    context = node = executor = None
    try:
        validate_topic(topic)
        validate_topic(uwb_topic)
        if topic == uwb_topic:
            raise ValueError("LiDAR와 UWB 토픽은 달라야 합니다")
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from geometry_msgs.msg import PointStamped
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

        context = Context()
        rclpy.init(args=[] if ros_args is None else ros_args,
                    context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node(f"smartcart_sensing_{os.getpid()}", context=context,
                    enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        def scan_callback(message):
            try:
                lidar_store.publish(scan_from_message(message, topic))
            except (ValueError, TypeError, OverflowError):
                lidar_store.set_error("잘못된 ROS LaserScan을 거부했습니다", rejected=True)

        if subscribe_scan:
            node.create_subscription(LaserScan, topic, scan_callback, qos)

        def diagnostics_message(status_items):
            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = node.get_clock().now().to_msg()
            diagnostics.status = []
            for item in status_items:
                status = DiagnosticStatus()
                # DiagnosticStatus.level은 ROS의 byte이므로 Humble Python에서는 길이 1의 bytes다.
                status.level, status.name = bytes([item["level"]]), item["name"]
                status.message, status.hardware_id = item["message"], item["hardware_id"]
                status.values = [KeyValue(key=key, value=value) for key, value in item["values"].items()]
                diagnostics.status.append(status)
            return diagnostics

        if publish_uwb:
            uwb_publisher = node.create_publisher(String, uwb_topic, qos)
            point_publisher = node.create_publisher(PointStamped, uwb_topic + "/position", qos)
            diagnostic_publisher = node.create_publisher(DiagnosticArray, uwb_topic + "/diagnostics", qos)
            last_position_line = None

            def publish_snapshot():
                nonlocal last_position_line
                snapshot = uwb_store.snapshot()
                message = String()
                message.data = encode_uwb_snapshot(snapshot)
                uwb_publisher.publish(message)
                diagnostic_publisher.publish(diagnostics_message([diagnostic_data(snapshot)]))
                position = position_data(snapshot)
                raw_line = (snapshot.get("measurement") or {}).get("raw_diagnostic_line")
                if position is not None and raw_line != last_position_line:
                    point = PointStamped()
                    point.header.frame_id = position["frame_id"]
                    # 수신 후 경과 시간을 빼서 오래된 측정에 현재 시각을 붙이지 않는다.
                    stamp_ns = max(0, node.get_clock().now().nanoseconds - int(position["age_ms"] * 1000000))
                    point.header.stamp.sec = stamp_ns // 1000000000
                    point.header.stamp.nanosec = stamp_ns % 1000000000
                    point.point.x, point.point.y, point.point.z = position["x"], position["y"], 0.0
                    point_publisher.publish(point)
                    last_position_line = raw_line

            node.create_timer(.1, publish_snapshot)
        elif subscribe_uwb:
            node.create_subscription(String, uwb_topic, lambda message: consume_uwb_json(uwb_store, message.data), qos)
        sensing_diagnostics = node.create_publisher(DiagnosticArray, SENSING_DIAGNOSTICS_TOPIC, qos)

        def publish_sensing_diagnostics():
            sensing_diagnostics.publish(diagnostics_message([
                lidar_diagnostic_data(lidar_store.snapshot()), diagnostic_data(uwb_store.snapshot()),
            ]))

        node.create_timer(.5, publish_sensing_diagnostics)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while not lidar_store.stop.is_set() and not uwb_store.stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=.2)
    except Exception as exc:
        lidar_store.set_error(f"ROS 2 센싱 연결 실패 ({type(exc).__name__})")
        if not publish_uwb:
            uwb_store.set_error(f"ROS 2 UWB 연결 실패 ({type(exc).__name__})")
    finally:
        # 일부 초기화 실패에도 나머지 자원을 정리한다.
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
