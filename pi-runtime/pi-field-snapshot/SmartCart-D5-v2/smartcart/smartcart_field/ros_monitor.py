"""센서·추적 ROS 토픽만 구독하는 현장 모니터. 발행자·서비스·장치 접근은 없다."""
from __future__ import annotations

import math
import os
from threading import Event
import time

from smartcart_sensing.ros import validate_topic
from smartcart_tracking.ros_runtime import decode_status
from .monitor import FieldMonitor


def run_monitor(monitor: FieldMonitor, *, duration_seconds=300, scan_topic="/scan",
                status_topic="/smartcart/uwb/dual/status", tracking_status_topic=None,
                stop=None, ros_args=None):
    """독립 문맥·20Hz 수명 검사로 제한 시간 동안 구독하고 항상 정리를 시도한다."""
    if not isinstance(monitor, FieldMonitor) or monitor.source_mode != "live":
        raise ValueError("실시간 현장 관측 모니터가 필요합니다")
    if (type(duration_seconds) not in (int, float) or not math.isfinite(duration_seconds)
            or not 1 <= duration_seconds <= 3600):
        raise ValueError("구독 시간은 1~3600초의 유한한 수여야 합니다")
    topics = [scan_topic, status_topic] + ([] if tracking_status_topic is None else [tracking_status_topic])
    for topic in topics:
        validate_topic(topic)
    if len(set(topics)) != len(topics):
        raise ValueError("현장 관측 구독 토픽은 서로 달라야 합니다")
    stop = Event() if stop is None else stop
    if not hasattr(stop, "is_set"):
        raise ValueError("종료 이벤트가 필요합니다")
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError("ROS 2 환경을 source한 Python이 필요합니다") from error
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    context = node = executor = None
    monitor.begin(time.monotonic_ns())
    try:
        context = Context()
        rclpy.init(args=[] if ros_args is None else ros_args, context=context,
                   signal_handler_options=SignalHandlerOptions.NO)
        node = Node("smartcart_field_monitor_" + str(os.getpid()), context=context,
                    use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
        # Humble은 서비스 비활성 여부와 별개로 /parameter_events 발행자를 자동 생성한다.
        # 공개 API로 제거하고 이후 매개변수 변경 없이 구독·타이머만 사용한다.
        for publisher in tuple(node.publishers):
            if not node.destroy_publisher(publisher):
                raise RuntimeError("자동 ROS 발행자를 제거하지 못했습니다")
        if tuple(node.publishers):
            raise RuntimeError("읽기 전용 모니터에 ROS 발행자가 남았습니다")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

        def status_callback(message):
            now = time.monotonic_ns()
            try:
                monitor.observe_status(decode_status(message.data), now)
            except Exception:
                monitor.note_error("status_json", now)

        def scan_callback(message):
            now = time.monotonic_ns()
            try:
                if (not 1 <= len(message.ranges) <= monitor.settings.max_rays
                        or len(message.intensities) > monitor.settings.max_rays):
                    raise ValueError()
                scan = {name: getattr(message, name) for name in (
                    "angle_min", "angle_max", "angle_increment", "range_min", "range_max", "scan_time", "time_increment")}
                scan["ranges"] = list(message.ranges)
                monitor.observe_scan(scan, now, frame_id=message.header.frame_id)
            except Exception:
                monitor.note_error("scan_message", now)

        def tracking_callback(message):
            now = time.monotonic_ns()
            try:
                monitor.observe_tracking_status(decode_status(message.data), now)
            except Exception:
                monitor.note_error("tracking_json", now)

        node.create_subscription(String, status_topic, status_callback, qos)
        node.create_subscription(LaserScan, scan_topic, scan_callback, qos)
        if tracking_status_topic is not None:
            node.create_subscription(String, tracking_status_topic, tracking_callback, qos)
        node.create_timer(.05, lambda: monitor.tick(time.monotonic_ns()))
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + duration_seconds
        while not stop.is_set() and context.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.05, max(0, deadline - time.monotonic())))
        if time.monotonic() < deadline:
            monitor.note_error("ros_interrupted", time.monotonic_ns())
    except KeyboardInterrupt:
        monitor.note_error("ros_interrupted", time.monotonic_ns())
        # CLI의 130 처리에 전달한다. finally에서 독립 ROS 문맥부터 정리한다.
        raise
    except Exception:
        monitor.note_error("ros_runtime", time.monotonic_ns())
    finally:
        # 한 정리 단계의 실패가 나머지 문맥 정리를 생략하지 않게 한다.
        for cleanup in (
            lambda: executor.shutdown(timeout_sec=1) if executor is not None else None,
            lambda: node.destroy_node() if node is not None else None,
            lambda: context.shutdown() if context is not None and context.ok() else None,
        ):
            try:
                cleanup()
            except Exception:
                monitor.note_error("ros_runtime", time.monotonic_ns())
        monitor.tick(time.monotonic_ns())
    return monitor.report()
