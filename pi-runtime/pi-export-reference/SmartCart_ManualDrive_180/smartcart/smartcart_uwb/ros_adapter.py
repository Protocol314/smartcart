"""실물 관측만 별도 ROS 토픽에 발행하며 기존 단일 UWB 토픽은 유지한다."""
from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time

from .collector import DualCollector
from .config import load_config
from .native import NativeParser
from .observation import OBSERVATION_SCHEMA, timestamp_ns
from .serial_reader import SerialReaders

OBSERVATION_TOPICS = {sensor: f"/smartcart/uwb/{side}/observation"
                      for sensor, side in (("uwb_left", "left"), ("uwb_right", "right"))}
STATUS_TOPIC = "/smartcart/uwb/dual/status"


def observation_message(observation: dict, message_type, *, now_ns: int):
    """출력 직전에도 TTL과 읽기 전용 출처를 확인한다. ROS 없이 단위 시험 가능하다."""
    if (not isinstance(observation, dict) or observation.get("schema") != OBSERVATION_SCHEMA
            or observation.get("sensor_id") not in OBSERVATION_TOPICS
            or observation.get("source_mode") != "live"
            or observation.get("timestamp_source") != "HOST_RX"
            or observation.get("drive_authorized") is not False):
        raise ValueError("실물 읽기 전용 관측만 ROS 실측 토픽으로 내보낼 수 있습니다")
    timestamp_ns(now_ns)
    rx = timestamp_ns(observation["rx_monotonic_ns"])
    deadline = timestamp_ns(observation["valid_until_monotonic_ns"])
    value = deepcopy(observation)
    if value["valid"] and (now_ns < rx or now_ns >= deadline):
        value.update(valid=False, invalid_reason="stale")
    message = message_type()
    for key in ("schema", "sensor_id", "frame_id", "adapter_session", "source_mode",
                "timestamp_source", "rx_monotonic_ns", "valid_until_monotonic_ns",
                "sample_sequence", "stream_epoch", "measurement_valid", "id_valid",
                "valid", "mounting_verified", "calibration_profile"):
        setattr(message, key, value[key])
    message.invalid_reason = value["invalid_reason"] or ""
    message.drive_authorized = False
    message.device_timestamp_available = False
    message.device_timestamp_ns = 0
    message.source_rx_available = value["source_rx_monotonic_ns"] is not None
    message.source_rx_monotonic_ns = value["source_rx_monotonic_ns"] or 0
    message.frame_available = value["tag_address"] is not None
    message.logical_target_id = value["logical_target_id"] if value["logical_target_id"] is not None else -1
    for key in ("tag_address", "device_sequence", "raw_angle_deg", "first_path_power", "rx_level"):
        setattr(message, key, value[key] if value[key] is not None else 0)
    for key in ("range_m", "bearing_rad"):
        number = value[key]
        if number is not None and not math.isfinite(number):
            raise ValueError("ROS 관측값은 유한한 수여야 합니다")
        setattr(message, key, float(number) if number is not None else 0.0)
    return message


class ObservationQueue:
    """용량을 넘으면 오래된 묶음을 버리고 수집 상태를 무효화한다."""

    def __init__(self, collector, capacity: int = 256):
        self.collector, self.capacity = collector, capacity
        self.lock = threading.Lock()
        self.items = deque()

    def append(self, observation: dict) -> None:
        overflow = False
        with self.lock:
            if len(self.items) >= self.capacity:
                self.items.clear()
                overflow = True
            else:
                self.items.append(deepcopy(observation))
        if overflow:
            self.collector.invalidate(observation["sensor_id"], reason="ros_queue_overflow")

    def drain(self) -> list[dict]:
        with self.lock:
            values = list(self.items)
            self.items.clear()
            return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="좌우 BU04 실측 ROS 어댑터; 주행 출력 없음")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--left-only", action="store_true", help="왼쪽 BU04만 읽습니다")
    options, ros_arguments = parser.parse_known_args(argv)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
        from smartcart_uwb_interfaces.msg import UwbObservation
    except ImportError as exc:
        raise RuntimeError("ROS 2와 빌드된 smartcart_uwb_interfaces가 필요합니다") from exc
    config = load_config(options.config)
    factory = lambda: NativeParser(options.native_library)
    with DualCollector(config, parser_factory=factory, source_mode="live") as collector:
        if options.left_only:
            collector.disconnected("uwb_right", reason="disabled_single_left")
        queue = ObservationQueue(collector)
        reader_options = {"observation_callback": queue.append}
        if options.left_only:
            reader_options["sensor_ids"] = ("uwb_left",)
        readers = SerialReaders(collector, **reader_options)
        context = rclpy.context.Context()
        node = None
        executor = None
        started = False
        try:
            rclpy.init(args=ros_arguments, context=context)
            node = rclpy.create_node("smartcart_dual_uwb", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
            publishers = {key: node.create_publisher(UwbObservation, topic, qos)
                          for key, topic in OBSERVATION_TOPICS.items()}
            status_publisher = node.create_publisher(String, STATUS_TOPIC, qos)

            def publish_observations():
                values = queue.drain()
                for value in values:
                    publishers[value["sensor_id"]].publish(
                        observation_message(value, UwbObservation, now_ns=time.monotonic_ns()))

            def publish_status():
                value = String()
                value.data = json.dumps(readers.tick(),
                                        ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                status_publisher.publish(value)

            node.create_timer(.02, publish_observations)
            node.create_timer(.1, publish_status)
            readers.start()
            started = True
            while context.ok() and not readers.stop.is_set():
                executor.spin_once(timeout_sec=.1)
            if readers.fatal_error:
                raise RuntimeError(readers.fatal_error)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                if started:
                    readers.close()
            finally:
                try:
                    if executor is not None:
                        executor.shutdown(timeout_sec=2)
                finally:
                    try:
                        if node is not None:
                            node.destroy_node()
                    finally:
                        if context.ok():
                            rclpy.shutdown(context=context)
    return 0
