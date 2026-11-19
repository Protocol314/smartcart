"""실제 rclpy/DDS 전송 검증. 모든 입력은 테스트가 만든 가상 자료이며 실물 측정이 아니다.

ROS 2가 없는 개발 PC에서는 명시적으로 건너뛴다. ROS 설치 환경에서는 전용
도메인 95·localhost·실행별 토픽을 사용하며 실제 /scan 또는 ESP32 포트를 열지 않는다.
도메인 245는 DDS 포트 상한을 초과하므로 사용하지 않는다.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import unittest
import uuid
from collections import deque
from unittest.mock import patch

from smartcart_radar.core import ScanStore
from smartcart_sensing.ros import encode_uwb_snapshot, ros_loop
from smartcart_sensing.uwb import UwbStore

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from geometry_msgs.msg import PointStamped
    from diagnostic_msgs.msg import DiagnosticArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


def test_diagnostic_line(sequence: int, *, accepted_id: int = 0) -> str:
    """전송 검증용 가상 ESP32 진단. 실제 장치와 연결하지 않는다."""
    return (f"C_KEY_DIAG_V3,{sequence * 20 % 4294967296},{sequence % 256},28185,0,{accepted_id},"
            "1,1,1,183,9,-8100,-7500,1825.0,1825.0,9.46,0.300,1.800,1.525,"
            f"WELCOME,0,{sequence % 4294967296},0,0,0")


@unittest.skipUnless(ROS_AVAILABLE, "rclpy가 없어 실제 ROS 2 전송 검증을 건너뜁니다")
class RosRuntimeTests(unittest.TestCase):
    """한 테스트의 관측 대기 상한 12초와 제한된 종료 대기를 적용한다."""
    def setUp(self):
        self.deadline = time.monotonic() + 12
        self.context = self.node = self.executor = self.worker = None
        self.stop = threading.Event()
        self.sequence = 0
        self.environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "95", "ROS_LOCALHOST_ONLY": "1"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.addCleanup(self.cleanup_ros)
        suffix = f"{os.getpid()}_{uuid.uuid4().hex[:8]}"
        self.scan_topic = f"/test_scan_{suffix}"
        self.uwb_topic = f"/test_uwb_{suffix}"
        self.diagnostic_topic = f"/test_sensing_diagnostics_{suffix}"
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node(f"smartcart_runtime_test_{suffix}", context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.scan_publisher = self.node.create_publisher(LaserScan, self.scan_topic, self.qos)
        self.diagnostics = deque(maxlen=100)
        self.node.create_subscription(DiagnosticArray, self.diagnostic_topic,
                                      self.diagnostics.append, self.qos)

    def cleanup_ros(self):
        self.stop.set()
        worker_alive = False
        if self.worker is not None:
            self.worker.join(timeout=1.5)
            worker_alive = self.worker.is_alive()
        try:
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=.5)
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            finally:
                if self.context is not None and self.context.ok():
                    self.context.shutdown()
        self.assertFalse(worker_alive, "ROS 센싱 worker가 종료되지 않았습니다")

    def start_bridge(self, uwb_store: UwbStore, *, publish_uwb: bool):
        self.lidar = ScanStore("ros", stale_seconds=.35)
        self.lidar.stop = self.stop
        self.uwb = uwb_store
        self.uwb.stop = self.stop
        self.worker = threading.Thread(target=ros_loop, args=(self.lidar, self.uwb), kwargs={
            "topic": self.scan_topic, "uwb_topic": self.uwb_topic, "publish_uwb": publish_uwb,
            "ros_args": ["--ros-args", "-r", "/smartcart/sensing/diagnostics:=" + self.diagnostic_topic],
        }, daemon=True)
        self.worker.start()

    def publish_test_scan(self):
        """ROS 표준 메시지는 출처 플래그가 없으므로 이름·프레임으로 테스트 입력임을 명시한다."""
        message = LaserScan()
        message.header.frame_id = "synthetic_test_laser"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angle_min, message.angle_max = -math.pi, math.pi
        message.angle_increment = math.pi / 2
        message.range_min, message.range_max = .05, 16.0
        message.scan_time, message.time_increment = .1, .02
        message.ranges = [1.0, 2.0, float("inf"), 3.0, 4.0]
        self.scan_publisher.publish(message)

    def next_line(self, *, accepted_id=0):
        self.sequence += 1
        return test_diagnostic_line(self.sequence, accepted_id=accepted_id)

    def wait_until(self, predicate, description, *, pump=None):
        while time.monotonic() < self.deadline:
            if pump is not None:
                pump()
            self.executor.spin_once(timeout_sec=.03)
            if predicate():
                return
            if self.worker is not None and not self.worker.is_alive():
                self.fail(f"ROS worker가 종료되었습니다: {description}, {self.lidar.snapshot()['error']}")
        self.fail(f"실제 ROS 전송 대기 시간 초과: {description}; "
                  f"LiDAR={self.lidar.snapshot()['state']}, UWB={self.uwb.snapshot()['state']}")

    def spin_for(self, seconds, *, pump=None):
        until = min(self.deadline, time.monotonic() + seconds)
        while time.monotonic() < until:
            if pump is not None:
                pump()
            self.executor.spin_once(timeout_sec=.03)
        self.assertLess(time.monotonic(), self.deadline, "ROS 테스트 전체 대기 상한을 초과했습니다")

    def levels(self):
        if not self.diagnostics:
            return {}
        return {status.name: int.from_bytes(status.level, "little")
                for status in self.diagnostics[-1].status}

    def test_laserscan_and_uwb_json_subscription_coordinates_duplicates_and_timeout(self):
        """독립 Context 발행→실제 DDS→통합 수신을 가상 입력으로 검증한다."""
        sender = UwbStore("serial", yaw_deg=90, offset_x=.2)
        publisher = self.node.create_publisher(String, self.uwb_topic, self.qos)
        self.start_bridge(UwbStore("ros"), publish_uwb=False)

        def publish_fresh():
            self.publish_test_scan()
            sender.publish_line(self.next_line())
            publisher.publish(String(data=encode_uwb_snapshot(sender.snapshot())))

        self.wait_until(lambda: self.lidar.snapshot()["state"] == "live"
                        and self.uwb.snapshot()["state"] == "live"
                        and self.levels() == {"smartcart/lidar": 0, "smartcart/uwb": 0},
                        "가상 LaserScan/UWB 수신과 통합 진단", pump=publish_fresh)
        scan = self.lidar.snapshot()["scan"]
        self.assertEqual(scan["ranges"], [1, 2, None, 3, 4])
        position = self.uwb.snapshot()["measurement"]["position"]
        self.assertAlmostEqual(position["x"], .5)
        self.assertAlmostEqual(position["y"], 1.8)
        self.assertEqual(position["frame_id"], "base_link")
        self.assertEqual(self.uwb.snapshot()["transform"]["yaw_deg"], 90)

        frozen_snapshot = sender.snapshot()
        frozen = encode_uwb_snapshot(frozen_snapshot)

        def publish_duplicate():
            self.publish_test_scan()
            publisher.publish(String(data=frozen))

        self.wait_until(lambda: (self.uwb.snapshot().get("measurement") or {}).get("raw_diagnostic_line")
                        == frozen_snapshot["measurement"]["raw_diagnostic_line"],
                        "중복 검증 기준 프레임 수신 동기화", pump=publish_duplicate)
        previous_time = self.uwb.last_received
        self.wait_until(lambda: self.uwb.snapshot()["state"] == "stale"
                        and self.levels() == {"smartcart/lidar": 0, "smartcart/uwb": 3},
                        "중복 UWB 수신의 생존 시간 만료", pump=publish_duplicate)
        self.assertEqual(self.uwb.last_received, previous_time)
        self.assertIsNone(self.uwb.snapshot()["measurement"]["position"])

        def publish_invalid_id():
            self.publish_test_scan()
            sender.publish_line(self.next_line(accepted_id=1))
            publisher.publish(String(data=encode_uwb_snapshot(sender.snapshot())))

        self.wait_until(lambda: self.uwb.snapshot()["state"] == "live"
                        and not self.uwb.snapshot()["measurement"]["valid"]
                        and self.levels().get("smartcart/uwb") == 2,
                        "실제 DDS에서 ID 불일치 무효화", pump=publish_invalid_id)
        self.assertIsNone(self.uwb.snapshot()["measurement"]["position"])
        self.wait_until(lambda: self.lidar.snapshot()["state"] == "stale"
                        and self.levels().get("smartcart/lidar") == 3,
                        "LaserScan 중단 후 통합 진단의 시간 초과")

    def test_serial_mode_store_publishes_json_position_and_diagnostics_without_stale_reuse(self):
        """가상 진단을 serial 모드 저장소에 주입하며 실제 UART 장치를 열지 않는다."""
        strings, points, legacy_diagnostics = deque(maxlen=200), deque(maxlen=200), deque(maxlen=200)
        self.node.create_subscription(String, self.uwb_topic, lambda message: strings.append(json.loads(message.data)), self.qos)
        self.node.create_subscription(PointStamped, self.uwb_topic + "/position", points.append, self.qos)
        self.node.create_subscription(DiagnosticArray, self.uwb_topic + "/diagnostics", legacy_diagnostics.append, self.qos)
        self.start_bridge(UwbStore("serial"), publish_uwb=True)

        def inject_fresh():
            self.publish_test_scan()
            self.uwb.publish_line(self.next_line())

        self.wait_until(lambda: bool(strings and points and legacy_diagnostics)
                        and self.levels() == {"smartcart/lidar": 0, "smartcart/uwb": 0},
                        "가상 serial 입력의 JSON/위치/진단 DDS 발행", pump=inject_fresh)
        self.assertEqual(strings[-1]["schema"], "smartcart.uwb.v1")
        self.assertEqual(strings[-1]["state"], "live")
        self.assertTrue(strings[-1]["measurement"]["valid"])
        self.assertEqual(points[-1].header.frame_id, "base_link")
        self.assertAlmostEqual(points[-1].point.x, 1.8)
        self.assertAlmostEqual(points[-1].point.y, -.3)
        self.assertGreater(points[-1].header.stamp.sec, 0)
        self.assertEqual(legacy_diagnostics[-1].status[0].name, "smartcart/uwb")

        self.wait_until(lambda: bool(strings) and strings[-1]["state"] == "stale"
                        and self.levels().get("smartcart/uwb") == 3,
                        "UWB 입력 중단 시 위치 발행 중지", pump=self.publish_test_scan)
        # 만료 직전에 전송되어 executor에 남은 메시지를 먼저 소진한다.
        self.spin_for(.15, pump=self.publish_test_scan)
        count_after_stale = len(points)
        self.spin_for(.25, pump=self.publish_test_scan)
        self.assertEqual(len(points), count_after_stale)
        self.assertIsNone(strings[-1]["measurement"]["position"])
        self.assertFalse(strings[-1]["measurement"]["valid"])

        self.uwb.set_error("테스트에서 주입한 수신 오류")
        self.wait_until(lambda: strings[-1]["state"] == "error"
                        and self.levels().get("smartcart/uwb") == 2,
                        "수신 오류 상태의 DDS 진단 전달", pump=self.publish_test_scan)
        self.assertEqual(len(points), count_after_stale)
        self.assertIsNone(strings[-1]["measurement"]["position"])


if __name__ == "__main__":
    unittest.main()
