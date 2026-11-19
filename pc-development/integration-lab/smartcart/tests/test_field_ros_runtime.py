"""Linux/Humble의 실제 DDS를 사용하는 구독 전용 현장 집계 통합 검사.

입력 두 개는 시험 노드가 생성한 합성 상태와 스캔이다. live는 운영 수신 경로의
출처 값이며 실물 USB·RF·사람·모터의 검수 완료를 뜻하지 않는다. 도메인 97과
고유 토픽을 사용하고 운영 모니터는 독립 문맥·스레드에서 실행한다.
"""
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

from smartcart_field.monitor import FieldMonitor
from smartcart_field.ros_monitor import run_monitor
from smartcart_tracking.config import TrackingConfig
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.observation import make_observation

ROOT = Path(__file__).resolve().parents[1]
SENSORS = ("uwb_left", "uwb_right", "lidar_center")
FORBIDDEN_FIELDS = {"position_xy_m", "predicted_position_xy_m", "track_id", "logical_target_id",
                    "tag_address", "range_m", "bearing_rad", "raw_angle_deg", "ranges",
                    "first_path_power", "rx_level", "adapter_session"}
ROS_AVAILABLE = False
ROS_REASON = "Linux에서 source한 ROS 2 Humble의 rclpy·sensor_msgs·std_msgs가 필요합니다"
try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    ROS_AVAILABLE = sys.platform.startswith("linux") and os.environ.get("ROS_DISTRO") == "humble"
except ImportError:
    pass


def assert_no_sensitive_fields(test, value):
    if isinstance(value, dict):
        test.assertFalse(FORBIDDEN_FIELDS.intersection(value), "원시 관측·위치·대상 식별자가 보관되었습니다")
        for item in value.values():
            assert_no_sensitive_fields(test, item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            assert_no_sensitive_fields(test, item)


@unittest.skipUnless(ROS_AVAILABLE, ROS_REASON)
class FieldRosRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.context = self.node = self.executor = self.worker = None
        self.stop = threading.Event()
        self.outcomes, self.errors = [], []
        self.sequence = 0
        self.environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "97", "ROS_LOCALHOST_ONLY": "1"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.addCleanup(self.cleanup_ros)
        suffix = str(os.getpid()) + "_" + uuid4().hex[:8]
        self.session = "synthetic_field_" + suffix
        self.scan_topic = "/synthetic_field_scan_" + suffix
        self.status_topic = "/synthetic_field_status_" + suffix
        self.monitor_node_name = "smartcart_field_monitor_" + str(os.getpid())
        self.mounting = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        self.uwb = EstimationConfig()
        self.settings = TrackingConfig()
        self.monitor = FieldMonitor(self.mounting, self.uwb, self.settings, window_size=32)
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("synthetic_field_input_" + suffix, context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        # 실제 DDS 입력 발행자는 이 시험 노드의 두 토픽뿐이다.
        self.scan_publisher = self.node.create_publisher(LaserScan, self.scan_topic, qos)
        self.status_publisher = self.node.create_publisher(String, self.status_topic, qos)

        def worker():
            try:
                self.outcomes.append(run_monitor(self.monitor, duration_seconds=20,
                    scan_topic=self.scan_topic, status_topic=self.status_topic, stop=self.stop))
            except BaseException as error:
                self.errors.append(type(error).__name__)

        self.worker = threading.Thread(target=worker, name="synthetic-field-readonly-dds", daemon=True)
        self.worker.start()

    def cleanup_ros(self):
        self.stop.set()
        if self.worker is not None and self.worker.ident is not None:
            self.worker.join(timeout=3)
        failures = []
        for cleanup in (
            lambda: self.executor.shutdown(timeout_sec=1) if self.executor is not None else None,
            lambda: self.node.destroy_node() if self.node is not None else None,
            lambda: self.context.shutdown() if self.context is not None and self.context.ok() else None,
        ):
            try:
                cleanup()
            except Exception:
                failures.append("observer_cleanup_failed")
        self.assertFalse(self.worker is not None and self.worker.is_alive(), "현장 DDS 모니터가 종료되지 않았습니다")
        self.assertEqual(failures, [])

    def wait_until(self, predicate, description, *, pump=None, timeout=6):
        deadline, next_pump = time.monotonic() + timeout, 0.0
        while time.monotonic() < deadline:
            if pump is not None and time.monotonic() >= next_pump:
                pump()
                next_pump = time.monotonic() + .04
            self.executor.spin_once(timeout_sec=.005)
            self.assertEqual(self.errors, [], "현장 모니터 스레드에 예외가 발생했습니다")
            if predicate():
                return
            if not self.worker.is_alive() and not self.stop.is_set():
                self.fail("운영 모니터가 조기에 종료되었습니다: " + description)
        self.fail("실제 DDS 대기 상한을 초과했습니다: " + description)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            self.assertTrue(self.worker.is_alive(), "운영 모니터가 수명 평가 중 종료되었습니다")
            self.assertEqual(self.errors, [])

    def publish_inputs(self):
        self.sequence += 1
        now = time.monotonic_ns()
        sensors = {}
        for name in ("uwb_left", "uwb_right"):
            observation = make_observation(sensor=self.mounting.sensors[name],
                frame={"sequence": self.sequence & 255, "tag_address": self.mounting.allowed_tag_address,
                       "angle_deg": 0, "distance_cm": 200, "first_path_power": -90, "rx_level": -80},
                allowed_tag_address=self.mounting.allowed_tag_address, logical_target_id=self.mounting.logical_target_id,
                sample_sequence=self.sequence, stream_epoch=0, adapter_session=self.session,
                source_mode="live", rx_ns=now, source_rx_ns=None,
                valid_until_ns=now + self.mounting.timeout_ms * 1_000_000)
            sensors[name] = {"state": "live", "connected": True, "stream_epoch": 0,
                             "latest": observation, "last_error": None}
        value = {"schema": "smartcart.dual_uwb_status.v1", "adapter_session": self.session,
                 "source_mode": "live", "drive_authorized": False, "sensors": sensors}
        self.status_publisher.publish(String(data=json.dumps(value, allow_nan=False)))
        scan = LaserScan()
        scan.header.frame_id = self.mounting.lidar.frame_id
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -.08, .08, .04
        scan.range_min, scan.range_max = .05, 10.0
        scan.scan_time, scan.time_increment = .02, .005
        scan.ranges = [2.0] * 5
        self.scan_publisher.publish(scan)

    def all_state(self, state, *, minimum_samples=1):
        sensors = self.monitor.report()["sensors"]
        return all(item["state"] == state and item["counts"].get("new_samples", 0) >= minimum_samples
                   for item in sensors.values())

    def assert_graph_readonly(self):
        self.assertIn((self.monitor_node_name, "/"), self.node.get_node_names_and_namespaces())
        self.assertEqual(self.node.get_publisher_names_and_types_by_node(self.monitor_node_name, "/"), [],
                         "현장 모니터 노드에 발행자가 있습니다")
        self.assertEqual(self.node.get_service_names_and_types_by_node(self.monitor_node_name, "/"), [],
                         "현장 모니터 노드에 서비스가 있습니다")
        subscriptions = {name for name, _ in self.node.get_subscriber_names_and_types_by_node(self.monitor_node_name, "/")}
        self.assertEqual(subscriptions, {self.scan_topic, self.status_topic})
        topics = {name for name, _ in self.node.get_topic_names_and_types()}
        self.assertFalse(topics & {"/cmd_vel", "/cmd_vel_unstamped", "/tf", "/tf_static"})

    def test_synthetic_dds_freshness_loss_recovery_readonly_graph_and_no_raw_retention(self):
        self.wait_until(lambda: self.node.count_subscribers(self.scan_topic) == 1 and
                        self.node.count_subscribers(self.status_topic) == 1 and
                        (self.monitor_node_name, "/") in self.node.get_node_names_and_namespaces(),
                        "독립 문맥의 현장 구독 노드 발견")
        self.wait_until(lambda: self.all_state("fresh", minimum_samples=4), "세 센서의 신선한 합성 입력",
                        pump=self.publish_inputs)
        self.assert_graph_readonly()
        before = self.monitor.report()
        self.assertGreaterEqual(before["counts"].get("status_messages", 0), 4)
        self.assertGreaterEqual(before["counts"].get("scan_messages", 0), 4)

        # 어떠한 수동 unplug/replug 선언도 하지 않고 DDS 입력만 중단한다.
        self.spin_for(.55)
        self.wait_until(lambda: self.all_state("stale"), "입력 중단 후 20Hz 타이머의 세 센서 수명 만료", timeout=2)
        stale = self.monitor.report()
        for name in SENSORS:
            self.assertGreaterEqual(stale["sensors"][name]["counts"].get("freshness_losses", 0), 1)
            self.assertGreaterEqual(stale["sensors"][name]["counts"].get("stale_transitions", 0), 1)

        minimum = max(item["counts"]["new_samples"] for item in stale["sensors"].values()) + 4
        self.wait_until(lambda: self.all_state("fresh", minimum_samples=minimum), "합성 DDS 새 입력의 관측 복구",
                        pump=self.publish_inputs)
        recovered = self.monitor.report()
        for name in SENSORS:
            self.assertGreaterEqual(recovered["sensors"][name]["counts"].get("recoveries", 0), 1)
            self.assertEqual(recovered["manual_unplug_tests"][name]["completed_cycles"], 0)
            self.assertEqual(recovered["manual_unplug_tests"][name]["status"], "미시험")
        self.assertFalse(recovered["completion"])
        self.assertFalse(recovered["physical_acceptance"])
        self.assertFalse(recovered["drive_authorized"])
        self.assertEqual(recovered["counts"].get("ros_runtime", 0), 0)
        self.assertEqual(recovered["counts"].get("invalid_status_messages", 0), 0)
        self.assertEqual(recovered["counts"].get("scan_message", 0), 0)
        self.assert_graph_readonly()
        assert_no_sensitive_fields(self, recovered)
        # 검증용 설정의 허용 태그는 필요하지만 수신 원시 관측이나 계산 위치를 보관하면 안 된다.
        for sensor in self.monitor._sensors.values():
            assert_no_sensitive_fields(self, vars(sensor))
        self.assertEqual(self.monitor._validator._seen, {})
        self.assertIsNone(self.monitor._validator._current)
        self.assertIsNone(self.monitor._validator._geometry_value)
        self.assertIsNone(self.monitor._tracking_latest)

        self.stop.set()
        self.worker.join(timeout=3)
        self.assertFalse(self.worker.is_alive())
        self.assertEqual(self.errors, [])
        self.assertEqual(len(self.outcomes), 1)
        self.assertFalse(self.outcomes[0]["completion"])
        assert_no_sensitive_fields(self, self.outcomes[0])
        self.wait_until(lambda: self.node.count_subscribers(self.scan_topic) == 0 and
                        self.node.count_subscribers(self.status_topic) == 0 and
                        (self.monitor_node_name, "/") not in self.node.get_node_names_and_namespaces(),
                        "정상 종료 후 DDS 구독·노드 자원 반환", timeout=3)


if __name__ == "__main__":
    unittest.main()
