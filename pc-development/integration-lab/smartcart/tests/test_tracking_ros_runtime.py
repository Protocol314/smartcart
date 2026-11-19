"""Linux/Humble의 실제 DDS 추적 통합 검사. 모든 센서 입력은 시험용 합성 자료다.

실물 USB·LiDAR·사람·모터를 사용하지 않는다. live 표시는 운영 어댑터의 수신 경로를
시험하기 위한 출처 값이며 실물 정확도 검증을 뜻하지 않는다. 도메인 96·localhost와
고유 입력 토픽을 사용한다. ROS가 없으면 명시적으로 건너뛰며 대역으로 통과시키지 않는다.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
import uuid
from unittest.mock import patch

from smartcart_radar.core import ScanStore
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore
from smartcart_sensing.dual_uwb import DualUwbStore
from smartcart_tracking.config import TrackingConfig, load_tracking_config
from smartcart_tracking.ros_runtime import LIVE_TOPICS, REPLAY_TOPICS, tracking_ros_loop, tracking_replay_ros_loop
from smartcart_tracking.view import TrackingStore
from smartcart_uwb.collector import STATUS_SCHEMA
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig, load_estimation_config
from smartcart_uwb.native import default_library_candidates
from smartcart_uwb.observation import make_observation

ROOT = Path(__file__).resolve().parents[1]
ROS_AVAILABLE = False
ROS_REASON = "Linux에서 source한 ROS 2 Humble(rclpy·sensor_msgs·std_msgs)이 필요합니다"
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


@unittest.skipUnless(ROS_AVAILABLE, ROS_REASON)
class TrackingRosRuntimeTests(unittest.TestCase):
    """실제 발행·구독·운영 worker·GUI 저장소를 연결하며 각 대기는 유한하다."""

    def setUp(self):
        self.context = self.node = self.executor = None
        self.worker = self.replay_worker = None
        self.stop = threading.Event()
        self.sequence, self.epoch = 0, 0
        self.last_status = None
        self.environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "96", "ROS_LOCALHOST_ONLY": "1"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.addCleanup(self.cleanup_ros)
        suffix = f"{os.getpid()}_{uuid.uuid4().hex[:8]}"
        self.session = "synthetic_dds_" + suffix
        self.scan_topic = "/synthetic_tracking_scan_" + suffix
        self.status_topic = "/synthetic_tracking_uwb_status_" + suffix
        self.mounting = load_config(ROOT / "config/dual_uwb.d5.example.json")
        self.uwb_settings = EstimationConfig(range_model="horizontal")
        self.settings = TrackingConfig()
        self.lidar = ScanStore("ros", stale_seconds=.3)
        self.lidar.stop = self.stop
        self.tracking = TrackingStore(self.mounting, self.settings, mode="ros", stop=self.stop)
        self.dual = DualUwbStore(self.mounting, self.uwb_settings, stop=self.stop)
        self.gui = SensingStore(self.lidar, UwbStore("off", stop=self.stop), tracking=self.tracking, dual_uwb=self.dual)
        self.received = {name: deque(maxlen=500) for name in LIVE_TOPICS}
        self.replayed = {name: deque(maxlen=500) for name in REPLAY_TOPICS}
        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("synthetic_tracking_probe_" + suffix, context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.scan_publisher = self.node.create_publisher(LaserScan, self.scan_topic, self.qos)
        self.status_publisher = self.node.create_publisher(String, self.status_topic, self.qos)
        for topics, received in ((LIVE_TOPICS, self.received), (REPLAY_TOPICS, self.replayed)):
            for name, topic in topics.items():
                self.node.create_subscription(String, topic,
                    lambda message, destination=received[name]: destination.append(json.loads(message.data)), self.qos)

    def cleanup_ros(self):
        self.stop.set()
        alive = []
        for worker in (self.worker, self.replay_worker):
            if worker is not None:
                worker.join(timeout=2)
                if worker.is_alive():
                    alive.append(worker.name)
        try:
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=1)
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            finally:
                if self.context is not None and self.context.ok():
                    self.context.shutdown()
        self.assertEqual(alive, [], "실제 ROS/재생 worker가 종료되지 않았습니다")

    def wait_until(self, predicate, description, *, pump=None, timeout=5):
        deadline, next_pump = time.monotonic() + timeout, 0.0
        while time.monotonic() < deadline:
            if pump is not None and time.monotonic() >= next_pump:
                pump()
                next_pump = time.monotonic() + .04
            self.executor.spin_once(timeout_sec=.005)
            if predicate():
                return
            if self.worker is not None and not self.worker.is_alive() and not self.stop.is_set():
                self.fail(f"운영 ROS worker 조기 종료: {description}; {self.tracking.snapshot().get('error')}")
        self.fail(f"실제 DDS 대기 시간 초과: {description}; 표시 상태={self.tracking.snapshot()['state']}")

    def start_live(self):
        self.worker = threading.Thread(target=tracking_ros_loop,
            args=(self.lidar, self.tracking, self.mounting, self.uwb_settings, self.settings),
            kwargs={"scan_topic": self.scan_topic, "status_topic": self.status_topic, "dual_uwb_store": self.dual},
            name="synthetic-live-tracking-dds", daemon=True)
        self.worker.start()
        self.wait_until(lambda: self.scan_publisher.get_subscription_count() > 0
                        and self.status_publisher.get_subscription_count() > 0
                        and self.node.count_publishers(LIVE_TOPICS["target"]) == 1,
                        "독립 Context 사이의 운영 구독·발행 발견")

    def synthetic_status(self):
        self.sequence += 1
        now = time.monotonic_ns()
        sensors = {}
        for sensor_id in ("uwb_left", "uwb_right"):
            observation = make_observation(sensor=self.mounting.sensors[sensor_id],
                frame={"sequence": self.sequence & 255, "tag_address": self.mounting.allowed_tag_address,
                       "angle_deg": 0, "distance_cm": 200, "first_path_power": 0, "rx_level": 0},
                allowed_tag_address=self.mounting.allowed_tag_address,
                logical_target_id=self.mounting.logical_target_id, sample_sequence=self.sequence,
                stream_epoch=self.epoch, adapter_session=self.session, source_mode="live",
                rx_ns=now, source_rx_ns=None, valid_until_ns=now + self.mounting.timeout_ms * 1_000_000)
            sensors[sensor_id] = {"state": "live", "connected": True, "stream_epoch": self.epoch,
                                  "latest": observation, "last_error": None}
        return {"schema": STATUS_SCHEMA, "adapter_session": self.session, "source_mode": "live",
                "drive_authorized": False, "sensors": sensors}

    def publish_scan(self, *, empty=False, frame_id=None):
        message = LaserScan()
        message.header.frame_id = self.mounting.lidar.frame_id if frame_id is None else frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angle_min, message.angle_max, message.angle_increment = -.08, .08, .04
        message.range_min, message.range_max = .05, 10.0
        message.scan_time, message.time_increment = .05, .01
        message.ranges = [float("inf") if empty else 2.0] * 5
        self.scan_publisher.publish(message)

    def publish_fresh(self, *, empty=False):
        self.last_status = self.synthetic_status()
        self.status_publisher.publish(String(data=json.dumps(self.last_status, allow_nan=False)))
        self.publish_scan(empty=empty)

    def acquire(self):
        self.wait_until(lambda: self.tracking.snapshot()["valid"], "합성 세 쌍의 실제 DDS 대상 획득",
                        pump=self.publish_fresh)
        return self.tracking.snapshot()["result"]["track"]

    def test_synthetic_full_status_laserscan_reach_dds_outputs_gui_and_cleanup(self):
        self.start_live()
        track = self.acquire()
        self.wait_until(lambda: all(self.received.values()) and any(item["valid"] for item in self.received["target"]),
                        "대상·군집·전체 상태의 실제 DDS 수신", pump=self.publish_fresh)
        self.assertEqual(track["frame_id"], "base_link")
        self.assertGreaterEqual(track["confirmation_count"], 3)
        self.assertAlmostEqual(track["position_xy_m"][0], 2.8138, delta=.02)
        self.assertAlmostEqual(track["position_xy_m"][1], 0, delta=.02)
        display = self.gui.snapshot()["tracking"]
        dual_display = self.gui.snapshot()['dual_uwb']
        self.assertTrue(dual_display['valid'])
        self.assertGreater(dual_display['sensors']['uwb_left']['range_m'], 0)
        self.assertGreater(dual_display['sensors']['uwb_right']['range_m'], 0)
        self.assertFalse(dual_display['drive_authorized'])
        self.assertTrue(display["valid"])
        self.assertEqual(display["source_mode"], "live")
        self.assertFalse(display["drive_authorized"])
        self.assertEqual(self.lidar.snapshot()["scan"]["source"]["kind"], "ros")
        for values in self.received.values():
            self.assertTrue(all(item["source_mode"] == "live" and item["drive_authorized"] is False for item in values))
        topics = {name for name, _ in self.node.get_topic_names_and_types()}
        self.assertFalse(topics & {"/cmd_vel", "/cmd_vel_unstamped", "/tf", "/tf_static", "/scan", "/smartcart/uwb"})
        self.assertFalse(any(self.replayed.values()))
        self.stop.set()
        self.worker.join(timeout=2)
        self.assertFalse(self.worker.is_alive())
        self.wait_until(lambda: all(self.node.count_publishers(topic) == 0 for topic in LIVE_TOPICS.values()),
                        "종료 후 운영 노드의 DDS 발행자 제거", timeout=3)

    def test_short_synthetic_occlusion_hides_measurement_and_reconfirms_same_track(self):
        self.start_live()
        original = self.acquire()["track_id"]
        self.wait_until(lambda: self.tracking.snapshot()["result"]["track"]["state"] == "coasting",
                        "합성 빈 스캔의 가림 상태", pump=lambda: self.publish_fresh(empty=True))
        hidden = self.gui.snapshot()["tracking"]["result"]["track"]
        self.assertFalse(hidden["valid"])
        self.assertIsNone(hidden["position_xy_m"])
        self.assertIsNotNone(hidden["predicted_position_xy_m"])
        self.assertEqual(hidden["confirmation_count"], 0)
        recovered = self.acquire()
        self.assertEqual(recovered["track_id"], original)
        self.assertGreaterEqual(recovered["confirmation_count"], 3)

    def test_full_status_disconnect_recovery_and_duplicate_ttl_are_observed_over_dds(self):
        self.start_live()
        original = self.acquire()["track_id"]
        disconnected = deepcopy(self.last_status)
        right = disconnected["sensors"]["uwb_right"]
        right.update(state="error", connected=False, last_error="disconnected")
        right["latest"].update(valid=False, invalid_reason="disconnected")
        encoded = json.dumps(disconnected, allow_nan=False)
        self.wait_until(lambda: bool(self.received["status"])
                        and self.received["status"][-1]["uwb_estimate"]["invalid_reason"] == "disconnected"
                        and self.tracking.snapshot()["result"]["track"]["track_id"] is None,
                        "전체 status의 명시적 단절 전달",
                        pump=lambda: self.status_publisher.publish(String(data=encoded)))
        self.assertIsNone(self.gui.snapshot()["tracking"]["result"]["track"]["position_xy_m"])
        self.epoch += 1
        recovered = self.acquire()
        self.assertNotEqual(recovered["track_id"], original)
        frozen = json.dumps(self.last_status, allow_nan=False)

        def duplicate_with_fresh_scan():
            self.status_publisher.publish(String(data=frozen))
            self.publish_scan()

        self.wait_until(lambda: not self.tracking.snapshot()["result"]["uwb_estimate"]["valid"]
                        and self.tracking.snapshot()["result"]["lidar_clusters"]["valid"],
                        "과거 status를 반복해도 UWB 원본 수명 만료", pump=duplicate_with_fresh_scan, timeout=2)
        expired = self.gui.snapshot()["tracking"]["result"]["track"]
        self.assertFalse(expired["valid"])
        self.assertIsNone(expired["position_xy_m"])
        self.assertIsNone(expired["predicted_position_xy_m"])
        self.wait_until(lambda: not self.tracking.snapshot()["result"]["lidar_clusters"]["valid"],
                        "모든 입력 중단 뒤 20Hz 타이머의 LiDAR 만료", timeout=2)

    def test_bad_json_and_wrong_scan_frame_clear_gui_without_stopping_worker(self):
        self.start_live()
        self.acquire()
        self.wait_until(lambda: bool(self.received["status"])
                        and self.received["status"][-1]["uwb_estimate"]["invalid_reason"] == "invalid_status_json",
                        "DDS String의 비유한 JSON 즉시 거부",
                        pump=lambda: self.status_publisher.publish(String(data='{"schema":NaN}')))
        self.assertIsNone(self.gui.snapshot()["tracking"]["result"]["track"]["position_xy_m"])
        self.acquire()
        self.wait_until(lambda: bool(self.received["clusters"])
                        and self.received["clusters"][-1]["invalid_reason"] == "invalid_ros_scan",
                        "DDS LaserScan frame 불일치 거부",
                        pump=lambda: self.publish_scan(frame_id="synthetic_wrong_frame"))
        self.assertTrue(self.worker.is_alive())
        self.assertIsNone(self.gui.snapshot()["tracking"]["result"]["track"]["position_xy_m"])
        self.assertTrue(self.acquire()["valid"])

    @unittest.skipUnless(any(path.is_file() for path in default_library_candidates()),
                         "합성 원시 재생 DDS 검사에는 이 Linux 플랫폼에서 빌드한 원본 C 파서가 필요합니다")
    def test_synthetic_raw_replay_reaches_only_replay_dds_topics_and_gui(self):
        from smartcart_tracking.service_runtime import replay_loop
        self.lidar = ScanStore("replay")
        self.lidar.stop = self.stop
        settings = load_tracking_config(ROOT / "config/target_tracking.synthetic.json")
        uwb = load_estimation_config(ROOT / "config/uwb_estimation.synthetic.json")
        self.tracking = TrackingStore(self.mounting, settings, mode="replay", stop=self.stop)
        self.gui = SensingStore(self.lidar, UwbStore("off", stop=self.stop), "replay", tracking=self.tracking)
        self.worker = threading.Thread(target=tracking_replay_ros_loop, args=(self.tracking,),
                                       name="synthetic-replay-dds", daemon=True)
        self.worker.start()
        self.wait_until(lambda: all(self.node.count_publishers(topic) == 1 for topic in REPLAY_TOPICS.values()),
                        "재생 DDS 발행자 발견")
        self.wait_until(lambda: bool(self.replayed["status"]) and self.replayed["status"][-1].get("state") == "waiting",
                        "재생 시작 전 좌표 없는 waiting 상태")
        self.replay_worker = threading.Thread(target=replay_loop,
            args=(self.lidar, self.tracking, self.mounting, uwb, settings,
                  ROOT / "samples/target_tracking.synthetic.jsonl"),
            kwargs={"speed": .25, "loop": True}, name="synthetic-raw-replay", daemon=True)
        self.replay_worker.start()
        self.wait_until(lambda: self.gui.snapshot()["tracking"]["valid"]
                        and all(self.replayed.values())
                        and any(item.get("valid") for item in self.replayed["target"])
                        and any(item.get("track", {}).get("valid") for item in self.replayed["status"]),
                        "원본 C 파서→재생 코어→실제 DDS→GUI 저장소", timeout=10)
        self.assertTrue(self.gui.snapshot()["sensing"]["synthetic"])
        self.assertFalse(any(self.received.values()))
        self.assertTrue(all(self.node.count_publishers(topic) == 0 for topic in LIVE_TOPICS.values()))
        self.assertFalse(self.tracking.snapshot()["drive_authorized"])
        self.assertTrue(all(item["source_mode"] == "replay" and item["drive_authorized"] is False
                            for values in self.replayed.values() for item in values))
        topics = {name for name, _ in self.node.get_topic_names_and_types()}
        self.assertFalse(topics & {"/cmd_vel", "/cmd_vel_unstamped", "/tf", "/tf_static"})


if __name__ == "__main__":
    unittest.main()
