"""ROS 없이 수행하는 메시지 계약·생존 시간·ROS 수명 주기 테스트."""
import copy
import math
import sys
import time
import types
import unittest
from unittest.mock import patch

from smartcart_radar.core import ScanStore
from smartcart_sensing.ros import (
    consume_uwb_json, diagnostic_data, encode_uwb_snapshot, lidar_diagnostic_data, position_data,
    ros_loop, scan_from_message, validate_topic,
)
from smartcart_sensing.uwb import MAX_ROS_JSON_BYTES, UwbStore, synthetic_line


class MessageTests(unittest.TestCase):
    def snapshot(self, mode="serial"):
        store = UwbStore(mode, yaw_deg=90, offset_x=.2)
        store.publish_line(synthetic_line(0))
        return store.snapshot()

    def test_ros_roundtrip_preserves_publisher_mount_transform(self):
        snapshot = self.snapshot()
        store = UwbStore("ros", yaw_deg=-90)
        self.assertTrue(consume_uwb_json(store, encode_uwb_snapshot(snapshot)))
        received = store.snapshot()
        self.assertEqual(received["state"], "live")
        self.assertEqual(received["measurement"]["position"], snapshot["measurement"]["position"])
        self.assertEqual(received["transform"]["yaw_deg"], 90)

    def test_ros_duplicate_does_not_extend_freshness(self):
        snapshot = self.snapshot()
        store = UwbStore("ros")
        self.assertTrue(store.publish_payload(snapshot, now=10))
        self.assertFalse(store.publish_payload(snapshot, now=10.4))
        self.assertEqual(store.snapshot(10.6)["state"], "stale")
        self.assertIsNone(store.snapshot(10.6)["measurement"]["position"])

    def test_ros_source_age_and_delivery_delay_are_counted(self):
        for age, delay in ((600, 0), (0, 1)):
            with self.subTest(age=age, delay=delay):
                snapshot = self.snapshot()
                snapshot["age_ms"] = age
                snapshot["server_unix"] = time.time() - delay
                store = UwbStore("ros")
                self.assertTrue(store.publish_payload(snapshot, now=10))
                self.assertEqual(store.snapshot(10)["state"], "stale")

    def test_ros_demo_remains_synthetic_and_no_position_topic(self):
        snapshot = self.snapshot("demo")
        store = UwbStore("ros")
        self.assertTrue(store.publish_payload(snapshot))
        self.assertEqual(store.snapshot()["state"], "synthetic")
        self.assertIsNone(position_data(store.snapshot()))
        self.assertEqual(diagnostic_data(store.snapshot())["level"], 1)

    def test_upstream_fault_immediately_removes_valid_position(self):
        store = UwbStore("ros")
        self.assertTrue(store.publish_payload(self.snapshot()))
        store.publish_payload({"schema": "smartcart.uwb.v1", "state": "error"})
        self.assertEqual(store.snapshot()["state"], "error")
        self.assertFalse(store.snapshot()["measurement"]["valid"])
        self.assertIsNone(store.snapshot()["measurement"]["position"])

    def test_malformed_json_and_payload_are_rejected(self):
        cases = ["{", "[]", '{"schema":"smartcart.uwb.v1","age_ms":NaN}',
                 "x" * (MAX_ROS_JSON_BYTES + 1), "[" * 1500 + "]" * 1500]
        for data in cases:
            with self.subTest(data=data[:20]):
                store = UwbStore("ros")
                self.assertFalse(consume_uwb_json(store, data))
                self.assertEqual(store.snapshot()["state"], "error")
                self.assertEqual(store.snapshot()["rejected_frames"], 1)

    def test_ros_malformed_transform_and_nonfinite_diagnostics_rejected(self):
        for field, value in (("yaw_deg", None), ("offset_y", math.inf), ("frame_id", "map")):
            with self.subTest(field=field):
                snapshot = self.snapshot()
                snapshot["transform"][field] = value
                store = UwbStore("ros")
                self.assertFalse(store.publish_payload(snapshot))
                self.assertIsNone(store.snapshot()["measurement"])

    def test_ros_valid_flag_cannot_override_invalid_raw_id(self):
        snapshot = self.snapshot()
        raw = snapshot["measurement"]["raw_diagnostic_line"].split(",")
        raw[5] = "1"
        snapshot["measurement"]["raw_diagnostic_line"] = ",".join(raw)
        snapshot["measurement"]["valid"] = True
        store = UwbStore("ros")
        self.assertTrue(store.publish_payload(snapshot))
        self.assertFalse(store.snapshot()["measurement"]["valid"])
        self.assertIsNone(position_data(store.snapshot()))

    def test_position_requires_fresh_authenticated_real_measurement(self):
        snapshot = self.snapshot()
        self.assertIsNotNone(position_data(snapshot))
        variants = []
        for state in ("stale", "error", "waiting", "synthetic"):
            variant = copy.deepcopy(snapshot)
            variant["state"] = state
            variants.append(variant)
        for key, value in (("valid", False), ("authenticated", False)):
            variant = copy.deepcopy(snapshot)
            variant["measurement"][key] = value
            variants.append(variant)
        variant = copy.deepcopy(snapshot)
        variant["age_ms"] = 501
        variants.append(variant)
        variant = copy.deepcopy(snapshot)
        variant["measurement"]["position"]["x"] = math.nan
        variants.append(variant)
        for variant in variants:
            with self.subTest(variant=variant):
                self.assertIsNone(position_data(variant))

    def test_diagnostics_explicit_for_invalid_measurements(self):
        snapshot = self.snapshot()
        self.assertEqual(diagnostic_data(snapshot)["level"], 0)
        snapshot["measurement"]["valid"] = False
        self.assertEqual(diagnostic_data(snapshot)["level"], 2)
        snapshot["state"] = "stale"
        self.assertEqual(diagnostic_data(snapshot)["level"], 3)

    def test_json_output_no_nonfinite_or_oversized_messages(self):
        snapshot = self.snapshot()
        snapshot["age_ms"] = math.nan
        with self.assertRaises(ValueError):
            encode_uwb_snapshot(snapshot)
        snapshot["age_ms"] = 1
        snapshot["extra"] = "a" * MAX_ROS_JSON_BYTES
        with self.assertRaises(ValueError):
            encode_uwb_snapshot(snapshot)

    def test_topic_validation(self):
        self.assertEqual(validate_topic("/smartcart/uwb"), "/smartcart/uwb")
        for name in ("scan", "/", "/scan;rm", "/scan//data", "/1topic"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                validate_topic(name)

    def test_lidar_diagnostics_separate_waiting_stale_error_synthetic_and_live(self):
        base = {"state": "live", "source_mode": "ros", "age_ms": 100, "stale_after_ms": 1500,
                "scan": {"source": {"kind": "ros"}, "stats": {"valid_sensor_rays": 3}}}
        self.assertEqual(lidar_diagnostic_data(base)["level"], 0)
        for state, expected in (("waiting", 3), ("stale", 3), ("error", 2), ("synthetic", 1),
                                ("replay", 1), ("ended", 1)):
            with self.subTest(state=state):
                snapshot = {**base, "state": state}
                status = lidar_diagnostic_data(snapshot)
                self.assertEqual(status["level"], expected)
                self.assertEqual(status["values"]["valid"], "false")
        expired = {**base, "age_ms": 1501}
        self.assertEqual(lidar_diagnostic_data(expired)["level"], 3)
        empty = copy.deepcopy(base)
        empty["scan"]["stats"]["valid_sensor_rays"] = 0
        self.assertEqual(lidar_diagnostic_data(empty)["level"], 2)

    def test_scan_indices_and_null_returns_preserved(self):
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="laser", stamp=types.SimpleNamespace(sec=1, nanosec=2)),
            angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi,
            scan_time=.1, time_increment=.01, range_min=.05, range_max=16,
            ranges=[1, math.inf, 2], intensities=[],
        )
        scan = scan_from_message(message)
        self.assertEqual(scan["ranges"], [1, None, 2])
        self.assertEqual(scan["angle_increment"], math.pi)
        self.assertEqual(scan["source"]["kind"], "ros")


def fake_ros(lidar_store, *, fail_subscription=False):
    """공식 rclpy 호출의 최소 모형으로 정리·QoS·발행 분기를 검사한다."""
    state = types.SimpleNamespace(nodes=[], contexts=[], executors=[], init_args=None)

    def init(**kwargs):
        state.init_args = kwargs.get("args")

    class Context:
        def __init__(self):
            self.active = True
            state.contexts.append(self)

        def ok(self):
            return self.active

        def shutdown(self):
            self.active = False

    class Message:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)
            self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")
            self.point = types.SimpleNamespace(x=0, y=0, z=0)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Node:
        def __init__(self, name, **kwargs):
            self.publishers, self.subscriptions, self.timers = {}, {}, []
            self.destroyed = False
            state.nodes.append(self)

        def create_subscription(self, cls, topic, callback, qos):
            if fail_subscription:
                raise RuntimeError("테스트 구독 초기화 실패")
            self.subscriptions[topic] = (callback, qos)

        def create_publisher(self, cls, topic, qos):
            publisher = Publisher()
            self.publishers[topic] = publisher
            return publisher

        def create_timer(self, seconds, callback):
            self.timers.append((seconds, callback))

        def get_clock(self):
            now = types.SimpleNamespace(nanoseconds=time.time_ns(), to_msg=lambda: types.SimpleNamespace(sec=1, nanosec=2))
            return types.SimpleNamespace(now=lambda: now)

        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self, **kwargs):
            self.closed = False
            state.executors.append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, **kwargs):
            for _, callback in self.node.timers:
                callback()
            lidar_store.stop.set()

        def shutdown(self, **kwargs):
            self.closed = True

    class QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    modules = {
        "rclpy": types.SimpleNamespace(init=init),
        "rclpy.context": types.SimpleNamespace(Context=Context),
        "rclpy.executors": types.SimpleNamespace(SingleThreadedExecutor=Executor),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rclpy.qos": types.SimpleNamespace(QoSProfile=QoS,
            ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
            HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
            DurabilityPolicy=types.SimpleNamespace(VOLATILE=1)),
        "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
        "sensor_msgs.msg": types.SimpleNamespace(LaserScan=Message),
        "std_msgs.msg": types.SimpleNamespace(String=Message),
        "geometry_msgs.msg": types.SimpleNamespace(PointStamped=Message),
        "diagnostic_msgs.msg": types.SimpleNamespace(DiagnosticArray=Message, DiagnosticStatus=Message, KeyValue=Message),
    }
    return state, modules


class LifecycleTests(unittest.TestCase):
    def test_ros_topics_qos_position_and_resource_cleanup(self):
        lidar = ScanStore()
        uwb = UwbStore(stop=lidar.stop)
        uwb.publish_line(synthetic_line(0))
        state, modules = fake_ros(lidar)
        with patch.dict(sys.modules, modules):
            ros_loop(lidar, uwb)
        node = state.nodes[0]
        self.assertEqual(node.subscriptions["/scan"][1].depth, 5)
        self.assertEqual(node.subscriptions["/scan"][1].reliability, 1)
        self.assertEqual(node.timers[0][0], .1)
        self.assertEqual(node.timers[1][0], .5)
        self.assertEqual(len(node.publishers["/smartcart/uwb"].messages), 1)
        self.assertEqual(len(node.publishers["/smartcart/uwb/position"].messages), 1)
        point = node.publishers["/smartcart/uwb/position"].messages[0]
        self.assertEqual((point.header.frame_id, point.point.x, point.point.y), ("base_link", 1.8, 0))
        combined = node.publishers["/smartcart/sensing/diagnostics"].messages[0]
        self.assertEqual([status.name for status in combined.status], ["smartcart/lidar", "smartcart/uwb"])
        self.assertEqual([status.level for status in combined.status], [b"\x03", b"\x00"])
        self.assertTrue(node.destroyed)
        self.assertTrue(state.executors[0].closed)
        self.assertFalse(state.contexts[0].active)

    def test_ros_launch_arguments_are_forwarded(self):
        lidar = ScanStore()
        uwb = UwbStore(stop=lidar.stop)
        state, modules = fake_ros(lidar)
        arguments = ["sensing", "--ros-args", "-r", "/scan:=/cart/scan"]
        with patch.dict(sys.modules, modules):
            ros_loop(lidar, uwb, ros_args=arguments)
        self.assertEqual(state.init_args, arguments)

    def test_stale_and_demo_publish_diagnostics_but_no_position(self):
        for mode, now in (("serial", time.monotonic() - 1), ("demo", time.monotonic())):
            with self.subTest(mode=mode):
                lidar = ScanStore()
                uwb = UwbStore(mode, stop=lidar.stop)
                uwb.publish_line(synthetic_line(0), now=now)
                state, modules = fake_ros(lidar)
                with patch.dict(sys.modules, modules):
                    ros_loop(lidar, uwb)
                node = state.nodes[0]
                self.assertEqual(len(node.publishers["/smartcart/uwb/position"].messages), 0)
                self.assertEqual(len(node.publishers["/smartcart/uwb/diagnostics"].messages), 1)

    def test_ros_input_mode_never_republishes(self):
        lidar = ScanStore()
        uwb = UwbStore("ros", stop=lidar.stop)
        state, modules = fake_ros(lidar)
        with patch.dict(sys.modules, modules):
            ros_loop(lidar, uwb, publish_uwb=False)
        self.assertIn("/smartcart/uwb", state.nodes[0].subscriptions)
        self.assertEqual(set(state.nodes[0].publishers), {"/smartcart/sensing/diagnostics"})
        self.assertEqual(state.nodes[0].timers[0][0], .5)
        combined = state.nodes[0].publishers["/smartcart/sensing/diagnostics"].messages[0]
        self.assertEqual([status.level for status in combined.status], [b"\x03", b"\x03"])

    def test_partial_initialization_cleans_up_and_reports_errors(self):
        lidar = ScanStore()
        uwb = UwbStore("ros", stop=lidar.stop)
        state, modules = fake_ros(lidar, fail_subscription=True)
        with patch.dict(sys.modules, modules):
            ros_loop(lidar, uwb, publish_uwb=False)
        self.assertEqual(lidar.snapshot()["state"], "error")
        self.assertEqual(uwb.snapshot()["state"], "error")
        self.assertTrue(state.nodes[0].destroyed)
        self.assertFalse(state.contexts[0].active)


if __name__ == "__main__":
    unittest.main()
