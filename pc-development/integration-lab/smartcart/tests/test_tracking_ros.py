"""실물·ROS 설치 없이 토픽 출처·문맥·타이머·오류 정리를 검사한다."""

from copy import deepcopy
import importlib.util
import json
import os
import sys
from threading import Event
import types
import unittest
from unittest.mock import patch

from smartcart_tracking.config import TrackingConfig
from smartcart_tracking.ros_runtime import LIVE_TOPICS, REPLAY_TOPICS, decode_status, tracking_ros_loop, tracking_replay_ros_loop
from smartcart_tracking.runtime import LiveTrackingRuntime
from smartcart_uwb.estimation_config import EstimationConfig
from test_tracking_runtime import mounting_config, raw_scan, status, SECOND


class Store:
    def __init__(self):
        self.stop = Event()
        self.values, self.errors = [], []
        self.envelope = None

    def publish(self, value, **kwargs):
        self.values.append((deepcopy(value), kwargs))

    def set_error(self, reason, **kwargs):
        self.errors.append(reason)

    def snapshot(self):
        return deepcopy(self.envelope)


def fake_ros(stop, actions=None, fail_subscribe=False, fail_shutdown=False):
    state = types.SimpleNamespace(nodes=[], contexts=[], executors=[], init_args=None)

    class Context:
        def __init__(self):
            self.active = False
            self.closed = False
            state.contexts.append(self)
        def ok(self):
            return self.active
        def shutdown(self):
            self.active = False
            self.closed = True

    def init(*, args, context, **kwargs):
        context.active = True
        state.init_args = args

    class Message:
        def __init__(self):
            self.data = ""

    class Publisher:
        def __init__(self):
            self.values = []
        def publish(self, message):
            self.values.append(message.data)

    class Node:
        def __init__(self, name, **kwargs):
            self.kwargs = kwargs
            self.publishers, self.subscriptions, self.timers = {}, {}, []
            self.destroyed = False
            state.nodes.append(self)
        def create_publisher(self, cls, topic, qos):
            publisher = Publisher()
            self.publishers[topic] = publisher
            return publisher
        def create_subscription(self, cls, topic, callback, qos):
            if fail_subscribe:
                raise RuntimeError("테스트 구독 오류")
            self.subscriptions[topic] = (callback, qos)
        def create_timer(self, interval, callback):
            self.timers.append((interval, callback))
        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self, *, context):
            self.context = context
            self.closed = False
            state.executors.append(self)
        def add_node(self, node):
            self.node = node
        def spin_once(self, **kwargs):
            if actions is not None:
                actions(self.node)
            for _, callback in self.node.timers:
                callback()
            stop.set()
        def shutdown(self, **kwargs):
            self.closed = True
            if fail_shutdown:
                raise RuntimeError("테스트 정리 오류")

    class QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    modules = {"rclpy": types.SimpleNamespace(init=init),
               "rclpy.context": types.SimpleNamespace(Context=Context),
               "rclpy.executors": types.SimpleNamespace(SingleThreadedExecutor=Executor),
               "rclpy.node": types.SimpleNamespace(Node=Node),
               "rclpy.qos": types.SimpleNamespace(QoSProfile=QoS, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
                   HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1), DurabilityPolicy=types.SimpleNamespace(VOLATILE=1)),
               "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
               "std_msgs.msg": types.SimpleNamespace(String=Message),
               "sensor_msgs.msg": types.SimpleNamespace(LaserScan=Message)}
    return state, modules


class RosTests(unittest.TestCase):
    def setUp(self):
        self.mounting = mounting_config()
        self.uwb = EstimationConfig(range_model="horizontal")
        self.settings = TrackingConfig()
        self.lidar, self.tracking = Store(), Store()
        self.lidar.stop = self.tracking.stop

    def run_live(self, **fake_options):
        state, modules = fake_ros(self.tracking.stop, **fake_options)
        with patch.dict(sys.modules, modules), patch("smartcart_tracking.ros_runtime.time.monotonic_ns", return_value=SECOND):
            tracking_ros_loop(self.lidar, self.tracking, self.mounting, self.uwb, self.settings,
                              ros_args=["tracking", "--ros-args"])
        return state

    def test_full_status_subscription_private_context_executor_qos_and_fixed_names(self):
        with patch.dict(os.environ, {"ROS_LOCALHOST_ONLY": "0"}):
            state = self.run_live()
            self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")
        node = state.nodes[0]
        self.assertEqual(set(node.subscriptions), {"/scan", "/smartcart/uwb/dual/status"})
        self.assertEqual(set(node.publishers), set(LIVE_TOPICS.values()))
        self.assertNotIn("/cmd_vel", node.publishers)
        self.assertEqual(node.subscriptions["/scan"][1].depth, 5)
        self.assertEqual(node.subscriptions["/scan"][1].reliability, 1)
        self.assertEqual(node.timers[0][0], .05)
        self.assertIs(state.executors[0].context, state.contexts[0])
        self.assertIs(node.kwargs["context"], state.contexts[0])
        self.assertFalse(node.kwargs["use_global_arguments"])
        self.assertEqual(state.init_args, ["tracking", "--ros-args"])
        self.assertTrue(node.destroyed)
        self.assertTrue(state.contexts[0].closed)
        self.assertTrue(state.executors[0].closed)

    def test_scan_is_normalised_for_legacy_view_and_tracking_result_is_live(self):
        def actions(node):
            node.subscriptions["/smartcart/uwb/dual/status"][0](types.SimpleNamespace(data=json.dumps(status(self.mounting))))
            data = raw_scan()
            message = types.SimpleNamespace(**data, intensities=[], header=types.SimpleNamespace(
                frame_id=self.mounting.lidar.frame_id, stamp=types.SimpleNamespace(sec=0, nanosec=0)))
            node.subscriptions["/scan"][0](message)
        state = self.run_live(actions=actions)
        self.assertEqual(self.lidar.values[0][0]["schema"], "smartcart.scan.v1")
        self.assertEqual(self.lidar.values[0][0]["source"]["kind"], "ros")
        result = self.tracking.values[-1][0]
        self.assertEqual(result["source_mode"], "live")
        self.assertFalse(result["drive_authorized"])
        payload = json.loads(state.nodes[0].publishers[LIVE_TOPICS["target"]].values[-1])
        self.assertEqual(payload["source_mode"], "live")
        self.assertIsNone(payload["position_xy_m"])

    def test_wrong_scan_frame_and_bad_json_invalidate_state(self):
        def actions(node):
            node.subscriptions["/smartcart/uwb/dual/status"][0](types.SimpleNamespace(data='{"a":NaN}'))
            node.subscriptions["/scan"][0](types.SimpleNamespace(
                header=types.SimpleNamespace(frame_id="wrong"), ranges=[1], intensities=[]))
        self.run_live(actions=actions)
        self.assertTrue(self.lidar.errors)
        result = self.tracking.values[-1][0]
        self.assertFalse(result["lidar_clusters"]["valid"])
        self.assertIsNone(result["track"]["position_xy_m"])

    def test_json_size_duplicate_and_nonfinite_literals_are_rejected(self):
        for value in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', " " * 65537, "[]", "\ud800"):
            with self.subTest(value=value[:20]):
                with self.assertRaises(ValueError):
                    decode_status(value)

    def test_partial_initialisation_and_cleanup_exception_still_close_context(self):
        for options in ({"fail_subscribe": True}, {"fail_shutdown": True}):
            with self.subTest(options=options):
                self.setUp()
                state = self.run_live(**options)
                self.assertTrue(state.nodes[0].destroyed)
                self.assertTrue(state.contexts[0].closed)
                if options.get("fail_subscribe"):
                    self.assertTrue(self.tracking.errors)

    def replay_envelope(self):
        result = LiveTrackingRuntime(self.mounting, self.uwb, self.settings).snapshot(SECOND)
        result["source_mode"] = "replay"
        for name in ("uwb_estimate", "lidar_clusters", "track"):
            result[name]["source_mode"] = "replay"
        return {"schema": "smartcart.tracking_view.v1", "mode": "replay", "source_mode": "replay",
                "state": "running", "drive_authorized": False,
                "result": {name: result[name] for name in ("uwb_estimate", "lidar_clusters", "track")}}

    def test_actual_tracking_store_payload_is_accepted_by_replay_publisher(self):
        from smartcart_tracking.view import TrackingStore
        store = TrackingStore(self.mounting, self.settings, mode="replay")
        payload = self.replay_envelope()["result"]
        result = {**payload, "source_mode": "replay", "drive_authorized": False}
        store.publish(result, now_ns=SECOND, wall_ns=SECOND)
        self.tracking.envelope = store.snapshot(now_ns=SECOND)
        state, modules = fake_ros(self.tracking.stop)
        with patch.dict(sys.modules, modules):
            tracking_replay_ros_loop(self.tracking)
        self.assertFalse(self.tracking.errors)
        status_output = json.loads(state.nodes[0].publishers[REPLAY_TOPICS["status"]].values[-1])
        self.assertEqual(status_output["source_mode"], "replay")
        self.assertFalse(status_output["drive_authorized"])

    def test_replay_ros_has_only_dedicated_publishers_and_no_sensor_subscriptions(self):
        self.tracking.envelope = self.replay_envelope()
        state, modules = fake_ros(self.tracking.stop)
        with patch.dict(sys.modules, modules):
            tracking_replay_ros_loop(self.tracking)
        node = state.nodes[0]
        self.assertFalse(node.subscriptions)
        self.assertEqual(set(node.publishers), set(REPLAY_TOPICS.values()))
        self.assertEqual([interval for interval, _ in node.timers], [.1, .5])
        for publisher in node.publishers.values():
            self.assertEqual(json.loads(publisher.values[-1])["source_mode"], "replay")

    def test_replay_ros_refuses_live_payload_or_ended_valid_position(self):
        for mode in ("live", "ended"):
            with self.subTest(mode=mode):
                self.setUp()
                self.tracking.envelope = self.replay_envelope()
                if mode == "live":
                    self.tracking.envelope["source_mode"] = "live"
                else:
                    self.tracking.envelope["state"] = "ended"
                    self.tracking.envelope["result"]["track"]["valid"] = True
                state, modules = fake_ros(self.tracking.stop)
                with patch.dict(sys.modules, modules):
                    tracking_replay_ros_loop(self.tracking)
                self.assertTrue(self.tracking.errors)
                self.assertFalse(any(p.values for p in state.nodes[0].publishers.values()))

    def test_replay_waiting_and_error_publish_only_coordinate_free_status(self):
        from smartcart_tracking.view import TrackingStore
        for expected_state in ("waiting", "error"):
            with self.subTest(state=expected_state):
                self.setUp()
                store = TrackingStore(self.mounting, self.settings, mode="replay")
                if expected_state == "error":
                    store.set_error("tracking_replay_error")
                self.tracking.envelope = store.snapshot(now_ns=SECOND)
                state, modules = fake_ros(self.tracking.stop)
                with patch.dict(sys.modules, modules):
                    tracking_replay_ros_loop(self.tracking)
                self.assertFalse(self.tracking.errors)
                publishers = state.nodes[0].publishers
                self.assertFalse(publishers[REPLAY_TOPICS["target"]].values)
                self.assertFalse(publishers[REPLAY_TOPICS["clusters"]].values)
                messages = publishers[REPLAY_TOPICS["status"]].values
                self.assertEqual(len(messages), 1)
                payload = json.loads(messages[0])
                self.assertEqual(payload["schema"], "smartcart.tracking_view.v1")
                self.assertEqual(payload["source_mode"], "replay")
                self.assertEqual(payload["state"], expected_state)
                self.assertFalse(payload["valid"])
                self.assertFalse(payload["drive_authorized"])
                self.assertIsNone(payload["result"])
                self.assertEqual(payload["error"], "tracking_replay_error" if expected_state == "error" else None)
                self.assertNotIn("position", messages[0])
                self.assertNotIn("lidar_transform", payload)

    def test_legacy_ros_subscriptions_can_be_disabled_without_changing_default(self):
        from smartcart_sensing.ros import ros_loop
        from smartcart_sensing.uwb import UwbStore
        from test_sensing_ros import fake_ros as legacy_fake
        from smartcart_radar.core import ScanStore
        lidar = ScanStore()
        uwb = UwbStore("ros", stop=lidar.stop)
        state, modules = legacy_fake(lidar)
        with patch.dict(sys.modules, modules):
            ros_loop(lidar, uwb, publish_uwb=False, subscribe_scan=False, subscribe_uwb=False)
        self.assertFalse(state.nodes[0].subscriptions)


try:
    HAS_ROS = importlib.util.find_spec("rclpy") is not None
except (ValueError, ModuleNotFoundError):
    HAS_ROS = False


@unittest.skipUnless(HAS_ROS, "이 호스트에는 rclpy가 없어 실제 ROS 문맥 검사를 건너뜁니다")
class ActualRosTests(unittest.TestCase):
    def test_explicit_context_initialises_and_shuts_down(self):
        import rclpy
        from rclpy.context import Context
        context = Context()
        try:
            rclpy.init(args=[], context=context)
            self.assertTrue(context.ok())
        finally:
            if context.ok():
                context.shutdown()


if __name__ == "__main__":
    unittest.main()
