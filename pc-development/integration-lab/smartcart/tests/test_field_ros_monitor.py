"""ROS 대역으로 구독 전용·시간 제한·형식 거부·독립 문맥 정리를 확인한다."""
import json
import os
import sys
from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smartcart_field.monitor import FieldMonitor
from smartcart_field.ros_monitor import run_monitor
from smartcart_tracking.config import TrackingConfig
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from test_field_monitor import ROOT, SECOND, scan, status


def fake_ros(*, actions=None, fail_subscription=False, fail_shutdown=False, fail_destroy_publisher=False,
             interrupt=False, spin_step=.2):
    state = SimpleNamespace(now=1., contexts=[], nodes=[], executors=[], spins=0)

    class Context:
        closed = False
        def __init__(self):
            state.contexts.append(self)
        def ok(self):
            return not self.closed
        def shutdown(self):
            self.closed = True

    class Node:
        def __init__(self, name, **kwargs):
            self.name, self.kwargs = name, kwargs
            self.subscriptions, self.timers = {}, []
            self.publishers = [SimpleNamespace(topic_name="/parameter_events")]
            self.destroyed_publishers = []
            self.destroyed = False
            state.nodes.append(self)
        def create_subscription(self, message_type, topic, callback, qos):
            if fail_subscription:
                raise RuntimeError("대역 구독 초기화 오류")
            self.subscriptions[topic] = (callback, qos)
        def create_timer(self, interval, callback):
            self.timers.append((interval, callback))
        def create_publisher(self, *_args, **_kwargs):
            raise AssertionError("현장 모니터의 발행은 금지된다")
        def destroy_publisher(self, publisher):
            if fail_destroy_publisher:
                return False
            self.publishers.remove(publisher)
            self.destroyed_publishers.append(publisher)
            return True
        def create_service(self, *_args, **_kwargs):
            raise AssertionError("현장 모니터의 서비스는 금지된다")
        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self, **kwargs):
            self.kwargs, self.closed = kwargs, False
            state.executors.append(self)
        def add_node(self, node):
            self.node = node
        def spin_once(self, *, timeout_sec):
            if interrupt:
                raise KeyboardInterrupt()
            if state.spins == 0 and actions is not None:
                actions(self.node)
            state.spins += 1
            state.now += spin_step
            for _interval, callback in self.node.timers:
                callback()
        def shutdown(self, *, timeout_sec):
            self.closed = True
            if fail_shutdown:
                raise RuntimeError("대역 실행기 정리 오류")

    class QoS:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    def init(**kwargs):
        state.init_args = kwargs

    modules = {"rclpy": SimpleNamespace(init=init), "rclpy.context": SimpleNamespace(Context=Context),
        "rclpy.executors": SimpleNamespace(SingleThreadedExecutor=Executor), "rclpy.node": SimpleNamespace(Node=Node),
        "rclpy.qos": SimpleNamespace(QoSProfile=QoS, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
             DurabilityPolicy=SimpleNamespace(VOLATILE=1), HistoryPolicy=SimpleNamespace(KEEP_LAST=1)),
        "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
        "sensor_msgs.msg": SimpleNamespace(LaserScan=type("LaserScan", (), {})),
        "std_msgs.msg": SimpleNamespace(String=type("String", (), {}))}
    return state, modules


class FieldRosMonitorTests(unittest.TestCase):
    def setUp(self):
        self.mounting = load_config(ROOT / "config/dual_uwb.d5.example.json")
        self.monitor = FieldMonitor(self.mounting, EstimationConfig(), TrackingConfig())

    def run_fake(self, *, fake_options=None, **options):
        state, modules = fake_ros(**(fake_options or {}))
        with patch.dict(sys.modules, modules), patch.dict(os.environ, {"ROS_LOCALHOST_ONLY": "0"}), \
                patch("smartcart_field.ros_monitor.time.monotonic", side_effect=lambda: state.now), \
                patch("smartcart_field.ros_monitor.time.monotonic_ns", side_effect=lambda: int(state.now * SECOND)):
            report = run_monitor(self.monitor, duration_seconds=1, **options)
            self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")
        return state, report

    def test_only_subscriptions_private_context_disabled_services_and_twenty_hz_timer(self):
        state, report = self.run_fake(tracking_status_topic="/smartcart/tracking/status", ros_args=["--ros-args"])
        node = state.nodes[0]
        self.assertEqual(set(node.subscriptions), {"/scan", "/smartcart/uwb/dual/status", "/smartcart/tracking/status"})
        self.assertFalse(node.kwargs["enable_rosout"])
        self.assertFalse(node.kwargs["start_parameter_services"])
        self.assertFalse(node.kwargs["use_global_arguments"])
        self.assertEqual(node.publishers, [])
        self.assertEqual([publisher.topic_name for publisher in node.destroyed_publishers], ["/parameter_events"])
        self.assertIs(node.kwargs["context"], state.contexts[0])
        self.assertIs(state.executors[0].kwargs["context"], state.contexts[0])
        self.assertEqual(node.timers[0][0], .05)
        self.assertEqual(node.subscriptions["/scan"][1].depth, 5)
        self.assertEqual(node.subscriptions["/scan"][1].reliability, 1)
        self.assertEqual(state.init_args["args"], ["--ros-args"])
        self.assertTrue(state.contexts[0].closed and node.destroyed and state.executors[0].closed)
        self.assertFalse(report["completion"])
        self.assertLessEqual(state.spins, 6)
        self.assertNotIn("ros_interrupted", report["counts"])

    def test_actual_callbacks_feed_status_and_scan_without_opening_native_parser(self):
        def actions(node):
            node.subscriptions["/smartcart/uwb/dual/status"][0](SimpleNamespace(data=json.dumps(status(self.mounting))))
            message = SimpleNamespace(**scan(), intensities=[], header=SimpleNamespace(frame_id=self.mounting.lidar.frame_id))
            node.subscriptions["/scan"][0](message)
        with patch("smartcart_uwb.native.NativeParser", side_effect=AssertionError("네이티브 파서 생성 금지")):
            _state, report = self.run_fake(fake_options={"actions": actions})
        self.assertEqual(report["counts"]["status_messages"], 1)
        self.assertEqual(report["counts"]["scan_messages"], 1)
        self.assertEqual(report["sensors"]["uwb_left"]["counts"]["new_samples"], 1)
        self.assertEqual(report["sensors"]["lidar_center"]["counts"]["stale_transitions"], 1)

    def test_bad_json_oversized_arrays_and_wrong_frame_are_counted_without_exception_leak(self):
        def actions(node):
            node.subscriptions["/smartcart/uwb/dual/status"][0](SimpleNamespace(data='{"a":NaN}'))
            message = SimpleNamespace(**scan(), intensities=[], header=SimpleNamespace(frame_id="wrong"))
            node.subscriptions["/scan"][0](message)
            message.ranges = [1.] * 8193
            node.subscriptions["/scan"][0](message)
            node.subscriptions["/smartcart/tracking/status"][0](SimpleNamespace(data=" " * 65537))
        _state, report = self.run_fake(fake_options={"actions": actions}, tracking_status_topic="/smartcart/tracking/status")
        self.assertEqual(report["counts"]["status_json"], 1)
        self.assertEqual(report["counts"]["scan_message"], 1)
        self.assertEqual(report["counts"]["tracking_json"], 1)
        self.assertEqual(report["sensors"]["lidar_center"]["counts"]["frame_mismatches"], 1)

    def test_partial_initialisation_and_executor_cleanup_failure_still_close_context(self):
        for options in ({"fail_subscription": True}, {"fail_shutdown": True}):
            with self.subTest(options=options):
                self.setUp()
                state, report = self.run_fake(fake_options=options)
                self.assertTrue(state.contexts[0].closed)
                self.assertTrue(state.nodes[0].destroyed)
                self.assertGreater(report["counts"]["ros_runtime"], 0)

    def test_automatic_publisher_removal_failure_is_incomplete_before_subscribing(self):
        self.complete_manual_cycles_before_ros_start()
        state, report = self.run_fake(fake_options={"fail_destroy_publisher": True})
        self.assertEqual(state.nodes[0].subscriptions, {})
        self.assertEqual(state.nodes[0].timers, [])
        self.assertEqual(state.executors, [])
        self.assertTrue(state.contexts[0].closed and state.nodes[0].destroyed)
        self.assertEqual(report["counts"]["ros_runtime"], 1)
        self.assertFalse(report["completion"])

    def test_preexisting_stop_event_exits_without_spinning(self):
        stop = Event()
        stop.set()
        state, report = self.run_fake(stop=stop)
        self.assertEqual(state.spins, 0)
        self.assertTrue(state.contexts[0].closed)
        self.assertEqual(report["counts"]["ros_interrupted"], 1)

    def complete_manual_cycles_before_ros_start(self):
        # DDS 시작 직전에도 신선한 실제 코어 상태를 만든다. 모의 완료 플래그는 주입하지 않는다.
        first, recovered = SECOND - 500_000_000, SECOND - 10_000_000
        self.monitor.observe_status(status(self.mounting, at=first), first)
        self.monitor.observe_scan(scan(), first, frame_id=self.mounting.lidar.frame_id)
        for name in ("uwb_left", "uwb_right", "lidar_center"):
            self.assertTrue(self.monitor.mark(name, "unplug", first + 1_000_000))
        self.monitor.tick(first + 301_000_000)
        for name in ("uwb_left", "uwb_right", "lidar_center"):
            self.assertTrue(self.monitor.mark(name, "replug", first + 301_000_000))
        self.monitor.observe_status(status(self.mounting, 2, at=recovered), recovered)
        self.monitor.observe_scan(scan(), recovered, frame_id=self.mounting.lidar.frame_id)
        self.assertTrue(self.monitor.report()["completion"])

    def test_early_stop_cannot_return_previously_completed_fresh_manual_state(self):
        self.complete_manual_cycles_before_ros_start()
        stop = Event()
        state, report = self.run_fake(stop=stop, fake_options={"spin_step": .02,
                                                             "actions": lambda _node: stop.set()})
        self.assertEqual(state.spins, 1)
        self.assertEqual(report["counts"]["ros_interrupted"], 1)
        self.assertFalse(report["completion"])
        self.assertTrue(all(value["state"] == "invalid" for value in report["sensors"].values()))

    def test_early_context_shutdown_is_incomplete_and_cleans_other_resources(self):
        self.complete_manual_cycles_before_ros_start()
        # 문맥의 외부 종료도 예정 시간이 끝난 것으로 해석하지 않는다.
        state, report = self.run_fake(fake_options={"spin_step": .02,
            "actions": lambda node: node.kwargs["context"].shutdown()})
        self.assertEqual(report["counts"]["ros_interrupted"], 1)
        self.assertFalse(report["completion"])
        self.assertTrue(state.nodes[0].destroyed and state.executors[0].closed)

    def test_keyboard_interrupt_cleans_then_propagates_for_cli_exit_130(self):
        self.complete_manual_cycles_before_ros_start()
        state, modules = fake_ros(interrupt=True)
        with patch.dict(sys.modules, modules), \
                patch("smartcart_field.ros_monitor.time.monotonic", side_effect=lambda: state.now), \
                patch("smartcart_field.ros_monitor.time.monotonic_ns", side_effect=lambda: int(state.now * SECOND)):
            with self.assertRaises(KeyboardInterrupt):
                run_monitor(self.monitor, duration_seconds=1)
        self.assertTrue(state.contexts[0].closed and state.nodes[0].destroyed and state.executors[0].closed)
        self.assertFalse(self.monitor.report()["completion"])
        self.assertEqual(self.monitor.report()["counts"]["ros_interrupted"], 1)

    def test_invalid_duration_topics_and_replay_are_rejected_before_ros_import(self):
        for options in ({"duration_seconds": 0}, {"duration_seconds": float("inf")},
                        {"scan_topic": "../../scan"}, {"scan_topic": "/same", "status_topic": "/same"}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                run_monitor(self.monitor, **options)
        replay = FieldMonitor(self.mounting, EstimationConfig(), TrackingConfig(), source_mode="replay")
        with self.assertRaises(ValueError):
            run_monitor(replay)

if __name__ == "__main__":
    unittest.main()
