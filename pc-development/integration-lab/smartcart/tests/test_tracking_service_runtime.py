"""벽시계 대기만 가상화하여 실제 C 파서·추적 재생 작업자의 수명과 정리를 검사한다."""
from copy import deepcopy
from functools import partial
import math
from pathlib import Path
from threading import Event
import unittest
from unittest.mock import patch

from smartcart_radar.core import ScanStore
from smartcart_tracking.config import load_tracking_config
from smartcart_tracking.pipeline import TrackingPipeline
from smartcart_tracking.replay import read_records
from smartcart_tracking.service_runtime import replay_loop
from smartcart_tracking.view import TrackingStore
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.native import NativeParser
from smartcart_uwb.replay import REPLAY_CLOCK_ORIGIN_NS

ROOT = Path(__file__).resolve().parents[1]


class VirtualClock:
    def __init__(self):
        self.value = 10_000_000_000

    def now(self):
        return self.value


class VirtualStop(Event):
    """실제 시간을 재우지 않고 작업자가 요청한 벽시계 대기만 진행시킨다."""
    def __init__(self, clock, stop_after_waits=None):
        super().__init__()
        self.clock = clock
        self.wait_count = 0
        self.stop_after_waits = stop_after_waits

    def wait(self, timeout=None):
        if self.is_set():
            return True
        if timeout is None or timeout < 0:
            raise AssertionError("재생 작업자가 무제한 또는 음수 대기를 요청했습니다")
        self.wait_count += 1
        self.clock.value += max(1, math.ceil(timeout * 1_000_000_000))
        if self.stop_after_waits is not None and self.wait_count >= self.stop_after_waits:
            self.set()
        return self.is_set()


class RecordingParser:
    def __init__(self, sink):
        self.parser = NativeParser()
        self.closed = False
        sink.append(self)

    def feed_events(self, data):
        return self.parser.feed_events(data)

    def reset(self):
        self.parser.reset()

    def close(self):
        self.closed = True
        self.parser.close()


class RecordingView(TrackingStore):
    def __init__(self, *args, stop_after_iteration=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.published = []
        self.iterations = []
        self.stop_after_iteration = stop_after_iteration

    def begin(self, *, replay=None):
        super().begin(replay=replay)
        self.iterations.append(replay["iteration"])
        if self.stop_after_iteration is not None and replay["iteration"] >= self.stop_after_iteration:
            self.stop.set()

    def publish(self, result, **kwargs):
        super().publish(result, **kwargs)
        self.published.append((kwargs["now_ns"], deepcopy(kwargs["replay"]), deepcopy(result)))


class TrackingServiceRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mounting = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        cls.uwb_settings = load_estimation_config(ROOT / "config" / "uwb_estimation.synthetic.json")
        cls.settings = load_tracking_config(ROOT / "config" / "target_tracking.synthetic.json")
        cls.capture = ROOT / "samples" / "target_tracking.synthetic.jsonl"
        cls.records = list(read_records(cls.capture))
        try:
            parser = NativeParser()
        except OSError as error:
            raise unittest.SkipTest("미리 빌드한 UWB 네이티브 라이브러리가 필요합니다") from error
        parser.close()

    def run_worker(self, *, speed=.25, records=None, loop=False, stop_after_waits=None,
                   stop_after_iteration=None):
        clock = VirtualClock()
        stop = VirtualStop(clock, stop_after_waits)
        lidar = ScanStore("replay")
        lidar.stop = stop
        view = RecordingView(self.mounting, self.settings, mode="replay", stop=stop,
                             stop_after_iteration=stop_after_iteration)
        parsers = []
        factory = lambda: RecordingParser(parsers)
        # 고정 세션은 배속에 따른 코어 결과의 완전 동일성을 비교하기 위한 시험 조건이다.
        pipeline = partial(TrackingPipeline, session_id="service-fixture")
        with patch("smartcart_tracking.service_runtime.time.monotonic_ns", clock.now), \
                patch("smartcart_tracking.service_runtime.TrackingPipeline", pipeline):
            if records is None:
                replay_loop(lidar, view, self.mounting, self.uwb_settings, self.settings,
                            self.capture, speed=speed, loop=loop, parser_factory=factory)
            else:
                with patch("smartcart_tracking.service_runtime.read_records", side_effect=lambda _: iter(records)):
                    replay_loop(lidar, view, self.mounting, self.uwb_settings, self.settings,
                                self.capture, speed=speed, loop=loop, parser_factory=factory)
            snapshot = view.snapshot(now_ns=clock.now())
        self.assertTrue(parsers)
        self.assertTrue(all(parser.closed for parser in parsers), "원시 C 파서가 닫히지 않았습니다")
        return lidar, view, stop, snapshot

    def event_results(self, view):
        found = {}
        for now_ns, metadata, result in view.published:
            index = metadata["event_index"]
            if now_ns == REPLAY_CLOCK_ORIGIN_NS + self.records[index]["at_ns"]:
                found[index] = result
        self.assertEqual(len(found), len(self.records))
        return [found[index] for index in range(len(self.records))]

    def test_actual_native_fixture_finishes_and_clears_display_markers(self):
        lidar, view, stop, snapshot = self.run_worker()
        self.assertFalse(stop.is_set())
        self.assertEqual(snapshot["state"], "ended")
        self.assertFalse(snapshot["valid"])
        self.assertIsNone(snapshot["result"]["track"]["position_xy_m"])
        self.assertEqual(lidar.snapshot()["state"], "ended")
        self.assertEqual(lidar.sequence, sum(record["event"] == "lidar_scan" for record in self.records))
        self.assertTrue(any(result["track"]["valid"] for _, _, result in view.published))

    def test_replay_speed_changes_waits_but_not_original_event_core_results(self):
        _, slow, slow_stop, _ = self.run_worker(speed=.25)
        _, fast, fast_stop, _ = self.run_worker(speed=4)
        self.assertGreater(slow_stop.wait_count, fast_stop.wait_count)
        self.assertEqual(self.event_results(slow), self.event_results(fast))

    def test_stop_interrupts_long_original_gap_and_closes_native_parsers(self):
        records = [{"schema": "smartcart.target_capture.v1", "event": "tick", "at_ns": at,
                    "source": "synthetic"} for at in (0, 60_000_000_000)]
        _, view, stop, snapshot = self.run_worker(records=records, stop_after_waits=3)
        self.assertTrue(stop.is_set())
        self.assertEqual(stop.wait_count, 3)
        self.assertEqual(snapshot["state"], "running")
        self.assertTrue(all(now < REPLAY_CLOCK_ORIGIN_NS + records[-1]["at_ns"]
                            for now, _, _ in view.published))
        self.assertFalse(snapshot["valid"])

    def test_mixed_capture_source_enters_error_and_removes_previous_result(self):
        records = deepcopy(self.records[:2])
        records[1]["source"] = "recorded"
        lidar, _, _, snapshot = self.run_worker(records=records)
        self.assertEqual(snapshot["state"], "error")
        self.assertEqual(snapshot["error"], "tracking_replay_error")
        self.assertIsNone(snapshot["result"])
        self.assertEqual(lidar.snapshot()["state"], "error")

    def test_empty_capture_enters_error_instead_of_finishing_as_valid(self):
        _, _, _, snapshot = self.run_worker(records=[])
        self.assertEqual(snapshot["state"], "error")
        self.assertIsNone(snapshot["result"])
        self.assertFalse(snapshot["valid"])

    def test_explicit_loop_begins_new_empty_iteration_and_remains_interruptible(self):
        records = [{"schema": "smartcart.target_capture.v1", "event": "tick", "at_ns": 0,
                    "source": "synthetic"}]
        _, view, stop, snapshot = self.run_worker(records=records, loop=True, stop_after_iteration=2)
        self.assertEqual(view.iterations, [1, 2])
        self.assertTrue(stop.is_set())
        self.assertEqual(snapshot["state"], "waiting")
        self.assertIsNone(snapshot["result"])
        self.assertEqual(snapshot["replay"]["iteration"], 2)
        self.assertFalse(snapshot["valid"])


if __name__ == "__main__":
    unittest.main()
