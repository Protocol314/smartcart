"""이미 위치가 있는 추정기에 잘못된 재생을 요청해도 과거 위치가 남지 않는다."""
from dataclasses import replace
from itertools import islice
from pathlib import Path
import unittest

from smartcart_uwb.collector import DualCollector
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.estimation_replay import estimate_records
from smartcart_uwb.estimator import TagEstimator
from smartcart_uwb.replay import read_records

ROOT = Path(__file__).resolve().parents[1]


class ReplayFailureGuards(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "config" / "dual_uwb.d5.example.json")
        self.settings = load_estimation_config(ROOT / "config" / "uwb_estimation.synthetic.json")
        self.collector = DualCollector(self.config, source_mode="replay")
        self.addCleanup(self.collector.close)
        self.estimator = TagEstimator(self.config, self.settings)
        records = islice(read_records(ROOT / "samples" / "dual_uwb.position.synthetic.jsonl"), 10)
        self.last = list(estimate_records(records, self.collector, self.estimator))[-1]
        self.assertTrue(self.last["estimate"]["valid"])

    def assert_old_position_cannot_return(self):
        result = self.estimator.update(self.last["snapshot"], self.last["replay_now_ns"])
        self.assertFalse(result["valid"])
        self.assertIsNone(result["position_xy_m"])
        self.assertFalse(result["drive_authorized"])

    def test_config_mismatch_discards_already_valid_position(self):
        with DualCollector(replace(self.config, timeout_ms=499), source_mode="replay") as wrong:
            with self.assertRaises(ValueError):
                list(estimate_records([], wrong, self.estimator))
        self.assert_old_position_cannot_return()

    def test_live_mode_rejected_and_previous_position_discarded(self):
        with DualCollector(self.config, source_mode="live") as wrong:
            with self.assertRaises(ValueError):
                list(estimate_records([], wrong, self.estimator))
        self.assert_old_position_cannot_return()

    def test_malformed_new_capture_discards_previous_position(self):
        with self.assertRaises(ValueError):
            list(estimate_records([{"schema": "private invalid input"}], self.collector, self.estimator))
        self.assert_old_position_cannot_return()


if __name__ == "__main__":
    unittest.main()
