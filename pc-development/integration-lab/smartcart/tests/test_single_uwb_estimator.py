"""단일 센서 위치는 오류와 만료 뒤 예전 표본을 재사용하지 않는다."""
import unittest
from types import SimpleNamespace

from test_dual_uwb_estimator import EstimatorTests, SECOND, MS
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.single_estimator import SingleLeftEstimator
from smartcart_uwb.serial_reader import SerialReaders
from test_dual_uwb_transport import assigned_config


class SingleEstimatorTests(unittest.TestCase):
    def setUp(self):
        self.fixture = EstimatorTests()
        self.fixture.setUp()
        self.estimator = SingleLeftEstimator(self.fixture.config, EstimationConfig(range_model="horizontal"))

    def test_left_only_and_deadline(self):
        snapshot = self.fixture.snapshot()
        snapshot["sensors"]["uwb_right"] = {"state": "error", "connected": False,
            "stream_epoch": 0, "latest": None, "last_error": "disconnected"}
        first = self.estimator.update(snapshot, SECOND)
        self.assertTrue(first["valid"])
        self.assertFalse(first["drive_authorized"])
        self.assertTrue(first["provisional"])
        self.assertEqual(first["valid_until_monotonic_ns"], SECOND + 200 * MS)
        self.assertFalse(self.estimator.update(snapshot, SECOND + 200 * MS)["valid"])
        self.assertFalse(self.estimator.update(snapshot, SECOND + 201 * MS)["valid"])
        snapshot["sensors"]["uwb_left"]["latest"] = self.fixture.observation(
            "uwb_left", sequence=2, rx=SECOND + 201 * MS)
        self.assertTrue(self.estimator.update(snapshot, SECOND + 201 * MS)["valid"])

    def test_invalid_frame_cannot_recover_old_sample(self):
        snapshot = self.fixture.snapshot()
        self.assertTrue(self.estimator.update(snapshot, SECOND)["valid"])
        bad = self.fixture.observation("uwb_left", sequence=2, rx=SECOND + MS, invalid="bad_frame:checksum")
        snapshot["sensors"]["uwb_left"].update(state="error", latest=bad, last_error="bad_frame:checksum")
        self.assertFalse(self.estimator.update(snapshot, SECOND + MS)["valid"])
        snapshot["sensors"]["uwb_left"].update(state="live", latest=self.fixture.observation("uwb_left"), last_error=None)
        self.assertFalse(self.estimator.update(snapshot, SECOND + 2 * MS)["valid"])

    def test_unknown_range_model_is_not_guessed(self):
        estimator = SingleLeftEstimator(self.fixture.config, EstimationConfig())
        snapshot = self.fixture.snapshot()
        for now in (SECOND, SECOND + MS):
            result = estimator.update(snapshot, now)
            self.assertFalse(result["valid"])
            self.assertEqual(result["invalid_reason"], "unknown_range_model")

    def test_reader_accepts_only_explicit_single_left_or_dual(self):
        collector = SimpleNamespace(source_mode="live", config=assigned_config())
        reader = SerialReaders(collector, sensor_ids=("uwb_left",))
        self.assertEqual(reader.sensor_ids, ("uwb_left",))
        with self.assertRaises(ValueError):
            SerialReaders(collector, sensor_ids=("uwb_right",))
