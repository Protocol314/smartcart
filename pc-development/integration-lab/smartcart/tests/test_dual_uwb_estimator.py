"""순번·시계·출처·오류 경계가 과거 위치를 되살리지 않는지 장치 없이 검사한다."""

from pathlib import Path
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import unittest
from unittest.mock import patch

from smartcart_uwb.collector import STATUS_SCHEMA
from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.estimator import ESTIMATE_SCHEMA, TagEstimator
from smartcart_uwb.observation import make_observation

EXAMPLE = Path(__file__).resolve().parents[1] / "config" / "dual_uwb.d5.example.json"
SECOND = 1_000_000_000
MS = 1_000_000


class EstimatorTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)
        self.settings = EstimationConfig(range_model="horizontal")
        self.estimator = TagEstimator(self.config, self.settings, source_mode="replay")

    def observation(self, sensor_id, *, sequence=1, epoch=0, rx=SECOND, session="estimator-tests",
                    angle=0, distance=200, source_mode="replay", invalid=None, tag=0x6E19,
                    device_sequence=None):
        frame = {"tag_address": tag, "sequence": sequence & 255 if device_sequence is None else device_sequence,
                 "angle_deg": angle, "distance_cm": distance, "first_path_power": -80, "rx_level": -75}
        return make_observation(
            sensor=self.config.sensors[sensor_id], frame=frame,
            allowed_tag_address=self.config.allowed_tag_address,
            logical_target_id=self.config.logical_target_id, sample_sequence=sequence,
            stream_epoch=epoch, adapter_session=session, source_mode=source_mode,
            rx_ns=rx, source_rx_ns=None, valid_until_ns=rx + self.config.timeout_ms * MS,
            invalid_reason=invalid,
        )

    def snapshot(self, left=None, right=None, *, session="estimator-tests", source_mode="replay"):
        observations = {
            "uwb_left": self.observation("uwb_left") if left is None else left,
            "uwb_right": self.observation("uwb_right") if right is None else right,
        }
        states = {}
        for sensor_id, observation in observations.items():
            states[sensor_id] = {"state": "live" if observation["valid"] else "error", "connected": True,
                                 "stream_epoch": observation["stream_epoch"], "latest": observation,
                                 "last_error": observation["invalid_reason"]}
        return {"schema": STATUS_SCHEMA, "source_mode": source_mode, "adapter_session": session,
                "drive_authorized": False, "sensors": states}

    def pair(self, sequence, rx, **values):
        return self.snapshot(self.observation("uwb_left", sequence=sequence, rx=rx, **values),
                             self.observation("uwb_right", sequence=sequence, rx=rx, **values),
                             session=values.get("session", "estimator-tests"),
                             source_mode=values.get("source_mode", "replay"))

    def assert_invalid(self, result, reason=None):
        self.assertFalse(result["valid"])
        self.assertIsNone(result["position_xy_m"])
        self.assertIsNone(result["error_radius_m"])
        self.assertEqual(result["valid_until_monotonic_ns"], 0)
        self.assertFalse(result["drive_authorized"])
        if reason is not None:
            self.assertEqual(result["invalid_reason"], reason)

    def test_conditional_estimate_contains_bounded_metadata_and_never_authorizes_drive(self):
        result = self.estimator.update(self.snapshot(), SECOND)
        self.assertTrue(result["valid"])
        self.assertEqual(result["schema"], ESTIMATE_SCHEMA)
        self.assertEqual(result["frame_id"], "base_link")
        self.assertAlmostEqual(result["position_xy_m"][0], 2.756)
        self.assertAlmostEqual(result["position_xy_m"][1], 0)
        self.assertEqual(set(result["samples"]), {"uwb_left", "uwb_right"})
        self.assertTrue(result["provisional"])
        self.assertFalse(result["drive_authorized"])
        self.assertEqual(result["estimate_sequence"], 1)
        self.assertEqual(result["valid_until_monotonic_ns"], SECOND + 200 * MS)

    def test_unknown_range_never_silently_uses_horizontal_distance(self):
        estimator = TagEstimator(self.config, EstimationConfig())
        for now in (SECOND, SECOND + MS):
            self.assert_invalid(estimator.update(self.snapshot(), now), "range_model_unknown")

    def test_tick_holds_position_and_deadline_but_increases_elapsed_error(self):
        snapshot = self.snapshot()
        first = self.estimator.update(snapshot, SECOND)
        second = self.estimator.update(snapshot, SECOND + 50 * MS)
        self.assertTrue(second["valid"])
        for field in ("estimate_sequence", "valid_until_monotonic_ns", "reference_monotonic_ns",
                      "position_xy_m", "error_at_reference_m"):
            self.assertEqual(first[field], second[field])
        self.assertAlmostEqual(second["error_radius_m"] - first["error_radius_m"], .15)
        self.assertEqual(second["estimate_age_ms"], 50)

    def test_one_new_sensor_cannot_extend_estimate_lifetime(self):
        first = self.estimator.update(self.snapshot(), SECOND)
        one_new = self.snapshot(self.observation("uwb_left", sequence=2, rx=SECOND + 10 * MS))
        held = self.estimator.update(one_new, SECOND + 10 * MS)
        self.assertTrue(held["valid"])
        self.assertEqual(first["valid_until_monotonic_ns"], held["valid_until_monotonic_ns"])
        self.assertEqual(held["estimate_sequence"], 1)
        both_new = self.pair(2, SECOND + 11 * MS)
        both_new["sensors"]["uwb_left"]["latest"] = one_new["sensors"]["uwb_left"]["latest"]
        fresh = self.estimator.update(both_new, SECOND + 11 * MS)
        self.assertTrue(fresh["valid"])
        self.assertEqual(fresh["estimate_sequence"], 2)

    def test_skew_failure_clears_old_position_and_matching_new_counterpart_can_recover(self):
        self.estimator.update(self.snapshot(), SECOND)
        lagging = self.snapshot(self.observation("uwb_left", sequence=2, rx=SECOND + 100 * MS))
        self.assert_invalid(self.estimator.update(lagging, SECOND + 100 * MS), "pair_skew")
        matching = self.snapshot(lagging["sensors"]["uwb_left"]["latest"],
                                 self.observation("uwb_right", sequence=2, rx=SECOND + 101 * MS))
        self.assertTrue(self.estimator.update(matching, SECOND + 101 * MS)["valid"])

    def test_equal_device_numbers_do_not_make_a_late_pair_simultaneous(self):
        snapshot = self.snapshot(self.observation("uwb_left", sequence=8, rx=SECOND),
                                 self.observation("uwb_right", sequence=8, rx=SECOND + 100 * MS))
        self.assert_invalid(self.estimator.update(snapshot, SECOND + 100 * MS), "pair_skew")

    def test_age_expires_before_collector_timeout_and_both_new_samples_are_required(self):
        snapshot = self.snapshot()
        self.estimator.update(snapshot, SECOND)
        self.assert_invalid(self.estimator.update(snapshot, SECOND + 200 * MS), "observation_stale")
        one_new = self.snapshot(self.observation("uwb_left", sequence=2, rx=SECOND + 201 * MS))
        self.assert_invalid(self.estimator.update(one_new, SECOND + 201 * MS))
        both = self.pair(2, SECOND + 202 * MS)
        both["sensors"]["uwb_left"]["latest"] = one_new["sensors"]["uwb_left"]["latest"]
        self.assertTrue(self.estimator.update(both, SECOND + 202 * MS)["valid"])

    def test_bad_frame_requires_both_fresh_and_same_fault_does_not_move_barrier_forever(self):
        self.estimator.update(self.snapshot(), SECOND)
        bad = self.observation("uwb_left", sequence=2, rx=SECOND + MS, invalid="bad_frame:checksum")
        self.assert_invalid(self.estimator.update(self.snapshot(bad), SECOND + MS), "bad_frame:checksum")
        right_new = self.observation("uwb_right", sequence=2, rx=SECOND + 2 * MS)
        self.assert_invalid(self.estimator.update(self.snapshot(bad, right_new), SECOND + 2 * MS))
        left_new = self.observation("uwb_left", sequence=3, rx=SECOND + 3 * MS)
        self.assertTrue(self.estimator.update(self.snapshot(left_new, right_new), SECOND + 3 * MS)["valid"])

    def test_invalidated_same_sample_cannot_be_relabelled_valid(self):
        self.estimator.update(self.snapshot(), SECOND)
        invalidated = self.snapshot()
        state = invalidated["sensors"]["uwb_left"]
        state.update(state="error", last_error="disconnected", connected=False)
        state["latest"].update(valid=False, invalid_reason="disconnected")
        self.assert_invalid(self.estimator.update(invalidated, SECOND + MS), "disconnected")
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND + 2 * MS), "sample_reuse")

    def test_explicit_reset_consumes_current_pair_and_preserves_session_and_mode(self):
        first = self.estimator.update(self.snapshot(), SECOND)
        reset = self.estimator.reset("mixed_original_sources")
        self.assert_invalid(reset, "mixed_original_sources")
        self.assertEqual(reset["adapter_session"], first["adapter_session"])
        self.assertEqual(reset["source_mode"], "replay")
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND + MS), "awaiting_fresh_pair")
        self.assertTrue(self.estimator.update(self.pair(2, SECOND + 2 * MS), SECOND + 2 * MS)["valid"])

    def test_epoch_change_requires_fresh_data_from_both_independent_epochs(self):
        self.estimator.update(self.snapshot(), SECOND)
        left = self.observation("uwb_left", sequence=2, epoch=1, rx=SECOND + MS)
        transition = self.snapshot(left)
        self.assert_invalid(self.estimator.update(transition, SECOND + MS), "epoch_changed")
        right = self.observation("uwb_right", sequence=2, epoch=0, rx=SECOND + 2 * MS)
        self.assert_invalid(self.estimator.update(self.snapshot(left, right), SECOND + 2 * MS), "awaiting_fresh_pair")
        left = self.observation("uwb_left", sequence=3, epoch=1, rx=SECOND + 3 * MS)
        self.assertTrue(self.estimator.update(self.snapshot(left, right), SECOND + 3 * MS)["valid"])

    def test_session_change_is_quarantined_and_retired_session_cannot_return(self):
        self.estimator.update(self.snapshot(), SECOND)
        session = "new-session"
        changed = self.pair(1, SECOND + MS, session=session)
        self.assert_invalid(self.estimator.update(changed, SECOND + MS), "session_changed")
        self.assertTrue(self.estimator.update(self.pair(2, SECOND + 2 * MS, session=session), SECOND + 2 * MS)["valid"])
        self.assert_invalid(self.estimator.update(self.pair(3, SECOND + 3 * MS), SECOND + 3 * MS), "retired_session")

    def test_mixed_source_or_session_clears_previously_valid_position(self):
        for field, value in (("source_mode", "live"), ("adapter_session", "other-session")):
            with self.subTest(field=field):
                estimator = TagEstimator(self.config, self.settings)
                estimator.update(self.snapshot(), SECOND)
                snapshot = self.pair(2, SECOND + MS)
                snapshot["sensors"]["uwb_right"]["latest"][field] = value
                self.assert_invalid(estimator.update(snapshot, SECOND + MS), "invalid_snapshot")
                self.assert_invalid(estimator.update(self.snapshot(), SECOND + 2 * MS))

    def test_same_sample_payload_and_timestamp_mutation_are_rejected(self):
        for field, value in (("range_m", 2.1), ("rx_monotonic_ns", SECOND + MS)):
            with self.subTest(field=field):
                estimator = TagEstimator(self.config, self.settings)
                estimator.update(self.snapshot(), SECOND)
                snapshot = self.snapshot()
                snapshot["sensors"]["uwb_left"]["latest"][field] = value
                if field == "rx_monotonic_ns":
                    snapshot["sensors"]["uwb_left"]["latest"]["valid_until_monotonic_ns"] += MS
                self.assert_invalid(estimator.update(snapshot, SECOND + MS), "sample_reuse")

    def test_host_clock_regression_invalidates_and_cannot_reuse_previous_pair(self):
        self.estimator.update(self.snapshot(), SECOND)
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND - 1), "non_monotonic_time")
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND + MS), "awaiting_fresh_pair")
        self.assertTrue(self.estimator.update(self.pair(2, SECOND + 2 * MS), SECOND + 2 * MS)["valid"])

    def test_geometry_disagreement_immediately_clears_position(self):
        self.estimator.update(self.snapshot(), SECOND)
        snapshot = self.snapshot(self.observation("uwb_left", sequence=2, angle=90, rx=SECOND + MS),
                                 self.observation("uwb_right", sequence=2, rx=SECOND + MS))
        self.assert_invalid(self.estimator.update(snapshot, SECOND + MS), "inconsistent_positions")
        self.assert_invalid(self.estimator.update(snapshot, SECOND + 2 * MS))
        self.assertTrue(self.estimator.update(self.pair(3, SECOND + 3 * MS), SECOND + 3 * MS)["valid"])

    def test_one_sensor_outlier_clears_held_position_before_other_sensor_updates(self):
        self.estimator.update(self.snapshot(), SECOND)
        left_bad = self.observation("uwb_left", sequence=2, angle=90, rx=SECOND + MS)
        snapshot = self.snapshot(left_bad)
        self.assert_invalid(self.estimator.update(snapshot, SECOND + MS), "inconsistent_positions")
        right_new = self.observation("uwb_right", sequence=2, rx=SECOND + 2 * MS)
        self.assert_invalid(self.estimator.update(self.snapshot(left_bad, right_new), SECOND + 2 * MS),
                            "awaiting_fresh_pair")
        left_new = self.observation("uwb_left", sequence=3, rx=SECOND + 3 * MS)
        self.assertTrue(self.estimator.update(self.snapshot(left_new, right_new), SECOND + 3 * MS)["valid"])

    def test_malformed_fields_flags_and_nonfinite_numbers_fail_closed(self):
        mutations = [
            lambda s: s.update(extra=True),
            lambda s: s.update(drive_authorized=True),
            lambda s: s["sensors"].update(extra={}),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(range_m=float("nan")),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(range_m=10**1000),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(sample_sequence=True),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(id_valid=1),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(frame_id="other"),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(mounting_verified=True),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(calibration_profile="frozen"),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(device_timestamp_ns=1),
            lambda s: s["sensors"]["uwb_left"]["latest"].update(invalid_reason="x" * 129),
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                estimator = TagEstimator(self.config, self.settings)
                estimator.update(self.snapshot(), SECOND)
                snapshot = self.pair(2, SECOND + MS)
                mutation(snapshot)
                self.assert_invalid(estimator.update(snapshot, SECOND + MS))
                self.assert_invalid(estimator.update(self.snapshot(), SECOND + 2 * MS))

    def test_wrong_tag_and_missing_sensor_are_invalid_not_fallback_position(self):
        self.estimator.update(self.snapshot(), SECOND)
        wrong = self.observation("uwb_left", sequence=2, rx=SECOND + MS, tag=1)
        self.assert_invalid(self.estimator.update(self.snapshot(wrong), SECOND + MS), "id_mismatch")
        waiting = self.snapshot()
        waiting["sensors"]["uwb_left"].update(state="waiting", latest=None, last_error=None)
        self.assert_invalid(self.estimator.update(waiting, SECOND + 2 * MS), "waiting_for_pair")

    def test_device_sequence_wrap_is_valid_but_duplicate_and_regression_are_not(self):
        for index, device_sequence in enumerate((254, 255, 0)):
            at = SECOND + index * MS
            snapshot = self.pair(index + 1, at, device_sequence=device_sequence)
            self.assertTrue(self.estimator.update(snapshot, at)["valid"])
        duplicate = self.pair(4, SECOND + 4 * MS, device_sequence=0)
        self.assert_invalid(self.estimator.update(duplicate, SECOND + 4 * MS), "device_sequence_regression")

    def test_original_capture_timestamp_is_metadata_not_a_second_clock(self):
        snapshot = self.snapshot()
        snapshot["sensors"]["uwb_left"]["latest"]["source_rx_monotonic_ns"] = 0
        snapshot["sensors"]["uwb_right"]["latest"]["source_rx_monotonic_ns"] = 999999999999
        self.assertTrue(self.estimator.update(snapshot, SECOND)["valid"])

    def test_outputs_are_copies_and_ticks_do_not_accumulate_position_history(self):
        snapshot = self.snapshot()
        result = self.estimator.update(snapshot, SECOND)
        result["position_xy_m"][0] = 999
        result["samples"]["uwb_left"]["sample_sequence"] = 999
        again = self.estimator.update(snapshot, SECOND + MS)
        self.assertAlmostEqual(again["position_xy_m"][0], 2.756)
        self.assertEqual(again["samples"]["uwb_left"]["sample_sequence"], 1)

    def test_unexpected_geometry_exception_cannot_leave_old_position_alive(self):
        self.estimator.update(self.snapshot(), SECOND)
        with patch("smartcart_uwb.estimator.fuse_pair", side_effect=RuntimeError("internal")):
            self.assert_invalid(self.estimator.update(self.pair(2, SECOND + MS), SECOND + MS), "estimation_error")
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND + 2 * MS))

    def test_invalid_now_type_clears_current_estimate(self):
        self.estimator.update(self.snapshot(), SECOND)
        self.assert_invalid(self.estimator.update(self.snapshot(), True))
        self.assert_invalid(self.estimator.update(self.snapshot(), SECOND + MS), "awaiting_fresh_pair")

    def test_verified_flags_do_not_turn_raw_profile_into_validated_calibration_or_drive(self):
        sensors = {key: replace(sensor, mounting_verified=True) for key, sensor in self.config.sensors.items()}
        self.config = replace(self.config, sensors=sensors)
        estimator = TagEstimator(self.config, replace(self.settings, assumptions_verified=True))
        result = estimator.update(self.snapshot(), SECOND)
        self.assertTrue(result["valid"])
        self.assertTrue(result["provisional"])
        self.assertFalse(result["drive_authorized"])

    def test_concurrent_ticks_are_consistent_and_do_not_increment_estimate_sequence(self):
        snapshot = self.snapshot()
        first = self.estimator.update(snapshot, SECOND)
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: self.estimator.update(snapshot, SECOND + MS), range(20)))
        for result in results:
            self.assertTrue(result["valid"])
            self.assertEqual(result["estimate_sequence"], first["estimate_sequence"])
            self.assertEqual(result["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])


if __name__ == "__main__":
    unittest.main()
