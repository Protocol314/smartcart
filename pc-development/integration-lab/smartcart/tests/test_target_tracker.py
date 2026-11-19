"""연관 모호성·새 표본 확인·가림·고장 복구가 유효 위치를 잘못 유지하지 않는지 검사한다."""

from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import math
import unittest

from smartcart_tracking.config import TrackingConfig
from smartcart_tracking.tracker import CLUSTERS_SCHEMA, TRACK_SCHEMA, TargetTracker
from smartcart_uwb.config import load_config
from smartcart_uwb.estimator import ESTIMATE_SCHEMA
from smartcart_uwb.geometry import ALIGNMENT_METHOD

SECOND = 1_000_000_000
MS = 1_000_000
EXAMPLE = Path(__file__).resolve().parents[1] / "config" / "dual_uwb.d5.example.json"


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)
        self.settings = TrackingConfig()
        self.tracker = TargetTracker(self.config, self.settings)

    def cluster(self, xy=(2.0, 0.0), identifier=0):
        return {"cluster_id": identifier, "centroid_xy_m": list(xy), "radius_m": .11, "width_m": .2,
                "point_count": 5, "first_ray": 10, "last_ray": 14, "error_radius_m": .03}

    def lidar(self, sequence=1, at=SECOND, clusters=None, epoch=0, session="lidar-tests"):
        return {"schema": CLUSTERS_SCHEMA, "sensor_id": "lidar_center", "frame_id": "base_link",
                "input_frame_id": self.config.lidar.frame_id, "adapter_session": session,
                "stream_epoch": epoch, "sample_sequence": sequence, "source_mode": "replay",
                "timestamp_source": "HOST_RX", "rx_monotonic_ns": at,
                "valid_until_monotonic_ns": at + self.settings.lidar_timeout_ms * MS,
                "valid": True, "invalid_reason": None,
                "clusters": [self.cluster()] if clusters is None else clusters,
                "drive_authorized": False, "provisional": True}

    def uwb(self, sequence=1, at=SECOND, xy=(2.0, 0.0), epoch=0, session="uwb-tests", error=.1):
        return {"schema": ESTIMATE_SCHEMA, "valid": True, "invalid_reason": None, "frame_id": "base_link",
                "position_xy_m": list(xy), "tag_height_m": None, "error_radius_m": error,
                "error_at_reference_m": error, "estimate_age_ms": 0.0, "pair_residual_m": .01,
                "rx_skew_ns": 0, "reference_monotonic_ns": at, "valid_until_monotonic_ns": at + 200 * MS,
                "estimate_sequence": sequence, "adapter_session": session, "source_mode": "replay",
                "provisional": True, "range_model": "horizontal", "alignment_method": ALIGNMENT_METHOD,
                "drive_authorized": False, "samples": {
                    name: {"stream_epoch": epoch, "sample_sequence": sequence,
                           "rx_monotonic_ns": at, "device_sequence": sequence & 255}
                    for name in ("uwb_left", "uwb_right")}}

    def invalid_uwb(self, sequence=3, at=SECOND, reason="bad_frame:checksum"):
        value = self.uwb(sequence, at)
        value.update(valid=False, invalid_reason=reason, valid_until_monotonic_ns=0)
        for name in ("position_xy_m", "tag_height_m", "error_radius_m", "error_at_reference_m", "estimate_age_ms",
                     "pair_residual_m", "rx_skew_ns", "reference_monotonic_ns", "alignment_method"):
            value[name] = None
        return value

    def cycle(self, sequence, *, at=None, xy=(2.0, 0.0), clusters=None, **values):
        at = SECOND + (sequence - 1) * 50 * MS if at is None else at
        return self.tracker.update(self.lidar(sequence, at, clusters=clusters, **values),
                                   self.uwb(sequence, at, xy=xy), at)

    def acquire(self):
        for sequence in range(1, 4):
            result = self.cycle(sequence)
        self.assertTrue(result["valid"])
        return result

    def assert_invalid(self, result, state=None, reason=None):
        self.assertFalse(result["valid"])
        self.assertIsNone(result["position_xy_m"])
        self.assertIsNone(result["velocity_xy_mps"])
        self.assertEqual(result["valid_until_monotonic_ns"], 0)
        self.assertFalse(result["drive_authorized"])
        if state is not None:
            self.assertEqual(result["state"], state)
        if reason is not None:
            self.assertEqual(result["invalid_reason"], reason)

    def test_three_new_pairs_confirm_a_unique_cluster_and_never_authorize_drive(self):
        for sequence in range(1, 4):
            result = self.cycle(sequence)
            self.assertEqual(result["confirmation_count"], sequence)
            self.assertEqual(result["valid"], sequence == 3)
        self.assertEqual(result["schema"], TRACK_SCHEMA)
        self.assertEqual(result["state"], "tracking")
        self.assertEqual(result["position_xy_m"], [2.0, 0.0])
        self.assertEqual(result["frame_id"], "base_link")
        self.assertTrue(result["provisional"])
        self.assertFalse(result["drive_authorized"])

    def test_initial_waiting_sequence_zero_does_not_create_an_error_or_block_first_pair(self):
        initial = self.lidar(sequence=0)
        initial.update(valid=False, invalid_reason="waiting_for_scan", clusters=[], valid_until_monotonic_ns=0)
        first = self.tracker.update(initial, None, SECOND)
        self.assert_invalid(first, "waiting")
        value = self.invalid_uwb(sequence=0, reason="waiting_for_pair")
        value.update(adapter_session=None, samples={})
        waiting = self.tracker.update(initial, value, SECOND)
        self.assert_invalid(waiting, "waiting")
        for sequence in (1, 2, 3):
            result = self.cycle(sequence)
            self.assertEqual(result["confirmation_count"], sequence)
        self.assertTrue(result["valid"])

    def test_bbox_diagonal_can_exceed_twice_radius_for_cross_shaped_points(self):
        cluster = self.cluster()
        cluster.update(radius_m=.1, width_m=2 * math.sqrt(2) * .1)
        result = self.cycle(1, clusters=[cluster])
        self.assert_invalid(result, "acquiring", "confirming")

    def test_same_scan_tick_and_one_new_input_cannot_add_confirmation_hits(self):
        first = self.cycle(1)
        lidar = self.lidar(1)
        for index in (2, 3, 4):
            at = SECOND + index * MS
            result = self.tracker.update(lidar, self.uwb(index, at), at)
            self.assert_invalid(result, "acquiring")
            self.assertEqual(result["confirmation_count"], first["confirmation_count"])
        scan2 = self.lidar(2, SECOND + 5 * MS)
        result = self.tracker.update(scan2, self.uwb(4, SECOND + 4 * MS), SECOND + 5 * MS)
        self.assertEqual(result["confirmation_count"], 2)
        for index in (3, 4):
            at = SECOND + (index + 4) * MS
            result = self.tracker.update(self.lidar(index, at), self.uwb(4, SECOND + 4 * MS), at)
            self.assertEqual(result["confirmation_count"], 2)

    def test_asynchronous_new_pair_is_counted_once_when_both_have_advanced(self):
        self.cycle(1)
        scan2 = self.lidar(2, SECOND + 20 * MS)
        result = self.tracker.update(scan2, self.uwb(1), SECOND + 20 * MS)
        self.assertEqual(result["confirmation_count"], 1)
        result = self.tracker.update(scan2, self.uwb(2, SECOND + 21 * MS), SECOND + 21 * MS)
        self.assertEqual(result["confirmation_count"], 2)
        result = self.tracker.update(scan2, self.uwb(3, SECOND + 22 * MS), SECOND + 22 * MS)
        self.assertEqual(result["confirmation_count"], 2)

    def test_valid_ticks_do_not_extend_track_deadline_or_sequence(self):
        first = self.acquire()
        at = SECOND + 120 * MS
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS), self.uwb(3, SECOND + 100 * MS), at)
        self.assertTrue(result["valid"])
        self.assertEqual(result["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])
        self.assertEqual(result["track_sequence"], first["track_sequence"])
        self.assertGreater(result["error_radius_m"], first["error_radius_m"])

    def test_tracking_new_lidar_can_reuse_fresh_uwb_without_extending_uwb_deadline(self):
        first = self.acquire()
        at = SECOND + 140 * MS
        result = self.tracker.update(self.lidar(4, at), self.uwb(3, SECOND + 100 * MS), at)
        self.assertTrue(result["valid"])
        self.assertEqual(result["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])
        self.assertEqual(result["track_sequence"], first["track_sequence"] + 1)

    def test_uwb_gate_ignores_far_clusters_but_does_not_pick_nearest_of_two_candidates(self):
        self.acquire()
        result = self.cycle(4, clusters=[self.cluster(), self.cluster((4, 0), 1)])
        self.assertTrue(result["valid"])
        self.assertEqual(result["candidate_count"], 1)
        ambiguous = self.cycle(5, clusters=[self.cluster(), self.cluster((2, .3), 1)])
        self.assert_invalid(ambiguous, "ambiguous", "ambiguous_candidates")
        self.assertEqual(ambiguous["candidate_count"], 2)
        self.assertEqual(ambiguous["confirmation_count"], 0)
        self.assertIsNotNone(ambiguous["predicted_position_xy_m"])

    def test_ambiguous_recovery_requires_three_new_pairs_and_keeps_short_track_id(self):
        first = self.acquire()
        self.cycle(4, clusters=[self.cluster(), self.cluster((2, .3), 1)])
        for sequence in range(5, 8):
            result = self.cycle(sequence)
            self.assertEqual(result["track_id"], first["track_id"])
            self.assertEqual(result["valid"], sequence == 7)

    def test_short_empty_occlusion_has_only_prediction_and_three_hit_reconfirmation(self):
        first = self.acquire()
        for sequence in (4, 5):
            result = self.cycle(sequence, clusters=[])
            self.assert_invalid(result, "coasting", "no_candidate")
            self.assertEqual(result["track_id"], first["track_id"])
            self.assertEqual(result["confirmation_count"], 0)
            self.assertIsNotNone(result["predicted_position_xy_m"])
        for sequence in (6, 7, 8):
            result = self.cycle(sequence)
            self.assertEqual(result["track_id"], first["track_id"])
            self.assertEqual(result["valid"], sequence == 8)

    def test_repeated_uwb_updates_during_empty_scan_do_not_move_reacquisition_boundary(self):
        self.acquire()
        empty = self.lidar(4, SECOND + 150 * MS, clusters=[])
        self.tracker.update(empty, self.uwb(4, SECOND + 150 * MS), SECOND + 150 * MS)
        for sequence in (5, 6):
            at = SECOND + (sequence - 1) * 50 * MS
            self.tracker.update(empty, self.uwb(sequence, at), at)
        for scan_sequence, uwb_sequence in ((5, 7), (6, 8), (7, 9)):
            at = SECOND + (uwb_sequence - 1) * 50 * MS
            scan = self.lidar(scan_sequence, at)
            result = self.tracker.update(scan, self.uwb(uwb_sequence, at), at)
            self.assertEqual(result["confirmation_count"], scan_sequence - 4)
            self.assertEqual(result["valid"], scan_sequence == 7)

    def test_empty_valid_scans_cannot_predict_after_occlusion_timeout(self):
        self.acquire()
        for sequence in range(4, 12):
            result = self.cycle(sequence, clusters=[])
        self.assert_invalid(result, "lost", "occlusion_timeout")
        self.assertIsNone(result["track_id"])
        self.assertIsNone(result["predicted_position_xy_m"])
        for sequence in (12, 13, 14):
            result = self.cycle(sequence)
        self.assertTrue(result["valid"])
        self.assertEqual(result["track_id"], 2)

    def test_stale_uwb_immediately_discards_id_and_prediction(self):
        self.acquire()
        at = SECOND + 300 * MS
        result = self.tracker.update(self.lidar(4, at), self.uwb(3, SECOND + 100 * MS), at)
        self.assert_invalid(result, "error", "uwb_stale")
        self.assertIsNone(result["track_id"])
        self.assertIsNone(result["predicted_position_xy_m"])

    def test_lidar_timeout_is_not_extended_to_occlusion_timeout(self):
        self.acquire()
        at = SECOND + 400 * MS
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS), self.uwb(4, at), at)
        self.assert_invalid(result, "error", "lidar_stale")
        self.assertIsNone(result["predicted_position_xy_m"])

    def test_invalid_uwb_requires_fresh_inputs_and_new_id_after_three_confirmations(self):
        first = self.acquire()
        at = SECOND + 150 * MS
        result = self.tracker.update(self.lidar(4, at), self.invalid_uwb(3, at), at)
        self.assert_invalid(result, "error", "uwb_invalid")
        for sequence in (5, 6, 7):
            result = self.cycle(sequence)
            self.assertEqual(result["valid"], sequence == 7)
        self.assertNotEqual(result["track_id"], first["track_id"])

    def test_continuity_jump_cannot_steal_old_id(self):
        first = self.acquire()
        result = self.cycle(4, xy=(4, 0), clusters=[self.cluster((4, 0))])
        self.assert_invalid(result, "lost", "continuity_jump")
        self.assertIsNone(result["track_id"])
        for sequence in (5, 6, 7):
            result = self.cycle(sequence, xy=(4, 0), clusters=[self.cluster((4, 0))])
        self.assertTrue(result["valid"])
        self.assertNotEqual(result["track_id"], first["track_id"])

    def test_cluster_ids_are_scan_local_and_not_identity(self):
        first = self.acquire()
        result = self.cycle(4, clusters=[self.cluster(identifier=7)])
        self.assertTrue(result["valid"])
        self.assertEqual(result["track_id"], first["track_id"])

    def test_relative_velocity_is_bounded_and_is_not_reported_as_world_velocity(self):
        self.acquire()
        for sequence in range(4, 9):
            xy = (2 + (sequence - 3) * .1, 0)
            result = self.cycle(sequence, xy=xy, clusters=[self.cluster(xy)])
            self.assertTrue(result["valid"])
            self.assertLessEqual(math.hypot(*result["velocity_xy_mps"]), self.settings.max_relative_speed_mps)
        self.assertNotIn("world_velocity", result)

    def test_large_uwb_error_and_time_skew_fail_closed(self):
        self.acquire()
        at = SECOND + 150 * MS
        result = self.tracker.update(self.lidar(4, at), self.uwb(4, at, error=1.3), at)
        self.assert_invalid(result, "error", "uwb_uncertain")
        fresh = TargetTracker(self.config, self.settings)
        result = fresh.update(self.lidar(1, SECOND + 101 * MS), self.uwb(), SECOND + 101 * MS)
        self.assert_invalid(result, "error", "sensor_skew")

    def test_same_sample_payload_mutation_or_source_mix_discards_previous_track(self):
        for field, value in (("source_mode", "live"), ("input_frame_id", "wrong"), ("sample_sequence", True)):
            with self.subTest(field=field):
                self.setUp()
                self.acquire()
                lidar = self.lidar(3, SECOND + 100 * MS)
                lidar[field] = value
                result = self.tracker.update(lidar, self.uwb(3, SECOND + 100 * MS), SECOND + 101 * MS)
                self.assert_invalid(result, "error")
                self.assertIsNone(result["track_id"])

    def test_nonfinite_large_unknown_fields_and_forged_flags_are_rejected(self):
        mutations = [lambda a, b: a.update(extra=True), lambda a, b: a.update(drive_authorized=True),
                     lambda a, b: a["clusters"][0].update(radius_m=float("nan")),
                     lambda a, b: a["clusters"][0].update(centroid_xy_m=[10**1000, 0]),
                     lambda a, b: a.update(clusters=[self.cluster()] * 129),
                     lambda a, b: b.update(error_radius_m=float("inf")),
                     lambda a, b: b.update(position_xy_m=[float("nan"), 0]),
                     lambda a, b: b.update(estimate_sequence=True),
                     lambda a, b: b.update(valid=1),
                     lambda a, b: b["samples"]["uwb_left"].update(stream_epoch=True)]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.setUp()
                self.acquire()
                lidar, uwb = self.lidar(4, SECOND + 150 * MS), self.uwb(4, SECOND + 150 * MS)
                mutation(lidar, uwb)
                result = self.tracker.update(lidar, uwb, SECOND + 150 * MS)
                self.assert_invalid(result, "error")
                self.assertIsNone(result["predicted_position_xy_m"])

    def test_epoch_and_session_transitions_require_new_confirmation(self):
        for kind in ("epoch", "session"):
            with self.subTest(kind=kind):
                self.setUp()
                first = self.acquire()
                values = {"epoch": 1} if kind == "epoch" else {"session": "lidar-new"}
                transition = self.cycle(4, **values)
                self.assert_invalid(transition, "error", "source_reset")
                for sequence in (5, 6, 7):
                    result = self.cycle(sequence, **values)
                self.assertTrue(result["valid"])
                self.assertNotEqual(result["track_id"], first["track_id"])

    def test_clock_regression_and_reset_cannot_reuse_old_pair(self):
        self.acquire()
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS), self.uwb(3, SECOND + 100 * MS), SECOND)
        self.assert_invalid(result, "error", "non_monotonic_time")
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS), self.uwb(3, SECOND + 100 * MS), SECOND + 101 * MS)
        self.assert_invalid(result, "waiting", "awaiting_fresh_inputs")
        self.tracker.reset("external_fault")
        for sequence in (4, 5, 6):
            result = self.cycle(sequence)
        self.assertTrue(result["valid"])

    def test_missing_input_has_no_fallback_and_returned_results_are_copies(self):
        first = self.acquire()
        first["position_xy_m"][0] = 999
        at = SECOND + 101 * MS
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS), self.uwb(3, SECOND + 100 * MS), at)
        self.assertEqual(result["position_xy_m"], [2.0, 0.0])
        self.assert_invalid(self.tracker.update(None, self.uwb(4, SECOND + 150 * MS), SECOND + 150 * MS),
                            "error", "missing_input")

    def test_concurrent_ticks_preserve_id_confirmation_and_fixed_deadline(self):
        first = self.acquire()
        lidar, uwb = self.lidar(3, SECOND + 100 * MS), self.uwb(3, SECOND + 100 * MS)
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: self.tracker.update(lidar, uwb, SECOND + 101 * MS), range(20)))
        for result in results:
            self.assertTrue(result["valid"])
            self.assertEqual(result["track_id"], first["track_id"])
            self.assertEqual(result["confirmation_count"], 3)
            self.assertEqual(result["valid_until_monotonic_ns"], first["valid_until_monotonic_ns"])

    def test_association_diagnostics_are_available_only_for_valid_tracking(self):
        acquiring = self.cycle(1)
        self.assertIsNone(acquiring["associated_cluster_id"])
        self.cycle(2)
        tracked = self.cycle(3)
        self.assertEqual(tracked["associated_cluster_id"], 0)
        self.assertEqual(tracked["association_distance_m"], 0)
        self.assertGreater(tracked["association_gate_m"], 0)
        ambiguous = self.cycle(4, clusters=[self.cluster(), self.cluster((2, .3), 1)])
        for key in ("associated_cluster_id", "association_distance_m", "association_gate_m"):
            self.assertIsNone(ambiguous[key])

    def test_same_scan_cannot_switch_to_another_near_cluster_on_uwb_update(self):
        clusters = [self.cluster(), self.cluster((2.2, 0), 1)]
        for sequence in (1, 2, 3):
            result = self.cycle(sequence, xy=(1.45, 0), clusters=clusters)
        self.assertTrue(result["valid"])
        at = SECOND + 101 * MS
        result = self.tracker.update(self.lidar(3, SECOND + 100 * MS, clusters=clusters),
                                     self.uwb(4, at, xy=(2.75, 0)), at)
        self.assert_invalid(result, "lost", "association_changed")
        self.assertIsNone(result["track_id"])


if __name__ == "__main__":
    unittest.main()
