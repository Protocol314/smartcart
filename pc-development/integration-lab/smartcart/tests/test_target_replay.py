"""원시 UWB와 가림을 고려한 합성 LiDAR를 대상 연결까지 통과시키는 회귀 검사.

물체는 분류된 사람이 아닌 원형 반사체다. 합성 자료는 실물 수락 검수나
주행 허가를 의미하지 않으며 모터·ROS·직렬 포트를 열지 않는다.
"""
from __future__ import annotations

import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.native import NativeParser
from test_dual_uwb_position_replay import position_frame

ROOT = Path(__file__).resolve().parents[1]
MOUNTING = ROOT / "config" / "dual_uwb.d5.example.json"
UWB_SETTINGS = ROOT / "config" / "uwb_estimation.synthetic.json"
TRACKING_SETTINGS = ROOT / "config" / "target_tracking.synthetic.json"
CAPTURE = ROOT / "samples" / "target_tracking.synthetic.jsonl"
TRUTH = ROOT / "samples" / "target_tracking.truth.json"
CAPTURE_SCHEMA = "smartcart.target_capture.v1"
TAG_POINT = [2.8, 0.0]
UNIQUE_SCENE = ((2.8, 0.0, .10), (3.2, 1.4, .12))
CROSSING_SCENE = ((2.8, -.22, .08), (2.8, .22, .08))
STRAY_SCENE = ((5.2, 1.8, .12),)


def raycast_scan(mounting, circles=(), walls=()):
    """각 빔의 가장 앞선 원·선분 교점만 사용하여 뒤쪽 물체를 그리지 않는다."""
    origin_x, origin_y, _ = mounting.lidar.translation_m
    yaw = math.radians(mounting.lidar.yaw_deg)
    count = 720
    step = 2 * math.pi / (count - 1)
    ranges = []
    for index in range(count):
        angle = -math.pi + index * step + yaw
        dx, dy = math.cos(angle), math.sin(angle)
        closest = math.inf
        for cx, cy, radius in circles:
            x, y = cx - origin_x, cy - origin_y
            dot = dx * x + dy * y
            discriminant = dot * dot - (x * x + y * y - radius * radius)
            if discriminant < 0:
                continue
            distance = dot - math.sqrt(discriminant)
            if .05 <= distance <= 16:
                closest = min(closest, distance)
        for (ax, ay), (bx, by) in walls:
            ax, ay = ax - origin_x, ay - origin_y
            bx, by = bx - origin_x, by - origin_y
            ex, ey = bx - ax, by - ay
            denominator = dx * ey - dy * ex
            if abs(denominator) < 1e-12:
                continue
            distance = (ax * ey - ay * ex) / denominator
            fraction = (ax * dy - ay * dx) / denominator
            if .05 <= distance <= 16 and 0 <= fraction <= 1:
                closest = min(closest, distance)
        ranges.append(round(closest, 6) if math.isfinite(closest) else None)
    return {"angle_min": -math.pi, "angle_max": math.pi, "angle_increment": step,
            "range_min": .05, "range_max": 16.0, "ranges": ranges,
            "scan_time": .02, "time_increment": .02 / (count - 1)}


def build_target_fixture(mounting):
    """커밋된 캡처와 체크포인트를 재생성하는 결정적 합성 도우미."""
    records, checkpoints = [], []
    cursor = 0
    sequence = {"uwb_left": 254, "uwb_right": 254}

    def add(event, *, sensor_id=None, data=None, scan=None, at_ns=None):
        nonlocal cursor
        moment = cursor if at_ns is None else at_ns
        if records and moment < records[-1]["at_ns"]:
            raise AssertionError("합성 사건 시각이 역행했습니다")
        record = {"schema": CAPTURE_SCHEMA, "event": event, "at_ns": moment, "source": "synthetic"}
        if sensor_id is not None:
            record["sensor_id"] = sensor_id
        if data is not None:
            record["data_hex"] = data.hex()
        if scan is not None:
            record["scan"] = scan
        records.append(record)
        cursor = max(cursor, moment + 1_000_000)
        return len(records) - 1

    def point(label, valid, *, event_index=None, same_as=None, different_from=None,
              candidate_min=None, track_id_null=False):
        checkpoints.append({"label": label, "event_index": len(records) - 1 if event_index is None else event_index,
                            "expected_valid": valid, "same_track_as": same_as,
                            "different_track_from": different_from, "candidate_min": candidate_min,
                            "track_id_null": track_id_null,
                            "expected_position_xy_m": TAG_POINT if valid else None,
                            "max_target_error_m": .25 if valid else None})

    def cycle(objects=UNIQUE_SCENE, *, wrong_tag=False, checksum_bad=False, scan=True):
        nonlocal cursor
        start = cursor
        indexes = {}
        for sensor_id, delta in (("uwb_left", 0), ("uwb_right", 5_000_000)):
            data = bytearray(position_frame(mounting.sensors[sensor_id], TAG_POINT, sequence[sensor_id],
                                           tag_address=0x1234 if wrong_tag and sensor_id == "uwb_right" else 0x6E19))
            sequence[sensor_id] = (sequence[sensor_id] + 1) & 255
            if checksum_bad and sensor_id == "uwb_right":
                data[29] ^= 1
            indexes[sensor_id] = add("uwb_bytes", sensor_id=sensor_id, data=bytes(data), at_ns=start + delta)
        if scan:
            indexes["lidar"] = add("lidar_scan", scan=raycast_scan(mounting, objects), at_ns=start + 10_000_000)
        # 다음 왼쪽 입력과 이전 오른쪽 입력도 UWB 40ms 시각차 한도 안에 둔다.
        cursor = start + 40_000_000
        return indexes

    def confirm(label, *, same_as=None, different_from=None):
        for index in range(3):
            cycle()
            if index < 2:
                point(label + f"_confirm_{index + 1}", False)
        point(label, True, same_as=same_as, different_from=different_from, candidate_min=1)

    add("uwb_reconnect", sensor_id="uwb_left")
    add("uwb_reconnect", sensor_id="uwb_right")
    add("lidar_reconnect")
    first_start = cursor
    cycle(); point("initial_confirm_1", False)
    for delta in (20_000_000, 25_000_000, 30_000_000):
        add("tick", at_ns=first_start + delta)
    point("cached_scan_does_not_confirm", False)
    cycle(); point("initial_confirm_2", False)
    cycle(); point("unique_acquired", True, candidate_min=1)
    cycle(); cycle(); point("unique_stable", True, same_as="unique_acquired", candidate_min=1)

    cycle(()); cycle(()); point("short_occlusion", False)
    confirm("short_recovery", same_as="unique_acquired")

    cycle(CROSSING_SCENE); cycle(CROSSING_SCENE)
    point("two_candidates_ambiguous", False, candidate_min=2)
    confirm("crossing_recovery", same_as="short_recovery")

    for _ in range(10):
        cycle(())
    point("long_occlusion_lost", False, track_id_null=True)
    confirm("long_recovery", different_from="crossing_recovery")

    cycle(STRAY_SCENE); point("stray_cannot_steal_id", False)
    confirm("stray_recovery", same_as="long_recovery")

    indices = cycle(wrong_tag=True)
    point("wrong_tag_immediate", False, event_index=indices["uwb_right"], track_id_null=True)
    confirm("wrong_tag_recovery", different_from="stray_recovery")
    indices = cycle(checksum_bad=True)
    point("checksum_immediate", False, event_index=indices["uwb_right"], track_id_null=True)
    confirm("checksum_recovery", different_from="wrong_tag_recovery")

    add("uwb_disconnect", sensor_id="uwb_left")
    point("uwb_disconnect_immediate", False, track_id_null=True)
    add("uwb_reconnect", sensor_id="uwb_left")
    confirm("uwb_reconnect_recovery", different_from="checksum_recovery")
    add("lidar_disconnect")
    point("lidar_disconnect_immediate", False, track_id_null=True)
    add("lidar_reconnect")
    confirm("lidar_reconnect_recovery", different_from="uwb_reconnect_recovery")

    for _ in range(9):
        cycle(scan=False)
    point("lidar_timeout", False, track_id_null=True)
    confirm("lidar_timeout_recovery", different_from="lidar_reconnect_recovery")
    return records, checkpoints


def moving_target_capture(mounting, count=30):
    """정지 카트 앞의 느린 이동 반사체와 고정된 다른 물체를 함께 생성한다."""
    records = []

    def position(at_ns):
        seconds = (at_ns - 10_000_000) / 1_000_000_000
        return [2.8 + .2 * seconds, .2 * math.sin(.2 * seconds)]

    for index, (event, sensor_id) in enumerate((("uwb_reconnect", "uwb_left"),
                                               ("uwb_reconnect", "uwb_right"),
                                               ("lidar_reconnect", None))):
        item = {"schema": CAPTURE_SCHEMA, "event": event, "at_ns": index * 1_000_000,
                "source": "synthetic"}
        if sensor_id is not None:
            item["sensor_id"] = sensor_id
        records.append(item)
    for index in range(count):
        start = 10_000_000 + index * 40_000_000
        for sensor_id, delta in (("uwb_left", 0), ("uwb_right", 5_000_000)):
            at_ns = start + delta
            data = position_frame(mounting.sensors[sensor_id], position(at_ns), (254 + index) & 255)
            records.append({"schema": CAPTURE_SCHEMA, "event": "uwb_bytes", "sensor_id": sensor_id,
                            "at_ns": at_ns, "source": "synthetic", "data_hex": data.hex()})
        at_ns = start + 10_000_000
        point = position(at_ns)
        records.append({"schema": CAPTURE_SCHEMA, "event": "lidar_scan", "at_ns": at_ns,
                        "source": "synthetic",
                        "scan": raycast_scan(mounting, ((point[0], point[1], .10), (3.3, 1.7, .12)))})
    return records, position


class TargetFixtureTests(unittest.TestCase):
    def test_fixture_and_truth_are_reproducible(self):
        mounting = load_config(MOUNTING)
        records, checkpoints = build_target_fixture(mounting)
        stored = [json.loads(line) for line in CAPTURE.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(records, stored)
        truth = json.loads(TRUTH.read_text(encoding="utf-8"))
        self.assertEqual(truth["source"], "synthetic")
        self.assertEqual(checkpoints, truth["checkpoints"])
        self.assertTrue(all(item["source"] == "synthetic" for item in records))
        self.assertEqual([item["at_ns"] for item in records], sorted(item["at_ns"] for item in records))

    def test_nearest_hit_occludes_rear_objects_and_wall_hides_target(self):
        mounting = load_config(MOUNTING)
        near = raycast_scan(mounting, ((2.0, 0, .15),))
        both = raycast_scan(mounting, ((2.0, 0, .15), (3.0, 0, .1)))
        self.assertEqual(near["ranges"][359:361], both["ranges"][359:361])
        wall = raycast_scan(mounting, ((2.8, 0, .1),), (((1.8, -1), (1.8, 1)),))
        self.assertTrue(all(value < 1.0 for value in wall["ranges"][359:361]))
        empty = raycast_scan(mounting)
        self.assertTrue(all(value is None for value in empty["ranges"]))
        self.assertEqual(len(empty["ranges"]), 720)
        self.assertEqual(len(empty), 8)


class TargetReplayEndToEndTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from smartcart_tracking.config import load_tracking_config
        from smartcart_tracking.pipeline import TrackingPipeline
        from smartcart_tracking.replay import track_file, track_records
        cls.pipeline_type = TrackingPipeline
        cls.track_file = staticmethod(track_file)
        cls.track_records = staticmethod(track_records)
        cls.mounting = load_config(MOUNTING)
        cls.uwb_settings = load_estimation_config(UWB_SETTINGS)
        cls.tracking_settings = load_tracking_config(TRACKING_SETTINGS)
        try:
            NativeParser().close()
        except (OSError, RuntimeError) as exc:
            raise unittest.SkipTest("명시적으로 빌드한 기존 C 파서가 필요합니다") from exc

    def pipeline(self):
        pipeline = self.pipeline_type(self.mounting, self.uwb_settings, self.tracking_settings,
                                      source_mode="replay", session_id="synthetic-target-test")
        self.addCleanup(pipeline.close)
        return pipeline

    def test_real_replay_track_drives_only_the_synthetic_follow_preview(self):
        from smartcart_tracking.follow_preview import simulate
        acquired = False
        for event in self.track_file(CAPTURE, self.pipeline()):
            track = event['track']
            if not acquired and track['valid']:
                now = track['reference_monotonic_ns']
                clearance = {'synthetic': True, 'valid': True, 'forward_clear': True,
                             'rotation_clear': True, 'observed_ns': now, 'valid_until_ns': now + 100_000_000}
                preview = simulate(track, clearance, now)
                self.assertEqual(preview['state'], 'preview_follow')
                self.assertGreater(preview['preview_forward_fraction'], 0)
                self.assertFalse(preview['drive_authorized'])
                acquired = True
            elif acquired and not track['valid']:
                preview = simulate(track, clearance, now)
                self.assertEqual(preview['state'], 'blocked')
                self.assertEqual(preview['preview_forward_fraction'], 0)
                self.assertEqual(preview['preview_turn_fraction'], 0)
                break
        self.assertTrue(acquired)

    def test_truth_acquisition_ambiguity_coasting_faults_and_recovery(self):
        events = list(self.track_file(CAPTURE, self.pipeline()))
        checkpoints = json.loads(TRUTH.read_text(encoding="utf-8"))["checkpoints"]
        ids = {}
        for expected in checkpoints:
            with self.subTest(label=expected["label"]):
                track = events[expected["event_index"]]["track"]
                self.assertEqual(track["valid"], expected["expected_valid"])
                self.assertFalse(track["drive_authorized"])
                self.assertEqual(track["source_mode"], "replay")
                if expected["candidate_min"] is not None:
                    self.assertGreaterEqual(track["candidate_count"], expected["candidate_min"])
                if expected["track_id_null"]:
                    self.assertIsNone(track["track_id"])
                if track["valid"]:
                    ids[expected["label"]] = track["track_id"]
                    self.assertIsNotNone(track["track_id"])
                    self.assertLessEqual(math.dist(track["position_xy_m"], expected["expected_position_xy_m"]),
                                         expected["max_target_error_m"])
                    if expected["same_track_as"] is not None:
                        self.assertIn(expected["same_track_as"], ids)
                        self.assertEqual(track["track_id"], ids[expected["same_track_as"]])
                    if expected["different_track_from"] is not None:
                        self.assertIn(expected["different_track_from"], ids)
                        self.assertNotEqual(track["track_id"], ids[expected["different_track_from"]])
                else:
                    self.assertIsNone(track["position_xy_m"])
        for event in events:
            self.assertFalse(event["drive_authorized"])
            self.assertFalse(event["track"]["drive_authorized"])
        json.dumps(events, allow_nan=False)

    def test_closer_distractor_is_ignored_for_unique_uwb_target(self):
        records, checkpoints = build_target_fixture(self.mounting)
        first = next(item for item in checkpoints if item["label"] == "unique_acquired")
        initial = records[:first["event_index"] + 1]
        # 태그 반사체보다 가까워도 UWB 위치와 맞지 않는 반사체는 선택하지 않는다.
        scene = ((2.8, 0.0, .10), (1.8, -1.0, .12))
        initial = [{**item, "scan": raycast_scan(self.mounting, scene)}
                   if item["event"] == "lidar_scan" else item for item in initial]
        events = list(self.track_records(initial, self.pipeline()))
        output = events[-1]
        self.assertTrue(output["track"]["valid"])
        self.assertLess(math.dist(output["track"]["position_xy_m"], TAG_POINT), .25)
        self.assertEqual(output["track"]["candidate_count"], 1)
        self.assertGreaterEqual(len(output["lidar_clusters"]["clusters"]), 2)
        nearest = min(output["lidar_clusters"]["clusters"],
                      key=lambda item: math.hypot(*item["centroid_xy_m"]))
        self.assertGreater(math.dist(nearest["centroid_xy_m"], TAG_POINT), 1.0)
        self.assertGreater(math.dist(nearest["centroid_xy_m"], output["track"]["position_xy_m"]), 1.0)

    def test_cached_scan_ticks_cannot_confirm_or_extend_lifetime(self):
        records, checkpoints = build_target_fixture(self.mounting)
        first = next(item for item in checkpoints if item["label"] == "initial_confirm_1")
        cached = next(item for item in checkpoints if item["label"] == "cached_scan_does_not_confirm")
        events = list(self.track_records(records[:cached["event_index"] + 1], self.pipeline()))
        before, after = events[first["event_index"]]["track"], events[-1]["track"]
        self.assertFalse(after["valid"])
        self.assertEqual(before["confirmation_count"], after["confirmation_count"])

    def test_moving_reflector_keeps_one_id_and_ignores_other_object(self):
        from smartcart_uwb.replay import REPLAY_CLOCK_ORIGIN_NS
        records, position = moving_target_capture(self.mounting)
        ids, scan_positions = set(), []
        for record, event in zip(records, self.track_records(records, self.pipeline())):
            track = event["track"]
            if not track["valid"]:
                self.assertIsNone(track["position_xy_m"])
                continue
            ids.add(track["track_id"])
            if record["event"] == "lidar_scan":
                actual = position(track["reference_monotonic_ns"] - REPLAY_CLOCK_ORIGIN_NS)
                self.assertLess(math.dist(actual, track["position_xy_m"]), .25)
                scan_positions.append(track["position_xy_m"])
                self.assertTrue(all(math.isfinite(value) for value in track["velocity_xy_mps"]))
        self.assertEqual(len(ids), 1)
        self.assertGreaterEqual(len(scan_positions), 25)
        self.assertGreater(scan_positions[-1][0] - scan_positions[0][0], .15)

    def test_confirmed_track_ticks_do_not_extend_original_deadline(self):
        records, checkpoints = build_target_fixture(self.mounting)
        end = next(item["event_index"] for item in checkpoints if item["label"] == "unique_stable")
        records = records[:end + 1]
        at_ns = records[-1]["at_ns"]
        records.extend({"schema": CAPTURE_SCHEMA, "event": "tick", "source": "synthetic",
                        "at_ns": at_ns + delta} for delta in (10_000_000, 20_000_000))
        events = list(self.track_records(records, self.pipeline()))
        confirmed = events[end]["track"]
        self.assertTrue(confirmed["valid"])
        for event in events[end + 1:]:
            self.assertTrue(event["track"]["valid"])
            self.assertEqual(event["track"]["track_id"], confirmed["track_id"])
            self.assertEqual(event["track"]["valid_until_monotonic_ns"], confirmed["valid_until_monotonic_ns"])

    def test_recovery_confirmation_counts_only_fresh_scans(self):
        records, checkpoints = build_target_fixture(self.mounting)
        lookup = {item["label"]: item["event_index"] for item in checkpoints}
        end = lookup["short_recovery"]
        events = list(self.track_records(records[:end + 1], self.pipeline()))
        for hit, label in ((1, "short_recovery_confirm_1"), (2, "short_recovery_confirm_2")):
            index = lookup[label]
            self.assertEqual(events[index]["track"]["confirmation_count"], hit)
            # 다음 왼쪽·오른쪽 UWB 입력은 같은 LiDAR를 다시 평가할 뿐이다.
            for event in events[index + 1:index + 3]:
                self.assertEqual(event["track"]["confirmation_count"], hit)
                self.assertFalse(event["track"]["valid"])
        self.assertTrue(events[-1]["track"]["valid"])
        self.assertEqual(events[-1]["track"]["confirmation_count"], 3)

    def test_inside_uwb_gate_jump_cannot_steal_existing_id(self):
        records, checkpoints = build_target_fixture(self.mounting)
        end = next(item["event_index"] for item in checkpoints if item["label"] == "unique_stable")
        initial = records[:end + 4]
        initial[-1] = {**initial[-1], "scan": raycast_scan(self.mounting, ((2.8, .42, .10),))}
        events = list(self.track_records(initial, self.pipeline()))
        previous, last = events[end], events[-1]
        self.assertTrue(previous["track"]["valid"])
        cluster = last["lidar_clusters"]["clusters"][0]
        estimate = last["uwb_estimate"]
        gate = min(self.tracking_settings.max_association_radius_m,
                   estimate["error_radius_m"] + self.tracking_settings.tag_cluster_offset_m
                   + cluster["error_radius_m"])
        self.assertLessEqual(math.dist(cluster["centroid_xy_m"], estimate["position_xy_m"]), gate)
        self.assertFalse(last["track"]["valid"])
        self.assertIsNone(last["track"]["position_xy_m"])
        self.assertIsNone(last["track"]["track_id"])
        self.assertEqual(last["track"]["invalid_reason"], "continuity_jump")

    def test_replay_wall_clock_changes_do_not_change_outputs(self):
        records, checkpoints = build_target_fixture(self.mounting)
        end = next(item["event_index"] for item in checkpoints if item["label"] == "unique_stable")
        results = []
        for host_clock in (1, 99_000_000_000_000):
            with patch("time.monotonic_ns", return_value=host_clock):
                results.append(list(self.track_records(records[:end + 1], self.pipeline())))
        self.assertEqual(results[0], results[1])

    def test_mixed_sources_and_reversed_time_are_rejected(self):
        records, _ = build_target_fixture(self.mounting)
        for invalid in ([records[0], {**records[1], "source": "recorded"}],
                        [records[1], records[0]]):
            with self.subTest(case=invalid), self.assertRaises(ValueError):
                list(self.track_records(invalid, self.pipeline()))

    def arguments(self):
        return ["--config", str(MOUNTING), "--uwb-estimation-config", str(UWB_SETTINGS),
                "--tracking-config", str(TRACKING_SETTINGS), "--input", str(CAPTURE)]

    def test_cli_default_summary_omits_private_fields(self):
        from smartcart_tracking.cli import main
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            status = main(self.arguments())
        self.assertEqual(status, 0)
        summary = json.loads(output.getvalue())
        self.assertFalse(summary["drive_authorized"])
        self.assertEqual(summary["source_mode"], "replay")
        for name in ("tag_address", "track_id", "position_xy_m", "velocity_xy_mps", "ranges"):
            self.assertNotIn(f'"{name}":', output.getvalue())

    def test_cli_opt_in_output_is_exclusive(self):
        from smartcart_tracking.cli import main
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            path = Path(folder) / "tracking.jsonl"
            args = self.arguments() + ["--output", str(path)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            before = path.read_bytes()
            self.assertTrue(before)
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertNotEqual(main(args), 0)
            self.assertEqual(path.read_bytes(), before)

    def test_cli_malformed_capture_does_not_echo_private_input(self):
        from smartcart_tracking.cli import main
        with tempfile.TemporaryDirectory(dir=ROOT) as folder:
            path = Path(folder) / "invalid.jsonl"
            marker = "PRIVATE_TRACK_POSITION_MUST_NOT_LEAK"
            path.write_text(json.dumps({"position_xy_m": marker}), encoding="utf-8")
            args = self.arguments()
            args[args.index("--input") + 1] = str(path)
            stdout, stderr = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                self.assertNotEqual(main(args), 0)
            self.assertEqual(stdout.getvalue(), "")
            self.assertNotIn(marker, stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
