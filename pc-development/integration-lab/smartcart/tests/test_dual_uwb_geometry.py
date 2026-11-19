"""높이 투영 특이점·장착 변환·오차 구간의 실제 점 포함을 장치 없이 검사한다."""
from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import unittest

from smartcart_uwb.config import load_config
from smartcart_uwb.estimation_config import EstimationConfig
from smartcart_uwb.geometry import ALIGNMENT_METHOD, GeometryError, fuse_pair, project_observation


EXAMPLE = Path(__file__).resolve().parents[1] / "config" / "dual_uwb.d5.example.json"


def settings(**values):
    defaults = {"range_model": "horizontal", "range_error_m": .02,
                "bearing_error_deg": 1.5, "mount_translation_error_m": .005,
                "mount_yaw_error_deg": .5, "min_horizontal_range_m": .05,
                "max_pair_skew_ms": 80, "max_observation_age_ms": 200,
                "max_relative_speed_mps": 2, "max_host_rx_delay_ms": 100,
                "position_error_floor_m": .03, "max_position_disagreement_m": .5}
    return EstimationConfig(**(defaults | values))


def observation(sensor, distance=2, bearing=0, rx=1_000_000_000):
    return {"sensor_id": sensor.sensor_id, "frame_id": sensor.frame_id,
            "valid": True, "measurement_valid": True, "id_valid": True,
            "range_m": distance, "bearing_rad": bearing, "rx_monotonic_ns": rx}


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)
        self.sensor = self.config.sensors["uwb_left"]

    def assert_reason(self, reason, call):
        with self.assertRaises(GeometryError) as caught:
            call()
        self.assertEqual(caught.exception.reason, reason)

    def test_horizontal_axis_left_sign_yaw_and_translation(self):
        for yaw in (-90, 0, 90, 180):
            sensor = replace(self.sensor, yaw_deg=yaw)
            for bearing in (-math.pi / 2, 0, math.pi / 2):
                with self.subTest(yaw=yaw, bearing=bearing):
                    result = project_observation(observation(sensor, bearing=bearing), sensor, settings())
                    angle = math.radians(yaw) + bearing
                    self.assertAlmostEqual(result["position_xy_m"][0], sensor.translation_m[0] + 2 * math.cos(angle))
                    self.assertAlmostEqual(result["position_xy_m"][1], sensor.translation_m[1] + 2 * math.sin(angle))

    def test_slant_height_projection_and_height_below_sensor(self):
        for dz in (-1, 1):
            with self.subTest(dz=dz):
                config = settings(range_model="slant_known_height", tag_height_m=self.sensor.translation_m[2] + dz,
                                  tag_height_tolerance_m=.01)
                result = project_observation(observation(self.sensor, math.sqrt(5)), self.sensor, config)
                self.assertAlmostEqual(result["position_xy_m"][0], self.sensor.translation_m[0] + 2)
                self.assertAlmostEqual(result["position_xy_m"][1], self.sensor.translation_m[1])

    def test_unknown_model_does_not_silently_assume_horizontal(self):
        self.assert_reason("unknown_range_model", lambda: project_observation(
            observation(self.sensor), self.sensor, EstimationConfig()))

    def test_impossible_near_vertical_and_interval_touch_are_rejected(self):
        for distance in (.5, 1, 1.01):
            with self.subTest(distance=distance):
                config = settings(range_model="slant_known_height", tag_height_m=self.sensor.translation_m[2] + 1)
                self.assert_reason("singular_projection", lambda: project_observation(
                    observation(self.sensor, distance), self.sensor, config))
        config = settings(range_error_m=.05, min_horizontal_range_m=.05)
        self.assert_reason("singular_projection", lambda: project_observation(
            observation(self.sensor, .1), self.sensor, config))

    def test_maximum_error_settings_reject_uncertain_projection(self):
        config = settings(range_model="slant_known_height", tag_height_m=.9,
                          range_error_m=2, tag_height_tolerance_m=1,
                          mount_translation_error_m=1, bearing_error_deg=45, mount_yaw_error_deg=45)
        self.assert_reason("singular_projection", lambda: project_observation(
            observation(self.sensor, 2), self.sensor, config))

    def test_height_interval_crosses_sensor_height(self):
        config = settings(range_model="slant_known_height", tag_height_m=self.sensor.translation_m[2],
                          tag_height_tolerance_m=.2)
        result = project_observation(observation(self.sensor, 2), self.sensor, config)
        self.assertAlmostEqual(result["position_xy_m"][0], self.sensor.translation_m[0] + 2)
        self.assertGreater(result["error_radius_m"], config.range_error_m)

    def test_declared_radius_contains_sampled_slant_interval_points(self):
        # 구간 끝점·내부와 양쪽 높이·각도·XY 설치 오차 조합의 실제 투영점을 검사한다.
        sensor = replace(self.sensor, yaw_deg=37)
        config = settings(range_model="slant_known_height", tag_height_m=.9,
                          tag_height_tolerance_m=.08, range_error_m=.1,
                          bearing_error_deg=8, mount_yaw_error_deg=3,
                          mount_translation_error_m=.03)
        obs = observation(sensor, 2.4, math.radians(-28))
        result = project_observation(obs, sensor, config)
        px, py = result["position_xy_m"]
        dz = config.tag_height_m - sensor.translation_m[2]
        height_error = config.tag_height_tolerance_m + config.mount_translation_error_m
        for dr in (-config.range_error_m, 0, config.range_error_m):
            for dh in (-height_error, 0, height_error):
                horizontal = math.sqrt((obs["range_m"] + dr) ** 2 - (dz + dh) ** 2)
                for da in (-11, -5.5, 0, 5.5, 11):
                    angle = obs["bearing_rad"] + math.radians(sensor.yaw_deg + da)
                    for translation_angle in (0, math.pi / 2, math.pi, 3 * math.pi / 2):
                        actual_x = sensor.translation_m[0] + horizontal * math.cos(angle) + config.mount_translation_error_m * math.cos(translation_angle)
                        actual_y = sensor.translation_m[1] + horizontal * math.sin(angle) + config.mount_translation_error_m * math.sin(translation_angle)
                        self.assertLessEqual(math.hypot(actual_x - px, actual_y - py), result["error_radius_m"] + 1e-12)

    def test_bad_observation_flags_numbers_and_sensor_mapping(self):
        for key, value in (("range_m", float("nan")), ("bearing_rad", float("inf")),
                           ("range_m", True), ("range_m", 10 ** 1000), ("range_m", 20.01),
                           ("bearing_rad", math.pi + .01), ("valid", False),
                           ("id_valid", False), ("measurement_valid", False)):
            with self.subTest(key=key, value=str(value)[:20]):
                obs = observation(self.sensor) | {key: value}
                self.assert_reason("invalid_observation", lambda: project_observation(obs, self.sensor, settings()))
        self.assert_reason("sensor_mismatch", lambda: project_observation(
            observation(self.sensor) | {"sensor_id": "uwb_right"}, self.sensor, settings()))
        self.assert_reason("sensor_mismatch", lambda: project_observation(
            observation(self.sensor) | {"frame_id": "wrong"}, self.sensor, settings()))

    def test_projection_does_not_change_input_or_settings(self):
        obs, config = observation(self.sensor), settings()
        original = dict(obs)
        project_observation(obs, self.sensor, config)
        self.assertEqual(obs, original)
        self.assertFalse(config.assumptions_verified)


class PairTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(EXAMPLE)

    def pair(self, left_position=(2, 0), right_position=(2, 0), skew_ns=50_000_000):
        observations = []
        for index, (sensor_id, target) in enumerate(zip(("uwb_left", "uwb_right"), (left_position, right_position))):
            sensor = self.config.sensors[sensor_id]
            dx, dy = target[0] - sensor.translation_m[0], target[1] - sensor.translation_m[1]
            observations.append(observation(sensor, math.hypot(dx, dy), math.atan2(dy, dx),
                                            1_000_000_000 + index * skew_ns))
        return observations

    def test_midpoint_latest_reference_and_motion_padding(self):
        left, right = self.pair(left_position=(2, 0), right_position=(2.1, .1))
        config = settings()
        result = fuse_pair(left, right, self.config, config)
        self.assertAlmostEqual(result["position_xy_m"][0], 2.05)
        self.assertAlmostEqual(result["position_xy_m"][1], .05)
        self.assertAlmostEqual(result["pair_residual_m"], math.sqrt(.02))
        individual = [project_observation(obs, self.config.sensors[obs["sensor_id"]], config)
                      for obs in (left, right)]
        expected = max(value["error_radius_m"] for value in individual) + math.sqrt(.02) / 2 + 2 * (.05 + .1)
        self.assertAlmostEqual(result["error_radius_m"], expected)
        self.assertEqual(result["reference_monotonic_ns"], right["rx_monotonic_ns"])
        self.assertEqual(result["rx_skew_ns"], 50_000_000)
        self.assertEqual(result["alignment_method"], ALIGNMENT_METHOD)
        self.assertIsNone(result["tag_height_m"])

    def test_pair_skew_and_residual_gate(self):
        for pair, reason in ((self.pair(skew_ns=80_000_001), "pair_skew"),
                             (self.pair(right_position=(3, 0)), "inconsistent_positions")):
            with self.subTest(reason=reason), self.assertRaises(GeometryError) as caught:
                fuse_pair(*pair, self.config, settings())
            self.assertEqual(caught.exception.reason, reason)
        # 이동 가능량만큼은 잔차 문턱에 더하되 좌표 자체를 예측 이동시키지 않는다.
        left, right = self.pair(right_position=(2.55, 0))
        result = fuse_pair(left, right, self.config, settings())
        self.assertAlmostEqual(result["position_xy_m"][0], 2.275)

    def test_earlier_right_reference_zero_timestamp_and_invalid_timestamp(self):
        left, right = self.pair(skew_ns=0)
        left["rx_monotonic_ns"], right["rx_monotonic_ns"] = 10_000_000, 0
        self.assertEqual(fuse_pair(left, right, self.config, settings())["reference_monotonic_ns"], 10_000_000)
        for invalid in (True, -1, 1.5, 1 << 63):
            with self.subTest(invalid=invalid), self.assertRaises(GeometryError) as caught:
                fuse_pair(left | {"rx_monotonic_ns": invalid}, right, self.config, settings())
            self.assertEqual(caught.exception.reason, "invalid_timestamp")

    def test_floor_and_no_independence_based_error_reduction(self):
        left, right = self.pair(skew_ns=0)
        config = settings(position_error_floor_m=.8, max_host_rx_delay_ms=0)
        result = fuse_pair(left, right, self.config, config)
        self.assertEqual(result["error_radius_m"], .8)
        single = project_observation(left, self.config.sensors["uwb_left"], config)
        self.assertGreaterEqual(result["error_radius_m"], single["error_radius_m"])

    def test_slant_pair_recovers_declared_height(self):
        target, height = (2, .3), .9
        pair = []
        for sensor_id in ("uwb_left", "uwb_right"):
            sensor = self.config.sensors[sensor_id]
            dx, dy, dz = target[0] - sensor.translation_m[0], target[1] - sensor.translation_m[1], height - sensor.translation_m[2]
            pair.append(observation(sensor, math.sqrt(dx * dx + dy * dy + dz * dz), math.atan2(dy, dx)))
        result = fuse_pair(*pair, self.config, settings(range_model="slant_known_height", tag_height_m=height))
        self.assertAlmostEqual(result["position_xy_m"][0], target[0])
        self.assertAlmostEqual(result["position_xy_m"][1], target[1])
        self.assertEqual(result["tag_height_m"], height)


if __name__ == "__main__":
    unittest.main()
