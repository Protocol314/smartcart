"""좌표계·장착 기하·운영자 증빙을 검수 전 상태와 구분하는 순수 검사."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from smartcart_field.config import CoordinateContract, FieldConfig, MountingEvidence
from smartcart_field.config import field_config_dict, field_config_from_dict, load_field_config
from smartcart_field.identity import check_identity, enumerate_inventory
from smartcart_field.mounting import inspect_mounting, readiness_summary
from smartcart_uwb.config import config_from_dict
from smartcart_uwb.estimation_config import EstimationConfig
from test_field_identity import assigned_configs, default_port_infos

ROOT = Path(__file__).resolve().parents[1]


def verified_evidence():
    return {role: MountingEvidence(True, True, True, True,
                                   "scan_origin" if role == "lidar_center" else "rf_phase_center",
                                   "시험이 지정한 운영자 증빙 선언")
            for role in ("uwb_left", "uwb_right", "lidar_center")}


def verified_settings():
    return FieldConfig(origin_verified=True, axes_verified=True, mounting_evidence=verified_evidence(),
                       range_model_verified=True, tag_height_verified=True)


class FieldMountingTests(unittest.TestCase):
    def setUp(self):
        self.payload = json.loads((ROOT / "config" / "dual_uwb.d5.example.json").read_text(encoding="utf-8"))
        self.mounting = config_from_dict(self.payload)
        self.estimation = EstimationConfig(range_model="slant_known_height", tag_height_m=.9, assumptions_verified=True)

    def measured_mounting(self):
        payload = deepcopy(self.payload)
        for item in (*payload["sensors"].values(), payload["lidar"]):
            item["mounting_verified"] = True
        return config_from_dict(payload)

    def test_cad_example_has_valid_geometry_but_stays_unverified_and_unchanged(self):
        report = inspect_mounting(self.mounting, FieldConfig(), self.estimation)
        self.assertTrue(report["valid"])
        self.assertFalse(report["ready"])
        self.assertTrue(all(item["state"] == "unverified" for item in report["roles"].values()))
        self.assertFalse(report["configuration_modified"])
        self.assertFalse(report["calibration_modified"])
        self.assertFalse(self.mounting.sensors["uwb_left"].mounting_verified)
        self.assertEqual(self.mounting.sensors["uwb_left"].translation_m, (.756, .244, .286))

    def test_all_declarations_only_satisfy_prerequisites_and_never_promote_calibration_or_drive(self):
        report = inspect_mounting(self.measured_mounting(), verified_settings(), self.estimation)
        self.assertTrue(report["ready"])
        self.assertEqual(report["scope"], "configuration_prerequisites_only")
        self.assertFalse(report["physical_measurement_performed"])
        self.assertFalse(report["drive_authorized"])
        self.assertTrue(report["provisional"])
        self.assertTrue(all(item["calibration_profile"] == "raw_unverified" and not item["calibration_verified"]
                            for item in report["roles"].values()))

    def test_field_declarations_cannot_replace_existing_mounting_verified_flags(self):
        report = inspect_mounting(self.mounting, verified_settings(), self.estimation)
        self.assertFalse(report["ready"])
        self.assertIn("mounting_flag_unverified", report["roles"]["uwb_left"]["reasons"])

    def test_translation_yaw_height_and_level_each_require_explicit_evidence(self):
        for name in ("translation", "yaw", "height", "level"):
            with self.subTest(name=name):
                evidence = verified_evidence()
                evidence["uwb_left"] = replace(evidence["uwb_left"], **{name + "_verified": False})
                settings = replace(verified_settings(), mounting_evidence=evidence)
                report = inspect_mounting(self.measured_mounting(), settings, self.estimation)
                self.assertFalse(report["ready"])
                self.assertIn(name + "_unverified", report["roles"]["uwb_left"]["reasons"])

    def test_phase_center_and_scan_origin_are_not_interchangeable(self):
        evidence = verified_evidence()
        evidence["uwb_left"] = replace(evidence["uwb_left"], reference_point="scan_origin")
        settings = replace(verified_settings(), mounting_evidence=evidence)
        report = inspect_mounting(self.measured_mounting(), settings, self.estimation)
        self.assertFalse(report["ready"])
        self.assertIn("sensor_reference_unverified", report["roles"]["uwb_left"]["reasons"])

    def test_blank_measurement_record_does_not_count_as_evidence(self):
        evidence = verified_evidence()
        evidence["lidar_center"] = replace(evidence["lidar_center"], evidence_note="")
        report = inspect_mounting(self.measured_mounting(), replace(verified_settings(), mounting_evidence=evidence), self.estimation)
        self.assertIn("measurement_evidence_missing", report["roles"]["lidar_center"]["reasons"])

    def test_coordinate_origin_axes_and_units_cannot_silently_change(self):
        for name, invalid in (("origin", "floor_projection"), ("x_axis", "right"), ("y_axis", "right"),
                              ("z_axis", "down"), ("translation_unit", "mm"), ("yaw_unit", "rad"),
                              ("frame_id", "laser")):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    CoordinateContract(**{name: invalid})

    def test_swapped_left_right_and_short_baseline_are_geometry_errors(self):
        for ys, code in (((-.244, .244), "uwb_side_assignment_inconsistent"),
                         ((.01, -.01), "uwb_baseline_too_small")):
            with self.subTest(code=code):
                payload = deepcopy(self.payload)
                payload["sensors"]["uwb_left"]["translation_m"][1] = ys[0]
                payload["sensors"]["uwb_right"]["translation_m"][1] = ys[1]
                report = inspect_mounting(config_from_dict(payload), FieldConfig(), self.estimation)
                self.assertFalse(report["valid"])
                self.assertIn(code, report["roles"]["uwb_left"]["reasons"])

    def test_lidar_must_match_expected_frame_and_central_mounting_requirement(self):
        payload = deepcopy(self.payload)
        payload["lidar"]["translation_m"][1] = .2
        report = inspect_mounting(config_from_dict(payload), FieldConfig(), self.estimation)
        self.assertIn("lidar_not_central", report["roles"]["lidar_center"]["reasons"])
        settings = replace(FieldConfig(), expected_lidar_frame_id="different_scan_frame")
        report = inspect_mounting(self.mounting, settings, self.estimation)
        self.assertIn("lidar_frame_mismatch", report["roles"]["lidar_center"]["reasons"])

    def test_yaw_is_not_normalised_automatically_and_heights_are_checked(self):
        invalid_yaw = deepcopy(self.payload)
        invalid_yaw["sensors"]["uwb_left"]["yaw_deg"] = 360
        with self.assertRaises(ValueError):
            config_from_dict(invalid_yaw)
        for name, value, code in (("translation_m", [.756, .244, 0], "sensor_height_nonpositive"),
                                  ("translation_m", [0, .244, .286], "sensor_not_in_front_of_rear_axle")):
            with self.subTest(code=code):
                payload = deepcopy(self.payload)
                payload["sensors"]["uwb_left"][name] = value
                mounting = config_from_dict(payload)
                report = inspect_mounting(mounting, FieldConfig(), self.estimation)
                self.assertFalse(report["valid"])
                self.assertIn(code, report["roles"]["uwb_left"]["reasons"])
                self.assertEqual(getattr(mounting.sensors["uwb_left"], name), tuple(value) if isinstance(value, list) else value)

    def test_unknown_range_and_missing_settings_do_not_become_ready(self):
        for estimation, expected in ((None, "estimation_settings_missing"),
                                     (EstimationConfig(), "range_model_unknown")):
            with self.subTest(expected=expected):
                report = inspect_mounting(self.measured_mounting(), verified_settings(), estimation)
                self.assertFalse(report["ready"])
                self.assertIn(expected, report["range_height"]["reasons"])

    def test_known_slant_height_requires_height_and_assumption_declarations(self):
        settings = replace(verified_settings(), tag_height_verified=False)
        report = inspect_mounting(self.measured_mounting(), settings, replace(self.estimation, assumptions_verified=False))
        self.assertFalse(report["range_height"]["ready"])
        self.assertIn("tag_height_unverified", report["range_height"]["reasons"])
        self.assertIn("estimation_assumptions_unverified", report["range_height"]["reasons"])

    def test_tag_height_is_relative_to_base_link_and_must_be_positive(self):
        estimation = replace(self.estimation, tag_height_m=-.1)
        report = inspect_mounting(self.measured_mounting(), verified_settings(), estimation)
        self.assertIn("tag_height_nonpositive", report["range_height"]["reasons"])
        self.assertFalse(report["ready"])

    def test_horizontal_model_does_not_invent_a_verified_tag_height(self):
        estimation = EstimationConfig(range_model="horizontal", assumptions_verified=True)
        settings = replace(verified_settings(), tag_height_verified=False)
        report = inspect_mounting(self.measured_mounting(), settings, estimation)
        self.assertTrue(report["ready"])
        self.assertEqual(report["range_height"]["tag_height_state"], "not_applicable")

    def test_config_roundtrip_and_example_preserve_all_unverified_defaults(self):
        example = load_field_config(ROOT / "config" / "field_acceptance.example.json")
        self.assertEqual(field_config_dict(example), field_config_dict(FieldConfig()))
        self.assertEqual(field_config_dict(field_config_from_dict(field_config_dict(example))), field_config_dict(example))
        with self.assertRaises(TypeError):
            example.mounting_evidence["lidar_center"] = MountingEvidence()

    def test_field_config_rejects_extra_missing_bool_numeric_and_duplicate_keys(self):
        payload = field_config_dict(FieldConfig())
        for mutation in (lambda value: value.update(extra=True), lambda value: value.pop("origin_verified"),
                         lambda value: value.update(min_uwb_baseline_m=True),
                         lambda value: value.update(max_lidar_lateral_offset_m=float("nan"))):
            with self.subTest(mutation=mutation):
                changed = deepcopy(payload)
                mutation(changed)
                with self.assertRaises(ValueError):
                    field_config_from_dict(changed)
        temporary_root = ROOT / "tests" / "build"
        temporary_root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as directory:
            path = Path(directory) / "field.json"
            for invalid in ('{"schema":1,"schema":2}', 'NaN', '{"broken":', ' ' * 65537):
                path.write_text(invalid, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_field_config(path)

    def test_combined_readiness_only_reports_configuration_prerequisites(self):
        inventory = enumerate_inventory(port_infos=default_port_infos())
        mounting, assigned = assigned_configs(inventory)
        mounting = replace(mounting, sensors={role: replace(sensor, mounting_verified=True)
                                             for role, sensor in mounting.sensors.items()},
                           lidar=replace(mounting.lidar, mounting_verified=True))
        settings = replace(verified_settings(), lidar_device=assigned.lidar_device)
        identity = check_identity(mounting, settings, inventory)
        measured = inspect_mounting(mounting, settings, self.estimation)
        summary = readiness_summary(identity, measured)
        self.assertTrue(summary["ready"])
        self.assertTrue(all(value["ready"] for value in summary["sensors"].values()))
        self.assertFalse(summary["drive_authorized"])
        self.assertFalse(summary["live_data_verified"])
        self.assertTrue(summary["provisional"])
        identity["drive_authorized"] = True
        with self.assertRaises(ValueError):
            readiness_summary(identity, measured)

    def test_invalid_programmatic_numbers_are_rejected_before_inspection(self):
        sensors = dict(self.mounting.sensors)
        with self.assertRaises(ValueError):
            replace(sensors["uwb_left"], translation_m=(float("nan"), .244, .286))
        report = inspect_mounting(self.mounting, FieldConfig(), self.estimation)
        json.dumps(report, allow_nan=False)


if __name__ == "__main__":
    unittest.main()
