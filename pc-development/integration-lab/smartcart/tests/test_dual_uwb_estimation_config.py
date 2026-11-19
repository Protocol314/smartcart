"""거리 의미·높이·시간·오차 가정의 엄격한 독립 설정 검사."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from smartcart_uwb.estimation_config import (EstimationConfig, config_dict,
                                            config_from_dict, load_estimation_config)


class EstimationConfigTests(unittest.TestCase):
    def test_default_preserves_unknown_range_and_provisional_assumptions(self):
        settings = EstimationConfig()
        self.assertEqual(settings.range_model, "unknown")
        self.assertIsNone(settings.tag_height_m)
        self.assertFalse(settings.assumptions_verified)
        self.assertEqual(config_from_dict(config_dict(settings)), settings)

    def test_known_height_is_explicit_and_relative_to_base_link(self):
        settings = EstimationConfig(range_model="slant_known_height", tag_height_m=.9)
        self.assertEqual(settings.tag_height_m, .9)
        for model in ("unknown", "horizontal"):
            with self.subTest(model=model), self.assertRaises(ValueError):
                EstimationConfig(range_model=model, tag_height_m=.9)
        with self.assertRaises(ValueError):
            EstimationConfig(range_model="slant_known_height")

    def test_no_unknown_or_missing_keys_or_versions(self):
        payload = config_dict(EstimationConfig())
        for change in ({**payload, "drive_authorized": True}, {**payload, "schema": "v2"},
                       {key: val for key, val in payload.items() if key != "range_error_m"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                config_from_dict(change)

    def test_all_numeric_fields_reject_nonfinite_bool_and_wrong_types(self):
        settings = EstimationConfig()
        names = set(config_dict(settings)) - {"schema", "range_model", "tag_height_m", "assumptions_verified"}
        for name in names:
            for value in (True, "1", None, float("nan"), float("inf"), -100, 100000):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    replace(settings, **{name: value})
        for value in (True, float("inf"), 10):
            with self.subTest(height=value), self.assertRaises(ValueError):
                EstimationConfig(range_model="slant_known_height", tag_height_m=value)

    def test_quantization_floors_and_time_relation(self):
        for change in ({"range_error_m": .009}, {"bearing_error_deg": .99},
                       {"max_pair_skew_ms": 200, "max_observation_age_ms": 100},
                       {"assumptions_verified": 1}, {"range_model": ["horizontal"]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                EstimationConfig(**change)

    def test_file_limits_duplicates_invalid_utf8_and_recursion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            raw = json.dumps(config_dict(EstimationConfig())).encode("utf-8")
            path.write_bytes(raw)
            self.assertEqual(load_estimation_config(path), EstimationConfig())
            for malformed in (b'{}', b'{"schema":1,"schema":2}', b'\xff',
                              b'{"value":NaN}', b' ' * 65537, b'[' * 2000 + b']' * 2000):
                path.write_bytes(malformed)
                with self.subTest(length=len(malformed)), self.assertRaises(ValueError):
                    load_estimation_config(path)


if __name__ == "__main__":
    unittest.main()
