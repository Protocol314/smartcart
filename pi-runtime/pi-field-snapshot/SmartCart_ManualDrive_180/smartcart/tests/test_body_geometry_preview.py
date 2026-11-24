"""직사각형 거리 계산과 미검증 상태를 가상 입력으로 확인한다."""
import math
import unittest

from smartcart_motor.body_geometry_preview import preview_body_envelope


class BodyGeometryTests(unittest.TestCase):
    def test_centered_rectangle(self):
        result = preview_body_envelope(.85, .55, .425, 0)
        self.assertAlmostEqual(result['body_corner_radius_m'], math.hypot(.425, .275))
        self.assertFalse(result['geometry_verified'])
        self.assertFalse(result['drive_authorized'])
        self.assertFalse(result['braking_distance_included'])
        self.assertFalse(result['swept_rotation_included'])

    def test_offcenter_and_edges(self):
        result = preview_body_envelope(.85, .55, .1, .1)
        self.assertAlmostEqual(result['body_corner_radius_m'], math.hypot(.75, .375))
        self.assertAlmostEqual(preview_body_envelope(.85, .55, 0, 0)['body_corner_radius_m'],
                               math.hypot(.85, .275))

    def test_reported_cart_dimensions_are_geometry_only(self):
        result = preview_body_envelope(.85, .55, .70, 0)
        self.assertAlmostEqual(result['body_corner_radius_m'], 0.7520804478245662)
        self.assertFalse(result['geometry_verified'])
        self.assertFalse(result['drive_authorized'])

    def test_reject_unmeasured_outside_and_nonfinite(self):
        for values in ((.85, .55, None, 0), (.85, .55, .86, 0),
                       (.85, .55, .1, .3), (.85, float('nan'), .1, 0),
                       (.85, .55, True, 0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                preview_body_envelope(*values)


if __name__ == '__main__':
    unittest.main()
