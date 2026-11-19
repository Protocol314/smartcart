"""ROS 콜백 형식 입력이 원래 관측 그대로 감독기에 전달되는지 가상으로 검사한다. rclpy 없음."""
import json
import math
import unittest

from smartcart_motor.control import MotorController
from smartcart_motor.drive_inputs import DriveInputs
from smartcart_motor.safety_provider import EstopReading
from smartcart_motor.supervisor import DriveSupervisor
from smartcart_tracking.config import TrackingConfig
from smartcart_tracking.runtime import LiveTrackingRuntime
from smartcart_uwb.estimation_config import EstimationConfig
from test_drive_supervisor import CLEAR, FOLLOW, VERIFIED
from test_motor_control import MS, SECOND, FakeBackend, known_config
from test_tracking_runtime import mounting_config, status


def full_scan():
    step = 2 * math.pi / 360
    return {'angle_min': -math.pi, 'angle_max': math.pi - step, 'angle_increment': step,
            'range_min': .05, 'range_max': 10., 'ranges': [2.] * 360, 'scan_time': .1,
            'time_increment': .0001}


class DriveInputsTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.mounting = mounting_config()
        self.supervisor = DriveSupervisor(MotorController(known_config(), FakeBackend()), acceptance=VERIFIED,
                                          clearance=CLEAR, follow=FOLLOW, clock=lambda: self.now)
        self.runtime = LiveTrackingRuntime(self.mounting, EstimationConfig(range_model='horizontal'),
                                           TrackingConfig(), single_left=True)

    def inputs(self, estop=None):
        if estop is None:
            return DriveInputs(self.supervisor, self.runtime)
        return DriveInputs(self.supervisor, self.runtime, estop=estop)

    def status_text(self, sequence):
        value = status(self.mounting, sequence, self.now)
        value['sensors']['uwb_right'] = {'state': 'error', 'connected': False, 'stream_epoch': 0,
                                         'latest': None, 'last_error': 'disabled_single_left'}
        return json.dumps(value)

    def test_default_has_no_estop_feedback_and_stays_locked(self):
        inputs = self.inputs()
        self.supervisor.set_mode('manual', self.now)
        inputs.on_status(self.status_text(1), self.now)
        reasons = inputs.on_scan(full_scan(), 'lidar_scan', self.now)['blocking_reasons']
        self.assertEqual(reasons, ['estop_feedback_unavailable'])
        self.assertEqual(self.supervisor.request_arm(self.now)['reason'], 'estop_feedback_unavailable')

    def test_original_observations_reach_supervisor_and_errors_block(self):
        inputs = self.inputs(lambda now: EstopReading(True, True, now + 200 * MS))
        self.supervisor.set_mode('manual', self.now)
        inputs.on_status(self.status_text(1), self.now)
        self.assertEqual(inputs.on_scan(full_scan(), 'lidar_scan', self.now)['blocking_reasons'], [])
        self.assertTrue(self.supervisor.request_arm(self.now)['accepted'])
        self.now += 40 * MS
        self.assertIn('lidar_invalid', inputs.on_scan(full_scan(), 'other_frame', self.now)['blocking_reasons'])
        self.assertIn('uwb_status_missing', inputs.on_status('{"schema":', self.now)['blocking_reasons'])
        self.assertEqual(self.supervisor.status(self.now)['latched_reason'], 'lidar_invalid')
        # 수신이 멈추면 타이머 평가에서 원래 만료가 지나 무효가 된다.
        inputs.on_status(self.status_text(2), self.now)
        inputs.on_scan(full_scan(), 'lidar_scan', self.now)
        self.now += 2 * SECOND
        reasons = inputs.tick(self.now)['blocking_reasons']
        self.assertTrue(any(code in reasons for code in ('lidar_stale', 'lidar_invalid')), reasons)
        self.assertIn('uwb_left_stale', reasons)

    def test_follow_with_current_tracker_output_is_never_authorized(self):
        inputs = self.inputs(lambda now: EstopReading(True, True, now + 200 * MS))
        self.supervisor.set_mode('follow', self.now)
        for sequence in range(1, 6):
            inputs.on_status(self.status_text(sequence), self.now)
            reasons = inputs.on_scan(full_scan(), 'lidar_scan', self.now)['blocking_reasons']
            self.now += 40 * MS
        self.assertTrue(any(code.startswith('tracking_') for code in reasons), reasons)
        self.assertFalse(self.supervisor.request_arm(self.now)['accepted'])


if __name__ == '__main__':
    unittest.main()
