import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from uwb_control import Config,UwbFeed,Follower,Observation
from runtime import ReportingBackend,DriveBridge
import runtime
import server
import zone_live


class 가상라이다:
    """모터 명령 시험에서 센서 실패를 명시적으로 주입한다."""
    def __init__(self, config):
        self.error = None

    def steer(self, observation, forward, turn, reason, now):
        self.check(forward, turn, now)
        return forward, turn, reason

    def check(self, forward, turn, now):
        if self.error:
            raise ValueError(self.error)
        return 1.

    def reset_recovery(self):
        pass

    def output_pwm_cap(self):
        return 30.

    def snapshot(self, now):
        return {'enabled': True, 'reason': self.error or 'synthetic_clear'}

def payload(sequence=1,distance=3.,angle=0.,age=0.):
    return {'source_mode':'live','state':'running','profile':'single_left','remaining_ms':500.,
            'sensors':{'uwb_left':{'connected':True,'valid':True,'measurement_valid':True,'id_valid':True,
            'sample_sequence':sequence,'range_m':distance,'raw_angle_deg':angle,'age_ms':age,'remaining_ms':500.-age}}}

class FeedTests(unittest.TestCase):
    def test_requires_three_new_samples(self):
        f=UwbFeed(Config())
        f.accept(payload(1),100,100)
        with self.assertRaisesRegex(ValueError,'confirming'):f.require(100)
        f.accept(payload(1),100.01,100.01)
        self.assertEqual(f.confirmations,1)
        for seq,t in ((2,100.05),(3,100.1)):f.accept(payload(seq),t,t)
        self.assertEqual(f.require(100.1).range_m,3.)

    def test_duplicate_does_not_extend(self):
        f=UwbFeed(Config());f.accept(payload(),100,100)
        first=f.current.deadline
        f.accept(payload(),100.1,100.1)
        self.assertLessEqual(f.current.deadline,first)
        f.accept(payload(),100.21,100.21)
        self.assertIsNone(f.current)

    def test_duplicate_cannot_restore_invalid(self):
        f=UwbFeed(Config());f.accept(payload(),100,100)
        f.invalidate('invalid');f.accept(payload(),100.01,100.01)
        self.assertIsNone(f.current)

    def test_invalid_id_and_measurement(self):
        for field in ('connected','valid','measurement_valid','id_valid'):
            f=UwbFeed(Config());p=payload();p['sensors']['uwb_left'][field]=False
            f.accept(p,100,100);self.assertIsNone(f.current)

    def test_old_sample(self):
        f=UwbFeed(Config());f.accept(payload(age=201),100,100)
        self.assertIsNone(f.current)

    def test_slow_response(self):
        f=UwbFeed(Config());f.accept(payload(age=150),100,100.06)
        self.assertIsNone(f.current)

    def test_sequence_regression(self):
        f=UwbFeed(Config());f.accept(payload(8),100,100)
        f.accept(payload(7),100.05,100.05)
        self.assertIn('regression',f.reason)

    def test_changed_duplicate(self):
        f=UwbFeed(Config());f.accept(payload(),100,100)
        f.accept(payload(distance=3.1),100.01,100.01)
        self.assertIsNone(f.current)

    def test_position_jump(self):
        f=UwbFeed(Config());f.accept(payload(),100,100)
        f.accept(payload(2,distance=8),100.05,100.05)
        self.assertEqual(f.reason,'uwb_position_jump')
        self.assertIsNone(f.current)

    def test_transport_or_invalid_numeric_never_holds_last_target(self):
        for reason in ('uwb_http_TimeoutError','invalid_numeric_input'):
            f=UwbFeed(Config())
            for seq,t in ((1,100.),(2,100.05),(3,100.1)):
                f.accept(payload(seq),t,t)
            f.require(100.1)
            f.invalidate(reason)
            with self.assertRaisesRegex(ValueError,reason):f.require(100.11)

    def test_private_observation_flags_not_required(self):
        f=UwbFeed(Config());p=payload();p['drive_authorized']=False;p['provisional']=True
        f.accept(p,100,100);self.assertIsNotNone(f.current)

class FollowTests(unittest.TestCase):
    def obs(self,distance,heading=0):return Observation(1,distance,math.radians(heading),100,100.2)

    def test_start_stop_hysteresis(self):
        f=Follower(Config())
        self.assertEqual(f.command(self.obs(2.15))[:2],(0,0))
        self.assertGreaterEqual(f.command(self.obs(2.3))[0],.1)
        self.assertGreaterEqual(f.command(self.obs(2.1))[0],.1)
        self.assertEqual(f.command(self.obs(2.0))[:2],(0,0))

    def test_never_inside_motor_deadzone(self):
        f=Follower(Config());f.command(self.obs(3))
        for d in (2.001,2.01,2.15,2.3,3,5):self.assertGreaterEqual(f.command(self.obs(d))[0],.1)

    def test_left_right(self):
        self.assertGreater(Follower(Config()).command(self.obs(3,20))[1],0)
        self.assertLess(Follower(Config()).command(self.obs(3,-20))[1],0)

    def test_pivot_and_rear_stop(self):
        self.assertEqual(Follower(Config()).command(self.obs(3,40))[0],0)
        with self.assertRaises(ValueError):Follower(Config()).command(self.obs(3,90))

    def test_config_rejects_wider_limits(self):
        for kw in ({'pwm_cap':100},{'age_limit_s':.21},{'confirmation_samples':1},{'lidar_enabled':False}):
            with self.assertRaises(ValueError):Config(**kw)

class MotorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # For source review on a PC, point this at the exported motor implementation.
        import os
        path=Path(os.environ.get('SMARTCART_TEST_DRIVE_DIR',str(Path.home()/'smartcart/SmartCart_ManualDrive_180/smartcart')))
        sys.path.insert(0,str(path))
        from smartcart_motor.manual_remote import ManualDrive,MockBackend
        cls.ManualDrive=ManualDrive;cls.MockBackend=MockBackend

    def setUp(self):
        patcher=patch.object(runtime,'LidarGuard',가상라이다)
        patcher.start();self.addCleanup(patcher.stop)
        self.now=100.
        self.backend=self.MockBackend()
        self.reporter=ReportingBackend(self.backend)
        self.drive=self.ManualDrive(self.reporter,clock=lambda:self.now)
        self.reporter.drive=self.drive
        self.bridge=DriveBridge(self.drive,self.reporter,Config(),clock=lambda:self.now)
        self.drive.acquire('one')
        for sequence,t in ((1,99.9),(2,99.95),(3,100.)):
            self.bridge.feed.accept(payload(sequence),t,t)

    def arm(self):
        s=self.drive.snapshot();self.bridge.command('one',dict(type='arm',stamp=s['stamp'],epoch=s['epoch'],seq=1,forward=0,turn=0,pwm=50))

    def auto(self,seq=2):
        s=self.drive.snapshot();self.bridge.command('one',dict(type='auto',stamp=s['stamp'],epoch=s['epoch'],seq=seq,pwm=50))

    def test_arm_required(self):
        with self.assertRaises(ValueError):self.auto()
        self.assertEqual(self.backend.values,(0,0))

    def test_auto_and_pwm_cap(self):
        self.arm();self.auto();self.assertTrue(self.bridge.enabled)
        self.assertLessEqual(self.drive.pwm,30)

    def test_applied_only_after_write(self):
        self.arm();self.auto()
        self.assertEqual(self.bridge.snapshot()['applied_seq'],-1)
        self.drive.tick(self.now)
        self.assertEqual(self.bridge.snapshot()['applied_seq'],2)

    def test_stop_and_no_automatic_restart(self):
        self.arm();self.auto();self.bridge.command('one',{'type':'stop'})
        self.assertFalse(self.bridge.enabled);self.assertFalse(self.drive.armed)
        with self.assertRaises(ValueError):self.auto(3)

    def test_sensor_failure_stops_without_browser_message(self):
        self.arm();self.auto();self.drive.tick(self.now)
        self.bridge.feed.invalidate('uwb_invalid');self.bridge.monitor_sensor()
        self.assertFalse(self.drive.armed);self.assertEqual(self.backend.values,(0,0))

    def test_sensor_expiry(self):
        self.arm();self.auto();self.now+=.21
        self.bridge.monitor_sensor();self.assertFalse(self.drive.armed)

    def test_lidar_failure_requires_new_arm_after_recovery(self):
        self.arm();self.auto();self.drive.tick(self.now)
        self.bridge.lidar.error='lidar_stale'
        self.bridge.monitor_sensor()
        self.assertFalse(self.drive.armed)
        self.assertFalse(self.bridge.enabled)
        self.assertEqual(self.backend.values,(0,0))
        self.bridge.lidar.error=None
        self.bridge.monitor_sensor()
        with self.assertRaisesRegex(ValueError,'rearm_required'):self.auto(3)
        self.assertFalse(self.drive.armed)
        self.assertEqual(self.backend.values,(0,0))

    def test_hardware_backend_is_rejected(self):
        class 실제출력:
            hardware=True
        with self.assertRaisesRegex(ValueError,'GPIO'):
            DriveBridge(self.drive,ReportingBackend(실제출력()),Config())

    def test_status_does_not_claim_hardware_interlock(self):
        self.arm();self.auto()
        self.assertIs(self.bridge.snapshot()['sensor_interlock'],False)

    def test_wrong_owner_cannot_stop_owner(self):
        self.arm();self.auto()
        with self.assertRaises(ValueError):self.bridge.command('other',{'type':'auto'})
        self.assertTrue(self.drive.armed)

    def test_manual_input_exits_autonomy(self):
        self.arm();self.auto();s=self.drive.snapshot()
        self.bridge.command('one',dict(type='drive',stamp=s['stamp'],epoch=s['epoch'],seq=3,forward=0,turn=0,pwm=30))
        self.assertFalse(self.bridge.enabled)

    def test_original_watchdog_still_stops(self):
        self.arm();self.auto();self.now+=.36;self.drive.tick(self.now)
        self.assertFalse(self.drive.armed)

    def test_failed_write_not_acknowledged(self):
        self.arm();self.auto()
        def fail(*a):raise RuntimeError('test backend failure')
        self.backend.write=fail;self.drive.tick(self.now)
        self.assertEqual(self.reporter.applied_seq,-1)
        self.assertFalse(self.drive.armed)

class 형상설정시험(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        (self.root/'live_zone_modules').mkdir()
        patcher=patch.object(zone_live,'Path',return_value=self.root/'zone_live.py')
        patcher.start();self.addCleanup(patcher.stop)

    def test_missing_geometry_fails_closed(self):
        with self.assertRaisesRegex(ValueError,'cart_geometry.json'):
            zone_live.ZoneGuard(Config())

    def test_synthetic_fixture_cannot_be_used_as_field_configuration(self):
        raw=(HERE/'tests/fixtures/cart_geometry.synthetic.json').read_text(encoding='utf-8')
        path=self.root/'live_zone_modules/cart_geometry.json'
        path.write_text(raw,encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'reactive_geometry_invalid'):
            zone_live.ZoneGuard(Config())

    def test_geometry_without_uwb_mount_is_rejected(self):
        data=json.loads((HERE/'tests/fixtures/cart_geometry.synthetic.json').read_text(encoding='utf-8'))
        data.pop('synthetic_test_only');data.pop('uwb_x_m')
        (self.root/'live_zone_modules/cart_geometry.json').write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'reactive_geometry_invalid'):
            zone_live.ZoneGuard(Config())

    def test_synthetic_geometry_only_in_temporary_test_directory(self):
        data=json.loads((HERE/'tests/fixtures/cart_geometry.synthetic.json').read_text(encoding='utf-8'))
        self.assertIs(data.pop('synthetic_test_only'),True)
        (self.root/'live_zone_modules/cart_geometry.json').write_text(json.dumps(data),encoding='utf-8')
        guard=zone_live.ZoneGuard(Config())
        with self.assertRaisesRegex(ValueError,'waiting_for_lidar'):
            guard.check(1,0,100)
        self.assertTrue(guard.blocked)

    def test_actual_lidar_guard_blocks_obstacle_and_stale_scan(self):
        data=json.loads((HERE/'tests/fixtures/cart_geometry.synthetic.json').read_text(encoding='utf-8'))
        data.pop('synthetic_test_only')
        (self.root/'live_zone_modules/cart_geometry.json').write_text(json.dumps(data),encoding='utf-8')
        guard=zone_live.ZoneGuard(Config())
        step=2*math.pi/360
        scan={'sequence':1,'source':{'kind':'ros'},'truncated':False,
              'ranges':[5.]*360,'angle_min':-math.pi,'angle_max':-math.pi+359*step,
              'angle_increment':step,'range_min':.05,'range_max':8.,
              'scan_time':.1,'time_increment':0.,'header':{'frame_id':'test','stamp':{'sec':1,'nanosec':0}}}
        frame={'source_mode':'ros','state':'live','error':None,'age_ms':0.,'stale_after_ms':500.,'scan':scan}
        guard.accept(frame,100.,100.)
        self.assertIsNotNone(guard.current)
        self.assertGreater(guard.check(1.,0.,100.),0.)
        with self.assertRaisesRegex(ValueError,'lidar_stale'):
            guard.check(1.,0.,100.21)
        scan['sequence']=2;scan['header']['stamp']['sec']=2;scan['ranges'][180]=.6
        guard.accept(frame,101.,101.)
        with self.assertRaisesRegex(ValueError,'reactive_front_obstacle_1m'):
            guard.check(1.,0.,101.)
        scan['sequence']=3;scan['header']['stamp']['sec']=3;scan['ranges'][180]=None
        guard.accept(frame,102.,102.)
        with self.assertRaisesRegex(ValueError,'reactive_front_unknown_scan'):
            guard.check(1.,0.,102.)

    def test_lidar_obstacle_stops_mock_motor_and_requires_rearm(self):
        import os
        path=Path(os.environ['SMARTCART_TEST_DRIVE_DIR'])
        sys.path.insert(0,str(path))
        from smartcart_motor.manual_remote import ManualDrive,MockBackend
        data=json.loads((HERE/'tests/fixtures/cart_geometry.synthetic.json').read_text(encoding='utf-8'))
        data.pop('synthetic_test_only')
        (self.root/'live_zone_modules/cart_geometry.json').write_text(json.dumps(data),encoding='utf-8')
        guard=zone_live.ZoneGuard(Config())
        clock=[100.]
        backend=MockBackend();reporter=ReportingBackend(backend)
        drive=ManualDrive(reporter,clock=lambda:clock[0]);reporter.drive=drive
        with patch.object(runtime,'LidarGuard',return_value=guard):
            bridge=DriveBridge(drive,reporter,Config(),clock=lambda:clock[0])
        drive.acquire('one')
        for seq,t in ((1,99.9),(2,99.95),(3,100.)):
            bridge.feed.accept(payload(seq),t,t)
        step=2*math.pi/360
        scan={'sequence':1,'source':{'kind':'ros'},'truncated':False,
              'ranges':[5.]*360,'angle_min':-math.pi,'angle_max':-math.pi+359*step,
              'angle_increment':step,'range_min':.05,'range_max':8.,'scan_time':.1,
              'time_increment':0.,'header':{'frame_id':'test','stamp':{'sec':1,'nanosec':0}}}
        frame={'source_mode':'ros','state':'live','error':None,'age_ms':0.,'stale_after_ms':500.,'scan':scan}
        guard.accept(frame,100.,100.)
        state=drive.snapshot()
        bridge.command('one',dict(type='arm',stamp=state['stamp'],epoch=state['epoch'],seq=1,forward=0,turn=0,pwm=30))
        state=drive.snapshot()
        bridge.command('one',dict(type='auto',stamp=state['stamp'],epoch=state['epoch'],seq=2,pwm=30))
        drive.tick(clock[0]);self.assertTrue(drive.armed)
        clock[0]=100.01;scan['sequence']=2;scan['header']['stamp']['sec']=2;scan['ranges'][180]=.6
        guard.accept(frame,clock[0],clock[0]);bridge.monitor_sensor()
        self.assertFalse(drive.armed);self.assertEqual(backend.values,(0,0))
        clock[0]=100.02;scan['sequence']=3;scan['header']['stamp']['sec']=3;scan['ranges'][180]=5.
        guard.accept(frame,clock[0],clock[0]);bridge.monitor_sensor()
        state=drive.snapshot()
        with self.assertRaisesRegex(ValueError,'rearm_required'):
            bridge.command('one',dict(type='auto',stamp=state['stamp'],epoch=state['epoch'],seq=3,pwm=30))
        self.assertEqual(backend.values,(0,0))

    def test_follow_fixture_requires_removing_test_marker(self):
        data=json.loads((HERE/'tests/fixtures/follow_config.synthetic.json').read_text(encoding='utf-8'))
        with self.assertRaises(TypeError):Config(**data)
        self.assertIs(data.pop('synthetic_test_only'),True)
        self.assertEqual(Config(**data).pwm_cap,30)

    def test_gpio_server_rejected_before_socket_or_config_access(self):
        with patch.object(sys,'argv',['server.py','--backend','gpio']), \
             patch.object(server.socket,'socket') as socket:
            with self.assertRaisesRegex(SystemExit,'GPIO'):
                server.main()
            socket.assert_not_called()

    def test_missing_follow_config_exits_before_backend_creation(self):
        with patch.object(sys,'argv',['server.py','--backend','mock','--address','192.168.43.2']), \
             patch.object(server,'HERE',self.root), patch.object(server.socket,'socket'), \
             patch.object(server,'load_base') as load_base:
            with self.assertRaisesRegex(SystemExit,'follow_config.json'):
                server.main()
            load_base.assert_not_called()


if __name__=='__main__':unittest.main()
