import copy
import math
from pathlib import Path
import sys
import unittest

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from uwb_control import Config,UwbFeed,Follower,Observation
from runtime import ReportingBackend,DriveBridge

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
        for kw in ({'pwm_cap':100},{'age_limit_s':1},{'confirmation_samples':1}):
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

if __name__=='__main__':unittest.main()
