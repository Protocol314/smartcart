"""센서 무효와 만료 시 가상 모터를 잠그는 회귀 시험."""
import copy
import math
import unittest
from smartcart_motor.sensor_guard import SensorGuard
from smartcart_motor.manual_remote import ManualDrive, MockBackend

class SensorSafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock = lambda: self.now
        self.guard = SensorGuard(clock=self.clock)
        step = 2*math.pi/360
        self.frame = {'schema':'smartcart.envelope.v1','source_mode':'ros','state':'live','age_ms':0,
          'scan':{'sequence':1,'angle_min':-math.pi,'angle_max':math.pi-step,'angle_increment':step,
                  'range_min':.05,'range_max':10.,'ranges':[2.]*360}}
        self.uwb = {'schema':'smartcart.dual_uwb_view.v1','source_mode':'live','state':'running',
          'profile':'single_left','drive_authorized':False,'sensors':{'uwb_left':{'state':'live',
          'connected':True,'valid':True,'measurement_valid':True,'id_valid':True,
          'remaining_ms':200,'age_ms':0,'sample_sequence':1}}}
        self.backend = MockBackend()
        self.drive = ManualDrive(self.backend,clock=self.clock,sensor_guard=self.guard)
        self.backend.open(); self.drive.acquire('owner')

    def command(self, kind, seq, f=0):
        self.drive.command('owner', {'type':kind,'seq':seq,'epoch':self.drive.epoch,
                           'stamp':self.now,'forward':f,'turn':0,'pwm':50})

    def start(self):
        self.guard.update(self.frame,self.uwb)
        self.command('arm',1)
        self.assertTrue(self.drive.armed)
        self.command('drive',2,1)
        self.drive.tick()
        self.now += .02
        self.drive.tick()
        self.assertGreater(self.drive.output[0],0)

    def test_faults_latch_stop_and_require_rearm(self):
        for fault in ('lidar','uwb','id','obstacle','unobserved','stale','mode','malformed'):
            with self.subTest(fault=fault):
                self.setUp();self.start()
                frame,uwb=copy.deepcopy(self.frame),copy.deepcopy(self.uwb)
                if fault=='lidar':frame['state']='stale'
                elif fault=='uwb':uwb['sensors']['uwb_left']['valid']=False
                elif fault=='id':uwb['sensors']['uwb_left']['id_valid']=False
                elif fault=='obstacle':frame['scan']['ranges'][0]=.5
                elif fault=='unobserved':frame['scan']['ranges'][2]=None
                elif fault=='stale':frame['age_ms']=301
                elif fault=='mode':frame['source_mode']='replay'
                else:uwb['sensors']=['잘못된 센서 형식']
                self.guard.update(frame,uwb);self.now+=.02;self.drive.tick()
                self.assertEqual(self.backend.values,(0,0));self.assertFalse(self.drive.armed)
                self.frame['scan']['sequence']=2;self.uwb['sensors']['uwb_left']['sample_sequence']=2
                self.guard.update(self.frame,self.uwb);self.now+=.02;self.drive.tick()
                self.assertFalse(self.drive.armed)

    def test_duplicate_cannot_refresh_and_stale_stops(self):
        self.start();self.now+=.15;self.guard.update(self.frame,self.uwb)
        self.now+=.04;self.drive.tick();self.assertEqual(self.backend.values,(0,0))
        self.assertFalse(self.drive.armed)

    def test_initial_arm_refused(self):
        self.command('arm',1);self.assertFalse(self.drive.armed)
        self.assertEqual(self.backend.values,(0,0))

    def test_poll_latency_counts(self):
        self.guard.update(self.frame,self.uwb,transport_ms=250)
        self.assertFalse(self.guard.status()[0])

    def test_hardware_output_cannot_be_constructed(self):
        class Hardware:
            hardware = True
        with self.assertRaises(ValueError):
            ManualDrive(Hardware(), sensor_guard=self.guard)

if __name__ == '__main__': unittest.main()
