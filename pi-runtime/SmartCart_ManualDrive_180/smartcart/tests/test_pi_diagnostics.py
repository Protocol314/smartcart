"""진단 출력이 무효 센서를 정상으로 판단하지 않는지 검증한다."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[3] / 'diagnose_pi.py'
spec = importlib.util.spec_from_file_location('pi_diagnosis', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class DiagnosisTests(unittest.TestCase):
    def test_lidar_validity_and_incomplete(self):
        step=2*math.pi/360
        frame={'schema':'smartcart.envelope.v1','source_mode':'ros','state':'live','age_ms':20,
               'scan':{'ranges':[2.]*360,'range_min':.05,'range_max':10.,
                       'angle_min':-math.pi,'angle_max':math.pi-step,'angle_increment':step}}
        self.assertTrue(module.scan_summary(frame)[0])
        for mutate in (lambda d:d['scan']['ranges'].__setitem__(0,None),
                       lambda d:d['scan']['ranges'].__setitem__(0,.5),
                       lambda d:d['scan']['ranges'].pop(),lambda d:d.update(source_mode='replay'),
                       lambda d:d.update(age_ms=300),lambda d:d.update(scan='invalid')):
            bad=copy.deepcopy(frame);mutate(bad);self.assertFalse(module.scan_summary(bad)[0])

    def test_uwb_identity_and_tracking(self):
        left={'state':'live','connected':True,'valid':True,'measurement_valid':True,'id_valid':True,'remaining_ms':100,'age_ms':10}
        uwb={'schema':'smartcart.dual_uwb_view.v1','source_mode':'live','state':'running','profile':'single_left','sensors':{'uwb_left':left}}
        self.assertTrue(module.uwb_summary(uwb)[0])
        for key in ('id_valid','measurement_valid','connected'):
            bad=copy.deepcopy(uwb);bad['sensors']['uwb_left'][key]=False;self.assertFalse(module.uwb_summary(bad)[0])
        track={'schema':'smartcart.tracking_view.v1','source_mode':'live','state':'running','valid':True,'remaining_ms':100,
               'result':{'track':{'state':'tracking','valid':True,'candidate_count':1,'confirmation_count':3}}}
        self.assertTrue(module.tracking_summary(track)[0])
        track['result']['track']['candidate_count']=2
        self.assertFalse(module.tracking_summary(track)[0])

if __name__=='__main__':unittest.main()
