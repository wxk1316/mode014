"""Independent corner-distance checks on the faster conservative separation."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from rda_dwa.motion_safety import minimum_gaps,VehicleGeometry,TrackedObstacle,rollout_rear
from rda_dwa.collision_checker import _rect_corners_vec,_rect_rect_gaps_vec


class MotionEnvelopeTests(unittest.TestCase):
    def test_sat_never_overstates_positive_exact_clearance(self):
        rng=np.random.default_rng(5);g=VehicleGeometry()
        poses=np.column_stack((rng.uniform(-4,4,1200),rng.uniform(-4,4,1200),rng.uniform(-3.14,3.14,1200)))
        for heading in (0.,.7,1.5):
            o=TrackedObstacle(.2,-.1,1.8,.7,heading)
            bounded=minimum_gaps(poses[:,None,:],[0.],[o],g)
            body=_rect_corners_vec(poses[:,0],poses[:,1],poses[:,2],g.length,g.width)
            box=_rect_corners_vec(np.array([o.x]),np.array([o.y]),np.array([o.heading]),o.length,o.width)[0]
            exact=_rect_rect_gaps_vec(body,box)
            self.assertTrue(np.all(bounded[exact>0]<=exact[exact>0]+1e-8))
            self.assertFalse(np.any((bounded>=.12)&(exact<.12-1e-8)))

    def test_rollout_matches_jerk_and_steering_acceleration_bounds(self):
        g=VehicleGeometry()
        rear,v,t,d,a,r=rollout_rear(0.,0.,0.,0.,0.,[1.],[.5],g,3.,.05,jerk=2.,steer_accel=1.7,with_actuator_state=True)
        self.assertLessEqual(np.max(np.abs(np.diff(a[0])/np.diff(t))),2.+1e-8)
        self.assertLessEqual(np.max(np.abs(np.diff(r[0])/np.diff(t))),1.7+1e-8)
        self.assertLessEqual(np.max(np.abs(r)),.62)
        self.assertGreater(v[0,-1],.9)


if __name__=='__main__':unittest.main()
