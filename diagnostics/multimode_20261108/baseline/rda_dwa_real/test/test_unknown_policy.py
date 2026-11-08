"""Requested fixed-box unknown policy and localization regression checks."""
import sys,math,unittest
import pytest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real'),str(ROOT/'diagnostics/engineering_20261108')]
from rda_dwa.motion_safety import VehicleGeometry,TrackedObstacle,minimum_gaps,obstacle_from_object
from rda_dwa_real.traj_tracker_core import TrajTrackerCore
from rda_dwa_real.kf_tracker import GlobalKFTracker

class UnknownPolicyTests(unittest.TestCase):
 def test_unknown_legacy_velocity_and_bound_cannot_translate_or_grow_box(self):
  poses=np.zeros((1,61,3));times=np.linspace(0.,3.,61);g=VehicleGeometry()
  unknown=TrackedObstacle(3.,0.,.4,.4,vx=-1.5,vy=.8,uncertain=True,uncertainty_speed=1.5)
  fixed=TrackedObstacle(3.,0.,.4,.4)
  self.assertEqual(unknown.speed,0.)
  for age in (0.,.2,2.):
   self.assertAlmostEqual(minimum_gaps(poses,times,[unknown],g,age).item(),minimum_gaps(poses,times,[fixed],g,age).item())
 def test_unknown_object_adapter_zeroes_reported_motion(self):
  o=obstacle_from_object(SimpleNamespace(x=3.,y=0.,length=.4,width=.4,heading=1.,speed=1.5,obstacle_type='unknown'))
  self.assertTrue(o.uncertain);self.assertEqual((o.vx,o.vy,o.uncertainty_speed),(0.,0.,0.))
 def test_confirmed_motion_still_translates_at_every_prediction_time(self):
  poses=np.zeros((1,61,3));times=np.linspace(0.,3.,61);g=VehicleGeometry()
  moving=TrackedObstacle(3.,0.,.4,.4,vx=-1.)
  fixed=TrackedObstacle(3.,0.,.4,.4,uncertain=True)
  self.assertLess(minimum_gaps(poses,times,[moving],g).item(),0.)
  self.assertGreater(minimum_gaps(poses,times,[fixed],g).item(),2.)
 def test_unconfirmed_track_stays_fixed_then_expires_on_third_miss(self):
  tracker=GlobalKFTracker()
  first=tracker.step([(0.,0.,'pedestrian',.4,.3,.7)],dt=.1)[0]
  self.assertEqual(first[5],'unknown');self.assertEqual(first[3],0.)
  for dt in (.1,.2):
   missed=tracker.step([],dt=dt)[0]
   self.assertEqual(missed[5],'unknown');self.assertEqual(missed[3],0.)
   self.assertEqual(missed[1:3],(0.,0.));self.assertEqual(missed[6:9],(.4,.3,.7))
  self.assertEqual(tracker.step([],dt=.3),[])
 def test_adaptive_preview_ignores_legacy_unknown_motion_bound(self):
  from rda_dwa_real.adaptive_preview import AdaptivePreview
  diagnostics=[]
  for bound in (0.,1.5):
   c=TrajTrackerCore();c.set_pose(0.,0.,0.,stamp=10.,arrival=10.)
   c.set_reference([(-.345,0.,0.,.5,0.,0.),(2.655,0.,0.,.5,3.,0.)],10.,seq=1)
   c.set_obstacles([TrackedObstacle(4.5+i*.01,4.5,.4,.4,uncertain=True,uncertainty_speed=bound) for i in range(20)],10.,stamp=10.)
   _,diag=AdaptivePreview().choose(c,0.,10.,.02);diagnostics.append(diag)
  self.assertEqual(diagnostics[0]['risk_relevant_obstacle_count'],diagnostics[1]['risk_relevant_obstacle_count'])
  self.assertEqual(diagnostics[1]['reason'],'open_smooth_path')
 def test_manual_map_relocalization_before_goal_is_accepted_while_stopped(self):
  c=TrajTrackerCore();c.set_safety('no_goal');c.set_pose(0.,0.,0.,stamp=10.,arrival=10.)
  c.set_pose(-78.,-12.,-1.57,stamp=10.05,arrival=10.05,allow_reinitialization=True)
  self.assertEqual(c._centre,(-78.,-12.));self.assertEqual(c._pose_reinitializations,1)
  self.assertAlmostEqual(c._pose_at,10.05);self.assertEqual(c.step(10.06,.02)['speed'],0.)
 def test_same_map_jump_during_active_motion_remains_rejected(self):
  c=TrajTrackerCore();c.set_pose(0.,0.,0.,stamp=10.,arrival=10.);c.ack_cmd(.3,0.)
  c.set_pose(-78.,-12.,-1.57,stamp=10.05,arrival=10.05,allow_reinitialization=True)
  self.assertEqual(c._centre,(0.,0.));self.assertEqual(c._safety_reason,'localization_jump')
  self.assertEqual(c.step(10.06,.02)['speed'],0.)
 @pytest.mark.xfail(strict=True,reason='Requested stationary unknown policy removes early motion protection: 5 m head-on actor unconfirmed for 0.6 s collides in this stress case')
 def test_uncertain_head_on_actor_is_avoided_after_confirmation(self):
  from closed_loop import run
  result=run('head_on_unconfirmed',.25,True,30.)
  self.assertTrue(result['reached'],result);self.assertGreaterEqual(result['min_physical_gap_m'],.12,result)
  self.assertLessEqual(result['max_command_jerk_m_s3'],2.+1e-7)
  self.assertLessEqual(result['max_steer_rate_deg_s'],30.+1e-7)

if __name__=='__main__':unittest.main()
