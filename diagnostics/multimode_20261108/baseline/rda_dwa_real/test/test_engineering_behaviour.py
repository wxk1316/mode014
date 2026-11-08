"""Current vehicle interfaces: source selection, noise, and actuator behaviour."""
from pathlib import Path
import math,sys,types,unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real/test'),str(ROOT/'diagnostics/engineering_20261108')]
import test_localization_input as localization_tests
transform=localization_tests.transform
from rda_dwa_real.pose_velocity import PoseVelocityEstimator
from rda_dwa.motion_safety import speed_step,VehicleGeometry,TrackedObstacle,minimum_gaps
from rda_dwa_real.kf_tracker import GlobalKFTracker

class EngineeringBehaviourTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):localization_tests.LocalizationInputTests.setUpClass()
 def test_newer_tf_preferred_over_frequently_received_old_topic(self):
  helper=localization_tests.LocalizationInputTests();source=helper.input('latest_available')
  source.receive_topic(helper.localization(stamp=997.).pose.pose,helper.localization(stamp=997.).header,received_at=50.)
  source.buffer.transform=transform(stamp=999.96,x=2.)
  pose,stamp,age=source.sample(2.,now=50.02)
  self.assertEqual(pose[0],2.);self.assertEqual(source.diagnostic()['selected_source'],'tf')
  self.assertAlmostEqual(age,.04);self.assertAlmostEqual(stamp,49.98)
  _,same,old_age=source.sample(2.,now=50.1)
  self.assertEqual(stamp,same);self.assertAlmostEqual(old_age,.12)
 def test_tf_unavailable_falls_back_and_cache_does_not_renew_age(self):
  helper=localization_tests.LocalizationInputTests();source=helper.input('latest_available');msg=helper.localization(stamp=997.)
  source.receive_topic(msg.pose.pose,msg.header,received_at=50.)
  self.assertEqual(source.sample(2.,now=50.2)[1],50.)
  self.assertEqual(source.diagnostic()['selected_source'],'latest_topic')
  self.assertIsNone(source.sample(2.,now=52.01))
 def test_noisy_stationary_and_accelerating_pose_velocity(self):
  rng=np.random.default_rng(9);e=PoseVelocityEstimator();values=[]
  for t in np.arange(0.,3.,.04):
   values.append(e.update(rng.normal(0,.003),rng.normal(0,.003),rng.normal(0,.003),t))
  self.assertLess(float(np.percentile(np.abs(np.array(values)[20:,0]),95)),.07)
  e.reset()
  for t in np.arange(0.,2.01,.04):result=e.update(.25*t*t,0.,.15*t*t,t)
  self.assertAlmostEqual(result[0],math.cos(.6),places=2)
  self.assertAlmostEqual(result[1],.6,places=2)
 def test_speed_target_crossings_never_snap_acceleration(self):
  v=a=0.;accelerations=[];speeds=[]
  for i in range(900):
   target=1. if i<200 else (.25 if i<400 else (0. if i<600 else .7))
   v,a=speed_step(v,a,target,.02,.7,1.2,2.);accelerations.append(float(a));speeds.append(float(v))
  self.assertLessEqual(max(abs(np.diff(accelerations)/.02)),2.+1e-8)
  self.assertGreaterEqual(min(speeds),0.);self.assertLessEqual(max(speeds),1.+1e-8)
  self.assertLess(max(speeds[550:600]),.001)
 def test_unknown_fixed_box_ignores_unconfirmed_velocity(self):
  g=VehicleGeometry();o=TrackedObstacle(2.,0.,.4,.4,vx=1.,uncertain=True,uncertainty_speed=1.5)
  moving=minimum_gaps(np.array([[0.,0.,0.],[0.,0.,0.]]),[0.,1.],[o],g)
  zero=minimum_gaps(np.array([[0.,0.,0.],[0.,0.,0.]]),[0.,1.],[TrackedObstacle(2.,0.,.4,.4,uncertain=True,uncertainty_speed=1.5)],g)
  self.assertAlmostEqual(float(moving),float(zero))
 def test_missed_mover_keeps_kf_motion_without_time_size_expansion(self):
  tracker=GlobalKFTracker()
  for i in range(35):out=tracker.step([(i*.08,0.,'pedestrian',.4,.4)],dt=.1)
  previous=out[0];missed=tracker.step([],dt=.2)[0]
  self.assertEqual(missed[5],'pedestrian');self.assertGreater(missed[3],.7)
  self.assertGreater(missed[1],previous[1]);self.assertEqual(missed[6:8],previous[6:8])
 def test_delayed_noisy_head_on_encounter_reaches_with_physical_clearance(self):
  from closed_loop import run
  result=run('head_on',.25,True,30.)
  self.assertTrue(result['reached'],result)
  self.assertGreaterEqual(result['min_physical_gap_m'],.12,result)
  self.assertLessEqual(result['max_command_jerk_m_s3'],2.+1e-7)
  self.assertLessEqual(result['max_steer_rate_deg_s'],30.+1e-7)
  self.assertLessEqual(result['max_steer_accel_deg_s2'],100.+1e-7)
 def test_straight_goal_approach_has_no_stop_then_accelerate_pulse(self):
  import json
  from closed_loop import run
  result=run('straight',.25,True,25.)
  self.assertTrue(result['reached'],result)
  trace=json.loads((ROOT/'diagnostics/engineering_20261108/straight_trace.json').read_text())
  speed=np.asarray([row[4] for row in trace]);active=np.flatnonzero(speed>.05)
  self.assertGreater(len(active),20)
  self.assertFalse(np.any(speed[active[0]:active[-1]]<.01))

if __name__=='__main__':unittest.main()
