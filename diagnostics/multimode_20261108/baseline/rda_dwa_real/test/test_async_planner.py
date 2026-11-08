"""Planning latency acceptance and state-consistency regression tests."""
from pathlib import Path
import queue
import sys
import time
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'rda_dwa_real'), str(ROOT/'rda_dwa')]
from rda_dwa_real.async_planner import AsyncPlanner, _work


def robot(x=0., yaw=0.,y=0.):
    return types.SimpleNamespace(x=x, y=y, theta=yaw, v=0.)


class SlowPlanner:
    def __init__(self, delay=.35):
        self.params = types.SimpleNamespace(dt=.1)
        self.delay = delay
        self.reference = []
        self.telemetry = {}

    def plan(self, pose, goal, obs, trap):
        time.sleep(self.delay)
        self.reference = [(0., 0., 0., .07, 0.), (.1, 0., 0., .07, 1.)]
        self.telemetry = {'reason': 'motion'}
        return True, .07, 0.


class AsyncPlannerTests(unittest.TestCase):
    def cached(self, elapsed, old_goal=(4., 0.), old_pose=None):
        planner = AsyncPlanner.__new__(AsyncPlanner)
        planner.base = SlowPlanner(0.)
        planner.max_age = .8
        planner.reference = []
        planner.telemetry = {}
        planner.current_steer = 0.
        planner.obstacle_age = 0.
        planner._pending = False
        planner._ready = True
        planner._seq = 1
        planner._input = queue.Queue(1)
        planner._output = queue.Queue(2)
        planner._process = types.SimpleNamespace(is_alive=lambda: True)
        planner._result = (1, 100.-elapsed, old_pose or robot(), old_goal,
            (True, .07, 0.), [(0.,0.,0.,.07,0.),(.1,0.,0.,.07,1.)],
            {'reason':'motion','worker_duration_s':.2})
        return planner

    def test_result_older_than_old_300ms_limit_is_accepted_within_800ms(self):
        planner = self.cached(.55)
        with patch('rda_dwa_real.async_planner.time.monotonic', return_value=100.):
            self.assertEqual(planner.plan(robot(), (4.,0.), []), (True,.07,0.))
        self.assertEqual(planner.telemetry['stale_causes'], [])
        self.assertEqual(planner.telemetry['max_result_age_s'], .8)
        self.assertTrue(planner.reference)

    def test_expired_goal_changed_pose_changed_and_yaw_changed_still_stop(self):
        for elapsed, goal, pose, cause in [
                (.81, (4.,0.), robot(), 'result_age'),
                (.2, (5.,0.), robot(), 'goal_changed'),
                (.2, (4.,0.), robot(y=.19), 'pose_displacement'),
                (.2, (4.,0.), robot(yaw=.25), 'yaw_change')]:
            with self.subTest(cause=cause):
                planner = self.cached(elapsed)
                with patch('rda_dwa_real.async_planner.time.monotonic', return_value=100.):
                    self.assertEqual(planner.plan(pose, goal, []), (False,0.,0.))
                self.assertEqual(planner.telemetry['reason'], 'planning_result_stale')
                self.assertIn(cause, planner.telemetry['stale_causes'])
                self.assertEqual(planner.reference, [])

    def test_stale_diagnostics_preserve_underlying_worker_budget_reason(self):
        planner = self.cached(.9)
        seq, stamp, pose, goal, _, _, _ = planner._result
        planner._result = (seq,stamp,pose,goal,(False,0.,0.),[],{'reason':'planning_budget'})
        with patch('rda_dwa_real.async_planner.time.monotonic', return_value=100.):
            self.assertEqual(planner.plan(robot(), (4.,0.), []), (False,0.,0.))
        self.assertEqual(planner.telemetry['worker_reason'], 'planning_budget')
        self.assertIn('result_age', planner.telemetry['stale_causes'])

    def test_worker_receives_current_control_dt(self):
        incoming, outgoing = queue.Queue(), queue.Queue()
        incoming.put((1,time.monotonic(),robot(),(4.,0.),[],None,0.,.1,.14))
        incoming.put(None)
        planner = SlowPlanner(0.)
        _work(incoming, outgoing, planner)
        self.assertEqual(planner.params.dt, .14)
        self.assertEqual(outgoing.get()[0], 'ready')
        _, packet = outgoing.get()
        self.assertEqual(packet[-1]['sampled_control_dt_s'], .14)

    def test_progress_on_curved_checked_prefix_is_accepted(self):
        planner=self.cached(.5)
        seq,stamp,pose,goal,result,_,telemetry=planner._result
        planner._result=(seq,stamp,pose,goal,result,
            [(0.,0.,0.,.8,0.),(.25,.06,.3,.8,.4),(.45,.15,.6,.8,.7)],telemetry)
        with patch('rda_dwa_real.async_planner.time.monotonic',return_value=100.):
            self.assertTrue(planner.plan(robot(x=.25,y=.06,yaw=.3),(4.,0.),[])[0])
        self.assertAlmostEqual(planner.telemetry['path_deviation_m'],0.)

    def test_distant_future_tail_cannot_accept_early_pose_progress(self):
        planner=self.cached(.2)
        seq,stamp,pose,goal,result,_,telemetry=planner._result
        planner._result=(seq,stamp,pose,goal,result,[(0.,0.,0.,.5,0.),(2.,0.,0.,.5,4.)],telemetry)
        with patch('rda_dwa_real.async_planner.time.monotonic',return_value=100.):
            self.assertFalse(planner.plan(robot(x=1.),(4.,0.),[])[0])

    def test_real_spawned_worker_returns_motion_after_350ms(self):
        planner = AsyncPlanner(SlowPlanner(), max_age=.8)
        try:
            deadline = time.monotonic()+5.
            result = (False,0.,0.)
            while time.monotonic()<deadline:
                result = planner.plan(robot(), (4.,0.), [])
                if result[0]:
                    break
                time.sleep(.02)
            self.assertTrue(result[0], planner.telemetry)
            self.assertGreater(planner.telemetry['worker_duration_s'], .3)
            self.assertLess(planner.telemetry['result_age_s'], .8)
        finally:
            planner.close()


if __name__ == '__main__':
    unittest.main()
