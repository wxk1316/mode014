"""Distinguish heading recovery from genuine dynamic-occupancy rejection."""
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from rda_dwa.module_config import module_config
from rda_dwa.realtime_ackermann import RealtimeAckermannPlanner
from rda_dwa.ackermann_model import AckermannParams,AckermannModel
from rda_dwa.motion_safety import TrackedObstacle
from rda_dwa_real.realtime_vehicle_core import RealtimeVehicleBridgeCore
from rda_dwa_real.vehicle_bridge import BridgeConfig


class DynamicYieldTests(unittest.TestCase):
    def planner(self):
        pp=module_config('v15_latesc')
        pp.dt=.1;pp.planning_budget_s=.25;pp.velocity_horizon_targets=True
        pp.command_acc=.7;pp.command_steer_rate=math.radians(30)
        pp.max_linear_velocity=1.;pp.sim_time=3.
        ack=AckermannParams(length=1.2,width=.8,wheelbase=.69)
        planner=RealtimeAckermannPlanner(pp,'ENHANCED_DWA',collision_model='rectangle',ack_params=ack)
        cfg=BridgeConfig();cfg.localization_timeout=2.;cfg.obstacle_timeout=.4
        core=RealtimeVehicleBridgeCore(planner,pp,config=cfg,ack_model=AckermannModel(ack))
        core.set_localization(0.,0.,0.,stamp=100.,arrival=0.)
        return planner,core

    def test_behind_goal_with_irrelevant_moving_object_is_orientation_stall(self):
        planner,core=self.planner()
        ok,v,_=planner.plan(core.robot,(-7.5,-.9),[TrackedObstacle(30.,30.,.3,.3,vx=.4)])
        self.assertTrue(ok)
        self.assertEqual(v,0.)
        self.assertEqual(planner.telemetry['reason'],'orientation_stall')
        self.assertTrue(planner.telemetry['motion_feasible'])
        self.assertGreater(planner.telemetry['feasible'],0)

    def test_recovery_clock_can_request_safe_spin_with_far_moving_object(self):
        _,core=self.planner();core.set_goal(-7.5,-.9,frame='map')
        spin=False
        for i in range(25):
            now=i*.1
            core.set_localization(0.,0.,0.,stamp=100.+now,arrival=now)
            core.set_obstacles([(30.,30.,'pedestrian',.3,.3,0.,.4,1)],stamp=100.+now,arrival=now)
            info=core.step(.1,now)
            if info['reason']=='spin_requested':spin=True;break
        self.assertTrue(spin,info)

    def test_overlapping_unknown_box_blocks_without_dynamic_yield(self):
        planner,core=self.planner()
        ok,v,_=planner.plan(core.robot,(4.,0.),[TrackedObstacle(.5,0.,.3,.3,uncertain=True,uncertainty_speed=1.5)])
        self.assertFalse(ok)
        self.assertEqual(v,0.)
        self.assertEqual(planner.telemetry['reason'],'blocked')
        self.assertEqual(planner.telemetry['feasible'],0)
        self.assertEqual(planner.telemetry['unknown_count'],1)
        self.assertEqual(planner.reference,[])

    def test_unknown_uses_same_static_guide_and_full_collision_horizon(self):
        planner,core=self.planner();other,_=self.planner()
        unknown=TrackedObstacle(2.5,0.,.6,.5,vx=-1.5,uncertain=True,uncertainty_speed=1.5)
        static=TrackedObstacle(2.5,0.,.6,.5)
        ans=planner.plan(core.robot,(5.,0.),[unknown]);baseline=other.plan(core.robot,(5.,0.),[static])
        self.assertEqual(ans,baseline)
        self.assertEqual(planner.reference,other.reference)
        self.assertEqual(planner.telemetry['guide_target'],other.telemetry['guide_target'])
        self.assertNotEqual(planner.telemetry['guide_target'],(5.,0.))
        self.assertEqual(planner.telemetry['unknown_policy'],'stationary')
        self.assertEqual(planner.telemetry['moving_count'],0)
        self.assertGreaterEqual(planner.telemetry['min_gap'],.12)

    def test_supervisor_unknown_packet_motion_is_zeroed(self):
        planner,core=self.planner()
        core.set_obstacles([(3.,2.,'unknown',.4,.4,1.,1.5,9,.3)],stamp=100.,arrival=0.)
        o=core._last_received_obs[0]
        self.assertTrue(o.uncertain);self.assertEqual((o.vx,o.vy,o.uncertainty_speed),(0.,0.,0.))
        self.assertTrue(core._safe_motion(.5,0.,[o],.1))
        core.set_obstacles([(.5,0.,'unknown',.4,.4,1.,1.5,9,.3)],stamp=100.1,arrival=.1)
        self.assertFalse(core._safe_motion(.5,0.,core._last_received_obs,.1))

    def test_static_scene_and_forward_motion_are_preserved(self):
        planner,core=self.planner()
        ok,v,_=planner.plan(core.robot,(4.,0.),[TrackedObstacle(3.,2.,.3,.3)])
        self.assertTrue(ok)
        self.assertGreater(v,0.)
        self.assertEqual(planner.telemetry['reason'],'motion')
        self.assertEqual(planner.telemetry['unknown_count'],0)
        self.assertGreater(len(planner.reference),2)


if __name__=='__main__':unittest.main()
