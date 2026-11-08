"""Experimental trigger ablation only; production source remains unchanged.

Same root actuator/oracle harness. Real motion without useful goal/route progress
is distinct from stationary execution-stall and from dynamic yielding. This
prototype relaxes *only the entry evidence*, never swept/temporal spin safety,
physical stopped/ready gates, cooldown, or per-episode attempt limits.
"""
from collections import deque
import importlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'docs/validation'),str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from check_ros_adapters import install_doubles
install_doubles()
from rda_dwa_real.vehicle_bridge import VehicleBridgeCore


class ExperimentalBridge(VehicleBridgeCore):
    def _arbitration_step(self,robot,obs,gx,gy,dt,blocked=False):
        history=getattr(self,'_experimental_motion_history',deque())
        self._experimental_motion_history=history
        if self.trap['active'] or self._bridge_state!=0 or self._arbit!='ACK':
            history.clear()
            return super()._arbitration_step(robot,obs,gx,gy,dt,blocked)
        cursor=self._static_search.cursor if self._static_search.path else -1
        distance=math.hypot(gx-robot.x,gy-robot.y)
        history.append((self._t_now,robot.x,robot.y,distance,cursor))
        while history and history[0][0]<self._t_now-6.0-1e-8:history.popleft()
        super()._arbitration_step(robot,obs,gx,gy,dt,blocked)
        if (not self.cfg.enable_trap or distance<1.0
                or self._t_now-self._ack_since<self.cfg.ack_guard_time
                or self._t_now<self._cooldown_until or self._episode_failed
                or self._spin_attempts>=self.cfg.max_spin_attempts):return
        if len(history)<2 or self._t_now-history[0][0]<5.9:return
        # Strict experiment: any moving/unknown source excludes this entry path.
        if not obs or any(o.obstacle_type not in ('block','static_large')
                          or o.speed>.05 or getattr(o,'velocity_unknown',False) for o in obs):return
        path=sum(math.hypot(b[1]-a[1],b[2]-a[2])for a,b in zip(history,list(history)[1:]))
        improvement=history[0][3]-min(a[3] for a in history)
        # Observable motion, little useful progress; route cursor advance grants
        # credit for an intentional detour moving away from the global goal.
        if path<.06 or improvement>.03 or (cursor>=0 and history[0][4]>=0 and cursor>history[0][4]):return
        # First use the supported Ackermann rear corridor, not an unsafe spin
        # inside a narrow cavity. Endpoint is guidance; DWA remains the executor.
        rear=(robot.x-3.*math.cos(robot.theta),robot.y-3.*math.sin(robot.theta))
        endpoint_safe=self.planner.cc.time_collision_free([[*rear,robot.theta]],[0.],obs,self.pp.safety_margin,0.)
        rear_safe=any(self.planner.command_safe(v,0.,robot,obs)for v in(-.05,-.1,-.15))
        if not endpoint_safe or not rear_safe:return
        self._activate_trap(robot,obs,(gx,gy))
        self.trap['sub_goal']=rear;self.trap['escape_heading']=robot.theta+math.pi
        self.trap['recovery_phase']='RETREAT';self.trap['rear_heading']=robot.theta
        self.cfg.trap_max_duration=24.0
        self._trap_initial_sub_goal=rear
        self._blocked_since=None;self._progress_anchor=None;self._stall_class='experimental_moving_local_minimum'
        history.clear()

    def step(self,*args,**kwargs):
        info=super().step(*args,**kwargs)
        if (self.trap['active'] and self.trap.get('recovery_phase') in ('ALIGN','RETURN_ALIGN')
                and self._arbit=='ACK' and info.get('ok',False)):
            if self.planner.command_safe(0.,0.,self.robot,self._last_obs):
                return self._stop_info(info,'trap_align_stop',abort=False)
            return self._stop_info(info,'trap_align_stop_unsafe')
        return info

    def _begin_phase_spin(self,target,obs):
        if abs((target-self.robot.theta+math.pi)%(2*math.pi)-math.pi)<math.radians(self.cfg.spin_entry_deg):
            self.trap['recovery_phase']='REJOIN' if self.trap['recovery_phase']=='RETURN_ALIGN' else 'ESCAPE'
            return
        if self._spin_attempts>=self.cfg.max_spin_attempts or not self._spin_safe(target,obs):return
        self._arbit='SPIN_REQUEST';self._spin_target=target;self._spin_req_since=self._t_now
        self._spin_anchor=(self.robot.x,self.robot.y);self._spin_attempts+=1;self._spin_in_trap_count+=1
        self._spin_used=True;self._spin_saw_exec=False;self._last_spin_result=None
        self.trap['phase_spin_started']=True

    def _update_trap(self,obs,dt):
        phase=self.trap.get('recovery_phase')
        if phase in ('RETREAT','ALIGN','ESCAPE','RETURN_ALIGN') and self.trap['active']:
            if self._bridge_state!=0 or self._arbit!='ACK':return
            self._trap_elapsed+=dt;self.trap_ctrl.trap_steps+=1;self.trap_ctrl.trap_total_steps+=1
            if self._trap_elapsed>self.cfg.trap_max_duration:
                self._force_trap_off('trap_timeout');self._cooldown_until=self._t_now+self.cfg.trap_cooldown;return
            sx,sy=self.trap['sub_goal']
            if phase=='RETREAT' and math.hypot(self.robot.x-sx,self.robot.y-sy)<.20:
                h=self.trap['rear_heading'];candidates=[]
                for sign in(-1.,1.):
                    gx,gy=self.robot.x-sign*2.*math.sin(h),self.robot.y+sign*2.*math.cos(h)
                    if self.planner.cc.time_collision_free([[gx,gy,h]],[0.],obs,self.pp.safety_margin,0.):
                        clearance=self.planner.free_distance(self.robot,obs,h+sign*math.pi/2,3.)
                        candidates.append((clearance,(gx,gy),h+sign*math.pi/2))
                if candidates:
                    _,goal,heading=max(candidates,key=lambda x:x[0]);self.trap['sub_goal']=goal
                    self.trap['escape_heading']=heading;self.trap['recovery_phase']='ALIGN'
                    self.trap['phase_spin_started']=False
            elif phase in ('ALIGN','RETURN_ALIGN'):
                if self.trap.get('phase_spin_started'):
                    self.trap['recovery_phase']='REJOIN' if phase=='RETURN_ALIGN' else 'ESCAPE'
                    self.trap['phase_spin_started']=False
                elif abs(self.robot.v)<=.03:
                    self._begin_phase_spin(self.trap['escape_heading'],obs)
            elif phase=='ESCAPE' and math.hypot(self.robot.x-sx,self.robot.y-sy)<.30:
                self.trap['recovery_phase']='RETURN_ALIGN'
                self.trap['escape_heading']=math.atan2(self._goal[1]-self.robot.y,self._goal[0]-self.robot.x)
                self.trap['phase_spin_started']=False
            return
        super()._update_trap(obs,dt)

    def _trap_exit(self,robot,obs,gx,gy):
        if self.trap.get('recovery_phase') in ('RETREAT','ALIGN','ESCAPE','RETURN_ALIGN'):return False
        return super()._trap_exit(robot,obs,gx,gy)


def main():
    adapter=importlib.import_module('rda_dwa_real.bridge_node')
    original=adapter.RealVehicleBridgeNode.__init__
    def experimental_init(self,*args,**kwargs):
        original(self,*args,**kwargs);self.core.__class__=ExperimentalBridge
    adapter.RealVehicleBridgeNode.__init__=experimental_init
    spec=importlib.util.spec_from_file_location('comparison',ROOT/'docs/research/validation/run_comparison.py')
    harness=importlib.util.module_from_spec(spec);spec.loader.exec_module(harness)
    from run_closed_loop import independent_circle_gap,clip_change
    harness.independent_circle_gap=independent_circle_gap;harness.clip_change=clip_change
    results=[]
    for case in ('multi_blocks','u_shape','thick_u','narrow_u','crossing','straight'):
        summary,rows=harness.run(case,60. if case in ('multi_blocks','u_shape','thick_u','narrow_u') else harness.SCENARIOS[case]['duration'])
        results.append(summary)
        folder=ROOT/'docs/research/validation/moving_stall_experiment';folder.mkdir(exist_ok=True)
        (folder/f'{case}_trace.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(case,summary['reached'],summary['distance_to_goal_m'],summary['mode_switch_count'],summary['minimum_oracle_gap_m'],flush=True)
    result={'scope':'experimental class injection into same root model-oracle harness; production unchanged; all-static moving-local-minimum trigger only',
        'thresholds':{'window_s':6.,'minimum_observed_path_m':.06,'maximum_best_goal_improvement_m':.03},'results':results}
    (ROOT/'docs/research/control_moving_stall_experiment.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
