"""Independent dynamic encounter experiment; monkeypatches only this process.

The frozen production source files remain unchanged. Uses the root's exact
50Hz bounded actuator harness and synthetic obstacle streams. Failure priority
is oracle collision, then configured-margin violation, then reachability.
"""
from __future__ import annotations
import argparse,importlib,importlib.util,json,math,sys,time,types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def configure(variant,head_start=5.,guide_along=1.5,guide_lateral=1.4):
    sys.path[:0]=[str(ROOT/'docs/validation'),str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
    from check_ros_adapters import install_doubles
    install_doubles()
    from run_closed_loop import independent_circle_gap,clip_change
    path=ROOT/'docs/research/validation/run_comparison.py'
    spec=importlib.util.spec_from_file_location('root_encounter_harness',path);harness=importlib.util.module_from_spec(spec);spec.loader.exec_module(harness)
    harness.independent_circle_gap=independent_circle_gap;harness.clip_change=clip_change
    original_obstacle=harness.obstacle
    def obstacles(case,t):
        if case=='head_on':return [(1,head_start-.7*t,0.,.5,.5,math.pi,.7,'pedestrian')]
        return original_obstacle(case,t)
    harness.obstacle=obstacles
    bridge_module=importlib.import_module('rda_dwa_real.bridge_node')
    original_init=bridge_module.RealVehicleBridgeNode.__init__
    def constructor(self,*args,**kwargs):
        original_init(self,*args,**kwargs)
        core=self.core
        if variant.startswith('full_target'):
            from rda_dwa.enhanced_dwa import VelocitySample
            from rda_dwa_real import vehicle_bridge as bridge_core_module
            # Isolated target-speed semantic experiment: hardware/50Hz tracker
            # receives desired speed, while each rollout starts at actual v and
            # ramps using the original acceleration bound. Both nominal and
            # post-adaptation guard check this same desired setpoint.
            bridge_core_module.limit_signed_speed=lambda target,current,acc,dec,dt: target
            def target_window(planner,robot,obs,stop_on_motion=False):
                p,ap=planner.params,planner.cc.ack
                velocities={0.,robot.v,p.min_linear_velocity,p.max_linear_velocity}
                ds={0.,planner.current_delta,-ap.delta_max,ap.delta_max}
                step=max(math.radians(1),p.steer_resolution)
                ds.update(k*step for k in range(-int(ap.delta_max/step),int(ap.delta_max/step)+1))
                window=[];seen=set()
                for v in sorted(velocities):
                    for delta in sorted(ds):
                        planner._check_deadline()
                        w=v*math.tan(delta)/ap.wheelbase if abs(v)>=ap.v_eps-1e-9 else 0.
                        key=(round(v,8),round(w,8))
                        if key in seen:continue
                        seen.add(key);sample=VelocitySample(v,w)
                        sample.feasible=planner.is_feasible(sample,robot,obs,True)
                        planner.last_checked_count=(planner.last_checked_count or 0)+1
                        window.append(sample)
                        if stop_on_motion and sample.feasible and abs(v)>=p.spin_v_eps-1e-9:return window
                planner.last_candidate_count=len(window);planner.last_feasible_count=sum(s.feasible for s in window)
                planner.last_has_forward=any(s.feasible and s.v>=p.spin_v_eps-1e-9 for s in window)
                planner.last_has_motion=any(s.feasible and abs(s.v)>=p.spin_v_eps-1e-9 for s in window)
                return window
            core.planner._executable_window=types.MethodType(target_window,core.planner)
        if variant=='horizon6':
            core.pp.sim_time=6.;core.pp.n_trajectory_samples=120
        elif variant in ('nominal6_brake3','full_target_nominal6'):
            original_reference=core.planner.reference_trajectory
            def reference(v,w,robot,horizon=None):
                return original_reference(v,w,robot,horizon=6. if horizon is None else horizon)
            core.planner.reference_trajectory=reference
        elif variant.startswith('early_side_guide') or variant=='full_target_guided':
            # A proposed waypoint is guidance only. Keep original full dynamic
            # input and unchanged nominal+braking hard guard.
            original_detour=core._update_static_detour
            anchor=[None]
            def guide(obs):
                if anchor[0] is not None:
                    target=anchor[0]
                    if math.hypot(core.robot.x-target[0],core.robot.y-target[1])<.5:
                        anchor[0]=None
                    else:return target
                goal=core._goal
                ex,ey=goal[0]-core.robot.x,goal[1]-core.robot.y;distance=math.hypot(ex,ey)
                if distance<1:return original_detour(obs)
                ex,ey=ex/distance,ey/distance;nx,ny=-ey,ex
                for o in obs:
                    if getattr(o,'velocity_unknown',False) or o.speed<.25:continue
                    vx=o.speed*math.cos(getattr(o,'velocity_heading',o.heading));vy=o.speed*math.sin(getattr(o,'velocity_heading',o.heading))
                    dx,dy=o.x-core.robot.x,o.y-core.robot.y
                    along=dx*ex+dy*ey;lateral=dx*nx+dy*ny
                    closing=-(vx*ex+vy*ey)
                    if not (closing>.25 and 2.<along<7. and abs(lateral)<.7):continue
                    side=1.
                    anchor[0]=(core.robot.x+ex*guide_along+nx*side*guide_lateral,core.robot.y+ey*guide_along+ny*side*guide_lateral)
                    return anchor[0]
                return original_detour(obs)
            core._update_static_detour=guide
    bridge_module.RealVehicleBridgeNode.__init__=constructor
    return harness


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--variant',default='baseline');parser.add_argument('--cases',nargs='*',default=['head_on']);parser.add_argument('--head-start',type=float,default=5.);parser.add_argument('--guide-along',type=float,default=1.5);parser.add_argument('--guide-lateral',type=float,default=1.4);args=parser.parse_args()
    harness=configure(args.variant,args.head_start,args.guide_along,args.guide_lateral)
    reports=[];outdir=ROOT/'docs/research/dynamic_encounter_traces'/args.variant;outdir.mkdir(parents=True,exist_ok=True)
    for case in args.cases:
        summary,rows=harness.run(case,harness.SCENARIOS[case]['duration'])
        gap=summary['minimum_conservative_circle_gap_m']
        summary['experiment_variant']=args.variant
        summary['experimental_parameters']={'head_start_m':args.head_start,'guide_along_m':args.guide_along,'guide_lateral_m':args.guide_lateral}
        summary['failure_type_priority']='oracle_collision' if gap is not None and gap<=0 else 'configured_margin_violation' if gap is not None and gap<.2 else 'reached' if summary['reached'] else 'no_reach_safe'
        summary['success_collision_free']=summary['reached'] and (gap is None or gap>0)
        reports.append(summary)
        (outdir/(case+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
        (outdir/(case+'_trace.json')).write_text(json.dumps(rows,indent=2)+'\n')
        print(json.dumps(summary,ensure_ascii=False),flush=True)
    (outdir/'results.json').write_text(json.dumps(reports,indent=2)+'\n')


if __name__=='__main__':main()
