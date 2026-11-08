"""Same-input offline baseline/integrated comparison; no ROS, DDS or UDP.

Uses the historical actuator harness without changing its physical constants.
All observations are synthetic oracle geometry; no raw-cloud perception or
vehicle calibration is inferred. Fault scenes describe safe-stop behaviour.
"""
from __future__ import annotations
import argparse, importlib, json, math, os, subprocess, sys, tempfile, time, zipfile
from pathlib import Path
SCENARIOS = {
 'straight': {'goal':(4.,0.),'duration':30.},
 'left_curve': {'goal':(3.,1.5),'duration':30.},
 'right_curve': {'goal':(3.,-1.5),'duration':30.},
 'static_block': {'goal':(4.,0.),'duration':35.},
 'multi_blocks': {'goal':(4.,0.),'duration':35.},
 'long_wall': {'goal':(4.,0.),'duration':35.},
 'u_shape': {'goal':(4.,0.),'duration':35.},
 'thick_u': {'goal':(4.,0.),'duration':35.},
 'narrow_u': {'goal':(4.,0.),'duration':35.},
 'narrow_passage': {'goal':(4.,0.),'duration':30.},
 'dead_end': {'goal':(4.,0.),'duration':20.},
 'crossing': {'goal':(4.,0.),'duration':35.},
 'head_on': {'goal':(4.,0.),'duration':30.},
 'overtaking': {'goal':(4.,0.),'duration':30.},
 'accelerate_reverse': {'goal':(4.,0.),'duration':30.},
 'occlusion': {'goal':(4.,0.),'duration':20.},
 'late_frame': {'goal':(4.,0.),'duration':10.},
 'false_velocity': {'goal':(4.,0.),'duration':10.},
 'reverse_goal': {'goal':(-2.,0.),'duration':30.},
 'rear_blocked': {'goal':(-2.,0.),'duration':5.},
 'localization_dropout': {'goal':(4.,0.),'duration':5.},
 'obstacle_dropout': {'goal':(4.,0.),'duration':5.},
}
ABLATION = 'integrated'

def box(i,x,y,l,w): return (i,x,y,l,w,0.,0.,'block')
def obstacle(case,t):
 if case in ('straight','left_curve','right_curve','reverse_goal','localization_dropout','obstacle_dropout'):return []
 if case == 'static_block':return [box(1,2.5,0.,.6,.6)]
 if case == 'multi_blocks':return [box(1,1.8,.15,.4,.5),box(2,3.,-.5,.5,.5)]
 if case == 'long_wall':return [box(1,2.,0.,.25,3.)]
 if case in ('u_shape','thick_u','narrow_u','dead_end'):
  width = .55 if case == 'thick_u' else .25
  side_y = .85 if case == 'narrow_u' else 1.3
  result=[box(1,2.,0.,width,2.*side_y+width),box(2,.5,side_y,3.,width),box(3,.5,-side_y,3.,width)]
  if case == 'dead_end':result.append(box(4,-1.2,0.,.25,3.))
  return result
 if case=='narrow_passage':return [box(1,2.,.8,6.,.2),box(2,2.,-.8,6.,.2)]
 if case=='crossing':return [(1,2.,1.8-t,.5,.5,-math.pi/2,1.,'pedestrian')]
 if case=='head_on':return [(1,5.-.7*t,0.,.5,.5,math.pi,.7,'pedestrian')]
 if case=='overtaking':return [(1,-3.+.7*t,.9,.5,.5,0.,.7,'pedestrian')]
 if case=='accelerate_reverse':
  yy=2.-.25*t if t<2 else 1.5-1.2*(t-2) if t<3.5 else -.3+.8*(t-3.5)
  vel=-.25 if t<2 else -1.2 if t<3.5 else .8
  return [(1,2.,yy,.5,.5,math.copysign(math.pi/2,vel),abs(vel),'pedestrian')]
 if case=='occlusion':return [(1,2.,max(-1.5,1.5-t),.5,.5,0.,-1.,'unknown')]
 if case=='late_frame':return [(1,2.,1.8-t,.5,.5,-math.pi/2,1.,'pedestrian')]
 if case=='false_velocity':return [(1,2.,.9,.5,.5,0.,-1.,'unknown')]
 if case=='rear_blocked':return [box(1,-1.,0.,.4,.4)]
 raise ValueError(case)

def independent_gap(x,y,yaw,ob,length,width,offset):
 if ob[-1] not in ('block','static_large'):
  return independent_circle_gap(x,y,yaw,ob,length,width,offset)
 _,ox,oy,ol,ow,_,_,_=ob
 cx,cy=x+offset*math.cos(yaw),y+offset*math.sin(yaw)
 c,s=math.cos(yaw),math.sin(yaw);dx,dy=ox-cx,oy-cy
 # Maximum separating-axis gap: conservative lower bound on polygon distance.
 gaps=[abs(dx)-(length*abs(c)+width*abs(s)+ol)/2,
       abs(dy)-(length*abs(s)+width*abs(c)+ow)/2,
       abs(dx*c+dy*s)-(length+ol*abs(c)+ow*abs(s))/2,
       abs(-dx*s+dy*c)-(width+ol*abs(s)+ow*abs(c))/2]
 return max(gaps)

def extra_metrics(rows,durations,summary,goal):
 if not rows:return {}
 path=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(rows,rows[1:]))
 accel=[(b['speed']-a['speed'])/(b['t']-a['t']) for a,b in zip(rows,rows[1:])]
 jerk=[abs(b-a)/.1 for a,b in zip(accel,accel[1:])]
 err=[]
 for row in rows:
  if math.hypot(goal[0]-row['x'],goal[1]-row['y'])>.3:
   target=math.atan2(goal[1]-row['y'],goal[0]-row['x'])
   if row['speed']<-.01:target+=math.pi
   err.append(math.atan2(math.sin(target-row['yaw']),math.cos(target-row['yaw'])))
 active=[row.get('trap',False) for row in rows]
 entries=sum(b and not a for a,b in zip([False]+active,active))
 times=sorted(durations)
 percentile=lambda q: times[min(len(times)-1,int(q*len(times)))]*1000
 return {'path_length_m_10hz_model':path,
  'yaw_to_global_goal_rmse_rad_10hz_model':math.sqrt(sum(v*v for v in err)/len(err)) if err else None,
  'speed_jerk_max_mps3_10hz_model':max(jerk,default=0.),
  'mode_switch_count':len(summary['mode_requests']),
  'mode_wait_s_10hz':sum(.1 for row in rows if row.get('ready')==-1),
  'trap_entries':entries,'trap_reentries':max(0,entries-1),
  'plan_p50_ms':percentile(.5),'plan_p99_ms':percentile(.99),
  'deadline_misses_100ms':sum(d>.1 for d in durations),'hard_deadline_misses_300ms':sum(d>.3 for d in durations),
  'failure_type':'reached' if summary['reached'] else 'safe_stop_or_local_failure' if summary['minimum_conservative_circle_gap_m'] is None or summary['minimum_conservative_circle_gap_m']>0 else 'oracle_collision',
  'minimum_oracle_gap_m':summary['minimum_conservative_circle_gap_m'],
  'ttc':None,'ttc_reason':'real vehicle TTC unavailable; separate synthetic CV-circle TTC reported when known' ,
  'actual_steering_measurement':None,'metrics_scope':'synthetic geometry / bounded actuator model; command steer rate; 10Hz model jerk, not vehicle measurements'}

def run(case, duration=40.):
    bridge = importlib.import_module('rda_dwa_real.bridge_node').RealVehicleBridgeNode().core
    tracker = importlib.import_module('rda_dwa_real.traj_tracker_node').TrajTrackerNode().core
    if ABLATION == 'experimental_recovery':
        bridge.cfg.moving_recovery_enable = True
    elif ABLATION == 'unbounded_scalar':
        bridge.pp.planning_budget_s = 0.
        bridge.pp.batch_temporal_enable = False
    elif ABLATION == 'no_local_guide':
        bridge.cfg.static_search_enable = False
    dt = .02
    goal = SCENARIOS[case]['goal']
    bridge.set_goal(*goal, frame='map')
    x = y = yaw = speed = steer = spin_speed = 0.
    target_mode = actual_mode = 0
    switch_started = -10.
    info = {'v_cmd': 0., 'w_cmd': 0., 'traj_mode': 0, 'spin_target': 0.}
    trajectory = [(0., 0., 0.)] * 60
    reference_seq = 0
    transitions = []
    steering_flips = 0
    last_sign = 0
    min_gap = math.inf
    min_physical_gap = math.inf
    min_ttc = math.inf
    oracle_margin_violation_ticks = 0
    known_dynamic_seen = False
    maximum_steer_rate = 0.
    plan_durations = []
    reasons = {}
    previous_angle = 0.
    rows = []
    reached = False
    for tick in range(int(duration/dt)):
        t = tick*dt
        if actual_mode != target_mode and t-switch_started >= 3.:
            actual_mode = target_mode
        ready = actual_mode if actual_mode == target_mode else -1
        tracker.set_mode_ready(ready, t)
        tracker.set_speed_feedback(speed, t)
        bridge.set_mode_ready(ready, arrival=t)
        bridge.set_bridge_state(tracker.bridge_state())
        bridge.set_wheel_feedback([speed]*4, stamp=t)
        if tick % 5 == 0:
            if not (case == 'localization_dropout' and t > 1.):
                bridge.set_localization(x, y, yaw, stamp=1000.+t, arrival=t)
            detections = [(ox, oy, typ, length, width, heading, velocity, ident)
                          for ident, ox, oy, length, width, heading, velocity, typ in obstacle(case, t-.45 if case == 'late_frame' else t)]
            if not (case == 'obstacle_dropout' and t > 1.):
                late = .45 if case == 'late_frame' else 0.
                bridge.set_obstacles(detections, stamp=1000.+t-late, frame='map', arrival=t, source_age=late)
            start = time.perf_counter()
            info = bridge.step(dt=.1, now=t)
            plan_durations.append(time.perf_counter()-start)
            reason = info.get('reason', 'motion')
            reasons[reason] = reasons.get(reason, 0)+1
            tracker.set_traj_mode(info.get('traj_mode', 0))
            trajectory = list(bridge._reference) or [(x, y, yaw)]*60
            reference_seq += 1
            if info.get('reached'):
                reached = True
        tracker.set_pose(x, y)
        if tick % 5 == 0 and hasattr(tracker, 'set_yaw_feedback'):
            tracker.set_yaw_feedback(yaw, 1000.+t, t)
        command = tracker.step(info['v_cmd'], info['w_cmd'], yaw, trajectory,
                               info.get('spin_target', yaw), t, dt, reference_seq)
        bridge.set_tracker_cmd_speed(command['speed'], command['angle'],
                                     command['turnmode'], arrival=t)
        if command['turnmode'] != target_mode and ready != -1:
            target_mode = command['turnmode']
            switch_started = t
            transitions.append({'t': round(t, 3), 'target_mode': target_mode})
        live = actual_mode == target_mode
        target_v = command['speed'] if live and actual_mode == 0 else 0.
        accelerating = abs(target_v) > abs(speed) and target_v*speed >= 0.
        speed = clip_change(speed, target_v, (.5 if accelerating else 1.5)*dt)
        target_w = command['speed'] if live and actual_mode == 4 else 0.
        spin_speed = clip_change(spin_speed, target_w, .8*dt)
        steer = clip_change(steer, math.radians(command['angle']), math.radians(45)*dt)
        old_x, old_y = x, y
        if actual_mode == 4:
            yaw += spin_speed*dt
        else:
            # The user confirmed the compensated localization point is the
            # geometric centre. Integrate the bicycle rear axle, then transform
            # back to that centre, assuming symmetric axles until calibrated.
            rear_x = x-.345*math.cos(yaw)
            rear_y = y-.345*math.sin(yaw)
            rear_x += speed*math.cos(yaw)*dt
            rear_y += speed*math.sin(yaw)*dt
            yaw += speed*math.tan(steer)/.69*dt
            x = rear_x+.345*math.cos(yaw)
            y = rear_y+.345*math.sin(yaw)
        if command['turnmode'] == 0 and abs(speed) > .05:
            sign = 1 if command['angle'] > .5 else -1 if command['angle'] < -.5 else 0
            if sign and last_sign and sign != last_sign:
                steering_flips += 1
            if sign:
                last_sign = sign
            maximum_steer_rate = max(maximum_steer_rate, abs(command['angle']-previous_angle)/dt)
        previous_angle = command['angle']
        for ob in obstacle(case, t+dt):
            gap = independent_gap(x, y, yaw, ob, bridge.ack_model.p.length,
                                         bridge.ack_model.p.width, bridge.ack_model.p.footprint_center_offset)
            min_gap = min(min_gap, gap)
            physical_ob = (*ob[:-1], 'block')
            physical_gap = independent_gap(x, y, yaw, physical_ob, bridge.ack_model.p.length,
                                            bridge.ack_model.p.width, bridge.ack_model.p.footprint_center_offset)
            min_physical_gap = min(min_physical_gap, physical_gap)
            oracle_margin_violation_ticks += int(gap < bridge._base_safety_margin-1e-9)
            if ob[-1] not in ('block','static_large','unknown') and ob[6]>=0.:
                known_dynamic_seen=True
                _,ox,oy,ol,ow,oh,ov,_=ob
                dx,dy=ox-x,oy-y
                rvx=ov*math.cos(oh)-(x-old_x)/dt
                rvy=ov*math.sin(oh)-(y-old_y)/dt
                radius=math.hypot(bridge.ack_model.p.length/2,bridge.ack_model.p.width/2)+math.hypot(ol/2,ow/2)
                aa=rvx*rvx+rvy*rvy;bb=2*(dx*rvx+dy*rvy);cc=dx*dx+dy*dy-radius*radius
                discriminant=bb*bb-4*aa*cc
                ttc=0. if cc<=0 else (-bb-math.sqrt(discriminant))/(2*aa) if aa>1e-12 and discriminant>=0 else math.inf
                if ttc>=0:min_ttc=min(min_ttc,ttc)
        if tick % 5 == 0:
            rows.append({'t': round(t, 3), 'x': x, 'y': y, 'yaw': yaw,
                         'speed': speed, 'angle': command['angle'],
                         'controller': tracker.mode, 'arbitration': bridge._arbit,
                         'reason': info.get('reason', ''), 'mode': actual_mode, 'trap':bool(bridge.trap['active']), 'ready':ready})
        if reached and speed < .001:
            break
    distances = sorted(plan_durations)
    summary = {
        'case': case, 'duration_s': round(t+dt, 3), 'reached': reached,
        'distance_to_goal_m': math.hypot(x-goal[0], y-goal[1]),
        'minimum_conservative_circle_gap_m': min_gap if math.isfinite(min_gap) else None,
        'mode_requests': transitions, 'steering_sign_flips_over_half_degree': steering_flips,
        'maximum_command_steer_rate_deg_s': maximum_steer_rate,
        'plan_p95_ms': distances[min(len(distances)-1, int(.95*len(distances)))]*1000,
        'plan_max_ms': max(distances)*1000, 'stop_reasons': reasons,
    }
    summary.update(extra_metrics(rows, plan_durations, summary, goal))
    summary['oracle_margin_violation_obstacle_ticks_50hz']=oracle_margin_violation_ticks
    summary['minimum_physical_aabb_oracle_gap_lower_bound_m']=min_physical_gap if math.isfinite(min_physical_gap) else None
    summary['physical_aabb_collision_free_oracle']=not math.isfinite(min_physical_gap) or min_physical_gap>0.
    summary['collision_free_oracle']=not math.isfinite(min_gap) or min_gap>0.
    summary['configured_margin_preserved_oracle']=not math.isfinite(min_gap) or min_gap>=bridge._base_safety_margin-1e-9
    summary['minimum_cv_circle_ttc_s_model']=min_ttc if math.isfinite(min_ttc) else None
    summary['ttc_model_known_dynamic_seen']=known_dynamic_seen
    summary['ttc_model_assumption']='constant velocities and enclosing circles from synthetic oracle; separate from safety checker and real TTC'
    summary['gap_method']='SAT separation lower bound for static AABB; exact point-to-vehicle rectangle minus circle radius for dynamic' 
    if math.isfinite(min_physical_gap) and min_physical_gap<=0.:
        summary['failure_type']='physical_aabb_oracle_collision'
    elif math.isfinite(min_gap) and min_gap<=0.:
        summary['failure_type']='conservative_envelope_overlap'
    summary['navigation_success_with_no_oracle_collision']=summary['reached'] and summary['physical_aabb_collision_free_oracle']
    summary['moving_recovery_enabled']=bool(getattr(bridge.cfg,'moving_recovery_enable',False))
    summary['static_search_enabled']=bool(getattr(bridge.cfg,'static_search_enable',False))
    return summary, rows


def worker(source,output,ablation,cases):
 global ABLATION, independent_circle_gap, clip_change
 ABLATION=ablation
 sys.path[:0]=[str(source/'docs/validation'),str(source/'rda_dwa'),str(source/'rda_dwa_real')]
 from check_ros_adapters import install_doubles
 install_doubles()
 from run_closed_loop import independent_circle_gap, clip_change
 output.mkdir(parents=True,exist_ok=True)
 results=[]
 for case in cases:
  result,rows=run(case,SCENARIOS[case]['duration']);results.append(result)
  (output/f'{case}_trace.json').write_text(json.dumps(rows,indent=2))
  print(json.dumps({'variant':ablation,**result},ensure_ascii=False),flush=True)
 (output/'results.json').write_text(json.dumps(results,indent=2,allow_nan=False))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--worker',action='store_true')
    parser.add_argument('--source',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--ablation',default='integrated')
    parser.add_argument('--cases',nargs='*',default=list(SCENARIOS))
    parser.add_argument('--duration',type=float)
    args=parser.parse_args()
    if args.duration is not None:
        if not math.isfinite(args.duration) or args.duration<=0:
            raise ValueError('duration must be finite and positive')
        for case in args.cases:SCENARIOS[case]['duration']=args.duration
    root=Path(__file__).resolve().parents[3]
    if args.worker:
        if args.source is None or args.output is None:
            parser.error('--worker requires --source and --output')
        return worker(args.source,args.output,args.ablation,args.cases)
    output=args.output or root/'docs/research/validation/comparison'
    output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rda-research-baseline-') as directory:
        baseline=Path(directory)
        with zipfile.ZipFile(root/'docs/research/baseline_sources.zip') as archive:
            for name in archive.namelist():
                if name.startswith(('rda_dwa/','rda_dwa_real/','docs/validation/')) and not name.endswith(('.json','.jsonl')):
                    archive.extract(name,baseline)
        for label,source in [('baseline',baseline),('integrated',root)]:
            command=[sys.executable,str(Path(__file__).resolve()),'--worker','--source',str(source),
                     '--output',str(output/label),'--ablation',label,'--cases',*args.cases]
            if args.duration is not None:command.extend(['--duration',str(args.duration)])
            subprocess.run(command,check=True)
    variants={name:json.loads((output/name/'results.json').read_text()) for name in ('baseline','integrated')}
    (output/'comparison.json').write_text(json.dumps({'date':'2026-11-06','same_harness':True,
        'same_scenarios':SCENARIOS,'scope':'offline fake ROS constructor + synthetic geometry + bounded actuator; no actual UDP/vehicle',
        'variants':variants},indent=2,allow_nan=False))

if __name__=='__main__':main()
