"""Same-input baseline and production DWA ablations; offline, no ROS or UDP.

Each worker imports either the frozen source snapshot or current sources in an
isolated interpreter. Input world obstacles are synthetic model snapshots, not
sensor replay or real logs. Quantiles are empirical finite-sample timings.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import zipfile
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


def worker(source,variant,repeats,worstcase=False):
    sys.path[:0]=[str(source/'rda_dwa'),str(source/'rda_dwa_real')]
    from rda_dwa.module_config import module_config
    from rda_dwa.enhanced_dwa import EnhancedDWA
    from rda_dwa.ackermann_model import AckermannParams
    from rda_dwa.robot_model import RobotState
    from rda_dwa.obstacle import new_obstacle
    def box(x,y,L=.4,W=.4,speed=0.,heading=0.,unknown=False):
        o=new_obstacle(id_=0,x=x,y=y,length=L,width=W,speed=speed,heading=heading,
                       obstacle_type='pedestrian' if speed or unknown else 'block')
        o.shape_heading=0.
        if unknown:o.velocity_unknown=True;o.max_unknown_speed=1.5
        return o
    def factory():
        p=module_config('v15_latesc');p.real_vehicle_safety=True
        p.max_linear_velocity,p.min_linear_velocity=.4,-.3
        p.max_angular_velocity,p.min_angular_velocity=.7,-.7
        p.acc_lin_max,p.dec_lin_max=.5,1.5
        p.safety_margin,p.safety_dt=.2,.05;p.alpha_smooth=.6
        p.enable_mpt_escape=p.backup_enabled=False
        p.batch_temporal_enable=variant!='scalar'
        p.reuse_nominal_rollouts=variant not in ('scalar','batch_only')
        p.planning_budget_s=.08 if variant=='full_budget' else 0.
        ap=AckermannParams(length=1.2,width=.8,wheelbase=.69,delta_max=math.radians(30),
            delta_dot_max=math.radians(45),v_min=-.3,v_max=.4,a_max=.5,dec_max=1.5,
            footprint_center_offset=0.,reference_rear_offset=-.345)
        return EnhancedDWA(p,'ENHANCED_DWA','rectangle',ack_params=ap)
    walls=lambda gap,length=4.: [box(2,gap/2,length,.15),box(2,-gap/2,length,.15)]
    cases={
        'straight':((6,0),[],.2,0.),
        'curve':((5,2),[],.2,.12),
        'multi_static':((6,0),[box(2.5,0,.6,.6),box(3.5,1.2,.6,.6),box(4.5,-1.,.6,.6)],.2,0.),
        'long_wall':((6,0),[box(2.5,.3,.2,5.)],.2,0.),
        'u':((-3,0),[box(1.5,0,.3,3.),box(0,1.5,3.,.3),box(0,-1.5,3.,.3)],0.,0.),
        'thick_u':((-3,0),[box(1.7,0,.6,3.4),box(0,1.7,3.4,.6),box(0,-1.7,3.4,.6)],0.,0.),
        'narrow_u':((-3,0),[box(1.5,0,.15,1.6),box(0,.8,3.,.15),box(0,-.8,3.,.15)],0.,0.),
        'narrow_corridor':((5,0),walls(1.5),.2,0.),
        'crossing':((6,0),[box(2,1.8,.5,.5,1.,-math.pi/2)],.2,0.),
        'headon':((6,0),[box(3,0,.5,.5,1.,math.pi)],.2,0.),
        'rear_approach':((6,0),[box(-3,0,.5,.5,1.5,0.)],.2,0.),
        'unknown_accel_or_reversal':((6,0),[box(2.8,0,.5,.5,0.,0.,True)],.2,0.),
        'late_frame_compensated_snapshot':((6,0),[box(2.3,0,.5,.5,.8,math.pi)],.2,0.),
        'rear_blocked':((-3,0),[box(-1.1,0,.4,2.)],0.,0.),
        'far_tiles_128':((6,0),[box(3+(i%16)*.5,3+(i//16)*.3,.4,.4) for i in range(128)],.2,0.),
        'far_tiles_512':((6,0),[box(3+(i%16)*.5,3+(i//16)*.3,.4,.4) for i in range(512)],.2,0.),
    }
    if worstcase:
        near = [box(1.3+(i%32)*.015,(1. if i%2 else -1.)*(.9+(i//32)*.025),.04,.04) for i in range(512)]
        dense = [box(1.+(i%32)*.09,(1. if i%2 else -1.)*(1.15+(i//32)*.015),.04,.04) for i in range(512)]
        mixed = [box(2.+(i%32)*.08,(1. if i%2 else -1.)*(1.25+(i//32)*.025),.05,.05,
                     .3 if i%3==0 else 0.,0.) for i in range(512)]
        cases = {'near_tiles512':((6,0),near,.2,0.),
                 'dense_tiles512':((6,0),dense,.2,0.),
                 'mixed_tiles512':((6,0),mixed,.2,0.),
                 'reverse_near_tiles512':((6,0),near,-.3,.3)}
    reports=[]
    for name,(goal,obs,speed,steer) in cases.items():
        a=factory();a.current_delta=steer
        r=RobotState(v=speed)
        for i,o in enumerate(obs):o.id=i
        durations=[];outputs=[];failures=[]
        for repeat in range(repeats):
            before=time.perf_counter();out=a.plan(r,goal,obs);durations.append(time.perf_counter()-before)
            outputs.append(list(out));failures.append(getattr(a,'last_failure_reason','' if out[0] else 'no_safe_candidate'))
        out=outputs[-1]
        gap=None;heading_error=None;path_length=None;ttc=None
        if out[0]:
            tr,ts=a.reference_trajectory(out[1],out[2],r)
            path_length=float(np.linalg.norm(np.diff(np.vstack(([r.x,r.y,r.theta],tr))[:,:2],axis=0),axis=1).sum())
            desired=math.atan2(goal[1]-tr[-1,1],goal[0]-tr[-1,0]);motion=tr[-1,2]+(math.pi if out[1]<0 else 0)
            heading_error=abs((desired-motion+math.pi)%(2*math.pi)-math.pi)
            if obs:
                gaps=np.column_stack([a.cc.temporal_gaps(tr,ts,o,uncertainty_rate=.05) for o in obs])
                gap=float(gaps.min())
                known=all(not getattr(o,'velocity_unknown',False) for o in obs)
                hit=np.flatnonzero(np.min(gaps,axis=1)<=.2)
                ttc=float(ts[hit[0]]) if known and len(hit) else None
        counts={'candidate_count':getattr(a,'last_candidate_count',None),'feasible_count':getattr(a,'last_feasible_count',None),'checked_count':getattr(a,'last_checked_count',None)}
        reports.append({'case':name,'variant':variant,'samples':repeats,'input_obstacles':len(obs),'outputs':outputs,
            'failure_reasons':failures,'latency_ms':dict(zip(['p50','p95','p99','max'],[float(x)*1000 for x in [*np.percentile(durations,[50,95,99]),max(durations)]])),
            'cycle_deadline_100ms_misses':sum(t>.1 for t in durations),'planner_budget_80ms_misses':sum(t>.08 for t in durations),
            'minimum_selected_nominal_model_gap_m':gap,'selected_nominal_path_m':path_length,'selected_endpoint_heading_error_rad':heading_error,
            'model_first_margin_TTC_s':ttc,'mode_switches':0,'trap_entries':0,'counts':counts})
        print(json.dumps({'case':name,'variant':variant,'p50_ms':reports[-1]['latency_ms']['p50'],'command':out}),file=sys.stderr,flush=True)
    return reports


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--worker',action='store_true');parser.add_argument('--source',type=Path);parser.add_argument('--variant');parser.add_argument('--repeats',type=int,default=12);parser.add_argument('--worstcase',action='store_true')
    args=parser.parse_args()
    if args.worker:
        print(json.dumps(worker(args.source,args.variant,args.repeats,args.worstcase)));return
    output=ROOT/'docs/research'/('planner_worstcase.json' if args.worstcase else 'planner_benchmark.json')
    with tempfile.TemporaryDirectory(prefix='rda-planner-baseline-') as temporary:
        base=Path(temporary)
        with zipfile.ZipFile(ROOT/'docs/research/baseline_sources.zip') as archive:
            for name in archive.namelist():
                if name.startswith(('rda_dwa/rda_dwa/','rda_dwa_real/rda_dwa_real/')) and name.endswith('.py'):
                    archive.extract(name,base)
        reports=[]
        for variant in ('baseline','batch_only','full','full_budget'):
            source=base if variant=='baseline' else ROOT
            command=[sys.executable,str(Path(__file__).resolve()),'--worker','--source',str(source),'--variant',variant,'--repeats',str(args.repeats)]
            if args.worstcase:command.append('--worstcase')
            result=subprocess.run(command,capture_output=True,text=True,check=True)
            print(result.stderr,flush=True)
            reports.extend(json.loads(result.stdout))
    baseline={r['case']:r for r in reports if r['variant']=='baseline'}
    equality={variant:all(r['outputs']==baseline[r['case']]['outputs'] for r in reports if r['variant']==variant) for variant in ('batch_only','full')}
    payload={'date':'2026-11-06','scope':'pure Python synthetic model snapshots; no ROS2/DDS/vehicle/real logs',
             'quantile_limit':f'{args.repeats} repeated fixed snapshots per case; empirical p99 is not a calibrated tail bound',
             'equal_commands_against_frozen_baseline':equality,'results':reports}
    output.write_text(json.dumps(payload,indent=2)+'\n')
    assert all(equality.values()),equality


if __name__=='__main__':main()
