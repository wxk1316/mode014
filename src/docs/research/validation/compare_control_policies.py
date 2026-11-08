"""Control-only counterfactuals from saved baseline sources and identical inputs.

Planner/perception dependencies are the current versions for BOTH policies. This
is a module ablation, not a replacement for the full-stack baseline scenarios.
No ROS, UDP, hardware, learned predictor or independent collision oracle is used.
"""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
import numpy as np
from test_planner_safety import planner, fresh
from rda_dwa.ackermann_model import AckermannModel
from rda_dwa_real import vehicle_bridge as patched_bridge, traj_tracker_core as patched_tracker


def baseline_module(relative,name):
    with zipfile.ZipFile(ROOT/'docs/research/baseline_sources.zip') as archive:
        source=archive.read(relative)
    spec=importlib.util.spec_from_loader(name,loader=None)
    module=importlib.util.module_from_spec(spec)
    exec(compile(source,'saved_baseline/'+relative,'exec'),module.__dict__)
    return module


base_bridge=baseline_module('rda_dwa_real/rda_dwa_real/vehicle_bridge.py','rda_dwa_real._baseline_bridge')
base_tracker=baseline_module('rda_dwa_real/rda_dwa_real/traj_tracker_core.py','rda_dwa_real._baseline_tracker')


def make_bridge(module):
    p,ap=planner();cfg=module.BridgeConfig();cfg.ack_guard_time=.2;cfg.blocked_duration=.5
    if hasattr(cfg,'static_search_enable'):cfg.static_search_enable=False
    c=module.VehicleBridgeCore(p,p.params,config=cfg,ack_model=AckermannModel(ap))
    c.set_goal(6.,0.)
    return c


def wall_trace(module,x):
    c=make_bridge(module);latency=[];requests=[];target=None
    dets=[(x,0.,'static_large',.4,2.,0.,0.,1),(-x,0.,'static_large',.4,2.,0.,0.,2)]
    for i in range(25):
        now=i*.1;fresh(c,now,dets)
        if target is not None:c.set_bridge_state(1)
        started=time.perf_counter();info=c.step(now=now);latency.append(time.perf_counter()-started)
        if info['traj_mode']==1 and target is None:
            target=info['spin_target'];requests.append({'time_s':now,'target_rad':target})
    percentiles=np.percentile(latency,[50,95,99])
    return {'spin_sessions':len(requests),'trap_active':c.trap['active'],
        'stall_class':getattr(c,'_stall_class','unclassified'), 'requests':requests,
        'latency_ms':dict(zip(['p50','p95','p99'],map(float,percentiles*1000))),
        'maximum_ms':max(latency)*1000,'deadline_misses_100ms':sum(v>.1 for v in latency)}


def trap_memory(module):
    c=make_bridge(module);c._activate_trap(c.robot,[],c._goal)
    c._spin_attempts=2;c._spin_anchor=(0,0);c.robot.y=.7;c._t_now=10.
    c._arbitration_step(c.robot,[],6,0,.1,blocked=False)
    allowance=c._spin_attempts
    c=make_bridge(module);c._activate_trap(c.robot,[],c._goal)
    c.robot.x=.6;c.trap['sub_goal']=(.6,0);c._t_now=2.;c._update_trap([],0.1)
    c._t_now=3.2;c._update_trap([],1.2)
    return {'attempts_after_internal_lateral_drift':allowance,
            'trap_released_without_rejoin_motion':not c.trap['active']}


def stop_gates(module):
    p=module.TrajTrackerParams()
    if hasattr(p,'spin_require_stopped_feedback'):p.spin_require_stopped_feedback=True
    c=module.TrajTrackerCore(p);c.set_traj_mode(1)
    first_mode4=None
    for i in range(100):
        now=(i+1)*.02;c.set_mode_ready(0,now);c.set_speed_feedback(.2,now)
        out=c.step(0.,0.,0.,[(0.,0.,0.)]*60,math.pi/2,now,.02,0)
        if out['turnmode']==4 and first_mode4 is None:first_mode4=now
    c=module.TrajTrackerCore()
    c._last_drive_direction=1
    reverse_nonzero=False
    for i in range(25):
        now=(i+1)*.02;c._control_now=now
        if hasattr(c,'_stop_feedback'):c.set_speed_feedback(.025,now,stop_speed=.1)
        else:c.set_speed_feedback(.025,now)
        reverse_nonzero |= c._ramp_speed(-.2,.02)<0
    return {'mode4_requested_while_measured_speed_point2':first_mode4,
            'reverse_allowed_with_one_wheel_point1':bool(reverse_nonzero)}


def main():
    result={'scope':'control-only module ablation; same current planner/perception; synthetic inputs; no ROS/DDS/UDP or independent collision oracle',
            'baseline_archive':'docs/research/baseline_sources.zip','results':{}}
    for name,b,t in [('baseline_control',base_bridge,base_tracker),('patched_control',patched_bridge,patched_tracker)]:
        result['results'][name]={'executable_but_unmoving_walls_1_32m':wall_trace(b,1.32),
            'true_static_blockage_walls_1_14m':wall_trace(b,1.14),
            'episode_memory':trap_memory(b),'physical_stop':stop_gates(t)}
    out=ROOT/'docs/research/control_module_comparison.json'
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    baseline=result['results']['baseline_control'];patched=result['results']['patched_control']
    assert baseline['executable_but_unmoving_walls_1_32m']['spin_sessions']==1
    assert patched['executable_but_unmoving_walls_1_32m']['spin_sessions']==0
    assert patched['true_static_blockage_walls_1_14m']['spin_sessions']==1
    assert patched['episode_memory']['attempts_after_internal_lateral_drift']==2
    assert not patched['episode_memory']['trap_released_without_rejoin_motion']
    assert patched['physical_stop']['mode4_requested_while_measured_speed_point2'] is None
    assert not patched['physical_stop']['reverse_allowed_with_one_wheel_point1']


if __name__=='__main__':main()
