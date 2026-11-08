"""Reproduce fixes and compare recorded raw clouds; no ROS or vehicle I/O.

Requires NumPy and PyYAML. The saved baseline is used only for comparison.
Includes one real spawned perception worker driven by recorded clouds at 20 Hz.
"""
from dataclasses import fields
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np
import yaml

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),
             str(ROOT/'src/rda_dwa_real/test'),str(ROOT/'diagnostics/engineering_20261108')]
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams,point_cloud_xyz
from rda_dwa_real.async_perception import AsyncPerception,cloud_packet
from rda_dwa_real.kf_tracker import GlobalKFTracker
from test_campus_perception import road,returns,extrema_bridge,u_shape,enclosed,grid_at
from cloud_stats import decode_cloud


def configured_params(module,config):
    cfg=yaml.safe_load(config.read_text())['lidar_track_node']['ros__parameters']
    aliases={'max_range':'max_obstacle_range','height_min':'obstacle_height_min',
             'height_max':'obstacle_height_max','ground_enabled':'ground_filter_enable',
             'x_min':'pass_x_min','x_max':'pass_x_max','y_min':'pass_y_min','y_max':'pass_y_max',
             'z_min':'pass_z_min','z_max':'pass_z_max'}
    values={f.name:cfg[aliases.get(f.name,f.name)] for f in fields(module.PerceptionParams)
            if aliases.get(f.name,f.name) in cfg}
    return module.PerceptionParams(**values)


def load_baseline():
    path=OUT/'before/src/rda_dwa_real/rda_dwa_real/perception_core.py'
    spec=importlib.util.spec_from_file_location('rda_dwa_real.perception_before',path)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module


def fix_cases(old):
    sparse=np.column_stack((np.linspace(1.,5.,16),np.full(16,2.)))
    x,y=np.meshgrid(np.linspace(1.5,2.5,51),np.linspace(-.5,.5,51))
    platform=np.column_stack((x.ravel(),y.ravel()))
    result={}
    for name,xy in [('raw_connectivity',extrema_bridge()),('sparse_structure',sparse),
                    ('small_u',u_shape()),('platform',platform)]:
        scene=np.vstack((road(),returns(xy)))
        before,bm=old.LidarPerceptionCore().process(scene,100.)
        after,am=LidarPerceptionCore().process(scene,100.)
        assert am['valid'] and enclosed(xy,after).all(),name
        if name=='small_u':assert not enclosed(np.array([[0.,0.]]),after).any()
        result[name]=dict(before_detections=before,after_detections=after,
                          before_count=len(before),after_count=len(after),
                          all_raw_supports_covered=True,ground_plane=am['ground_plane'])
    x,y=np.meshgrid(np.linspace(-4.,4.,81),np.linspace(-4.,4.,81))
    for sign in (-1.,1.):
        xyz=np.column_stack((x.ravel(),y.ravel(),(-.81+np.where(np.abs(y)<=.8,0.,sign*.1)).ravel()))
        d,m=LidarPerceptionCore().process(xyz,100.)
        assert m['valid'] and d
        g=m['drivable_grid'];assert grid_at(g,(2.,0.))==0
        assert grid_at(g,(2.,2.))==grid_at(g,(2.,-2.))==100
        result['ten_cm_raised_sidewalk' if sign>0 else 'ten_cm_dropoff']=dict(
            ground_plane=m['ground_plane'],detection_count=len(d),road_cell=0,
            left_sidewalk_cell=100,right_sidewalk_cell=100,unseen_cell=grid_at(g,(5.,5.)))
    return result


def raw_clouds(limit=100):
    bag=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
    with sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True) as db:
        topic=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
        rows=db.execute('select data from messages where topic_id=? order by timestamp desc limit ?', (topic,limit)).fetchall()[::-1]
    return [decode_cloud(row[0]) for row in rows]


def core_profile(core,clouds):
    durations=[];counts=[];reasons={};queries=[]
    for msg in clouds:
        xyz=point_cloud_xyz(msg);stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        started=time.monotonic();detections,metrics=core.process(xyz,stamp);durations.append(time.monotonic()-started)
        counts.append(len(detections));queries.append(metrics.get('exact_distance_tests',0))
        reasons[metrics['reason']]=reasons.get(metrics['reason'],0)+1
    return dict(frames=len(clouds),reasons=reasons,detection_range=[min(counts),max(counts)],
                duration_p50_ms=float(np.median(durations)*1000),duration_p95_ms=float(np.percentile(durations,95)*1000),
                duration_max_ms=float(max(durations)*1000),exact_distance_tests_max=max(queries),
                last_ground_plane=metrics['ground_plane'])


def worker_profile(params,clouds):
    worker=AsyncPerception(params,GlobalKFTracker(dt=.05))
    packets=[cloud_packet(msg) for msg in clouds[-30:]];samples=[];sequence=0
    begin=time.monotonic();next_frame=begin;submit_durations=[]
    try:
        while time.monotonic()-begin<3.5:
            now=time.monotonic()
            if now>=next_frame:
                sequence+=1;packet=dict(packets[(sequence-1)%len(packets)])
                stamp=777000.+now-begin;packet.update(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))
                started=time.monotonic();worker.submit((sequence,now,packet,(0.,0.,0.),dict(source_age_at_receive_s=.01)))
                submit_durations.append(time.monotonic()-started);next_frame=now+.05
            result=worker.poll()
            if result is not None:
                seq,received,_,_,tracks,m=result;age=.01+time.monotonic()-received
                samples.append(dict(sequence=seq,source_age_s=age,duration_s=m['worker_duration_s'],
                                    reason=m['reason'],tracks=len(tracks) if tracks is not None else None,
                                    eligible=tracks is not None and age<=.4))
            time.sleep(.005)
    finally:worker.close()
    warm=[r for r in samples if r['sequence']>10]
    return dict(scope='Local spawned worker: parsing, ground, exact clusters, grid, KF, queues; excludes ROS/DDS and Jetson CPU.',
                input_frames=sequence,results=len(samples),warm_eligible_fraction=sum(r['eligible'] for r in warm)/len(warm) if warm else 0.,
                warm_source_age_p95_ms=float(np.percentile([r['source_age_s'] for r in warm],95)*1000) if warm else None,
                worker_duration_p95_ms=float(np.percentile([r['duration_s'] for r in warm],95)*1000) if warm else None,
                submit_duration_p95_ms=float(np.percentile(submit_durations,95)*1000),samples=samples)


def main():
    old=load_baseline();current=sys.modules['rda_dwa_real.perception_core']
    cfg=ROOT/'src/rda_dwa_real/config/real_vehicle.yaml'
    params=configured_params(current,cfg);old_params=configured_params(old,OUT/'before/src/rda_dwa_real/config/real_vehicle.yaml')
    clouds=raw_clouds();cases=fix_cases(old)
    comparison=dict(before=core_profile(old.LidarPerceptionCore(old_params),clouds),
                    after=core_profile(LidarPerceptionCore(params),clouds),
                    scope='Same last 100 recorded raw clouds, fixed pose, respective production configurations. Core profile excludes parsing/KF/ROS/DDS; no truth-labelled obstacles.')
    (OUT/'raw_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    worker=worker_profile(params,clouds)
    sources=json.loads((OUT/'before_sha256.json').read_text())
    report=dict(date='2026-11-08',cases=cases,raw_comparison=comparison,worker=worker,
                source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(cases={k:{a:b for a,b in v.items() if a not in ('before_detections','after_detections')} for k,v in cases.items()},
                         raw_comparison=comparison,worker={k:v for k,v in worker.items() if k!='samples'}),indent=2))


if __name__=='__main__':main()
