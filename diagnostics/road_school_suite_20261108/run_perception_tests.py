"""Synthetic static clouds through current filtering/clustering, then KF+loop.
The initial snapshot's detections are repeated in map for the closed loop.
This isolates shape approximation; does not simulate viewpoint/occlusion changes.
"""
import sys,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(OUT),str(ROOT/'src/rda_dwa_real')]
from run_suite import Box,definitions,corners,exact_gaps,run,CFG
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams

def cloud(objects):
 x,y=np.meshgrid(np.arange(-6.,6.01,.15),np.arange(-6.,6.01,.15));parts=[np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.81)))]
 for o in objects:
  height=.5 if 'cone' in o.name else (.6 if o.name=='bench' else (1.8 if o.name in ('wall','u_side','u_back','tree','pillar') else .8))
  for h in np.linspace(.14,height,5):
   shrink=max(0.,1.-h/.5) if 'cone' in o.name else 1.
   length=o.length*shrink;width=o.width*shrink
   nx=max(3,math.ceil(o.length/.06)+1);ny=max(3,math.ceil(o.width/.06)+1)
   points=np.vstack((np.column_stack((np.linspace(-length/2,length/2,nx),np.full(nx,-width/2))),
                     np.column_stack((np.linspace(-length/2,length/2,nx),np.full(nx,width/2))),
                     np.column_stack((np.full(ny,-length/2),np.linspace(-width/2,width/2,ny))),
                     np.column_stack((np.full(ny,length/2),np.linspace(-width/2,width/2,ny)))))
   c,s=math.cos(o.heading),math.sin(o.heading);xy=points@np.array([[c,s],[-s,c]])+np.array([o.x,o.y])
   parts.append(np.column_stack((xy,np.full(len(xy),-.81+h))))
 return np.vstack(parts)

p=CFG['lidar_track_node']['ros__parameters']
params=PerceptionParams(max_range=p['max_obstacle_range'],y_min=p['pass_y_min'],y_max=p['pass_y_max'],voxel_size=p['voxel_size'],
 cluster_gate=p['cluster_gate'],cluster_min_points=p['cluster_min_points'],cluster_min_extent=p['cluster_min_extent'],
 structural_length=p['structural_length'],structure_tile_size=p['structure_tile_size'],height_min=p['obstacle_height_min'],height_max=p['obstacle_height_max'])
rows=[];results=[]
for case in definitions():
 if case['group']=='dynamic':continue
 points=cloud(case['objects']);det,metrics=LidarPerceptionCore(params).process(points,100.,(0.,0.,0.))
 detected=[Box(q[0],q[1],q[3],q[4],q[5],name='perception_cluster') for q in det]
 body=corners(0.,0.,0.,1.2,.8)
 initial_gap=float(exact_gaps(body,[corners(q.x,q.y,q.heading,q.length,q.width) for q in detected]).min()) if detected else None
 row={'name':case['name'],'raw_points':len(points),'physical_object_count':len(case['objects']),'detection_count':len(det),
  'detections':det,'metrics':metrics,'initial_reported_gap_m':initial_gap,'initial_false_occupied':initial_gap is not None and initial_gap<0.}
 rows.append(row)
 if not metrics['valid']:print(json.dumps({'name':case['name'],'perception_error':metrics}),flush=True);continue
 test=dict(case,detections_override=detected,input_mode='cluster_snapshot')
 result=run(test,seed=19,noise=False,warm=False,duration=65.,boundary_model='tiled');results.append(result)
 print(json.dumps({'perception':{k:v for k,v in row.items() if k not in ('metrics','detections')},'result':result}),flush=True)
(OUT/'perception_snapshots.json').write_text(json.dumps(rows,indent=2));(OUT/'perception_closed_loop_results.json').write_text(json.dumps(results,indent=2))
