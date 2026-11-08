import sys,sqlite3,math,json
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(Path(__file__).parent)]
from cloud_stats import decode_cloud,xyz
from rda_dwa_real.perception_core import LidarPerceptionCore
p=ROOT/'record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3'
with sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True) as db:
 ident=db.execute('select id from topics where name=?',('/rslidar_points',)).fetchone()[0]
 _,data=db.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit 1',(ident,)).fetchone()
cloud=decode_cloud(data);points=xyz(cloud);core=LidarPerceptionCore();d,m=core.process(points,0.)
points=points[np.linalg.norm(points[:,:2],axis=1)<6];height=points[:,2]+.45-(points[:,:2]@core.plane[:2]+core.plane[2]);obj=points[(height>=.12)&(height<=2.2)]
fig,ax=plt.subplots(figsize=(9,8));ax.scatter(obj[:,0],obj[:,1],s=1,c=obj[:,2],cmap='viridis')
for i,(x,y,t,L,W,h) in enumerate(d):
 c,s=math.cos(h),math.sin(h);corners=np.array([[L/2,W/2],[-L/2,W/2],[-L/2,-W/2],[L/2,-W/2]])@np.array([[c,s],[-s,c]])+np.array([x,y])
 ax.add_patch(Polygon(corners,fill=False,color='red',lw=.6));ax.text(x,y,str(i),fontsize=7)
ax.add_patch(Polygon([[-.6,-.4],[.6,-.4],[.6,.4],[-.6,.4]],color='grey'));ax.set(xlim=(-6,6),ylim=(-6,6),xlabel='body x [m]',ylabel='body y [m]',title='Historical lidar supports and fitted boxes');ax.set_aspect('equal');ax.grid();fig.savefig(Path(__file__).with_name('cloud_boxes.png'),dpi=140)
Path(__file__).with_name('cloud_boxes.json').write_text(json.dumps({'detections':d,'metrics':m},indent=2))
