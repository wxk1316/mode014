"""Render actual saved before/after boxes and a synthetic campus surface grid."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon,Rectangle,Patch
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path[:0]=[str(ROOT/'src/rda_dwa_real'),str(ROOT/'src/rda_dwa'),str(ROOT/'src/rda_dwa_real/test')]
from test_campus_perception import u_shape
from rda_dwa_real.perception_core import LidarPerceptionCore

report=json.loads((OUT/'validation.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(13,4.3),layout='constrained')
points=u_shape();case=report['cases']['small_u']
for ax,key,title in zip(axes[:2],['before_detections','after_detections'],['Before: U cavity filled','After: observed walls retained']):
    ax.scatter(points[:,0],points[:,1],s=8,color='#286090')
    for cx,cy,_,length,width,angle in case[key]:
        c,s=np.cos(angle),np.sin(angle)
        corners=np.array([[-1,-1],[-1,1],[1,1],[1,-1]])*np.array([length,width])/2
        corners=corners@np.array([[c,s],[-s,c]])+np.array([cx,cy])
        ax.add_patch(Polygon(corners,facecolor='#efa064',edgecolor='#b84d1c',alpha=.35))
    ax.add_patch(Rectangle((-.6,-.4),1.2,.8,facecolor='#455566',edgecolor='black',alpha=.6))
    ax.set(xlim=(-1.4,1.4),ylim=(-1.4,1.4),title=title,xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)
x,y=np.meshgrid(np.linspace(-4.,4.,81),np.linspace(-4.,4.,81))
cloud=np.column_stack((x.ravel(),y.ravel(),(-.81+np.where(np.abs(y)<=.8,0.,.1)).ravel()))
_,metrics=LidarPerceptionCore().process(cloud,100.)
grid=metrics['drivable_grid'];values=np.array(grid['data']).reshape(grid['height'],grid['width'])
colours=np.where(values<0,0,np.where(values==0,1,2));origin=grid['origin_x'];end=origin+grid['width']*grid['resolution']
axes[2].imshow(colours,origin='lower',extent=(origin,end,origin,end),cmap=ListedColormap(['#e2e6eb','#77bd98','#efa064']),vmin=0,vmax=2,interpolation='nearest')
axes[2].add_patch(Rectangle((-.6,-.4),1.2,.8,facecolor='#455566',edgecolor='black'))
axes[2].set(xlim=(-4.5,4.5),ylim=(-4.5,4.5),title='10 cm roadside step (synthetic)',xlabel='x (m)',ylabel='y (m)')
axes[2].legend(handles=[Patch(color='#77bd98',label='Observed road'),Patch(color='#efa064',label='Raised / blocked'),Patch(color='#e2e6eb',label='Unknown')],loc='upper right',fontsize=8)
fig.savefig(OUT/'campus_perception.png',dpi=170)
print(OUT/'campus_perception.png')
