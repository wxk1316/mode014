"""Actual closed-loop traces and mode timeline, not planned paths."""
import json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from campaign import OUT,corners
cases=json.loads((OUT/'scenario_expectations.json').read_text());case_map={c['name']:c for c in cases}
colors={0:'#247c62',1:'#217bb4',4:'#8b56a4'}
plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})


def figure(names,path):
 fig,axes=plt.subplots(math.ceil(len(names)/2),2,figsize=(13,3.3*math.ceil(len(names)/2)),layout='constrained',squeeze=False)
 for ax,name in zip(axes.flat,names):
  case=case_map[name];rows=json.loads((OUT/'candidate_31'/f'{name}_trace.json').read_text());result=json.loads((OUT/'candidate_31'/f'{name}_result.json').read_text())
  for ob in case['objects']:
   ax.add_patch(Polygon(corners(ob['x'],ob['y'],ob['heading'],ob['length'],ob['width']),facecolor='#b7bdc4',edgecolor='#6b7179',alpha=.7))
   if abs(ob['vx'])+abs(ob['vy'])>.01:
    ax.annotate('',xy=(ob['x']+4*ob['vx'],ob['y']+4*ob['vy']),xytext=(ob['x'],ob['y']),arrowprops={'arrowstyle':'->','color':'#bd4c4c','lw':1.7})
  baseline=OUT/'baseline_31'/f'{name}_trace.json'
  if baseline.exists():
   old=json.loads(baseline.read_text());ax.plot([r['x'] for r in old],[r['y'] for r in old],ls='--',color='#a0a4aa',lw=1.3,label='ACK+SPIN baseline')
  for mode in (0,1,4):
   for i in range(len(rows)-1):
    if rows[i]['mode']==mode:ax.plot([rows[i]['x'],rows[i+1]['x']],[rows[i]['y'],rows[i+1]['y']],color=colors[mode],lw=2)
  for row in rows[::max(1,len(rows)//9)]:
   ax.add_patch(Polygon(corners(row['x'],row['y'],row['yaw'],1.2,.8),fill=False,edgecolor=colors[row['mode']],lw=.65,alpha=.35))
  ax.scatter([0],[0],s=35,color='#303641',label='Start',zorder=8);ax.scatter([case['goal'][0]],[case['goal'][1]],marker='*',s=95,color='#df9e22',label='Goal',zorder=8)
  ax.axhline(2,color='#53606b',lw=1);ax.axhline(-2,color='#53606b',lw=1)
  ax.set_xlim(min(-3,min(r['x'] for r in rows)-.8),6);ax.set_ylim(-2.2,2.2);ax.set_aspect('equal');ax.grid(alpha=.12)
  status='reached' if result['reached'] else 'stopped / incomplete'
  ax.set_title(f"{name}: {status}, {result['duration_s']:.1f}s, gap {result['min_gap_m']:.3f}m\n{len(result['switches'])} switches; {result['spin_count']} spins")
  ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)')
 for ax in list(axes.flat)[len(names):]:ax.set_visible(False)
 from matplotlib.lines import Line2D
 handles=[Line2D([0],[0],color=colors[m],lw=2,label=label) for m,label in [(0,'ACK'),(1,'CRAB'),(4,'SPIN')]]
 handles.append(Line2D([0],[0],color='#a0a4aa',ls='--',label='Baseline'))
 fig.legend(handles=handles,loc='outside lower center',ncol=4)
 fig.suptitle('Closed-loop simulation: actual vehicle-centre motion\n4 m road, 1.2 x 0.8 m vehicle; real C++ timing policy, simulated worker delay 0.25 s',fontsize=12)
 fig.savefig(OUT/path,dpi=150);plt.close(fig)

figure(['pedestrian_0.5','car_1.0','single_barrier','staggered_barriers','u_window','lateral_goal'],'requested_scenarios.png')
figure(['campus_bollards','campus_bench','campus_bins','campus_planter','campus_chicane','campus_wall_gap'],'campus_scenarios.png')
fig,axes=plt.subplots(3,1,figsize=(11,7),layout='constrained')
for name in ['car_1.0','u_window','lateral_goal']:
 rows=json.loads((OUT/'candidate_31'/f'{name}_trace.json').read_text());t=[r['t'] for r in rows]
 axes[0].plot(t,[r['v'] for r in rows],label=name);axes[1].step(t,[r['mode'] for r in rows],where='post',label=name);axes[2].plot(t,[r['gap'] for r in rows],label=name)
axes[0].set_ylabel('Signed speed (m/s)');axes[1].set_ylabel('Mode ID');axes[1].set_yticks([0,1,4],['ACK 0','CRAB 1','SPIN 4']);axes[2].set_ylabel('Physical gap (m)');axes[2].axhline(.12,ls='--',color='#bd4c4c',label='Required 0.12 m');axes[2].set_xlabel('Simulation time (s)')
for ax in axes:ax.grid(alpha=.2);ax.legend(loc='upper right')
fig.suptitle('Actual commands, modes and independently evaluated clearance');fig.savefig(OUT/'mode_timeline.png',dpi=150);plt.close(fig)
print('Saved 3 verified-source trace figures')
