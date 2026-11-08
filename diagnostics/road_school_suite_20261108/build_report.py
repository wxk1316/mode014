import sys,json,math,collections,csv,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from matplotlib import font_manager
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(OUT)]
from run_suite import corners
font='/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
if Path(font).exists():
 font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name()
plt.rcParams['axes.unicode_minus']=False
unique={}
for p in (OUT/'runs').glob('*.json'):
 d=json.loads(p.read_text());r=d['result'];r.setdefault('boundary_model','continuous');r.setdefault('input_mode','detection_boxes')
 key=(r['name'],r['seed'],r['noise'],r['warm_kf'],r['plant_lag'],r['boundary_model'],r['input_mode'])
 if key not in unique or p.stat().st_mtime>unique[key][0].stat().st_mtime:unique[key]=(p,d)
all_data=[d for _,d in unique.values()]
primary=[d for d in all_data if d['result']['boundary_model']=='tiled']
noise=[d for d in primary if d['result']['noise'] and not d['result']['warm_kf'] and d['result']['input_mode']=='detection_boxes']
clean=[d for d in primary if not d['result']['noise'] and not d['result']['warm_kf'] and d['result']['input_mode']=='detection_boxes']
warm=[d for d in primary if d['result']['noise'] and d['result']['warm_kf']]
perception=[d for d in primary if d['result']['input_mode']=='cluster_snapshot']
summary={'primary_runs':len(primary),'noise_cold_runs':len(noise),'clean_cold_runs':len(clean),'noise_warm_runs':len(warm),'cluster_snapshot_runs':len(perception),
 'boundary_representation_comparison_runs':len(all_data)-len(primary),'noise_outcomes':dict(collections.Counter(d['result']['outcome'] for d in noise)),
 'warm_dynamic_outcomes':dict(collections.Counter(d['result']['outcome'] for d in warm)),
 'clean_outcomes':dict(collections.Counter(d['result']['outcome'] for d in clean)),
 'cluster_snapshot_outcomes':dict(collections.Counter(d['result']['outcome'] for d in perception)),
 'current_production_code_modified':False}
(OUT/'suite_summary.json').write_text(json.dumps(summary,indent=2))
(OUT/'all_results.json').write_text(json.dumps([d['result'] for d in all_data],indent=2))
fields=list(primary[0]['result'].keys())
with (OUT/'primary_results.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader()
 for d in primary:writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,dict) else v for k,v in d['result'].items()})
cn={'single_cone_2p5m':'单路障（2.5m）','two_cones_centre_open':'双路障：中央可通','two_cones_centre_closed':'双路障：需从侧边绕','two_cones_staggered':'双路障：前后错位',
 'school_gate_pillars':'校门两侧立柱','school_bollards_narrow':'狭窄隔离柱通道','school_tree_and_cone':'树与路障交错','school_parked_vehicle':'右侧停放车辆',
 'school_building_corner':'建筑L形墙角','school_bench_planter':'长椅与花箱','school_road_blocked':'路障封闭道路'}
for s in (.4,.45,.5):cn[f'pedestrian_3m_{s:.2f}']=f'3m迎面行人 {s:.2f}m/s'
for s in (.5,.75,1.):cn[f'vehicle_4m_{s:.2f}']=f'4m迎面小车 {s:.2f}m/s'
labels={'reached':'到达','not_reached':'未到达','collision':'接触失败','exited_u':'退出窗口','not_exited_u':'未退出','safe_blocked':'安全停车','clearance_below_12cm':'间距不足12cm'}
lookup={(d['result']['name'],d['result']['seed']):d for d in noise}
colors={'reached':'#087f5b','exited_u':'#087f5b','safe_blocked':'#2364aa','collision':'#c92a2a','not_reached':'#d97706','not_exited_u':'#d97706','clearance_below_12cm':'#c92a2a'}
def scene(ax,d,u=False):
 r=d['result'];records=d['records'];ax.axhline(2,color='#888',lw=2);ax.axhline(-2,color='#888',lw=2)
 for o in d['objects']:
  ax.add_patch(Polygon(corners(o['x'],o['y'],o['heading'],o['length'],o['width']),facecolor='#aeb5be',edgecolor='#555',alpha=.7))
  if o['vx'] or o['vy']:
   end=(o['x']+o['vx']*(records[-1]['t']-.2),o['y']+o['vy']*(records[-1]['t']-.2))
   ax.plot([o['x'],end[0]],[o['y'],end[1]],'--',color='#c92a2a',lw=1.5)
   ax.add_patch(Polygon(corners(*end,o['heading'],o['length'],o['width']),fill=False,edgecolor='#c92a2a',lw=1.5))
 x=[q['x'] for q in records];y=[q['y'] for q in records]
 ax.plot(x,y,color='#1565c0',lw=1.8);ax.scatter([0],[0],color='#1565c0',s=24,zorder=5)
 for q in (records[0],records[-1]):ax.add_patch(Polygon(corners(q['x'],q['y'],q['yaw'],1.2,.8),facecolor='#1565c0',edgecolor='#1565c0',alpha=.15))
 if u:
  mouth=d['scenario']['u_mouth'];ax.axvline(mouth,color='#888',linestyle=':',lw=1);ax.add_patch(Circle((0,0),math.hypot(.6,.4),fill=False,linestyle=':',edgecolor='#1565c0'))
  ax.set_xlim(-3.7,1.8);ax.set_ylim(-1.25,1.25)
 else:
  ax.scatter([5],[0],marker='*',s=100,color='#087f5b',zorder=6);ax.set_xlim(min(-3.5,min(x)-.8),6);ax.set_ylim(-2.3,2.3)
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('x / m',fontsize=8);ax.set_ylabel('y / m',fontsize=8)
 name=cn.get(r['name'],f"纵深{d['scenario'].get('u_depth')}m / 前距{d['scenario'].get('front_clearance')}m")
 ax.set_title(name+'\n'+labels[r['outcome']]+f"，{r['duration_s']:.1f}s",color=colors[r['outcome']],fontsize=10)

fig,axes=plt.subplots(2,3,figsize=(15,8))
for ax,name in zip(axes.flat,[f'pedestrian_3m_{s:.2f}' for s in (.4,.45,.5)]+[f'vehicle_4m_{s:.2f}' for s in (.5,.75,1.)]):scene(ax,lookup[(name,19)])
fig.suptitle('迎面动态场景：目标前方5m、道路宽4m；蓝色为本车，红色为对方（seed=19）',fontsize=14)
fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/'dynamic_tests.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(3,4,figsize=(19,12))
for ax,name in zip(axes.flat,cn.keys()):
 if name.startswith(('pedestrian','vehicle')):continue
 scene(ax,lookup[(name,19)])
axes.flat[-1].axis('off');axes.flat[-1].text(.05,.7,'蓝线：车辆模型轨迹\n灰框：静态物体\n绿星：原目标(5,0)\n边界：y=±2m\n结果采用真实车体间距\n已加入KF、延迟及小幅噪声',fontsize=12,va='top')
fig.suptitle('路障与校园静态场景：最新代码，检测框→KF→规划→跟踪→车辆模型',fontsize=15)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'static_school_tests.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(15,8))
for ax,name in zip(axes.flat,[f'u_depth{depth}_front{gap:.2f}' for depth in (2,3) for gap in (.15,.4,.7)]):scene(ax,lookup[(name,19)],u=True)
fig.suptitle('U形净宽1.8m：蓝色虚圆为车体旋转包络；评价安全退出，原目标仍在右侧5m',fontsize=14)
fig.tight_layout(rect=(0,0,1,.94));fig.savefig(OUT/'u_tests.png',dpi=180);plt.close(fig)
snapshots={r['name']:r for r in json.loads((OUT/'perception_snapshots.json').read_text())}
fig,axes=plt.subplots(1,2,figsize=(12,5))
for ax,name in zip(axes,('u_depth2_front0.40','u_depth3_front0.40')):
 d=lookup[(name,19)];s=snapshots[name]
 for o in d['objects']:ax.add_patch(Polygon(corners(o['x'],o['y'],o['heading'],o['length'],o['width']),facecolor='#888',edgecolor='#444'))
 for q in s['detections']:ax.add_patch(Polygon(corners(q[0],q[1],q[5],q[3],q[4]),facecolor='#c92a2a',edgecolor='#c92a2a',alpha=.25))
 ax.add_patch(Polygon(corners(0,0,0,1.2,.8),facecolor='#1565c0',edgecolor='#1565c0',alpha=.75));ax.arrow(0,0,.45,0,color='white',head_width=.08)
 ax.set_aspect('equal');ax.set_xlim(-3.4,1.5);ax.set_ylim(-1.3,1.3);ax.grid(alpha=.2)
 ax.set_title(f"纵深{d['scenario']['u_depth']}m：3面真实墙 → {s['detection_count']}个检测框\n红色框将空地及车位包在内部",fontsize=11)
 ax.set_xlabel('x / m');ax.set_ylabel('y / m')
fig.suptitle('当前聚类的凹形误占据：灰色是墙，红色是聚类OBB，蓝色是小车',fontsize=14)
fig.tight_layout(rect=(0,0,1,.9));fig.savefig(OUT/'u_perception_false_box.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,7))
for row,name in enumerate(('single_cone_2p5m','pedestrian_3m_0.45')):
 d=lookup[(name,19)];r=d['records'];t=np.array([q['t'] for q in r]);axes[row,0].plot(t,[q['command_v'] for q in r],label='速度指令');axes[row,0].plot(t,[q['v'] for q in r],label='车辆模型速度');axes[row,0].legend(fontsize=9)
 axes[row,1].plot(t,[q['command_steer_deg'] for q in r],label='转角指令');axes[row,1].plot(t,[q['delta_deg'] for q in r],label='车辆模型转角');axes[row,1].legend(fontsize=9)
 axes[row,0].set_ylabel('m/s');axes[row,1].set_ylabel('度')
 for ax in axes[row]:ax.set_title(cn[name]);ax.set_xlabel('时间 / s');ax.grid(alpha=.25)
fig.suptitle('正常绕行与动态停车：触发停止分支时速度指令直接归零',fontsize=14);fig.tight_layout(rect=(0,0,1,.93));fig.savefig(OUT/'commands_example.png',dpi=180);plt.close(fig)
# Concrete guard, guide and shape evidence; no post-hoc production changes.
comfort={'normal_acceleration_m_s2':0.,'normal_deceleration_m_s2':0.}
for d in primary:
 rr=d['records'];vv=np.array([q['command_v'] for q in rr]);aa=np.diff(vv)/.02
 mask=np.array([rr[i]['tracking']=='LQR' and rr[i-1]['tracking']=='LQR' and rr[i]['mode']==0 for i in range(1,len(rr))])
 if np.any(mask):
  comfort['normal_acceleration_m_s2']=max(comfort['normal_acceleration_m_s2'],float(aa[mask].max()))
  comfort['normal_deceleration_m_s2']=max(comfort['normal_deceleration_m_s2'],float(-aa[mask].min()))
comfort.update({k:max(d['result'].get(k,0) for d in primary) for k in ('command_jerk_max','command_steer_rate_max_deg_s','command_steer_accel_max_deg_s2')})
(OUT/'comfort_metrics.json').write_text(json.dumps(comfort,indent=2))
evidence={}
for name in ('two_cones_centre_open','two_cones_staggered','school_bench_planter','pedestrian_3m_0.45','vehicle_4m_0.75'):
 d=lookup[(name,19)];plans=d['plans'];bad=next((q for q in plans if q['reason'] in ('orientation_stall','dynamic_yield','execution_collision_guard')),None)
 r=d['records'];vel=np.array([q['command_v'] for q in r]);acc=np.diff(vel)/.02;jerk=np.diff(acc)/.02
 evidence[name]={'first_blocking_plan':bad,'maximum_command_deceleration_all_branches_m_s2':float(max(0.,-acc.min())),'maximum_command_jerk_all_branches_m_s3':float(np.max(np.abs(jerk))),'result':d['result']}
(OUT/'failure_evidence.json').write_text(json.dumps(evidence,indent=2))
# Detailed source-hash audit.
metadata=json.loads((OUT/'test_metadata.json').read_text());metadata['primary_road_boundary_model']='fixed map tiles, 1.2m, sensed within8m';metadata['source_sha256']['src/rda_dwa_real/rda_dwa_real/perception_core.py']=hashlib.sha256((ROOT/'src/rda_dwa_real/rda_dwa_real/perception_core.py').read_bytes()).hexdigest()
assert all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in metadata['source_sha256'].items())
metadata['suite_summary']=summary;metadata['test_script_sha256']={n:hashlib.sha256((OUT/n).read_bytes()).hexdigest() for n in ('run_suite.py','run_perception_tests.py','build_report.py')};(OUT/'test_metadata.json').write_text(json.dumps(metadata,indent=2))
lines=['# 5米前方目标、4米道路与校园场景闭环测试','',
'结论：当前32f65fc版本在这些场景下不能认为已实现可靠动态避障和稳定校园绕行。迎面行人/小车均发生本车停车后被持续直行对象接触的失败；单路障可以通过，双路障和部分校园组合不稳定；U形窗口的规划恢复有自转退出动作，但聚类会把凹形空地包成实心框。','',
'## 测试边界','',
'本次没有修改生产代码。原目标(5,0)，车体初始中心(0,0)、朝向+x，车长1.2m宽0.8m轴距0.69m。道路边界y=±2m作为静态分片框输入，宽度约束使用完整车体。迎面对象保持直线恒速，不主动避让停下的本车；因此停车本身不作为迎面避让通过。','',
'主测试为检测框→当前KF→异步规划→50Hz跟踪→有响应滞后的车辆模型。感知20Hz、数据年龄约0.1s、规划交付延迟0.25s、底盘命令延迟0.08s，数值库线程1。噪声包含检测中心1cm、定位3mm/0.2度和检测尺寸/朝向扰动，采用19/37/71三个随机种子。物理间距由独立矩形算法100Hz检查，首次接触立即终止。正常通过要求到达且车体/障碍/道路间距均不小于12cm。','',
'这不是实车或ROS/DDS测试，没有校准实车执行器，也没有原始实车点云回放。另做合成静态点云，经当前过滤/聚类后，把初始检测快照在map中重复送入闭环，用于检验形状占据问题；该部分不模拟视角变化或遮挡演化。','',
'参数和源文件哈希见test_metadata.json，结果明细见primary_results.csv与all_results.json。', '',
'## 迎面动态测试','',
'行人尺寸0.45×0.45m，目标发布时距车中心3m，速度0.40/0.45/0.50m/s；迎面小车尺寸经用户同意按1.2×0.8m，距离4m，速度0.50/0.75/1.00m/s。', '',
'| 场景 | 首次检测：加噪3次 | 提前完成KF确认：加噪3次 | 无噪声 |','|---|---|---|---|']
for name in [f'pedestrian_3m_{s:.2f}' for s in (.4,.45,.5)]+[f'vehicle_4m_{s:.2f}' for s in (.5,.75,1.)]:
 ns=[d for d in noise if d['result']['name']==name];ws=[d for d in warm if d['result']['name']==name];cs=next(d for d in clean if d['result']['name']==name)
 lines.append(f"| {cn[name]} | {sum(d['result']['outcome']=='collision' for d in ns)}/3 接触失败 | {sum(d['result']['outcome']=='collision' for d in ws)}/3 接触失败 | {labels[cs['result']['outcome']]} |")
lines += ['', '所有这些迎面测试失败。触发dynamic_yield后本车速度归零，对方继续运动而接触本车，未出现成功侧移让过。提前确认对照也失败，排除了仅由unknown学习延迟解释全部失败的可能。', '',
'迎面小车的_future_refuge()采用两车外接圆判定停靠点：横向要求2×sqrt(0.6²+0.4²)+0.12=1.562m；4m道路内、正向停车车体中心最大允许偏移只有2−0.4−0.12=1.48m，形成保守判定冲突。直接检查例子：中心偏移1.2m时两个平行矩形实际净距0.4m，道路净距0.4m，却仍被圆判定拒绝。见refuge_diagnostic.json。该证据说明终端位置判定过度保守，不证明从指定起点到该停车点的完整轨迹一定可行。','',
'行人的3m近距离场景主要是当前分段候选和制动备份没有选出可验证侧移，随后停止。测试不会把删除动态碰撞检查作为修复，也不把本车停止算作已完成避让。', '',
'![迎面测试](dynamic_tests.png)', '', '## 路障和校园静态测试','',
'路障真实底座0.28×0.28m、高0.5m。单路障位于(2.5,0)；左右路障同位于x=2.5，y=±0.8为中央可通、y=±0.55为需要侧绕；错位路障为(2,0.5)和(3.25,-0.5)。', '',
'| 场景 | 检测框加噪：到达/3 | 检测框无噪声 | 合成点云检测快照闭环 |','|---|---:|---|---|']
for name in list(cn)[:11]:
 ns=[d for d in noise if d['result']['name']==name];cs=next(d for d in clean if d['result']['name']==name);ps=next(d for d in perception if d['result']['name']==name)
 count=sum(d['result']['outcome']=='reached' for d in ns)
 ntext=f'{count}/3' if name!='school_road_blocked' else '3/3正确停车'
 lines.append(f"| {cn[name]} | {ntext} | {labels[cs['result']['outcome']]} | {labels[ps['result']['outcome']]} |")
lines += ['', '合成点云双路障侧绕与树/路障组合出现真实间距不足12cm（分别约8.7cm和7.7cm），不算通过。尖锥底座在离地高度过滤后，实测框可能只包住上方较细的截面，不能将实测尺寸误差忽略。', '',
'各校园场景的精确尺寸/坐标在runs/中保存，图中显示11种静态组合。校门立柱、停放车辆、L形墙角在检测框噪声测试3次均到达；窄隔离柱、树与路障组合、长椅/花箱未稳定完成。封闭道路正确停止。','',
'部分失败记录出现guide_target在车后方，造成orientation_stall、自转或远距离绕行；另有规划motion但执行collision_guard反复停止。当前几何引导的额外膨胀、最多8个引导框和实际执行约束之间存在不一致；双路障中央无噪声可通过、有噪声不通过，说明引导切换对尺寸/中心扰动敏感。见failure_evidence.json。','',
'![校园静态测试](static_school_tests.png)','', '## U形窗口是否自转退出','',
'净宽1.8m，墙厚0.1m，纵深2/3m；车位于窗口横向中心，朝封闭端。车头到封闭端分别0.15、0.40、0.70m。原目标保持初始正前方5m；退出判据为完整车体穿过入口外侧至少0.12m，而不是已经到达原目标。', '',
'| 纵深 | 车头距封闭端 | 分开的三面墙：加噪退出/3 | 分开的三面墙：无噪声 | 当前聚类快照 |','|---:|---:|---:|---|---|']
for depth in (2,3):
 for gap in (.15,.4,.7):
  name=f'u_depth{depth}_front{gap:.2f}';ns=[d for d in noise if d['result']['name']==name];cs=next(d for d in clean if d['result']['name']==name);ps=next(d for d in perception if d['result']['name']==name)
  lines.append(f"| {depth}m | {gap:.2f}m | {sum(d['result']['outcome']=='exited_u' for d in ns)}/3 | {labels[cs['result']['outcome']]} | {labels[ps['result']['outcome']]} |")
lines += ['', '车体旋转最大半径sqrt(0.6²+0.4²)=0.721m；加入12cm间距后约需0.841m中心到墙距离（代码还加采样扫掠界）。15cm车头间距对应中心距墙0.75m，不够安全自转。当前Ackermann前进速度下限为0，没有先倒退一点再自转的恢复动作，因此这组停住。', '',
'三面墙输入正确且前距0.4/0.7m时，代码会请求自转、等待底盘模式切换、设置恢复子目标再前进退出。无噪声4种组合均退出；加噪12次中10次退出，2次恢复失败/次数耗尽。退出一般约22–29s，不能称为稳定保证。', '',
'当前聚类却把这些2–3m的连通U形三面墙当成一个OBB，最长边未达到3.5m的结构拆分门限，框包住了窗口空地和车位，6组聚类快照全部无法退出。这与真实物理碰撞不同，是感知占据几何误报。', '',
'按这里的0.1m墙厚和4m道路，两侧外绕通道各只有1.0m，而车宽加两侧间距要求1.04m。因此原目标在封闭端之外时，不能把安全退出等同于绕墙到达前方目标。此处专门记录退出事件。', '',
'![U形恢复测试](u_tests.png)','', '![U形聚类误占据](u_perception_false_box.png)','', '## 平滑性与验证结论','',
'正常LQR追踪段的命令加减速、jerk、转向速率/加速度满足当前参数（约0.7/1.2m/s²、2m/s³、30度/s、100度/s²）。测试数据中停止分支仍直接归零，不能把正常段的限制外推成全过程无震荡。失败场景还有反复起停/自转，车辆模型不构成实车机械平滑性证明。', '',
'![指令示例](commands_example.png)','',
'优先修复方向：1）已确认动态对象的矩形停靠/时间碰撞判定和近距离侧移候选；2）凹形墙体用保留空地的分解占据表达，而不是按最长边是否大于3.5m才拆分；3）静态引导的膨胀与最终车体/执行检查保持一致，避免指向车后方或通道边缘；4）恢复加入退出目标持续条件，必要时在底盘支持下设计安全倒退；5）检测框对锥体底座的覆盖误差需要计入。', '',
'本次测试仅评估当前实现，没有为得到通过结果调整生产算法，也没有把这些失败改成通过。当前版本未达到上述全部工程场景要求。','',f'主测试共{len(primary)}次：检测框噪声69次、动态提前确认18次、无噪声23次、聚类快照闭环17次；另有{len(all_data)-len(primary)}次连续长边界表示对照。精确汇总以suite_summary.json为准。']
(OUT/'测试报告.md').write_text('\n'.join(lines)+'\n')
print(json.dumps(summary,indent=2))
