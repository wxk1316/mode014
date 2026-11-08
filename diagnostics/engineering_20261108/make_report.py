import json,math,sys,hashlib,zipfile,shutil
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
results=json.loads((OUT/'closed_loop_results.json').read_text())
for r in results:
 r['physical_clearance_pass']=r['min_physical_gap_m'] is None or r['min_physical_gap_m']>=.12
 r['scenario_pass']=r['reached'] and r['physical_clearance_pass']
(OUT/'closed_loop_results.json').write_text(json.dumps(results,indent=2)+'\n')
fig,axes=plt.subplots(1,2,figsize=(12,5))
for ax,name in zip(axes,('static_barrier','head_on')):
 trace=json.loads((OUT/(name+'_trace.json')).read_text());a=np.asarray([r[:10] for r in trace],dtype=float)
 ax.plot(a[:,1],a[:,2],label='vehicle centre',lw=2)
 for r in trace[::100]:
  t,x,y,th=r[:4];c,s=math.cos(th),math.sin(th);box=np.array([[.6,.4],[-.6,.4],[-.6,-.4],[.6,-.4]])@np.array([[c,s],[-s,c]])+np.array([x,y]);ax.add_patch(Polygon(box,fill=False,color='tab:blue',alpha=.35))
 if name=='static_barrier':ax.add_patch(Polygon([[2.7,-.25],[3.3,-.25],[3.3,.25],[2.7,.25]],color='black'))
 else:
  ax.plot([5.,-2.],[0.,0.],'--',color='tab:red',label='actor 1.0 m/s')
  for t in (0,1,2,3,4,5):ax.scatter([5.-t],[0.],color='tab:red',s=20);ax.annotate(f'{t}s',(5.-t,0.),xytext=(0,-15),textcoords='offset points',fontsize=8)
 ax.scatter([7.],[0.],marker='*',s=100,color='green',label='manual goal');ax.set(xlim=(-1,8),ylim=(-1,4),xlabel='map x [m]',ylabel='map y [m]',title=name.replace('_',' '));ax.set_aspect('equal');ax.grid();ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(OUT/'avoidance_validation.png',dpi=150)
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
for name in ('straight','static_barrier','head_on'):
 trace=json.loads((OUT/(name+'_trace.json')).read_text());a=np.asarray([r[:10] for r in trace],dtype=float)
 axes[0].plot(a[:,0],a[:,4],label=name);axes[1].plot(a[:,0],np.rad2deg(a[:,5]));axes[2].plot(a[:,0],a[:,8])
axes[0].set_ylabel('command speed [m/s]');axes[1].set_ylabel('command steer [deg]');axes[2].set_ylabel('lateral error [m]');axes[2].set_xlabel('time [s]');axes[0].legend();[ax.grid() for ax in axes];fig.tight_layout();fig.savefig(OUT/'command_smoothness.png',dpi=150)
cloud=json.loads((OUT/'tf_cloud_replay_200.json').read_text());frames=cloud['frames']
valid=[r['perception_duration_s'] for r in frames];counts=[sum(r['types'].values()) for r in frames];unknown=[r['types'].get('unknown',0) for r in frames]
stats=dict(frames=len(frames),perception_duration_p50_s=float(np.percentile(valid,50)),perception_duration_p95_s=float(np.percentile(valid,95)),last_detection_count=cloud['final_perception']['detection_count'],last_track_count=cloud['final_perception']['track_count'],last_unknown_count=cloud['final_perception']['unknown_count'],selected_localization=cloud['last_frames'][-1]['localization_source'],planning_reasons=cloud['planner_reasons'],forward_probe=cloud['final_forward_probe'])
(OUT/'replay_summary.json').write_text(json.dumps(stats,indent=2)+'\n')
rows=[]
for r in results:
 gap='无障碍' if r['min_physical_gap_m'] is None else f"{r['min_physical_gap_m']:.3f} m"
 status='通过' if r['scenario_pass'] else ('保护停车' if r['scenario']=='unknown_nearby' else '失败（保留）')
 rows.append(f"| {r['scenario']} | {status} | {gap} | {r['goal_distance_m']:.3f} m |")
text=f'''# 2026-11-08 感知、局部规划与平滑控制工程修改

## 结论和当前实车状态

这是一版完成代码接线与离线验证的工程修改，不是实车验收结论。原始记录没有更改。本次未运行 ROS/DDS 节点、UDP 底盘驱动或实车。GitHub 仅上传修改的生产文件，测试、日志、bag、图和报告留在本地。

用户最新日志明确是 `dynamic_yield`，定位接入有效、规划耗时和结果年龄已在有效范围内；此时不是原来的时间戳/预算丢弃。聚类几何名称不是语义识别：静态物体产生 `unknown` 和虚假速度后，1.5 m/s 全方向可达包络会使候选被拒绝。不能靠清空障碍物或把所有 unknown 改成静态让车动。

## 生产修改

- 点云：解析字段偏移、浮点类型、字节序和行填充；按照雷达到车体的 +0.45 m 高度转换。官方消息字段约定可核对 [ROS PointCloud2 文档](https://docs.ros.org/en/rolling/p/sensor_msgs_py/sensor_msgs_py.point_cloud2.html)。
- 地面：近场观测估计鲁棒平面、限制倾角与支撑范围；保留地面上 0.12–2.2 m 的点，适配普通路障、行人、自行车、汽车和建筑低层表面。无地面证据超过复用时限时不发布虚构空场景。
- 聚类：0.10 m XY 体素，0.30 m 邻接，至少 10 个原始点和 0.20 m 长边。目标是普通路障，不覆盖特别细小物体或 10 cm 以下路沿。包围框使用全部原始支撑点。超过 3.5 m 的连接分量以固定世界网格 1.2 m 分段，抑制墙面框跨过院落；连续墙面端部合并，孤立弱片段剔除。检测范围扩大到 8 m，提前看到迎面目标，但额外计算耗时需在 Jetson 上测量。
- 跟踪：真实观测才推进运动确认；短漏检不把已确认静止目标变成动态。关联考虑源时间、速度可达距离以及有限尺寸变化，保留对交替小聚类串成假运动的限制。未确认/漏检动态目标仍保护为 unknown，其包络以最后观测为中心并覆盖漏检期间，避免叠加不可信速度后漏掉反方向运动。
- 定位接入：`latest_available` 比较现有 map→body TF 与定位话题的源时间，取较新有效输出；TF 不可用时取最新接收话题。有效更新统一为单调时钟；重复取缓存不刷新断连时限。保留 2 s 断连保护，手动 RViz 定位和手动目标点流程不变，定位算法/TF 发布器不改。
- 规划：采用限加速度、jerk、转向速度/加速度和 0.10 s 执行延迟的轨迹预测；控制使用同样的速度/转向伺服模型。分阶段转出、反向回正、减速候选与侧向避让锁定，优先执行仍可行的已选延续。迎面逃逸只有完整时序轨迹和最终侧向停驻位置通过检查才接受；检查已知匀速对象的后续运动射线。普通动态场景继续检查反应后停车备份。无可行侧向停驻方案时输出具体 `yield_cause`。
- 跟踪/控制：原始定位姿态不滤波；短时间窗回归估计速度/角速度，降低相邻差分噪声。新增横向和航向反馈死区。速度与转向不在目标交叉时突变加速度；正常目标/动态减速传递非空参考和明确 `goal_braking` / `dynamic_braking`，安全停车仍立即生效。参考携带目标速度与目标转角，控制执行对应限幅；速度候选增加中间减速目标，直行和单路障闭环不再出现中途停车再加速的脉冲。
- 异步：沿结果年龄所允许的参考前缀检查当前位置/航向，不再把沿已检路径正常行进当作起点位移超限。年龄、目标变更和离轨检查仍存在。当前障碍在监督层和控制层继续检查。

## 验证

{(OUT/'unit_tests.log').read_text().strip()}

测试包含 ROS 接口替身、真实 spawn 工作进程、地面/路障/连接墙角几何、交替小聚类、真实运动/漏检、时钟/源切换、离轨拒绝、速度与转向目标交叉、以及带延迟和噪声的迎面闭环。未做 ROS Humble 实际编译、DDS 集成或底盘测试。

闭环设定：50 Hz 跟踪，0.25 s 串行规划延迟，20 Hz 量化目标/约 0.08 s 执行延迟，位置 3 mm/航向 0.2° 噪声、20–60 ms 不规则定位采样，障碍观测 0.10 s 延迟。底盘按量化速度/转角执行的运动学模型是近似；不包含真实轮胎滑移和内环动力学。间隙用独立矩形角点/边距离核对，不以“到达目标”代替碰撞验证。

| 场景 | 结果 | 最小物理间隙 | 最后目标距离 |
|---|---|---|---|
{chr(10).join(rows)}

`head_on_slow` 是 0.7 m/s、从 5 m 迎面走来；`head_on` 是 1.0 m/s、5 m；`head_on_early` 是 1.5 m/s、7.5 m 提前检测。`head_on_fast` 同为 1.5 m/s 但只有 5 m 初始距离，未找到及时逃逸，持续直行的仿真行人撞上停住的小车；`head_on_narrow` 两侧墙之间无足够侧向空间，也发生同样失败。这两项不是通过项。小车停车不能保证非协作对象不会撞来。本实现不能宣称任意行人速度、突现距离或窄路都可避让，学校实车运行需要按这种物理边界设定检测范围、速度和通行规则。

正常 LQR 运动指令的 jerk ≤ 2 m/s³，转向速率 ≤ 30°/s，转向加速度 ≤ 100°/s²（浮点误差除外）；加速 ≤ 0.7、减速 ≤ 1.2 m/s²。安全急停不受舒适性限制。轨迹中的一次避让再回正是有意转向，不能将转角变号一概算作震荡。没有真实转向角反馈，因此这里只证明指令和模型行为，不能证明实车实际转角无震荡。

![静态路障与迎面避让](avoidance_validation.png)

![速度与转向指令](command_smoothness.png)

## 原始点云回放

使用原始 bag 最后 200 帧点云及在点云到达前已收到的 TF 进行离线三维链组合；时间插值是回放近似，不是 tf2 中间件测试。Mac 上感知 p50={stats['perception_duration_p50_s']:.4f} s，p95={stats['perception_duration_p95_s']:.4f} s，不能作为 Jetson 运行时间。

最后一帧 {stats['last_detection_count']} 个分段检测、{stats['last_track_count']} 条跟踪、{stats['last_unknown_count']} 条 unknown。分段墙面增加框数不能解释为增加真实障碍。该历史场景的前方 2 m 探针仍为 `{stats['forward_probe']['reason']}`，后方目标统计为 {stats['planning_reasons']}；历史场景没有足够证据证明所有剩余未确认目标都是误报，所以没有解除保护。**不能据此宣布当前实车不动已完全解决。** 最新运行需要检查 `yield_cause`、unknown 的 id/框/漏检，以及 RViz 原始点云是否与车体重叠。

## 部署

把修改文件包按 src/ 相对目录覆盖到实车现有完整工作区。plan2 是补丁仓库，不包含原项目的全部依赖，不能作为完整工程直接构建。

```bash
cd ~/Desktop/Wxk/car_gpt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rda_dwa rda_dwa_real
source install/setup.bash
ros2 launch rda_dwa_real rl_dwa_car.launch.py config:=$PWD/src/rda_dwa_real/config/real_vehicle.yaml
```

本次修改两个包，不能只编译 rda_dwa_real。若底盘驱动已经单独启动，launch 加 `with_car_ctr:=false`。按原流程完成 RViz 手动定位，再手动发目标；没有自动初始定位或自动目标。

```bash
ros2 topic echo /rda/perception_diag --once --full-length
ros2 topic echo /rda/diag --once --full-length
ros2 topic echo /rda/control_diag --once --full-length
```

关注 selected_source、source_age/receive_age、ground_plane、weak_clusters_removed、track_outputs、unknown_count、staged_feasible/selected、avoidance_side、yield_cause、stale_causes、控制参考点数与指令。Jetson 感知过期/规划预算失败必须测耗时后调整，不能无条件取消保护。

## 文件与回滚

modified_files.json 列出生产文件及 SHA-256。修改包只包含这 13 个生产文件；回滚包包含修改前的 11 个已有文件。新建的 perception_core.py 与 pose_velocity.py 在回滚所有引用文件后可删除。报告和验证数据不会上传 GitHub。
'''
text=text.replace('(avoidance_validation.png)',f'({OUT}/avoidance_validation.png)').replace('(command_smoothness.png)',f'({OUT}/command_smoothness.png)')
(OUT/'工程修改与验证.md').write_text(text)
print(json.dumps({'positive_closed_loop_passes':sum(r['scenario_pass'] for r in results),'scenarios':len(results),'report':str(OUT/'工程修改与验证.md'),'replay_summary':stats},ensure_ascii=False,indent=2))
