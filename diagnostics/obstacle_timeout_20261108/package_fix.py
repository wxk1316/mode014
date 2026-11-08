from pathlib import Path
import ast,json,hashlib,zipfile,shutil,subprocess
import yaml
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;CLONE=Path('/tmp/plan2-rviz-upload-20261107')
files=['src/rda_dwa_real/rda_dwa_real/perception_core.py','src/rda_dwa_real/rda_dwa_real/async_perception.py','src/rda_dwa_real/rda_dwa_real/lidar_track_node.py','src/rda_dwa_real/config/real_vehicle.yaml']
for name in files:
 if name.endswith('.py'):ast.parse((ROOT/name).read_text())
config=yaml.safe_load((ROOT/files[-1]).read_text());lidar=config['lidar_track_node']['ros__parameters']
assert lidar['async_perception'] and lidar['perception_max_source_age_s']==.4 and lidar['track_reset_gap_s']==2.
assert config['rda_dwa_real_bridge']['ros__parameters']['obstacle_timeout']==.4
assert config['traj_tracker_node']['ros__parameters']['obstacle_timeout']==.4
manifest={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}
for name in files:
 dest=CLONE/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dest)
with zipfile.ZipFile(OUT/'car_gpt_obstacle_timeout_fix.zip','w',zipfile.ZIP_DEFLATED) as z:
 for name in files:z.write(ROOT/name,name)
with zipfile.ZipFile(OUT/'car_gpt_obstacle_timeout_rollback.zip','w',zipfile.ZIP_DEFLATED) as z:
 for name in files:
  previous=OUT/'before'/name
  if previous.exists():z.write(previous,name)
for file in ('car_gpt_obstacle_timeout_fix.zip','car_gpt_obstacle_timeout_rollback.zip'):
 with zipfile.ZipFile(OUT/file) as z:assert z.testzip() is None
(OUT/'modified_files.json').write_text(json.dumps(manifest,indent=2)+'\n')
subprocess.run(['git','add','--',*files],cwd=CLONE,check=True)
actual=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=CLONE,text=True).splitlines();assert set(actual)==set(files)
profile=json.loads((OUT/'profile_comparison.json').read_text());worker=json.loads((OUT/'worker_profile.json').read_text())
report=f'''# obstacle_timeout 现场诊断与修复

## 现场结论

用户实车输入已足够定位这次阻断。点云刚到达时 source_age_s=0.0121 s，地面拟合有效，3851 个支撑点；filter_cluster_duration_s=1.5807 s，总处理时间 1.6000 s，障碍输出约 0.7 Hz。监督层观测年龄 1.8799 s、控制层 2.0901 s，均超过 0.4 s。定位 TF 在监督层年龄仅 0.0305 s。规划诊断为空，说明当时尚未进入规划。

另一条确定的连锁问题是 source_dt_s=1.5503 s，而旧代码超过 0.5 s 就 reset 跟踪器，每帧反复重建，因而 detection_count=track_count=unknown_count=56。感知节点的重计算占用单线程 ROS 回调，感知自己的定位接收年龄也达到 1.5138 s。RViz 的 queue full 不能直接等同此感知故障；现场点云到达年龄显示感知已经收到新点云。

## 修改

1. 过滤/聚类优化：避免大型点集循环、逐点密集邻接循环、每簇重复扫描全部原始点。以占据体素的四个极值支撑点建立精确欧式邻接，邻接计算、分量合并、整数键排序和支撑分组改用 NumPy 原生数组。保留全部原始支撑点拟合框。地面拟合使用居中矩计算，框拟合避免频繁的小矩阵 BLAS 调用。
2. 独立感知进程：ROS 回调只缓存最新点云与当时的定位快照，不执行聚类；只保留一个待处理观测，新的覆盖旧的，阻止队列积压。工作进程的数值库线程设为 1，父进程环境恢复。
3. 保留真实采集时间：输出沿用原始点云 header，过期的待处理输入/完成结果不发布为空场景。0.4 s 障碍新鲜度阈值保持，新增 source_age_at_receive_s、source_age_s（结果发布前总年龄）、worker_queue_delay_s、worker_duration_s、ground/voxel/component/box_duration_s、obstacle_published、track_reset 诊断。
4. 跟踪重置：间隔超过 2.0 s 才按数据中断重建；实际观测时间差用于 KF，取消截断到 0.5 s。无效地面/解析失败不冒充成功观测；下一有效帧按距上次有效观测的真实间隔推进。

## 验证

{(OUT/'unit_tests.log').read_text().strip()}

新增检查：真实 spawn 工作进程、等待槽只留最新帧、重计算不在大点云 ROS 回调执行、1.55 s 间隔不再每帧 reset、过期待处理/完成观测不更新障碍、有效空观测保留原始采集时间、随机图与原欧式聚类一致。

同样 30 帧已有原始 bag，在本地 Mac 上：处理中位时间 {profile['before_p50_s']*1000:.2f} → {profile['after_p50_s']*1000:.2f} ms，约 {profile['before_p50_s']/profile['after_p50_s']:.2f} 倍；p95 {profile['before_p95_s']*1000:.2f} → {profile['after_p95_s']*1000:.2f} ms。检测数量全部一致，框差异最大 {profile['max_box_change_m']:.3g} m，属于浮点舍入。不能将此耗时当作 Jetson 实测。

本地实际异步进程用原始点云以约 20 Hz 目标频率输入，{worker['input_frames']} 帧提交、{worker['completed_frames']} 帧完成；预热后的观测全部在 0.4 s 内，源年龄 p95={worker['warm_source_age_p95_s']*1000:.2f} ms，工作进程耗时 p95={worker['worker_duration_p95_s']*1000:.2f} ms，跟踪重置 {worker['track_resets']} 次。测试没有 ROS/DDS、Jetson 或底盘实车。

这次聚类几何语义不变，历史记录仍有 unknown 保护让行；本次仅解决算力/调度/跟踪重置的已定位问题，不保证部署后其他避障条件都会通过。

## 部署

在当前 62260f9 版本上，将 car_gpt_obstacle_timeout_fix.zip 内四个文件按 src/ 路径一起覆盖到实车完整工作区，包含新增 async_perception.py。本次不增加依赖，不修改定位算法、TF 发布器、目标发布流程或底盘协议。

```bash
cd ~/Desktop/Wxk/car_gpt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rda_dwa_real
source install/setup.bash
ros2 launch rda_dwa_real rl_dwa_car.launch.py config:=$PWD/src/rda_dwa_real/config/real_vehicle.yaml
```

重启感知、规划和跟踪节点使配置生效，按原 RViz 定位/目标流程操作。若底盘节点已独立启动，launch 追加 with_car_ctr:=false。启动日志应显示 async_perception=True、max_source_age=0.40s、track_reset_gap=2.00s。

```bash
ros2 topic echo /rda/perception_diag --once --full-length
ros2 topic hz /rda/obstacles
ros2 topic echo /rda/diag --once --full-length
```

关注工作进程处理时间、四个阶段耗时、队列延迟、结果源年龄、obstacle_published 与 track_reset。目标是有效观测持续更新，监督/控制层 obstacle_age_s 保持低于 0.4 s；不能只看到 published 或输出频率就认为年龄合格。如果仍超时，新的阶段指标能识别 Jetson 的具体瓶颈，无需先上传大 bag。

## 交付与回滚

本次仅上传以上四个生产文件到 plan2。报告、测试、性能数据和 bag 留在本地。回滚包包含修改前的三个已有文件；恢复它们后可删除新建的 async_perception.py。modified_files.json 给出源码哈希。
'''
(OUT/'超时原因与修复.md').write_text(report)
print(json.dumps({'production_files':len(files),'patch_zip':str(OUT/'car_gpt_obstacle_timeout_fix.zip'),'report':str(OUT/'超时原因与修复.md')},ensure_ascii=False,indent=2))
