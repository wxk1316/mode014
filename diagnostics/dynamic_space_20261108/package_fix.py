import ast,json,hashlib,zipfile,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;CLONE=Path('/tmp/plan2-rviz-upload-20261107')
before=json.loads((OUT/'before_sha256.json').read_text());files=[f for f,old in before.items() if hashlib.sha256((ROOT/f).read_bytes()).hexdigest()!=old]
assert set(files)=={'src/rda_dwa/rda_dwa/motion_safety.py','src/rda_dwa/rda_dwa/realtime_ackermann.py','src/rda_dwa_real/rda_dwa_real/realtime_vehicle_core.py','src/rda_dwa_real/rda_dwa_real/traj_tracker_core.py','src/rda_dwa_real/rda_dwa_real/traj_tracker_node.py'}
manifest={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}
for f in files:
 ast.parse((ROOT/f).read_text());dest=CLONE/f;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/f,dest)
for name,base in [('car_gpt_dynamic_space_fix.zip',ROOT),('car_gpt_dynamic_space_rollback.zip',OUT/'before')]:
 with zipfile.ZipFile(OUT/name,'w',zipfile.ZIP_DEFLATED) as z:
  for f in files:z.write(base/f,f)
 with zipfile.ZipFile(OUT/name) as z:assert z.testzip() is None
subprocess.run(['git','add','--',*files],cwd=CLONE,check=True)
actual=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=CLONE,text=True).splitlines();assert set(actual)==set(files)
(OUT/'modified_files.json').write_text(json.dumps(manifest,indent=2)+'\n')
s=json.loads((OUT/'run_summary.json').read_text());replay=json.loads((OUT/'planner_replay.json').read_text())
positive=[json.loads(line) for line in (OUT/'closed_loop_run.log').read_text().splitlines()];unknown_cases=[json.loads(line) for line in (OUT/'unconfirmed_scenarios.log').read_text().splitlines()]
rows=[]
for r in positive+unknown_cases:
 gap=r['min_physical_gap_m'];status='通过' if r['reached'] and (gap is None or gap>=.12) else ('保护停/缓行' if r['scenario']=='unknown_nearby' else '失败，保留')
 rows.append(f"| {r['scenario']} | {status} | {'无障碍' if gap is None else f'{gap:.3f} m'} |")
report=f'''# 空地中的 dynamic_yield：新实车记录诊断与修复

## 确认的原因

数据来源：用户提供 run_19700110_080449_336119475 的 JSONL 与 RViz 截图。记录器写入 65429 条、丢失 0 条。本次未修改原始数据。上传目录没有 raw_bag，因此本次采用已记录的观测、诊断与指令，不把它称作原始点云回放。

1. 感知延迟问题已改善。障碍输出 {s['obstacle_rate_hz']:.2f} Hz；感知 p50={s['perception_duration_s']['50']*1000:.1f} ms，p99={s['perception_duration_s']['99']*1000:.1f} ms；障碍年龄 p50={s['obstacle_age_s']['50']*1000:.1f} ms，最大={s['obstacle_age_s']['100']*1000:.1f} ms；跟踪器重置 0 次。记录文件的生产源码/配置哈希与修改前工作区一致。
2. 目标在记录 t=44.802 s 手动发出，约在车头方向前方 17.8 m。目标后 557 个规划周期中：497 dynamic_yield，58 motion，2 planning_pending。多个帧 static_motion_feasible=true、moving_count=0、stale_causes=[]、yield_cause=unknown_reachable_envelope。未知目标数中位 6、最大 17。unknown 并不表示语义类别“行人”，也不证明物体确实移动；这里包括初始未确认或短暂漏检的聚类跟踪。
3. 当前安全余量为 0.12 m；造成大范围封锁的是“全部 unknown 在整段 3 秒内可向任何方向以 1.5 m/s 运动”的模型。扩张增加 4.5 m，另外仍包含观测年龄及漏检期间的扩张。RViz 标记显示当前占据框，未显示这种完整未来区域，所以看着空地很多但候选全部被覆盖。多个单独的 unknown 就能拒绝所有候选，且在不同 id 间轮换，形成持续让行。
4. 控制层还错误保留了手动地图定位前的坐标：目标后 t=44.81–61.16 s，控制器 centre 仍约 (-0.006,0.014)，输入 TF 则在 (-78,-12) 附近且年龄约 0.06 s。大约 78 m 的坐标初始化跳变被普通运动跳变检查拒绝，控制器 pose age 增长到 26.22 s 才被旧的 dt 阈值放行。这是控制接入初始化问题，不是定位算法失效。期间有 819 个控制周期为 localization_timeout。

记录虽有 126 条短时非零指令，但没有持续运动。不能仅由定位漂移推断实际移动距离；没有新实车的轮速测量来做这种结论。

## 修改后的检查方法

- 占据框、1.5 m/s 未确认目标速度上限、漏检的当前尺寸扩张及 0.12 m 余量保留，未把 unknown 强制转为静态，未删除任何障碍。
- 静态几何和已确认动态目标仍验证完整规划轨迹。unknown 验证 0.4 s 的下一段动作以及带 jerk、转向约束的完整刹停备份，使用较慢的 0.8 m/s² 制动模型。每候选检查持续到速度低于 0.01 m/s 并额外保留 0.2 s 占据；这个时限会随初速、目标速度、加速度状态改变，不是统一把未知物体预测削到固定小数。每次新观测与控制周期重新检查。
- unknown 位于车前通行走廊时，提早在经过验证的左右候选中选择侧向准备，不必等它被确认运动才开始转向。保持避让侧，空间相近的 id 变化及 confirmed→unknown 切换尽量继承原侧；所有动作仍需通过未知目标的刹停检查。
- 规划、监督与控制使用一致的 unknown 动作/刹停规则。控制频率下的单候选刹停采用等价标量计算，避免每个积分步分配小 NumPy 数组；等价性另有测试。
- 车辆没有目标且停止时，允许接入 RViz 手动定位造成的地图坐标初始化跳变，清除旧路径和旧速度历史，立即使用最新有效定位。有效目标运动阶段的跳变检查仍存在。未改变定位算法、TF 发布器或 RViz 手动定位再发布目标的流程。
- 新增 unknown_policy、unknown_reaction_s、unknown_stop_horizon_s、unknown_stop_min_gap、provisional_avoidance_id、avoidance_side、pose_reinitializations 诊断字段。

这种方法验证模型下下一段动作及可用刹停；它不声称停住后任意非协作对象永远不会撞车。仍依赖配置的车辆/执行响应、障碍速度上限与及时观测，不能宣称所有现场都通过。

## 相同实车快照比较

原先让行的 {replay['frames']} 个快照，同样的姿态、目标和障碍消息进行旧/新检查：旧代码 {replay['before_positive']} 个运动候选，新代码 {replay['after_positive']} 个运动候选；新代码 {replay['after_reasons'].get('dynamic_yield',0)} 个继续让行。旧回放 {replay['before_reasons'].get('dynamic_yield',0)} 次与现场 497 次略有差异，因为输入按接收时间近似匹配，初始加速度/转向率未记录到规划快照，比较双方均置零。

**这是冻结车辆状态的候选回放，不是新闭环轨迹，不能声称实车已经走通 17.8 m。** 新运动候选均保留 unknown，并通过本次动作/刹停检查；没有用“把全部未知障碍设为静态”的诊断对照作为实际策略。

![预测区域解释]({OUT}/unknown_space_comparison.png)

图中蓝色为车辆、绿色为已确认当前框、橙色为 unknown 当前框、红色为未来可达区域示意。左右两图的当前占据框完全相同；只对不同检查时限作解释，完整刹停检查用时间索引轨迹而非一张静态区域图。

## 回归与闭环验证

{(OUT/'unit_tests.log').read_text().strip()}

| 场景 | 结果 | 最小物理间隙 |
|---|---|---|
{chr(10).join(rows)}

闭环含 0.25 s 规划延迟、50 Hz 控制、20 Hz 量化目标、约 0.08 s 执行延迟、位置/航向噪声。unconfirmed 场景的运动对象在前 0.6 s 保持 unknown 再确认；5 m 处 1.0 m/s 迎面、7.5 m 处 1.5 m/s 迎面及自行车横穿通过。间隙由独立矩形距离检查。

较晚的 5 m/1.5 m/s 迎面与没有侧向空间的窄通道仍是失败压力用例，非协作仿真对象撞到停止车辆，这两项不算通过。长期不确认的近处 unknown 仍可能停/缓行。不要把“到达目标”或“输出 motion”直接当成无碰撞证据。

正常控制指令的 jerk ≤2 m/s³、转向速率 ≤30°/s、转向加速度 ≤100°/s²，急停除外。上述为本地模型验证；未运行 ROS/DDS、Jetson 编译或新实车，实际平滑性还需验证。

## 部署和核对

本次补丁包含五个修改的生产文件，覆盖到当前 c583aff 版本的完整工作区后：

```bash
cd ~/Desktop/Wxk/car_gpt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rda_dwa rda_dwa_real
source install/setup.bash
ros2 launch rda_dwa_real rl_dwa_car.launch.py config:=$PWD/src/rda_dwa_real/config/real_vehicle.yaml
```

两个包都修改了，不能只编译 rda_dwa_real。重启节点，按原流程手动完成 RViz 定位后发布目标。若底盘已经单独启动，launch 加 with_car_ctr:=false。

```bash
ros2 topic echo /rda/diag --once --full-length
ros2 topic echo /rda/control_diag --once --full-length
```

核对 planner.unknown_policy=action_and_brake，unknown_stop_horizon_s 随状态变化，unknown_stop_min_gap 合格，参考点数/速度有连续输出，输入定位有效时控制器 pose age 不再因旧地图坐标持续增长。若继续让行，以完整快照核对是哪一个 unknown 的刹停区域仍阻挡，或是否已确认动态目标确实冲突。

## 交付

只上传五个修改的生产文件到 plan2，不上传用户日志、bag、图、测试和报告。修改包与回滚包均保留 src/ 相对路径，modified_files.json 给出 SHA-256。
'''
(OUT/'空地停车原因与修复.md').write_text(report)
print(json.dumps({'files':files,'patch':str(OUT/'car_gpt_dynamic_space_fix.zip'),'report':str(OUT/'空地停车原因与修复.md')},ensure_ascii=False,indent=2))
