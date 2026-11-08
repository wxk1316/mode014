# C：控制、模式、trap 工作摘要

检索/代码核对截止：2026-11-06（Asia/Shanghai）。无车辆、ROS/DDS或UDP运动测试。

## 已读文件与当前证据

已读任务书、日志指南、当前两个launch、real_vehicle.yaml、vehicle_bridge.py、traj_tracker_core/node、ClassicTrapController、escape、mode_switch_guard.hpp、car_ctr.cpp、原tests/control与planner_safety、五场景/自转验证脚本及历史control_audit/implementation。

- 当前已经修复：ACK w=0不会结束独立yaw自转会话；3秒+ready4/ready0；原子command/Path epoch；源定位超时/连续性；短路径后轴纯追踪几何；即时安全减速；倒车停稳握手。不能重复记作当前缺陷。
- 当前确证：`_arbitration_step(blocked=...)`原本不使用blocked；任意障碍+观测静止就可触发trap。原±1.32m墙fixture合法v=.05，观测完全不动却旧代码切模式；这是执行停滞假设，不能当几何无解。±1.20m仍合法v=.05（3s端点x=.14875，前向净距.25125>.2）；±1.14m静态probe=False、自转-75°安全且一次锁存，作为真正受困fixture。
- 当前确证：trap rejoin hold原来持续0指令，仅以同位置安全条件+墙钟退出；每次距离spin_anchor>.5就reset，trap内部横移可刷新尝试预算。
- 当前确证：停稳只用平均轮速；单轮.1m/s、其余0，均值.025<.03会放行；现在生产节点传max|wheel speed|作为停稳证据，均值只供方向估计。
- 当前确证：自转入口先发0保持.3秒但不检查真实停稳；底层可无限重发机械请求；未知速度speed0对象原_spin_safe未用整自转盘预测，只剩假定角度时序足迹。

## 坐标、单位、时钟与契约

map几何中心；车1.2×.8m，L=.69m，中心→后轴.345m仅轴对称假设。δ/CarCtr.angle degree；内部弧与yaw rad；ACK v m/s，SPIN speed rad/s，UDP -2已标定不改。控制/模式等待用monotonic秒；定位和障碍source stamp不由到达重复刷新。command与Path共同整数ns epoch；Path z是速度标签，点header是发布版本不是未来时刻。

融合snapshot尺寸包含过去源年龄膨胀；unknown未来半径增长率=max(config uncertainty_rate,max_unknown_speed)，不重复叠加过去age。自转整盘检查同一未来率，实际/模拟的3秒等待均检查障碍可达域。

## 文献（已读原文4篇；另原始与2021补充见control_literature.md）

RPP 2023 §2/3/5/6：几何追踪不含动力学，Ackermann输入路径须可执行；保留默认前馈和安全后验，不借论文放开未经验证转角修正。
Kim等2024 APF/WF §V-A/VI：hit/leave记忆约束避免重复环绕，固定方向；TurtleBot4圆盘与5Hz仿真，不能套成本车3秒机械系统完备保证。采用episode全局目标进展门+有限预算/冷却。
Wang/Tayebi 2024 TAC §II/III/IV：混合流/跳变与hysteresis，GAS需synergy/Lyapunov/静态sphere-world等条件，本车仅工程状态机guard借鉴，未声称定理满足。
cHyRRT/cHySST 2024 §3–6：状态/输入/flow/jump/unsafe set与真实混合时间；bounce/tensegrity实验非Ackermann；启发离线多模式停止/换轮/动作/制动/恢复时间贯通，不声称概率完备。

## 已修改/文件所有权

C独占vehicle_bridge、traj_tracker_core/node、mode_switch_guard/car_ctr；未改定位、TF、补偿、YAML/bridge_node（root接线）。新增BridgeConfig `trap_exit_progress=.10`、`trap_cooldown=5`、`trap_static_probe_budget=.02`、`static_search_enable=True`、`static_search_budget=.015`、`static_search_max_expansions=200`。生产tracker `spin_require_stopped_feedback=True`（纯legacy harness默认为False）、`spin_stop_timeout=3`。C++ `mode_max_retries=2`（初发+两次重发，request生命周期reset）；未知命令模式1/2当前不放行，B提供默认禁用仿真契约。

规划接口由B提供：last_failure_reason='deadline'不可当static无解；has_safe_motion(robot,static_obs,budget_s)返回True/False/None，None仅预算不足；LocalStaticGuide返回waypoint/None与status，所有动态安全仍由DWA复核。guide/DWA/probe共享剩余周期预算，root硬.3秒guard最后停车。

## 验证/未决/下一步

原45条control测试与C++原机械等待测试通过。新增同输入baseline/控制模块消融、动态让行、执行停滞、true static blocked、episode漂移、timeout latch、rejoin没动/真进展、unknown等待、自转移动未停/单轮未停/反馈丢失测试进行中。

未决：真实轮速新鲜度与物理制动/转角/角加速度、未测后轴中心、定位小修正、CV非合作障碍/看不见区域；source clock同步、DDS参考/ready交付仍需实车日志。没有证据支持开启控制jerk滤波或tracking correction。

## 最终整合追加：移动中的局部极小

根35秒trace证明U/窄U真的前后行驶（末15秒3.876/2.984m）仍无全局进展；厚U/多块微移往复（.272/.200m、x跨度.0278/.0172m），不应全称execution-stall。新的独立6秒窗口不随每帧blocked/feasible交替清零，只由新定位源更新；实际pose路径≥.06m、全局或锁路线最佳进展≤.03m，还必须累计≥1秒新鲜四轮非零rolling（maxabs>.005m/s，仅定性轮滚动证据不是纵速标定）。新鲜全零轮+1cm pose jitter及缺轮速不得触发。动态/unknown、fault、机械期间不累积此检测。

新增phase RETREAT3m→真实停稳/ALIGN→侧向3m ESCAPE→RETURN_ALIGN→原安全REJOIN。狭窄U内不强制spin，退到腔外再判断整盘；至多2次spin且phase/位移不刷新次数。有效ACK预算生产24秒（pure默认16保留旧回归）；机械期间冻结、单会话24秒/机械8秒仍有界。旧35/60秒负结果和失败prototype均保留。

phase全局rejoin修复：EnhancedDWA会以active trap.sub_goal覆盖传入goal；确认重接时传trap=None并核build_dynamic_window实际global goal。active dynamic/unknown且实际近零让行冻结ACK计时（仍跑安全DWA/phase检查），planner/hardnode deadline用tick checkpoint恢复phase/ACK预算；请求尝试数保守保留，不能因算超时renew预算。源码source hash见control_frozen_manifest.json；本轮只是源码/模型验证，未实车运动。

90秒同输入/同horizon整栈对照（static_recovery_90s/comparison.json）：基线多块36.34秒到达，其余U/厚U/窄U90秒失败；成熟版U83.14秒与窄U82.78秒到达，厚U90秒距.3083仍失败，多块90秒距.9244未达（明确回退）。所有4静态新模型无碰撞/.2margin违规，零deadline miss。U基线trap4次/再进入3次/7模式切换→新1/0/4；厚U3/2/6→1/0/4。不能用这些证明全场景完备。多块后退3米动作成本不必要，已向root提出安全侧向spin优先候选及回退修正，不擅掩盖负结果。

**最终默认选择修订**：侧绕优先仍没有恢复多块90秒到达，故moving_recovery_enable默认False，仅观测moving minimum；实验True有强goal-relative confinement前提（法向±2m固定yaw实际足迹及sweep都受阻）。几何探测确实分开multi/free与3U/blocked，但confinement-only multi90仍距3.3502、没有导航到达，因此不把它包装成默认优先方案收益。旧静态blocked恢复与默认guard/perception/kernel保持。原自动phase U83.14/厚U87.44/窄U82.78成功及multi失败数据保留为实验，不作新默认结果。新增默认关闭而仍有观察、实验True wheel/pose窗口、原输入断流/timeout/statebudget等测试；定向137+8subtests通过。最终源码/开关见control_frozen_manifest.json；不再进行行为或CPU迭代。
