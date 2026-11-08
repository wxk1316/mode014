# B 规划验证与冻结说明

源码冻结：2026-11-06（冻结后只补实验脚本/报告，源码hash在planner_benchmark.json与planner_worstcase.json）。定位生产代码、TF和补偿未修改。候选/safety/评分保持同输入，新增静态guidance由C接入，未知模式只在显式离线仿真接入。

## 基线与消融

`planner_benchmark.py`在独立解释器解压并导入本轮冻结zip中的planner源码，不把历史review作为代码基线。16场景×12次同输入；batch-only与full命令均与冻结baseline逐项相同。碰撞/制动和软代价的变化只是等价计算与本轮nominal复用；小场景≤4障碍走标量避免批量开销。

| 合成输入 | baseline p50 ms | batch-only p50 ms | full p50 ms | full p95/p99/max ms |
|---|---:|---:|---:|---|
| 直行 | 18.46 | 17.01 | 10.12 | 14.07 / 15.30 / 15.61 |
| 静态多块 | 39.19 | 38.16 | 30.74 | 32.27 / 32.27 / 32.27 |
| 长墙 | 29.29 | 29.26 | 21.19 | 21.74 / 21.77 / 21.78 |
| 横穿CV | 28.55 | 26.64 | 20.22 | 20.46 / 20.47 / 20.47 |
| 远场128瓦片 | 674.90 | 33.90 | 27.29 | 29.19 / 29.30 / 29.32 |
| 远场512瓦片 | 2680.77 | 38.53 | 29.44 | 31.80 / 32.96 / 33.25 |

这是固定snapshot纯Python计算；far512中的合成几何可能超出active 12m感知半径，因此只证明算术热点，不证明实际可见512片分布。near/dense/mixed下均在约4m内，单独压力测试更贴近有效范围。有限12/3样本的p99不是统计尾部界，没有点云解码/ROS/DDS/车机CPU/实车误差。

## 有效近场压力与deadline失败

`planner_worstcase.json`四组512观测、每组3重复，均确认有效map AABB或known-CV动态circle；保留完整阈值。unbounded full与baseline命令相同。

| 512对象场景 | baseline p50 ms | full p50/p95/max ms | .08s预算输出 |
|---|---:|---|---|
| near静态 | 2272.85 | 68.57 / 69.68 / 69.80 | 完整可行命令；p50 70.63ms |
| dense静态 | 2522.26 | 52.23 / 52.58 / 52.62 | 完整可行命令；p50 49.65ms |
| mixed动态/静态 | 2428.35 | 56.94 / 57.48 / 57.54 | 完整可行命令；p50 54.53ms |
| 已在-.3m/s倒车、转角估计.3rad、近场侧障，目标在前方 | 2739.52 | 118.14 / 119.07 / 119.17 | False / v=0 / w=0，reason=deadline；p50 80.18 / p95 80.21 / max80.21ms |

最后一组完整窗口111候选/111可行，在评分阶段超过budget，被整轮丢弃；checked111是已完成可行验证数量，不能据此发布尚未完整评分的某个命令。停车命令用同自行车/CV/响应/制动模型重新独立检验为safe；这不是实车制动标定或永不被外来目标撞击保证。原子NumPy调用导致约.2ms budget越界，hard.3s外层guard仍在。

候选遥测：startup没有plan时count=None；在窗口尚未构建完整时candidate/feasible仍None、checked可能部分。构建完整但评分deadline时candidate/feasible可知。C4回退可能再次构建同一窗口，所以checked是**候选验证调用次数**，可大于最终窗口candidate_count；不是唯一候选集合大小。has_safe_motion反事实保存/恢复所有真实plan遥测，不混入该计数。

## 有意义的自动回归

新增`tests/test_research_planner.py`12个测试通过，包括40随机matrix/scalar混合矩形/circle/unknown/generic OBB一致、120随机完整时域判定一致、8组同输入采样选择、invalid值fail closed、snapshot不能跨更新复用、deadline清理和计数、static反事实状态恢复、有界SE2持久路径与U反向出口、unknown/新goal/timeout、未知模式生产禁用/离线轨迹、3s中动态逼近、ready超时/丢失制动/后方阻塞。最终全栈测试由主agent在冻结后统一执行。

`planner_mode_comparison.json`30组动作/场景+2ready场景：ACK直行/转向、ACK倒车、SPIN、CRAB、TRANSLATE。全日程含等待和制动，open中ACK约3.15s、SPIN约9.0s、crab/translate约9.15s，其中6s为两次机械等待。窄通道ACK直行/倒车可行而spin扫掠或侧移不适用；side obstacle下侧移/自转被风险筛选；rear blocked倒车禁止；3s等待中横穿到车旁阻止switch动作。不同动作终点任务不同，这些数据比较通行能力/风险/动作时间，不能伪装同目标最优路线。CRAB/TRANSLATE protocol_verified=False，不能生产部署。

## 实车验收与回滚

部署仍使用原`ros2 launch car_launch run_rda_dwa.launch.py`，默认无新模型/求解器依赖；新增Python模块在原package目录，colcon安装须由车机构建。根参数配置/下游日志接线由主/C负责，本分支未启动ROS、车辆或UDP。

- 静态同目标原始点云rosbag与标定足迹：直/弯/多块/长墙/U/厚U/窄U/通道，报告真实最小净空/到达/路径与deadline，搜索无解/超时不误记“确实无路”。
- 动态known/unknown/遮挡/急反向及ready等待中逼近：由受控目标执行、完整源时间/速度假设，检查未来误差和实际停稳，当前CV不能保证对任意加速目标安全。
- 执行轨迹受滤波/控制或参数调整改变，必须重新做制动/碰撞验证；实际转角无反馈、轮速未独立标定、后轴距离假定是测量缺口。
- 回滚按主baseline_manifest和zip/hash逐文件恢复；batch_temporal_enable/reuse_nominal_rollouts可分别关闭作消融，未知mode默认继续disabled。关闭deadline只供离线性能比较，不能当车辆失联/超时安全策略。
