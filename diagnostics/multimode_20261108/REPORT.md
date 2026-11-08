# 三模式改进与闭环实验报告

日期：2026-11-08。本文是本地闭环仿真、独立几何检查与组件测试；没有启动实车ROS/UDP，没有将仿真当成实车测试。

## 改动结果

加入ACK=0、CRAB=1（连续±90°，含反向平移）、SPIN=4。规划选择模式、跟踪执行模式专用控制，C++驱动统一串行切换。跨模式至少等待3秒；停稳、复位和目标轮角准备另按实际时序计算。模式回显不是机械到位。

普通规划保留异步worker；时间紧迫的迎面避让用小规模最新状态候选，完整动作锁存，持续重新检查。倒车动作包含反打和制动时序，终点未到安全位置时不按定时器提前结束。单路障保留原ACK可见性引导，密集场景使用车体矩形的局部引导。

## 实验口径

车辆几何中心起点(0,0)、航向0，默认目标(5,0)，道路y∈[-2,2]。车体1.2×0.8m，轴距0.69m。行人0.45×0.45m，迎面小车1.2×0.8m，标准路障0.6×0.6m。U形内宽1.8m、壁厚0.1m、后方开口；另有窄U压力测试和出口目标测试。

跟踪50Hz；驱动策略通过编译的C++桥直接运行；独立车辆积分和多边形距离检查50Hz。普通worker交付延迟0.25s使用实际AsyncPlanner队列/结果校验，虚拟时间含交付延迟；主线程计算墙钟另行统计。紧急父进程计算期间虚拟时钟不推进，因此数据不是实际ROS线程调度的实测。

基线为修改前保存源码，采用其旧3秒驱动模型与真值停稳门禁。候选版使用新C++策略和定位估计停稳，两种驱动口径逐结果标注，不能称基线也执行新策略。

扰动包含定位2mm/航向0.002rad噪声、60–180ms不均匀反馈、120ms观测年龄和间歇缺帧。所有开发失败日志保留；随机种子是工程回归，不宣称统计泛化或全局最优。

## 每个环境的预期与实际

| 场景 | 测试前预期 | 实际 | 时间s | 最小净距m | 基线到达/碰撞 |
|---|---|---|---:|---:|---|
| open | Keep ACK; no trap or spin; brake at goal. | 到达；模式0；trap记录0 | 10.62 | 1.600 | True/False |
| pedestrian_0.4 | Early safe side avoidance; compare ACK with 3s CRAB waiting. No trap for a passing actor. | 到达；模式0→1；trap记录0 | 27.52 | 0.176 | False/True |
| pedestrian_0.45 | Early safe side avoidance; compare ACK with 3s CRAB waiting. No trap for a passing actor. | 到达；模式0→1；trap记录0 | 26.72 | 0.180 | False/True |
| pedestrian_0.5 | Early safe side avoidance; compare ACK with 3s CRAB waiting. No trap for a passing actor. | 到达；模式0→1；trap记录0 | 26.52 | 0.228 | False/True |
| car_0.5 | Early side avoidance and checked waiting refuge. If switch cannot finish before contact, use ACK or stop safely; no unnecessary spin. | 到达；模式0→1；trap记录0 | 44.62 | 0.211 | False/True |
| car_0.75 | Early side avoidance and checked waiting refuge. If switch cannot finish before contact, use ACK or stop safely; no unnecessary spin. | 到达；模式0→1；trap记录0 | 29.52 | 0.300 | False/True |
| car_1.0 | Early side avoidance and checked waiting refuge. If switch cannot finish before contact, use ACK or stop safely; no unnecessary spin. | 到达；模式0→1；trap记录0 | 28.42 | 0.268 | False/True |
| single_barrier | ACK side arc if cheaper; otherwise one CRAB segment, hold side until obstacle passed. | 到达；模式0；trap记录0 | 15.12 | 0.185 | True/False |
| staggered_barriers | Continuous safe S/translation route; no left-right mode chattering or false trap. | 到达；模式0→1；trap记录0 | 36.82 | 0.162 | False/False |
| u_window | Check full spin sweep first. Rotate toward rear opening if safe, exit, then route around U to default goal; road-side corridors may be too narrow at .12m clearance. | 已退出U，原目标未达；模式0→4→0；trap记录223 | 90.00 | 0.179 | False/False |
| u_exit_only | Compare CRAB backing with SPIN+ACK; preserve body yaw when backing is cheaper. | 到达；模式0→1；trap记录0 | 12.12 | 0.500 | True/False |
| u_spin_blocked | Refuse unsafe rotation; compare CRAB backing exit without rotating body. | 到达；模式0→1；trap记录0 | 12.12 | 0.250 | False/False |
| staggered_tight | Retain original tight geometry as pressure test; safe failure allowed when route does not fit. | 未达/停车；模式0；trap记录0 | 90.00 | 1.100 | False/False |
| campus_bollards | ACK through central opening; no mode switch. | 到达；模式0；trap记录0 | 10.62 | 0.175 | False/False |
| campus_bench | Pass on right within road boundaries. | 到达；模式0；trap记录0 | 14.02 | 0.199 | True/False |
| campus_bins | Smooth alternating avoidance; avoid repeated switching. | 到达；模式0→1；trap记录0 | 38.42 | 0.159 | True/False |
| campus_planter | Side passage with full body clearance; return toward goal. | 到达；模式0；trap记录0 | 15.52 | 0.194 | True/False |
| campus_chicane | Use feasible S passage or CRAB segments; evaluate 3s cost. | 未达/停车；模式0；trap记录0 | 90.00 | 1.000 | False/False |
| campus_wall_gap | Straight ACK through wide door, no trap. | 到达；模式0；trap记录0 | 10.62 | 0.400 | True/False |
| campus_full_block | Safe bounded failure; do not claim goal reachable or spin indefinitely. | 未达/停车；模式0；trap记录0 | 90.00 | 1.600 | False/False |
| lateral_goal | One CRAB +90deg switch, translate without yaw; remain CRAB at arrival. | 到达；模式0→1；trap记录0 | 11.02 | 0.546 | True/False |

注：未达场景时间是观察预算，不能作为成功耗时。曲线来自实际积分的车体中心；最小净距是50Hz独立几何采样值，规划和执行预测还使用区间扫掠界。

## U形与不通过场景

给定1.8m内宽大于车体旋转扫掠直径约1.442m加两侧0.12m余量，允许自转。默认前方5m目标在闭合端外，车成功退出不等于能绕到该目标：外侧通道约1.0m，小于0.8+2×0.12=1.04m，因此此配置不满足绕行余量。

窄U、紧交错障碍、密集垃圾桶、道路全堵等测试的实际成功或失败均在上表保留。狭窄压力测试的安全未达不是“已脱困”。只在一个车体朝向下局部搜索失败，也不能宣称数学上的全局无解。

## 扰动结果

- seed=223：17/21到达，碰撞0，最小净距0.144m。
- seed=347：17/21到达，碰撞0，最小净距0.138m。
- seed=509：17/21到达，碰撞0，最小净距0.160m。

## 失败归因与修正

- 规划：旧引导可能选择长道路边界远端角点；3s短预测难以比较3s换模式成本。新增完整动作、矩形局部引导、低速近端候选、模式收益滞回。
- 跟踪/时序：原前向速度估计丢弃vy；负零倒车丢失方向；空等待计划沿用旧路；自转完成未等命令归零；反打输入与传输延迟未贯穿全时序。逐项修复并保留复现测试。
- 执行：切换前实际停稳、串行命令时隙、唯一READY、超时与反馈年龄；硬编码自转速度-2改成名义轮面系数0.429和可配置方向。
- 感知：主试验直接提供正确矩形来隔离算法；另运行实际GlobalKFTracker合成检测探查。未测试真实点云分割、遮挡和地面漏检，因此不能把实车感知问题判定为已排除。

## 参数选择

有限配置以安全、成功数、耗时、切换数依次排序；selected_config.json和tuning_results.json保留全部配置与失败。此处最佳仅指测试配置范围。

主要值：ACK输入±30°；CRAB±90°、转角速率30°/s、速度上限0.7m/s；ACK转角速率45°/s；a=0.7、dec=1.2、jerk=2；SPIN最大0.6rad/s，普通角精度4°、窄通路对准1°；换模式≥3s，切换截止12s，反馈硬门禁0.5s，停稳0.035m/s和0.06rad/s。

倒车采样20/25°为反馈留余量，控制上限仍30°；末端航向和停放射线检查。CRAB负速度与同模式换向先停稳；零速准备轮角不会重复发送切模式。

## 验证与限制

回归记录见regression_final.log；实际multiprocessing ACK/CRAB进程见real_worker_smoke_results.json；六条有向切换见transition_matrix_results.txt。没有在本机运行colcon/ROS全栈，C++测试验证的是独立策略，不能替代车机ROS编译。

实车必须核对angle_sign、spin_sign、自转omega到轮面速度系数、反馈速度单位、实际转向延迟与轮角到位。当前READY标注timed_open_loop；通信物理断链时上位机零速可能送不到，文档中车端保持末速的问题仍需固件看门狗/独立急停解决。

## 复现与文件

```bash
diagnostics/multimode_20261108/.venv/bin/python diagnostics/multimode_20261108/campaign.py --worker-delay .25 --duration 90
diagnostics/multimode_20261108/.venv/bin/python diagnostics/multimode_20261108/robustness.py
diagnostics/multimode_20261108/.venv/bin/python -m pytest src/rda_dwa_real/test -q
```

源文件补丁：three_mode_changes.patch；原始源码：baseline/。结果/逐场景轨迹：candidate_31/、baseline_31/和candidate_noise_<seed>/。图：requested_scenarios.png、campus_scenarios.png、mode_timeline.png。

## 假设与审查

两轮新上下文审查记录保留，实际模型身份未知、独立性未知，结果仅为provisional。末轮剩余基线标签问题已修正；之后的修正由执行者与测试验证，不宣称第三轮独立通过。

# 假设记录
<!-- ASK mode: never -->

| ID | 请求未确定的部分 | 本次选择 | Class | Source |
|---|---|---|---|---|
| A-001 | 90°命令接口 | 模式1，连续±90°，模式映射ACK=0/CRAB=1/SPIN=4 | interface | user |
| A-002 | 实验形式 | 本地闭环仿真；位置为车体中心；车体1.2×0.8m、轴距0.69m；含摆角/3s事务，不等价于实车标定 | semantic | default |
| A-003 | 未给出的障碍尺寸 | 行人0.45×0.45m，小车1.2×0.8m，路障0.6×0.6m；完整参数随场景保存 | semantic | default |
| A-004 | 目标和道路 | 初始中心(0,0),yaw=0；默认目标(5,0)；边界y=±2，作为可见固定障碍；U形开口朝后，额外出口目标用于区分退出和最终到达 | semantic | default |
| A-005 | 最优含义 | 有限配置网格中的验证最佳；安全优先，其次成功率、耗时、切换数；保留失败结果；不声明全局最优 | semantic | default |
| A-006 | 感知归因 | 主试验使用标注几何隔离规划/控制；额外输入噪声、延迟、丢帧；未使用真实雷达点云时不归因于实车感知 | semantic | default |
| A-007 | 计划协议 | v2显式语义mode和epoch；兼容现有v1的mode1=自转，模式内轨迹与切换事务分别编号 | interface | default |
| A-008 | 驱动到位证据 | 先保留回显后至少3s；无真实编码器明确open_loop等待，必要反馈失效不放行；单位/角度映射显式配置 | interface | default |
| A-009 | 回包与停稳 | 不均匀回包只更新状态；定位全平面速度窗口有效且持续停稳才切换；失效数据不能算静止 | interface | default |
| A-010 | U形尺寸 | 内净宽1.8m；壁厚0.1m；深度与停车位显式保存；窄到扫掠不可行时必须拒绝自转，不能强求成功 | semantic | default |
| A-011 | 参数选择验证 | 调参集为pedestrian_0.5/car_1.0/single_barrier/staggered_barriers/campus_planter/lateral_goal；其余校园几何、其余动态速度与噪声种子为验证集；同预算同输入比较保存基线和改进版 | semantic | default |

## 说明

物理停车、轮胎滑移和底层速度单位尚未实车标定。模拟器为独立积分与几何检查，不直接把规划预测作为真实运动；每次试验先保存预期动作，再检查日志。

| A-012 | 动态避障调参范围 | 转角速率30/40/45°每秒（接口上限45），横向加速度0.6/0.8/0.9m/s²，guide_extra 0.03/0.08/0.12；仿真选择后不称实车已验证 | interface | default |

| A-013 | 近距离快速迎面目标的可行性 | 先评估前进ACK，近迎面动作达不到0.4m操作余量时比较已有底盘支持的ACK倒车侧避；3s切换影响另外的CRAB候选，不作为倒车分支的独立门槛；实际执行保持mode0，倒车须先停稳并按独立带符号模型检查 | interface | default |
| A-014 | 紧交错障碍的未指定间距 | 原始(2,.65)/(3.4,-.65)保留为staggered_tight压力测试；常规主场景采用(1.5,.65)/(3.7,-.65)，两种均保留结果与失败 | semantic | default |

| A-015 | 跨模式后续航向成本 | ACK预测末端航向使后续静态矩形通路不可行而当前航向CRAB可行时，将方向纠正的双切换6s纳入ACK比较；属于工程启发式，不宣称精确任务时间 | semantic | default |
| A-016 | CRAB速度调参 | 0.3/0.5/0.7m/s有限比较，角速率仍≤45°/s；配置选择仅经仿真，不等于实车轮胎/动力学校准 | interface | default |

| A-017 | 跟踪安全回退 | CRAB和倒车执行同时检查立即制动尾迹；SPIN按当前实际yaw_rate预测短时旋转扫掠，不用3s后的理想旋转替代当前执行检查 | interface | default |

| A-018 | 零速度时的方向语义 | v2携带direction=±1；倒车制动到-0仍保留倒车控制分支，空参考的等待计划清除旧运动参考 | interface | sweep |
| A-019 | 异步实验含义 | 调参主循环同步求解，虚拟物理时间不含主线程计算墙钟；追加使用实际AsyncPlanner结果检查和0.25s模拟worker交付延迟的闭环验证，另报告实测求解耗时；不称真实ROS调度测试 | semantic | sweep |
| A-020 | 动态前进与倒车选择 | 先评估迎面目标的前进ACK完整动作；要求0.4m操作净距才保持前进，否则比较倒车；非迎面和远目标不无条件倒车；0.4是根据开发噪声失败调整的仿真操作余量 | interface | sweep |
| A-021 | 新目标打断自转 | 新目标撤销旧SPIN意图并在当前机械模式停稳，安全返回ACK后重新规划；不沿用旧目标角或旧结果会话 | interface | sweep |

| A-022 | 连续停稳遇到失效数据 | 驱动和跟踪都清除停稳起点；RESET阶段回到BRAKE；失效间隔不能计入停稳或自转完成保持 | interface | sweep |
| A-023 | 仿真切换停稳证据 | 候选版C++策略使用与生产相同的定位速度估计和有效窗口，不使用真值速度放行；真值仅用于独立运动积分、碰撞和评价 | semantic | sweep |
| A-024 | ready轨迹字段 | 候选版保存C++driver.ready，基线为旧模型ready_at；另存command_in_actual_mode；模式相同不能被标记为机械已完成 | semantic | sweep |
| A-025 | 异步父进程候选状态 | 父进程switch_candidate显式同步当前摆角/观测年龄/加速度/模式epoch，与worker输入一致 | semantic | sweep |
| A-026 | SPIN完成语义 | 实际停稳、输出角速度近零且满足配置spin_stop_hold才报告reached，不用单次位姿判定替代 | semantic | sweep |

| A-027 | 基线驱动与候选驱动的区别 | 基线保留旧3s时序模型及真值停稳门禁；候选执行新C++策略并使用定位估计停稳。scope/driver_model/ready字段逐结果注明；不能称基线也执行新C++策略 | semantic | sweep |
| A-028 | 时间紧迫的动态避让 | 已确认迎面目标的短TTC或活跃逃逸宏动作，用最新状态执行小规模父进程规划，普通轨迹继续异步；实测预算单独记录，不把同步紧急路径称为异步worker | interface | default |
| A-029 | 逃逸过程的停车回退 | 普通动作须有安全制动尾迹；正在执行完整、当前观测再次验证且末端在运动目标未来射线外的逃逸宏动作时，可继续该宏动作并报告制动尾迹间距，不能用原车道停车替代逃逸。失联仍零驱动，不能保证持续冲撞者避让 | semantic | default |

| A-030 | 低速定位导数噪声 | 位移跨度≤0.03m且二阶端点速度≤0.08m/s时用同窗口一阶回归估计低速，不强制置零；高速/加速保留原二阶导数 | interface | default |

| A-031 | 噪声验证用于修正后的划分 | 种子7/19/43属于开发验证，发现失败后用于调整；最终额外用101/127/179验证，保留原始失败，不宣称开发验证仍为留出集 | semantic | default |
| A-032 | 窄通路与航向保持启发式 | 两侧近邻静态障碍且ACK中间航向使后续矩形通路不可行时，计入潜在6s纠正代价；单侧障碍不自动套用此额外代价 | interface | default |

| A-033 | 自转单位 | 上层CarCtr模式4速度为rad/s，UDP轮面速度默认omega×0.429m并继承spin_sign=-1；删除硬编码-2，提供参数，0.429是名义几何值而非实车标定 | interface | default |
| A-034 | 倒车跟踪余量 | 逃逸基元采样20/25°留出至30°的跟踪余量；末端航向误差≤15°且路径/停放射线净距≥0.22m；位置和航向反馈增益1.5/1.8；非全局最优保证 | interface | default |
| A-035 | 恢复目标的航向基准 | 保存原目标的初始航向基准，临时恢复子目标不覆盖它；避免恢复结束后按错误基准反复自转 | interface | default |
| A-036 | 最终调参时间模型 | 初始同步探索结果保留；最终六组配置用75s预算、0.25s模拟worker交付，与最终验证一致 | semantic | default |

| A-037 | 反向规划速度状态 | 仅运动预测中融合定位速度与80ms命令滞后模型；停稳仍必须使用新鲜定位证据，不用命令速度证明停车 | interface | default |
| A-038 | 命令延迟 | ACK/倒车模型对整个输入时序施加延迟，不仅对最初时段；反打和制动段同样延迟 | interface | default |
| A-039 | 逃逸完成 | 反向动作的输入时序锁存；末端未到时允许受约束低速补足进度；到达安全位置、航向及实际停稳后才结束 | semantic | default |
| A-040 | 双向CRAB | 模式1允许带符号平移速度；轮角仍限±90°；同模式速度换向先停稳，不能把负速度判为停滞 | interface | default |
| A-041 | 零速轮角准备 | 当前CRAB模式的目标角未到且移动候选不足时，验证停车等待并零速准备；不重新发送切模式 | interface | default |
| A-042 | 航向与原子性 | CRAB段航向参考锁存，累计漂移触发纠正；原子模式下不消费旧的独立路径清除消息 | interface | default |
| A-043 | 最终扰动口径 | 7/19/43与101/127/179为开发诊断；最终使用223/347/509，90s观察预算。所有开发失败保留，不把它们计作最终通过 | semantic | default |

| A-044 | 短引导点的模式候选 | 切换候选速度按局部引导点距离降低，加入启动加速时间；不因恒定高速越过短目标而误判该模式无效 | interface | default |
| A-045 | 换向等待计时 | 同模式换向等待从本次开始计时，不沿用此前模式请求或程序启动的12s截止 | interface | default |
| A-046 | 最终版本标记 | 最终仿真预算90s；程序中的数值改进不能替代实车轮角/速度标定。生成结果使用最终脚本，失败与未达仍保留 | semantic | default |


## 工具引用

图表采用Matplotlib技能；工具方法来源：[Scientific Agent Skills: A Library of Procedural Knowledge for Research Agents](https://arxiv.org/abs/2609.00065)，Timothy Kassis、Vinayak Agarwal、Yuhuan He、Darshil Patel、Aubrey M. Brueckner（2026）。该引用不是车辆安全性证据。
