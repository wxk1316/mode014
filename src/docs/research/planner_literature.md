# B 规划参考目录、路线与理论到代码

核查/检索截止：2026-11-06。优先原文、作者版本、官方仓库与注册 DOI 元数据。下列“影响”使用可核实采用/后续原文引用，而非未经核对的引用数字。读取的每个原文只在此简短概述；原文链接提供完整论证。未将差速、飞行器或理想执行的实验等同本车。

## 分类参考目录

| ID/主题 | 作者、标题、年份与刊物 | DOI/稳定原文、官方代码/许可 | 实验条件、影响证据与本车边界 |
|---|---|---|---|
| B01 规划性能/可停安全 | Dieter Fox, Wolfram Burgard, Sebastian Thrun. **The Dynamic Window Approach to Collision Avoidance**. 1997, IEEE Robotics & Automation Magazine 4(1), 23–33 | [DOI 10.1109/100.580977](https://doi.org/10.1109/100.580977)，[CMU 原文](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)。未核实原作者可部署代码/许可；无需权重 | 同步驱动 RHINO，实物约0.95m/s；作为GF-DWA2025的基础引用，持续影响可查。速度可达集与可停集原则适用，原始圆弧/同步驱动不能代替本车转角速率、倒车停稳和动态可达域。 |
| B02 静态/非完整搜索 | Dmitri Dolgov, Sebastian Thrun, Michael Montemerlo, James Diebel. **Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments**. 2010, IJRR 29(5), 485–501 | [DOI 10.1177/0278364909359210](https://doi.org/10.1177/0278364909359210)，[作者早期原文](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)。未核实该作者版完整官方运行代码/许可；无需权重 | DARPA Urban Challenge/停车场，SE(2) Hybrid-A*后优化；Smac原文明确继承。可借鉴非完整基元和拓扑引导，不能由有限局部激光地图推导全局完备/动力学执行保证。 |
| B03 2019规划性能/优化 | Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu, Shaojie Shen. **Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight**. 2019, IEEE RA-L 4(4), 3529–3536 | [DOI 10.1109/LRA.2019.2927938](https://doi.org/10.1109/LRA.2019.2927938)，[原文](https://arxiv.org/abs/1907.01531)，[官方 Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)，GPL-3.0，ROS/C++、无需训练权重 | 3D quadrotor、离散加速度搜索+B-spline/时间调整；EGO原文用其作基线，是明确后续采用。3D双积分控制不能直接套车的曲率/转角；仅参考“搜索提供安全拓扑、优化改善平滑”的系统分层。 |
| B04 2020静态拓扑/局部极小 | Boyu Zhou, Fei Gao, Jie Pan, Shaojie Shen. **Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths**. 2020, ICRA, 1208–1214 | [DOI 10.1109/ICRA40945.2020.9196996](https://doi.org/10.1109/ICRA40945.2020.9196996)，[原文](https://arxiv.org/abs/1912.12644)，[Fast-Planner官方库](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)，GPL-3.0、无需权重 | quadrotor拓扑多路径/引导优化、室内外飞行。证明局部代价极小需要更大拓扑信息的研究线索；本车采用单一持久搜索路径指导DWA，未复制论文的并行优化或其性能结论。 |
| B05 2021规划计算/优化 | Xin Zhou, Zhepei Wang, Hongkai Ye, Chao Xu, Fei Gao. **EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors**. 2021, IEEE RA-L 6(2), 478–485 | [DOI 10.1109/LRA.2020.3047728](https://doi.org/10.1109/LRA.2020.3047728)，[原文v2](https://arxiv.org/abs/2008.08835)，[官方代码](https://github.com/ZJU-FAST-Lab/ego-planner)，GPL-3.0、ROS/C++、无需预训练权重 | ESDF-free障碍必要信息+B-spline/L-BFGS、实物飞行与规划对照；官方后续EGO-Swarm可查。借鉴避免冗余信息/重复计算；本轮批量SAT/保守剔除是独立等价工程实现，不称为EGO优化器。注意v1只有4作者，正式DOI/当前版本5作者。 |
| B06 2024预印本/2026工程搜索 | Steve Macenski, Matthew Booker, Joshua Wallace, Tobias Fischer. **Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics**. arXiv v1=2024-01-23、v2=2025-05-08；截至2026-11-06正式刊物/DOI未核实，2026官方工程维护 | [原文 2401.13078](https://arxiv.org/abs/2401.13078)，[官方 Nav2 Smac](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner)，[package.xml Apache-2.0](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/package.xml)。ROS2/C++/OMPL/costmap，无需权重 | 原文随机10,000m²图、1000起终点、Ryzen5600X；仓库Nav2默认支持/维护，是工程影响证据。Sec IV/V启发缓存、Hybrid-A*/lattice、非圆形足迹适配本车；依赖完整costmap和C++栈，当前局部观测/算力未实测，先用轻量有界SE2引导，不能复用论文20–300ms为本车时延。 |
| B07 2025静态凹形/动态场 | Ze Zhang, Yifan Xue, Nadia Figueroa, Knut Åkesson. **Gradient Field-Based Dynamic Window Approach for Collision Avoidance in Complex Environments**. 2025, IROS, 19669–19674 | [DOI 10.1109/IROS60139.2025.11246091](https://doi.org/10.1109/IROS60139.2025.11246091)，[原文](https://arxiv.org/abs/2504.03260)，[作者官方库](https://github.com/Woodenonez/CollisionAvoidance-GPDF-DWA)，MIT；JAX/AOT、无训练/预学权重 | SecIV unicycle、dt=.2s、N20、至多84候选，i7-9750H GPDF推断约8ms；五静态含U及多机器人仿真，与DWA/MPC/DDPG比较。新论文有原文/官方代码，尚不称持续高影响。全局形状梯度值得研究，但没有本车轮型时序/真实执行模型，故不替换硬动态安全筛选。 |
| B08 2026优化/批量几何/混合模式 | **EXACT-MPPI**, arXiv 2605.29663（作者/完整元数据由主目录记录） | [原文](https://arxiv.org/abs/2605.29663)，[作者官方库](https://github.com/caseypen/EXACT-mppi)，Readme.md存在；仓库license未明确，不能假定许可。Ubuntu22.04/Python3.10/JAX CPU或CUDA12/ROS2 Humble/Gazebo/Docker，无训练权重 | Sec IV-D批量K×T×N几何、IV-F模式成本/切换惩罚/冷却/停车与当前方向相符；Sec V-F动态仅当前点云，无显式未来预测。属新方法参考，未验证影响/本车预算；不导入依赖、不替代unknown/制动/3s真实ready检查。 |
| B09 专业几何 | Christer Ericson. **Real-Time Collision Detection**. 2005, Morgan Kaufmann | [作者书站](https://realtimecollisiondetection.net/books/rtcd/)，ISBN 9781558607323，非开源算法包许可 | bounding volume、分离轴和保守距离下界的专业依据。本轮实现闭式SAT/三角不等式下界，非某个论文新算法或安全认证。 |

补充动态系统路线来源：Kim P. Wabersich, Melanie N. Zeilinger. **A predictive safety filter for learning-based control of constrained nonlinear dynamical systems**. 2021, Automatica 129, 109597. [DOI 10.1016/j.automatica.2021.109597](https://doi.org/10.1016/j.automatica.2021.109597)，[作者原文](https://arxiv.org/abs/1812.05506)。本轮未核实对应官方可部署实现/许可或权重，未引入依赖。用于说明预测认证/安全policy与模型误差约束的条件；当前DWA有限backup未建立其终端安全/递归可行条件，不能直接援引理论保证。动态迎面追加实验见dynamic_encounter_experiment.md/json，生产改法已明确拒绝。

## 路线比較与选择

| 路线 | 收益及条件 | 依赖/预算/失败路径 | 本轮选择 |
|---|---|---|---|
| 保留DWA、等价批量/本帧复用/保守剔除 | 先消除已复现热点，候选/评分/动态硬约束保持；观测同输入可直接比较 | NumPy已有依赖；O(C·T·N)上界，N较多时常数下降；内部预算超时丢弃本轮输出 | 已实施默认开启；小场景≤4障碍保留标量backend避免批量开销 |
| 持久局部SE2搜索指导DWA | 多块/凹形需要拓扑绕路；带前后向曲率基元，不靠单最近对象的两点引导 | 纯Python/heapq、.3m/24heading、6基元，15ms可配置/200扩展，2s重试；无解/超时无引导，DWA安全停或有限恢复 | 已实现LocalStaticGuide，C接入；有限观测不是全局完备地图 |
| 完整Smac/lattice + DWA/控制安全层 | 长墙、复杂U、死胡同更广的系统路线，精细costmap/启发提高成功与质量 | ROS2 Nav2/C++/OMPL、地图契约/启动迁移；观测unknown/动态层/反馈时序需专门适配 | 作为下一阶段系统选项，未把官方速度当车机速度 |
| B-spline/SQP/MPC/MPPI + 硬安全复查 | 可优化jerk/多段控制/模式成本，需要执行器滞后、状态和误差界 | 新solver/JAX/地图；局部极小/超时必须安全停；真实转角无反馈、轮速未独立标定 | 不优先整体替换；保留比较及必要输入补齐条件 |
| GF-DWA/GPDF梯度引导 | 已给出凹形几何仿真原文；可以拓扑/局部梯度合用 | JAX依赖/GP训练矩阵/AOT；原动力学是unicycle；没有模式等待动障安全 | 未导入，仅解释静态拓扑信息价值 |

## 已实施的证据 → 来源 → 条件 → 代码 → 失败 → 对照

### P-B1 规划热点与安全等价提速

当前证据：`EnhancedDWA._executable_window`每样本调用`command_safe`生成nominal，`_evaluate_production_window`再次生成nominal；时间足迹逐瓦片分别创建NumPy数组。冻结基线512远场静态瓦片可复现秒级周期；旧审查里“碰撞只看当前位置”已经修复，不再作为动机。

来源：B01可达/可停控制原则；B09分离轴/包络距离下界；B05必要信息/避免冗余的工程启发；B08批量几何是路线比较。这里只宣称原有安全模型的等价计算提速。

状态为几何中心`[x,y,yaw]`，控制`[v,delta]`，单位m/rad/s；自行车后轴到中心保留-.345m参数。车辆支持函数`h_A(n)=L/2|n·e_long|+W/2|n·e_lat|`，矩形分离下界`g=max_n(|n·(cB-cA)|-h_A(n)-h_B(n))`，轴集合为双方两独立边法线。动障circle与unknown可达域维持`pB(t)=pB0+vB t`、半径`rB+max(q,unknownMaxSpeed)t`。检测依赖源时间已外推到now；不能把运动heading当shape yaw。

采样间安全阈值仍`margin+(max translation+footprintRadius·|deltaYaw|)/2+movingBound·maxDt/2`。整个时域的circ下界，只在其大于阈值时跳过硬撞检；soft最小SAT用必含world x/y轴的保守下界剪枝，generic OBB始终保留。动态加速/遮挡/急转若突破已给速度/误差界，等价提速不能生成新的安全保证。

代码：`CollisionChecker.begin_scene/end_scene`限单次`plan`、`_prepare_temporal_scene/temporal_gaps_many/_temporal_gaps_prepared/min_temporal_gap/time_collision_free`；`VelocitySample.nominal_trajectory/nominal_times`仅同输入本轮存储；`EnhancedDWA.plan`deadline wrapper、`_evaluate_production_window`复用。生产适配后的command_safe独立复查不读取候选缓存，安全复用不跨障碍revision/位姿/转角更新。

预算：原O(C·T·N)上界不变，准备O(N)、固定64对象块，常见case硬检查Nnear、soft遍历O(N)廉价下界后处理Nnear·T。100Hz/10Hz等周期承诺必须实测；ROS当前10Hz预算.08s由root接线。超时`last_failure_reason='deadline'`且输出False/0/0、candidate/feasible total未知，checked为部分数量。不能让计算故障触发trap几何判断。

对照：`tests/test_research_planner.py`随机matrix/scalar SAT/circle/OBB/unknown约束、120组整时域判定、同输入命令、快照更新、deadline清理；`planner_benchmark.py`从冻结zip隔离解释器加载baseline，与batch-only/full/full-budget对照16场景×12重复，输出有限样本p50/p95/p99/max/deadline、nominal模型净空/航向/路径与失败类型。真实日志/车机测时仍未获得。

### P-B2 多块/凹形的局部持久引导

证据：当前两阶段detour只锁单最近障碍，仿真GlobalGuide依赖oracle点且不在当前launch。来源B02、B06的SE2基元/启发搜索，B04的拓扑引导；没有将其global completeness套本车。

状态key`[round(x/.3),round(y/.3),round(yaw/(2pi/24)),direction]`，连续pose保留；基元±方向、δ=0/±30°、长.9m、6步几何中心路径。每弧矩形+sweep查确认静态，heuristic Euclidean distance，reverse/转角/换向附非负成本；这些是方向偏好，不是机械时间证明。输出仅map waypoint，真正转角速率、动障、后方、停稳和制动由DWA重新验证。

代码`local_guide.LocalStaticGuide.update/reset/_arc/_key`；new goal>.3m清空；单调cursor保持所选路线2s；过期/无解/扩展上限/预算无路径并2s冷却。静态确认必须type block/static_large、speed≤.05且unknown=False，不能偷偷把动态让行变静态。有限8m以内patch、200扩展默认15ms可由调用override剩余周期；旧引导不能放行不安全轨迹。

验证：单块搜索能得到绕行路径；U开口朝后可产生后退基元；timeout/unknown/new goal回归。benchmark闭环与C集成结果另列；不将搜索waypoint成功记为车到达。

### P-B3 多运动模式离线基元与机械等待

证据/来源：硬件确认0 ACK/4 SPIN与3s真实ready；B02/B06非完整±基元，B08混合成本，标准平面刚体运动学；等待规则为用户事实/专业工程，非瞬时切换论文定理。

`hybrid_primitives.evaluate_maneuver`贯通当前pose/速度估计→停止→至少3s+ready→ACK±/SPIN/CRAB/TRANSLATE参考路径→制动→至少3s恢复等待→全时域足迹/动态风险。ACK用`yawdot=v tan(delta)/.69`与后轴转换；spin中心原地yaw；CRAB/TRANSLATE假设`xdot=s cos(yaw+alpha),ydot=s sin(yaw+alpha),yawdot=0`。未知command/feedback/angle/speed单位参数化，default disabled/unverified；production=False只能显式simulation评估。它不发布ROS或UDP，生产controller仍保留真实模式到位/反馈/速度门。

全schedule时域检查包括等待中动障接近、ready晚到/超时和丢失后的制动尾段；reverse includes停稳确认假设.2s。未知模式即使仿真安全仍不能部署；方位/速度语义、实际四轮角度和轮型余量需受控实测。斜行/平移和自转是否比ACK绕行更优需把两次3s等待、动态接近及净空计入，不能只比较几何路径长度。
