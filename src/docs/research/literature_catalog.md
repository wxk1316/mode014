# 分类参考目录与理论条件

检索截止 **2026-11-06（Asia/Shanghai）**。原文/作者版本/官方仓库和许可已由并行agent逐项核对。历史review不是本輪检索证明；全文未取到、正式出版状态或代码许可未确认的条目明确标出。论文、专业理论、官方工程和本车启发式分开。

用户补充要求的近三至五年8篇重点见 [近期动态/跟踪重点对应表](recent_dynamic_tracking_focus.md)。Crossref本日登记被引数：预测安全过滤275、SimpleTrack145、RPP112、Ruckig95、Patchwork++186；这是登记链接，不是Google Scholar总量。GF-DWA为3，近期新论文不冒充高引。完整可复核记录见 [引用元数据](validation/recent_paper_impact.json)。

|主题|条目及阅读入口|本车采用边界|
|---|---|---|
|规划性能|B01、B03/B05、B07、R26-EX；下方B目录|batch/SAT/单plan复用是独立等价实现，不称EGO/GPDF优化器|
|动态避障|R25-DRA、R21-PSF重点表、A02/A03；[迎面追加实验](dynamic_encounter_experiment.md)|CV/unknown可达域不是校准概率；安全停车不阻止非合作目标撞击|
|静态与非完整约束|B02/B04/B06、C02|局部SE2非全局完备；倒车、轮型等待和足迹独立验证|
|平滑跟踪|C01/C03/C04|FF默认保留，不能只后滤波；缺实际转角和真实执行误差，未启OTG|
|多模式/混合系统/trap|C05/C06/C07、R26-EX|流/跳变/记忆结构适配，不继承单积分/静态sphere-world的GAS或采样完备定理|
|非模型感知/估计|A01—A08|Source dt、关联/生命周期/unknown；地面模型须raw bag真值|
|可选开源模型|A09/A10|没有安装模型；代码/数据/权重许可证分开，CPU/GPU/内存实测缺失|

同文 aliases：B08=R26-EX。模型与理论到当前类/函数/公式/失败路径分别在本目录各模块段和 [总体方案及验证](decisions_and_validation.md)。下面保留完整元数据及出处，供审阅与下一阶段恢复。


---

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

---

# 跟踪、多模式与 trap 原始文献核查

检索截止 **2026-11-06（Asia/Shanghai）**。优先原论文、作者版本、官方实现；先读v1再核查最新元数据。下面的工程实现不声称继承论文中未满足的稳定性、完备性或安全定理。

| ID / 类别 | 作者、标题、年份及载体 | DOI / 稳定原文 | 官方实现、许可、权重 | 核查实验条件与可核实影响证据 |
|---|---|---|---|---|
| C01 原始专业几何 | R. Craig Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, 1992, CMU-RI-TR-92-01 | [CMU原报告页](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/) | 原文未指定现代官方ROS实现；权重不适用。报告版权仍属作者/权利人 | 车辆几何推导与前视参数影响；延续性由C03原论文仍将PP/APP作为基线证明，未编造引用数 |
| C02 原始非完整约束 | James A. Reeds, Lawrence A. Shepp, *Optimal paths for a car that goes both forwards and backwards*, 1990, Pacific Journal of Mathematics 145(2):367–393 | [原期刊](https://msp.org/pjm/1990/145-2/p06.xhtml), DOI **10.2140/pjm.1990.145.367** | 原文无作者代码/学习权重；不引入第三方实现 | 有最小转弯半径、方向端点、前后向cusp的解析最短路径；障碍、停稳时间、转角速率不在该结论中。不是本车多点掉头安全证明 |
| C03 2023 / 平滑追踪与工程安全 | Steve Macenski, Shrijit Singh, Francisco Martín, Jonatan Ginés, *Regulated Pure Pursuit for Robot Path Tracking*, 2023, Autonomous Robots 47:685–694 | [作者全文](https://arxiv.org/html/2305.20026v1), DOI **10.1007/s10514-023-10097-6** | [Nav2官方包](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)；[package.xml许可Apache-2.0](https://raw.githubusercontent.com/ros-navigation/navigation2/main/nav2_regulated_pure_pursuit_controller/package.xml)；无权重 | §5 TurtleBot3 Gazebo理想定位与工业服务机器人、室内约70m路线；§6明确不含车辆动力学，Ackermann路径必须先满足运动学。影响证据是官方Nav2可用实现及原论文报告的部署，不把差速原地转向直接移植到本车 |
| C04 2021 / jerk优化候选 | Lars Berscheid, Torsten Kröger, *Jerk-limited Real-time Trajectory Generation with Arbitrary Target States*, 2021, Robotics: Science and Systems XVII | [RSS原文页](https://roboticsproceedings.org/rss17/p015.html), DOI **10.15607/RSS.2021.XVII.015** | [作者Ruckig](https://github.com/pantor/ruckig)；[MIT许可](https://raw.githubusercontent.com/pantor/ruckig/main/LICENSE)；C++/Python，无学习权重 | OTG多DoF位置/速度/加速度目标与v/a/jerk约束，论文十亿随机轨迹及实际应用。影响证据：作者持续开放实现、官方仓库约1.4k star（检索时观察值，不作为安全质量证明）。OTG不自动保证车辆足迹碰撞安全 |
| C05 2024 / 混合系统理论 | Miaomiao Wang, Abdelhamid Tayebi, *Hybrid Feedback for Affine Nonlinear Systems with Application to Global Obstacle Avoidance*, 2024, IEEE Transactions on Automatic Control（2023预印本） | [作者扩展全文v4](https://arxiv.org/html/2305.04104v4/), DOI **10.1109/TAC.2024.3372463** | 原文未核实独立作者代码许可/权重；实验用MATLAB HyEQ，不额外引入依赖 | §II流/跳变，§III synergy与Lyapunov，§IV单积分sphere-world、单圆障碍数值仿真。期刊版本/完整证明可核实；不宣称已有广泛引用，不能把GAS结论套到本车机械切换与动态障碍 |
| C06 2024 / trap记忆 | Joonkyung Kim, Sangjin Park, Wonjong Lee, Woojun Kim, Nakju Doh, Changjoo Nam, *Escaping Local Minima: Hybrid Artificial Potential Field with Wall-Follower for Decentralized Multi-Robot Navigation*, 2024, arXiv预印本 | [原文v1](https://arxiv.org/html/2409.10332), DOI **10.48550/arXiv.2409.10332** | 原文未找到可核实官方代码/权重及许可。RS规则版本不需模型；LS用ViT学习switch，本项目不用LS | §V-A记录hit/leave点、固定方向与目标距离进展以避免重复环绕；§VI TurtleBot4圆盘r=.17m、10m射线、PyBullet5Hz、Ryzen5900X/3070Ti/32GB与2–3实物机器人。新预印本不夸大影响，仅作适用机制证据 |
| C07 2024→2025 / 多模式工程 | Beverly Xu, Nan Wang, Ricardo Sanfelice, v1 *C++/OMPL Motion Planning Tools for Hybrid Dynamical Systems* (2024), v3 *cHyRRT and cHySST: Two Motion Planning Tools for Hybrid Dynamical Systems* (2025), IEEE CASE 2025扩展作者版 | [v1已读全文](https://arxiv.org/html/2411.11812v1), [v3元数据](https://arxiv.org/abs/2411.11812), DOI **10.48550/arXiv.2411.11812** | [官方cHyRRT](https://github.com/HybridSystemsLab/hybridRRT-Ccode), [LICENSE BSD三条款](https://raw.githubusercontent.com/HybridSystemsLab/hybridRRT-Ccode/main/LICENSE)；作者还列hybridSST-Ccode。C++/OMPL，无模型权重 | §3–5显式状态/控制、flow/jump set、unsafe set与混合时间；v1 bouncing-ball/tensegrity，v3 pinball扩展。上游HyRRT CDC2022与HySST CDC2023被持续官方实现支持；有限时间采样不足以继承其概率完备/近优性质 |

**知识类型分开**：C01/C02运动几何是专业原始理论；C03/C04/C07是论文及官方工程实现；本项目3秒+ready、最大重试2、.10m/1秒退出确认、5秒冷却、动态反事实分类和观测停稳门是结合本车契约的工程guard。后者不是某篇论文给出的通用最优阈值。

## 理论到当前代码

| 问题证据 | 来源 / 理论条件 | 本车适配与变量 | 实施 / 预算 / 失败 | 对照验证 |
|---|---|---|---|---|
| 重规划有效但定位/轮速一直不动，旧仲裁仍切自转 | C03 §6区分可执行参考与真实跟踪；专业系统辨识区分执行停滞和几何不可行。C06 §V-A用目标进展记忆，不以任意静止为trap证明 | ACK已ready且远目标，主DWA `last_has_motion`；静态反事实 `has_safe_motion` True/False/None；动态或unknown造成无解归yield，执行不动归execution_stall | vehicle_bridge._arbitration_step；只在blocked确认后调用20ms或cycle剩余probe；None不进入trap，停留原安全控制 | ±1.32m两墙有v=.05但观测0：基线1次自转→补丁0，staticprobe True；±1.14m真正无ACK可行：两版本均1次安全锁存 |
| trap内每横移>.5m重置次数，退出时没有实际重接推进 | C06 §V-A hit/leave及较优目标距离的更新条件；C05混合guard只作结构借鉴，不继承Lyapunov证明 | episode保存激活全局距离，内部位移不得reset；重接真实全局距离减少≥.10m、持续≥1s且安全；退出冷却5s。失败不清预算，新目标或失败后全局推进≥.5m才reset | vehicle_bridge._activate_trap/_update_trap/_arbitration_step；有限16s有效ACK动作预算，max_spin_attempts=2；timeout失败锁存但正常DWA仍能使用可行命令 | 同输入侧移.7m：旧次数2→0，补丁保留2；同位置重接1.2s：旧释放、补丁继续；真前进.15m后补丁释放/冷却 |
| 自转停止指令/固定.5s不证明yaw停稳，平均轮速也不证明四轮停 | C05 flow/jump分离＋本车机械协议；停稳/制动属于专业运动学guard，阈值为待实测工程值 | ACK max|wheel speed|≤.03m/s且fresh.3s保持.3s；停止自转新源yaw差分|ω|≤.03rad/s fresh.3s保持.5s、角误差仍≤4°。新源重复不刷新 | tracker/core＋node新源callback；STOP漂移→RUN原12s时钟不reset；停稳3s无证据FAULT保持当前模式；随后ACK仍≥3s+ready0 | 移动v=.2时旧.3s请求4，补丁不请求；一轮.1m/s均值.025旧允许反向，补丁禁止；停止漂移/旧源/超时/稳定到位6项新增反例 |
| unknown speed=0的自转盘/延迟后的ACK出口漏未来可达集 | 专业集合包含与CV边界，不称统计置信区间；B/感知的未来风险契约 | `p_o(t)=p_o+v_ot`，`r_o(t)=r_snapshot+max(.05,v_unknown_max)t`；snapshot已含过去age，停止/换轮/旋转/恢复delay再加未来域 | bridge._spin_safe/_obstacles_after_delay；所有等待同一时间轴；危险取消模式-1停车，不假装安全等待；O(MT)向量化 | 绕过假定角度时序的整盘检查仍拒绝5m未知对象；6s出口rebase使半径增加9m，原snapshot不变 |
| 长墙、多块/凹形只靠两点绕行范围不足 | C02非完整约束；B目录Hybrid-A*原始路线。搜索结果必须再经C03强调的可执行性/安全后验 | 局部SE2状态(x,y,yaw,方向)，.3m格、24heading bins、前后向3转角弧；静态路由只给map waypoint，unknown不变静态 | B的LocalStaticGuide接bridge，15ms/200exp、2s记忆/失败冷却，nearfootprint净空≤margin+.3m禁新前置搜索，DWA最终复查全动态/制动；不完备 | root full-stack/no_static_search闭环负责；原near/unknown不接guide安全测试保持 |
| 需要jerk改善但目前无实际转角反馈/制动完整标定 | C04 OTG须完整状态与v/a/jerk约束，C03 §6几何不含动力学 | 未来状态[v,δ,a,δdot]与jerk模型必须进入规划安全积分，不能只在50Hz后处理滤波 | 本轮保留前馈/45°s/hold和即时安全降速；不新增Ruckig生产依赖，不启tracking correction。下一阶段拿日志辨识执行器，再做统一rollout+OTG消融 | 本轮仅记录jerk/反号指标；真实平滑改善须车辆日志，不把平滑后轨迹当既定安全 |

2019–2026检索覆盖由主目录与B/A条目联合承担；C主要查2021、2023、2024、2025及持续影响原始方法。没有找到已验证适合本车的2026控制模型便不虚构目录条目。

---

# 感知、估计与可选模型参考目录

检索截止 **2026-11-06（Asia/Shanghai）**。优先读原文、作者仓库和许可文本；综述未作为实施依据。论文速度/检测基准与本车点云、雷达安装、低矮障碍、车机CPU和3秒换轮条件不同。以下“影响证据”采用原文后续比较、官方实现的使用关系或当日公开仓库计数；不虚报学术引用数。标记“专业理论”“官方实现”“工程启发式”区分证据等级。

## 已核实条目

| ID / 类别 | 作者、标题、年份及出处 | 原文/DOI | 官方实现、许可、权重 | 实验条件与影响证据 | 本车适用范围 / 读到位置 |
|---|---|---|---|---|---|
| A01 估计原始理论 | R. E. Kalman, **A New Approach to Linear Filtering and Prediction Problems**, 1960, *Journal of Basic Engineering* 82(1):35–45 | [出版社DOI](https://doi.org/10.1115/1.3662552)，Crossref出版元数据已核；出版社正文403，两个流传PDF链接404；另下载到的MIT PDF实际是Tony Lacey教程，未冒充原论文 | 原年代无官方代码/权重；本项目独立实现CV KF，未复制第三方受限实现 | AB3DMOT原文引用该方法，长期算法采用证据；不以KF名义保证真实分布已校准 | 专业KF状态空间/协方差公式；线性、白噪声和噪声协方差适配是条件；原全文未获取，未宣称章节阅读 |
| A02 非模型估计/跟踪 | Xinshuo Weng, Jianren Wang, David Held, Kris Kitani, **3D Multi-Object Tracking: A Baseline and New Evaluation Metrics**, 2020, IROS；2019先行稿标题为A Baseline for 3D Multi-Object Tracking | [最终作者稿](https://arxiv.org/abs/1907.03961)（PDF已读）、[2020短稿](https://arxiv.org/abs/2008.08063) | [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT)；[许可](https://github.com/xinshuoweng/AB3DMOT/blob/master/LICENSE)是**仅学术/非营利非商业研究**，不是MIT；KF本身无权重，前端检测权重另算 | KITTI/nuScenes、已检测3D box；最终稿报告207.4 FPS是跟踪范围，不含本车原始点云端到端。SimpleTrack/Poly-MOT原文与代码采用/比较；2026-11-06仓库API观察1850 stars仅工程采用信号 | 已读III-B/C/D/E及实验；保留“预测→匹配→更新→birth/death”顺序，不能把检测置信度或3D IoU硬搬到规则瓦片 |
| A03 非模型估计/跟踪 | Ziqi Pang, Zhichao Li, Naiyan Wang, **SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking**, arXiv 2021，ECCV 2022 Workshops | [原文](https://arxiv.org/abs/2111.09621)，[会议DOI](https://doi.org/10.1007/978-3-031-25056-9_43) | [SimpleTrack](https://github.com/tusen-ai/SimpleTrack)，[MIT](https://github.com/tusen-ai/SimpleTrack/blob/main/LICENSE)；跟踪无独立权重，前端CenterPoint检测/置信度另需模型；Python+NumPy/SciPy等 | Waymo/nuScenes；§5.1低质量高频检测可损害跟踪，20Hz不必优于10Hz。Poly-MOT明确比较，2026-11-06仓库观察400 stars | 已读§3、§4.2/4.3/4.4、§5.1、§6；借鉴组件消融/生命周期与“高频不等于高可信”，不复制GIoU或置信度门（本车没有可靠shape yaw/score） |
| A04 非模型地面 | Hyungtae Lim, Minho Oh, Hyun Myung, **Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor**, 2021, *IEEE RA-L* 6(4):6458–6465 | [作者稿](https://arxiv.org/abs/2108.05560)，[DOI](https://doi.org/10.1109/LRA.2021.3093009) | [作者实现](https://github.com/LimHyungTae/patchwork)；当前许可须随选定commit核对，未嵌入生产；非学习模型、无权重 | SemanticKITTI与粗糙地面；Patchwork++为明确后续扩展，构成方法延续证据 | 元数据与后续原文依据已核；当前固定z假设的替代路线，需原始点云/姿态/地面真值；本轮不以缺真值的synthetic地面删点部署 |
| A05 非模型地面 | Seungjae Lee, Hyungtae Lim, Hyun Myung, **Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud**, 2022, IROS:13276–13283 | [原文](https://arxiv.org/abs/2207.11919)，[DOI](https://doi.org/10.1109/IROS47612.2022.9981561) | [官方代码](https://github.com/url-kaist/patchwork-plusplus)，[BSD-2-Clause](https://github.com/url-kaist/patchwork-plusplus/blob/master/LICENSE)；C++/Eigen、pybind11、ROS2 Humble接口；无权重/训练 | SemanticKITTI；自适应地面似然、TGR/R-VPF/RNR。官方2026-11-06 API观察1128 stars、ROS2/C++/Python多接口为工程采用证据；不是本车漏检保证 | 已读§III-A—E、§IV；安装高度/反射/地面姿态条件要重新验收。本轮不引入其“ground”删除结果到总安全输入 |
| A06 非模型估计/路线对比 | Xiaoyu Li, Tao Xie, Dedong Liu, Jinghan Gao, Kun Dai, Zhiqiang Jiang, Lijun Zhao, Ke Wang, **Poly-MOT: A Polyhedral Framework For 3D Multi-Object Tracking**, 2023, IROS | [原文](https://arxiv.org/abs/2307.16675) | [官方代码](https://github.com/lixiaoyu2000/Poly-MOT)，README声称MIT，但仓库API未识别license文件，本轮不复制；检测器权重独立 | nuScenes、Python、Intel 9940X无GPU跟踪；§IV-C报告3 FPS，说明更复杂几何/关联不当然更快；Easy-Poly延续对比及官方README列SimpleTrack/AB3DMOT依赖，观察197 stars | 已读§III-B/C/D、§IV-B/C；类别特定CTRA/bicycle需要可靠类别与朝向，本车规则type不可当semantic class，因此不选其整套替换 |
| A07 近期跟踪探索 | Peng Zhang, Xin Li, Xin Lin, Liang He, **Easy-Poly: An Easy Polyhedral Framework For 3D Multi-Object Tracking**, arXiv 2025，v3于2026-02-13更新；未核实正式会刊 | [原文](https://arxiv.org/abs/2502.17822)，[稳定DOI](https://doi.org/10.48550/arXiv.2502.17822) | 原文模板仍有占位会议DOI，不能当正式出版DOI；公开官方实现/明确许可/权重未查到，不引入 | nuScenes多模态检测、confidence weighted KF/自适应协方差；新方法持续影响证据尚不足，只记最新研究 | 已读§3.3/3.4/3.5、§4；本车没有神经置信度，不能伪造score驱动R/Q；仅作为后续校准的导航 |
| A08 近期非模型地面 | Muhammad Haider Khan Lodhi, Christoph Hertzberg, **GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds**, ICRCV 2025作者接收稿，2026-03-04上传 | [原文](https://arxiv.org/abs/2603.04208)，[出版DOI](https://doi.org/10.1109/ICRCV67407.2025.11349133)；年份按正式2025与上传2026分开 | 原文说代码接收后将公开，未核实官方已公开仓库/许可；无学习权重，不引入 | SemanticKITTI，局部特征值/梯度扩张，原文§V-E列复杂植被/运行时波动；新文无长期影响断言 | 已读§III-B/C/D、§V-E；用作对固定z平地假设的最新对照；本车低障碍false-ground需独立真值验证 |
| A09 可选学习检测（未接入） | Alex H. Lang, Sourabh Vora, Holger Caesar, Lubing Zhou, Jiong Yang, Oscar Beijbom, **PointPillars: Fast Encoders for Object Detection from Point Clouds**, 2019, CVPR | [原文](https://arxiv.org/abs/1812.05784) | [作者代码](https://github.com/nutonomy/second.pytorch)，[MIT](https://github.com/nutonomy/second.pytorch/blob/master/LICENSE)；原仓库README未核到可下载预训练链接，后续实现的权重须另核 | KITTI；§6 Intel i7+1080Ti报告16.2ms全pipeline/约62Hz，不能套Jetson/CPU；CenterPoint等使用pillar backbone为影响证据 | 已读§2、§6；相对稠密真值车/人检测有用，未知低矮静态物体不应靠网络过滤规则点集 |
| A10 可选学习检测/跟踪（未接入） | Tianwei Yin, Xingyi Zhou, Philipp Krähenbühl, **Center-based 3D Object Detection and Tracking**, 2021, CVPR | [原文](https://arxiv.org/abs/2006.11275) | [作者代码](https://github.com/tianweiy/CenterPoint)，[MIT及NOTICE](https://github.com/tianweiy/CenterPoint/blob/master/docs/NOTICE)；[官方模型表](https://github.com/tianweiy/CenterPoint/blob/master/configs/nusc/README.md)列SharePoint下载目录，文件未下载/哈希验证；nuScenes/Waymo数据含非商业约束，模型训练数据条款独立核对 | nuScenes/Waymo，预训练模型，中心+速度回归、最近邻跟踪。SimpleTrack/Poly-MOT显式采用/比较是可核影响证据 | 已读§4、§5、附录A/B；仅条件候选，不声称本车易部署或实时，详下表 |

## 优先实现与理论对应

A02/03支持保持轻量模块、采用源帧顺序和生命周期、对运动模型/关联单独消融；**论文并没有直接提出本补丁所有阈值或本车unknown策略**。本轮具体门与恢复是工程启发式；圆包络可达界是专业集合/三角不等式推导。

- CV KF变量 `s=[x,y,vx,vy]`，位置m、速度m/s，`F(dt)`用真实源间隔，`P⁻=FPFᵀ+Q`、`K=PHᵀ(HPHᵀ+R)⁻¹`。`Q/R`未统计校准，不能称95%置信域。
- 关联约束：两次观测中心距离 `||z_k-z_last|| ≤ v_max Δt + 2 ε_p`。前提是障碍中心速度界和两次位置误差界都成立；`v_max=1.5m/s`、`ε_p=.15m`是现运行假设，不是完整实车/行人标定。失败边仍保留新unknown和旧轨TTL。
- 速度条件：最初第二量测差分，后续约`.2s`真实源窗口差分用于速度界核对；同时瞬时速度差 `||v_fd-v_pred||>.8m/s` 且位置创新 `||z-Hs⁻||>.15m` 为模型不一致。位置立即使用本次框，速度设未知；非重复且间隔不大于窗口的兼容量测累计`.2s`重确认。
- unknown未来集：`p(t)∈B(z_current,v_max t)`，与已观测足迹圆包络作Minkowski和。source age已在snapshot包络；模式动作时间重新从0起算时，必须先把等待/动作/recover `delay`的可达半径带到新的time0。C分支因此新增`_obstacles_after_delay`，不能省略已等待的unknown域。
- 不能保证超过1.5m/s的真实目标、看不见的实体、非高斯/更大质心跳变、错误定位初始化或车辆不能按制动模型停下。阈值/恢复必须记录实际源频率、误差、unknown时长、误停车与漏障；当前代码不会让其成为概率或形式安全认证。

## 可选模型的部署门槛（未实施、无比较收益断言）

| 项目 | CenterPoint 条件候选 |
|---|---|
| 依赖/平台 | [作者INSTALL](https://github.com/tianweiy/CenterPoint/blob/master/docs/INSTALL.md)：Linux、Python3.6+、PyTorch1.1+、CUDA10+、CMake、spconv1/2、可选APEX/旋转NMS扩展；作者测试旧Ubuntu/PyTorch组合，当前车机适配要重新验证 |
| 权重 | 官方模型表有pillar/voxel下载目录；未下载、校验hash或证明链接当前匿名可用，不把README链接等同可部署成品 |
| ROS2接口 | 新适配应消费`PointCloud2`原source stamp，输出同一map AABB/独立velocity contract；推理异步depth1、超时不刷新，模型对象与规则占据取保守并集，避免漏检把低障碍删除。当前没有新推理node |
| CPU/GPU/内存/时延 | 本车均**未测**；官方pillar表报告31 FPS只属于作者环境，不作为车机周期预算。原80k规则pipeline历史约234ms且排除DDS/解码；不以这些异构数值算提升比例 |
| 训练需求 | 提供预训练则不必从零训，但雷达线数/安装高度/场景不同，需要标注验证和可能fine tune；需要真实低矮障碍、遮挡和动静真值 |
| 许可 | 代码MIT及NOTICE；数据/下载模型条款独立，不用MIT覆盖数据的非商业条件 |
| 失败回退/验收 | 推理失败/超时沿现守卫停车并保留规则占据；同一真实bag测对象召回、未知障碍覆盖、track IDS、速度误差、净空/误停车与p50/95/99/max/deadline misses后才能选择。没有GPU/bag时不实施模型替换 |

---

# 主agent补充来源核查

截止2026-11-06（Asia/Shanghai）；实际联网搜索+原文/作者仓库读取。原始网页摘要/代码元数据在validation/source_root与exact_readme，禁止把搜索标题当已部署事实。下面来源支持系统路线比较，不引入新依赖。

|ID/类别|作者/题目/年份/刊物|稳定原文|官方实现/许可/权重|实验与适用边界|影响力证据/本车对应|
|---|---|---|---|---|---|
|R26-EX/规划性能、多模式|Chen Peng, Zhikang Ge, Wenwu Lu, Haiming Gao, Stavros Vougioukas, Peng Wei. EXACT-MPPI: Exact Signed-Distance Navigation for Arbitrary-Footprint Robots from Point Clouds via Path Integral Control. 2026，arXiv预印本v2；未核实会议发表|[arXiv/DOI](https://doi.org/10.48550/arXiv.2605.29663)，[原文v1](https://arxiv.org/html/2605.29663v1)|[作者仓库](https://github.com/caseypen/EXACT-mppi)，Readme.md确认JAX CPU/CUDA12、Ubuntu22.04/Python3.10、Humble/Gazebo/Docker；无需训练权重，根LICENSE未找到/APIlicense null，许可待确认|多底盘2D足迹/低速运动学；SecIV-D批量距离，IV-F分模式+切换惩罚/冷却；V-F动态只重规划当前点云，无显式未来运动预测|2026-11-06作者仓库API98stars（短期关注证据，非持续影响证明）。本车采用独立NumPy批量时序撞检思想与模式成本比较，保留未来障碍、3秒/ready条件；不移植MPPI实现或定理|
|R25-DRA/动态避障|Elia Trevisan, Khaled A. Mustafa, Godert Notten, Xinwei Wang, Javier Alonso-Mora. Dynamic Risk-Aware MPPI for Mobile Robots in Crowds via Efficient Monte Carlo Approximations. 2025，arXivv2作者注明IROS2025 Accepted Version|[原文](https://arxiv.org/html/2506.21205v2)，[DOI](https://doi.org/10.48550/arXiv.2506.21205)|本轮原文仅核实pedsim实验链接，未找到作者算法官方代码/权重/许可；不当部署候选|人群随机未来轨迹的MonteCarlo联合碰撞概率，实车与仿真比较S-MPC/Frenet/MPPI；概率可信度需要预测分布|IROS2025接受是评审状态证据，引用量未查。作为概率风险路线比较；当前CV/unknown可达包络缺统计校准，不能输出论文意义CP|
|E-MPPI/官方工程|Nav2 MPPI Controller，滚动官方文档与README（2026访问），非单独论文条目|[Jazzy官方配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)|[官方实现](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)，license需按子包检查（整个repo API Other不足判断）|支持DiffDrive/Omni/Ackermann；batch与rollout/critic、最小曲率约束；不能用文档示例时间证明本机预算|Nav2发布实现提供可核实采用；本轮比较完整优化替换成本与依赖，不直接覆盖既有模式ready/反馈链|
|E-SMAC/官方工程|Nav2 Smac Hybrid-A* Planner，滚动官方文档（2026访问）|[官方配置](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)|[官方实现](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner)；许可需子包核对，无模型权重|SE2非完整约束、倒车/转弯半径/解析连接代价与最大长度；离散地图/全局规划输入要求不同|官方生产实现可核实；本轮轻量有界局部SE2引导独立实现，后续DWA保持动态时序与制动检查，无完备性声称|

专业理论和本车推导：DWA/SE2/混合模式的共同状态可写z=(x,y,yaw,v,delta,m,phase,t_wait)。ACK用后轴bicycle再转换中心，SPIN中心yaw变化；未知模式满足protocol_confirmed才可用。候选可行条件不是仅J小，而是每时刻足迹与障碍可达集合分离，换轮等待与返回换轮也在H内。deadline要求未完整验证的结果丢弃；安全停车仍依赖反应/制动/速度上界/感知覆盖的配置假设。此推导属于本车工程适配，不是以上来源直接给出的本车安全证明。
