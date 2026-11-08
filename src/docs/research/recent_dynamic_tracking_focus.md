# 近期动态避障、跟踪与平滑控制：8篇重点对应

检索截止2026-11-06（Asia/Shanghai）。响应用户追加提醒，以下8篇按**当前框架与代码反例**选择，不按题名堆叠算法。Crossref登记被引数实查于本日，属于出版社登记引用链接，**不是Google Scholar总引用或统一影响排名**；元数据见validation/recent_paper_impact.json。新论文引用不足就如实标注，原始DWA/KF/Reeds–Shepp另见literature_catalog。作者、载体、许可/实验详见各完整目录。

|优先文献/作者/年份/载体|影响证据|当前框架对应、理论条件|改进代码框架 / 预期效果 / 决策|
|---|---|---|---|
|[Regulated Pure Pursuit for Robot Path Tracking](https://doi.org/10.1007/s10514-023-10097-6)，Steve Macenski, Shrijit Singh, Francisco Martín, Jonatan Ginés，2023 Autonomous Robots（C03）|Crossref登记112；[Nav2官方Apache2实现](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)持续采用|论文§2/3/5/6：几何追踪与调速/碰撞前验；Ackermann参考必须运动学可执行，论文不含实车动力学与本车3秒换轮。当前默认correction=0，不是完整PP误差闭环|traj_tracker_core/node执行已验证参考；真实轮滚动/新源yaw停稳门，VehicleBridge实际global rejoin不再被trap subgoal覆盖。预期减少错误模式回切、原地重接和控制模式抖动；保留FF转角，不以论文名义启用未经安全模型校准的反馈纠偏|
|[Jerk-limited Real-time Trajectory Generation with Arbitrary Target States](https://doi.org/10.15607/RSS.2021.XVII.015)，Lars Berscheid, Torsten Kröger，RSS2021（C04）|Crossref登记95；[作者Ruckig MIT实现](https://github.com/pantor/ruckig)维护|OTG状态(q,v,a)及速度/加速度/jerk约束；生成平滑运动不自动保证足迹碰撞。当前无真实转角，也未独立标定轮速/纵速与制动|下一统一MotionProfile=(v,delta,a,delta_dot)，规划rollout与tracker同模型、后验安全复查。预期降低角/速jerk，但本轮**不启生产后滤波/新依赖**；先记录模型/实测指标与执行滞后，再做OTG消融。安全急停不应被jerk限幅延迟|
|[A predictive safety filter for learning-based control of constrained nonlinear dynamical systems](https://doi.org/10.1016/j.automatica.2021.109597)，Kim P. Wabersich, Melanie N. Zeilinger，Automatica2021；[作者稿](https://arxiv.org/abs/1812.05506)（R21-PSF）|Crossref登记275，元数据核对作者/载体；官方完整实现/许可本轮未核实，不引入|显式有限时域安全控制序列、终端安全集合/backup与误差条件，不能把任意零指令叫递归安全。当前5m/.7m/s迎面会撞停车车，说明stop backup不是持续安全refuge|设计下一sequence_planner+SafetyBackup：z=(x,y,yaw,v,delta,mode,phase)，terminal refuge、时序障碍与模式等待约束；超时用已验证backup而非未经条件的“永远安全停”。本轮只保留现硬guard+明确deadline，拒未有效的H6/desired-speed更改，预期系统方案需实测模型误差/终端条件才能成立|
|[Dynamic Risk-Aware MPPI for Mobile Robots in Crowds via Efficient Monte Carlo Approximations](https://arxiv.org/abs/2506.21205)，Elia Trevisan, Khaled A. Mustafa, Godert Notten, Xinwei Wang, Javier Alonso-Mora，IROS2025作者接受版（R25-DRA）|接受状态/原文实车与仿真可查；长期高引用与算法官方权重/许可未核实，不伪报|非高斯未来轨迹MonteCarlo联合碰撞概率需可信随机预测。本车当前CV/unknown规则及Q/R未校准，不能把可达包络叫碰撞概率|保持TrackManager源时外推和CollisionChecker时序可达域；KF异常速度不clip成known。动态risk字段区分known CV/unknown reach/source_age，未来可接概率预测但硬安全保持。预期减少假速度误决策；真实加速/非合作迎面仍是明确失败边界|
|[SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking](https://doi.org/10.1007/978-3-031-25056-9_43)，Ziqi Pang, Zhichao Li, Naiyan Wang，ECCV2022 Workshops（2021作者稿A03）|Crossref登记145；Poly-MOT后续比较/采用，[官方MIT代码](https://github.com/tusen-ai/SimpleTrack)|§3/4/5.1：关联、生命周期与高频低质量检测组件消融。本车规则cluster没有检测score/可靠形状yaw，不能直接套3D GIoU/置信门|kf_tracker对真实source dt物理可达关联、.2s短窗模型一致性/unknown重确认；lidar父KF→瓦片传播，frame重复不重新测量。预期压假速度/急反向迟滞；反向3s oracle包络3/61→61/61，120轨KF p95增加约.31ms，噪声保守性完整列出|
|[Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics](https://arxiv.org/abs/2401.13078)，Steve Macenski, Matthew Booker, Joshua Wallace, Tobias Fischer，2024预印本/2025v2，2026工程维护（B06）|[Nav2 Smac官方Apache2包](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner)采用；正式刊物/DOI与引用总数未核实|SE2非完整基元、反向、启发/足迹；完整costmap与本车局部感知不同，有限激光patch不完备|local_guide.LocalStaticGuide(.3m,24yaw bins,ACK±弧)只给waypoint；VehicleBridge route持久锁定，DWA保留全动态/制动/后方检查。预期减少单最近障碍两点引导的局部极小；15ms/200exp无解/超时回退，不能将搜索waypoint记到达|
|[Gradient Field-Based Dynamic Window Approach for Collision Avoidance in Complex Environments](https://doi.org/10.1109/IROS60139.2025.11246091)，Ze Zhang, Yifan Xue, Nadia Figueroa, Knut Åkesson，IROS2025（B07）|Crossref登记3，明确是近期低引研究；[作者MIT代码](https://github.com/Woodenonez/CollisionAvoidance-GPDF-DWA)|unicycle、dt=.2/N20/至多84候选、JAX梯度场；没有本车ACK转角与3秒轮型/未知动态条件|比较GF-DWA与轻量DWA/SE2路线，**不引入JAX/GPDF**。实际优先enhanced_dwa/collision_checker批量等价+单plan复用+保守下界，避免因新算法破坏当前硬安全；512近场算力结果与deadline停已实测离线|
|[Escaping Local Minima: Hybrid Artificial Potential Field with Wall-Follower for Decentralized Multi-Robot Navigation](https://arxiv.org/abs/2409.10332)，Joonkyung Kim, Sangjin Park, Wonjong Lee, Woojun Kim, Nakju Doh, Changjoo Nam，2024预印本（C06）|原文与实物/仿真条件核查；新文无高引用声称，官方代码/许可未核实|§V-A hit/leave记忆、固定侧与目标进展；TurtleBot圆盘/5Hz不等于本车混合机械系统的完备性|VehicleBridge增加6s新源真实运动+累计轮滚动、goal/route无进展通道；侧绕或rear3m→ALIGN→ESCAPE3m→RETURN_ALIGN→真实REJOIN，有限24s ACK action/2 spin/冷却；动态让行和计算fault冻结。预期减少运动中局部循环/提前退出/无限刷新预算；U与窄U matched90结果、multi/厚U负结果另列|

## 共同的数据契约和公式

当前链为规则点云→父CV KF→map瓦片→TrackManager源年龄→ACK DWA→static guide/trap→同epochPath/Twist→50Hz tracker→3秒真实ready UDP。原定位/TF/补偿保持0；状态是几何中心，后轴.345m仅轴对称假设。

- KF `s=[x,y,vx,vy]`，`F(dt)`随真实源间隔；关联`||z-z_last||≤v_bound dt+2ε`。异常速度/位置创新进入unknown，保留观测框和旧TTL；.15m/.8m/s/.2s不是统计标定。
- ACK `yaw_dot=v*tan(delta)/.69`，先在后轴积分再回中心，速度/转角速率受原配置限制；倒车先实测停稳，后方全时域检查。`v,delta`不是实际转角反馈。
- 预测安全要求车辆足迹与同一时间的障碍集合分离：known `p(t)=p0+v*t`，unknown `B(p0,r0+v_bound*t)`；过去source age在snapshot，换轮/动作/返回delay重基准还保留未来域。采样间sweep、response/brake/park都在硬guard；soft评分不放宽。
- mode状态含STOP_CONFIRM/WAIT≥3s+ready/RUN/STOP_yaw/RECOVER≥3s+ready/FAULT；成功必须新源yaw低速、仍对齐，ACK恢复失败不由mode0 heartbeat无限重启。
- trap不是“v小就进入”：static blocked反事实True/False/None（None预算不足），加独立有真实wheel-roll+pose运动但无goal/route进展的local minimum；故障、让行、近目标不混入。退出用真正global build_dynamic_window目标、持续安全+实际.1m进展，不以停车墙钟证明成功。

所有后处理会改变实际轨迹，所以本轮把jerk滤波/完整MPC/概率模型列作有条件下一系统路线；已经实施的补丁与可回滚配置、真实和模型指标分列见decisions_and_validation。预期效果不能覆盖独立迎面物理失败或高噪声不确定性边界。


**最终默认选择**：moving_minimum的6s新源pose/轮滚动和两侧足迹阻塞判据默认只日志观测；`moving_recovery_enable:false`。当前源码实验true在U/厚U/窄U83.14/87.44/82.78秒到达，旧同输入90秒失败，但开放多块自动恢复回退，因此不是默认上线收益。静态blocked反事实、episode与真实重接、安全停稳/模式时序、KF和碰撞计算改进仍默认生效。

额外单基元研究见mode_opportunity_experiment.json：在给定模型中完整5秒ACK转向日程可通过5m迎面检查（间隙下界.238m），而滚动DWA的中间停车backup失败；这再次表明“当前候选无解”不等于完整物理可行控制集为空。该结果没有建立中间安全停车、失联时backup或真实误差界，未批准生产连续开环动作。7m初距下横移/倒斜行的模型机会也需完整物理协议与实测限值，不能仅凭单位/±45确认就驱动车辆。
