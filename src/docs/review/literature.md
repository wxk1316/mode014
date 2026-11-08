# 无人车平稳避障修复的文献与工程依据

查阅日期：2026-11-06。以作者原论文、作者机构页面和 ROS2 Nav2 官方文档为依据。本文把来源明确支持的结论与面向本车的工程推导分开；引用不等于实车安全认证，不照搬文献中的底盘或示例参数。

## 1. DWA 的安全条件包含“能够停下”，不只是前方预测点没碰撞

Fox、Burgard、Thrun 的原始 DWA 在速度空间中考虑短周期内可达速度，并仅保留沿候选运动轨迹能够安全停车的 admissible velocities，再用目标进展、速度、障碍距离评价。作者机构摘要明确这一停车要求；1997期刊版对应 DOI `10.1109/100.580977`。[作者机构原论文页（IROS 1996）](https://www.ri.cmu.edu/publications/controlling-synchro-drive-robots-with-the-dynamic-window-approach-to-collision-avoidance/)，[IEEE期刊版（1997）](https://xplorestaging.ieee.org/document/580977/)

**本车工程推导**：原论文的底盘是 synchro-drive，不能直接当作阿克曼或四轮自转运动学模型。候选应从实际 v、转角开始，以真实加减速度和转角速率积分；检查候选执行段后继续检查制动段。沿直线的最低停止距离可先以 `v*τ_delay + v²/(2*a_brake)` 估算；这只是必要的一维条件，弯道、自转、行人相对运动仍须做完整时间足迹检查。规划1.5m/s²而控制实际仅0.2m/s²的减速假设必须统一，否则原DWA的admissibility条件并未成立。模式切换3秒的静止期间也不能略过。

## 2. 动态避障必须使用运动信息和相对运动

Fiorini、Shiller 1998 的 Velocity Obstacles 方法在速度空间根据机器人和障碍的当前位置及速度选择避让；速度障碍表示会导致碰撞的速度集合。来源为原论文摘要记录，不据此宣称已经复现其全部算法。[原论文记录与摘要（OSTI）](https://www.osti.gov/biblio/665350)

**本车工程推导**：维持现有DWA时，最小修复是同一时刻 `vehicle_pose(t)` 对 `obstacle_pose(t)` 的足迹检查，不能算了障碍未来p却仍传o.x/y给矩形撞检。检查必须覆盖执行、模式等待、旋转及制动全过程。匀速假设不能涵盖行人突然换向，应随预测残差、丢检年龄增大不确定性包络，必要时停车等待。不能把暂时让行误判成trap并在行人旁进入自转。VO论文本身也不能证明本车固定3秒换轮期安全。

## 3. 纯跟踪需要速度调节和执行指令碰撞检查

Macenski 等（2023）Regulated Pure Pursuit 在 Adaptive Pure Pursuit 上增加速度调节，针对工业服务机器人狭窄及部分可见环境。Nav2 官方实现提供曲率调节、速度相关前视、近障减速和当前速度指令的前向碰撞模拟。[原论文 arXiv 2305.20026](https://arxiv.org/html/2305.20026v1)，[Nav2 Jazzy RPP 官方参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)

**本车工程推导**：先修当前纯跟踪实际目标点距离平方的分母、更新路径后的索引重置、目标在车后时的处理，再限制转角变化率；按曲率/障碍净空减速，不能只把发布频率升到50Hz。RPP的 `use_fixed_curvature_lookahead` 可减少速度与前视变化的相互影响，可作为后续方案；任何转角平滑都可能改变已验证轨迹，必须重新安全检查或预留跟踪误差包络。RPP普通原地转向配置不处理本车机械换轮，必须经过独立模式状态机。

## 4. 进入和退出转向的不同阈值构成明确滞回

Nav2 Rotation Shim 官方文档区分进入角阈值 `angular_dist_threshold` 与退出角阈值 `angular_disengage_threshold`，并提供旋转角加速度和前向安全模拟参数。其示例进入阈值大于退出阈值，支持“粗对齐进入、精对齐退出”的状态保持。[Nav2 Jazzy Rotation Shim 官方参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/)

**本车工程推导**：保留用户的机械3秒约束，使用 `ACK → STOP → WAIT_SPIN_READY → RUN_YAW_CLOSED_LOOP → SETTLE → WAIT_ACK_READY → ACK`。RUN计时从机械ready开始；目标航向在会话锁存；yaw误差生成自转w，独立于阿克曼规划w=0。退出由角误差和稳定保持时间决定，失败/传感器失效单独停车并上报。时间冷却、进展位移门和重复请求去重用于抑制trap与模式来回跳。Nav2文档只提供角度滞回/旋转控制依据，机械ready协议和3秒时序是本车工程设计，不能声称来源替本车证明。

## 5. 速度平滑需要独立加减速限制与可信反馈

Nav2 Velocity Smoother 官方文档分别提供 `max_accel`、`max_decel`，支持开环指令估计和闭环里程计反馈；这说明速度平滑的执行模型应显式建模。[Nav2 Jazzy Velocity Smoother 官方参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)

**本车工程推导**：用独立参数限制加速与制动，安全停车不经普通小变化死区拖延。机械模式、定位和轮速都须有有效性与年龄；50Hz重发旧定位不是闭环新反馈。转角限速应按实际底盘45°/s或实测更保守值设置，车辆执行能力需要日志标定。

## 6. 本次采用与后续边界

本次采用：时间配对碰撞检查、真实初态与制动段、全周足迹、安全输入超时、稳定目标航向、自转角度滞回、真实模式ready、转角斜率和加减速分离。优先修正现有代码接口，不引入未经本车验证的复杂MPC/学习策略替换。

后续可比较RPP、TEB或MPC，但需要先固定感知时钟、物理足迹/转向中心、模式执行反馈和跟踪误差，建立同一套横穿、狭窄绕行与模式等待基准。此次离线回归能够证明指定反例被封住，不能证明任意人群场景或任意定位故障下安全。
