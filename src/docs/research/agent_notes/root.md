# 主agent工作摘要

已读任务书/日志指南/当前top与stack launch/YAML/bridge_node主要运行与诊断/历史验证与cross_review/literature。确认当前车中心0、ACK0/SPIN4、3s、当前固定DWA与前馈；实际rclpy/ros2不可用，已有venv可离线。基线171+12。完整快照先于并行修改。

已完成A/B/C各模块与主agent集成；以下阶段笔记保留历史过程，当前结论以README、decisions_and_validation和final_validation_summary为准。真实日志目前只有人工样例不可用于症状归因；定位保护hash最终已复核。时间monotonic控制、ROS source年龄、整数ns配对分别保留。


阶段摘要：优先方案已授权实施A源时运动异常unknown+B批量时序安全/有限deadline/局部SE2guide+Cepisode进展/预算及真实停稳机械反馈。root已bridge/YAML全部新增参数接线、明确CarCtr AABB契约注释、mode_max_retries launch参数。中间集成误差None intent已交C修复；原1.32/1.20m墙仍安全v=.05，改1.14m保留latchedspin断言+新增原执行停滞禁止spin。近场guide旧安全语义需C保留。最终前不将中间失败当完成。

同输入初5场景通过且轨迹一致，root扩22场景bench（独立SAT静态距离界/动态circle/有界执行器）。指标含model路径、10Hz model jerk/yaw-to-global-goal（非path tracking实测）、命令45°/s、trap/mode/等待、p50/95/99/max、deadline；TTC与实测转角null。BASE与新源码隔离加载；本机CPU结果非车机预算。

主额外文献：2026 EXACT-MPPI arxiv2605.29663v2作者Peng/Ge/Lu/Gao/Vougioukas/Wei，SecIV-D GPU KTHN batch，IV-F混合cost+switchpenalty+cooldown/physical stopping，V-F明确仅当前点云动态无预测/低速平面模型。官方github(caseypen/EXACT-mppi)Readme.md已核查JAX CPU/CUDA12/Humble/Gazebo/Python3.10；未找到许可，APIlicense null，不采用依赖。2025 DRA-MPPI arxiv2506.21205v2，Trevisan/Mustafa/Notten/Wang/Alonso-Mora，IROS2025接受，MonteCarlo joint CP需可信随机预测；本车unknown deterministic reachable不能标成校准概率。官方Nav2 MPPI与Smac最新迁移URL已原页查读，官方工程不是论文定理。


阶段四综合验证：初22基线/新全跑揭示非合作迎面物理碰撞与部分静态无目标进展（不是全pass）。另headon原harness reached在碰撞后仍True，root修报告失败优先并加入物理AABBoracle vs circle包络；晚帧fixture改为真正源时位置t-.45，最终baseline22已在隔离zip重跑完成，新方案待C移动minimum成熟补丁冻结后重跑。B额外25变体迎面全失败、包括H6/fullsetpoint/早旁路，已拒production改动，保留当前接口。静态movingminimum正在C授权按实际新源6s/全static/routegoal进展区分、bounded rear3m→escape→rejoin实验；生产ACKaction预算YAML24s（purecore旧16保持原timeout回归）。C专用ACKmode timeout latch已冻结测试。B停止所有CPU实验便最终公平计时。

用户协议资料已用async询问斜行/横移command/feedback/angle/speed，当前仍未核实所以生产disabled；独立compare_modes与B30mode结果已保存，均明确different endpoint primitivecapability而非同目标最优。


最新用户提醒已写user_priorities：近期动态/跟踪/平滑重点5–8篇与highcitation可核证据，正在Crossref取引用元数据，不能把newpapers说高引用。部分新模式语义已用户确认横移线速度only、CRAB线速度angle±45，rootModelProtocol更新但缺IDs/方向仍生产禁用。

历史阶段：成熟C恢复RETREAT→ALIGN→ESCAPE3m→RETURN_ALIGN→REJOIN首次U83s成功后，A审查发现dynamicwaiting/deadline误扣ACK24budget，以及freshwheel0+1cmposejitter误moving恢复。C已完成最小修复，新增自动恢复必须共同轮滚动证据；预算回滚仅计算/输入fault不得刷新attempts，正常成功控制保持。随后完成最终全回归与同输入对照，不再等待冻结或重读全部论文原文。

## 最终交付断点（2026-11-06）

- 已读证据：最终框架/参数、27文件changes_manifest、8篇重点映射、final_comparison及final_experimental_recovery、补丁/回滚独立核验、最终安全复核。原paper全文缓存仅供内部核对，交付包排除。
- 当前源码：18个原文件修改、9个新增源码/测试；vehicle_bridge SHA256=2b6fa0c79cc1073e7c3a9bd8654709528767bd4b618e31ee1cb4431d1e6af1fc。定位/TF/补偿40个保护文件和28个旧审查/验证文件保持原字节。
- 验证：248 passed+12 subtests，C++17原/扩展guard通过，73 Python AST无误，参数接口检查通过；隔离patch应用/回滚hash、拒绝后来编辑及幂等检查均通过。没有ROS2/DDS/实车/UDP执行。
- 默认：motion_guard、批量DWA/80ms预算、有限SE2guide、静态反事实、episode/实际global rejoin、逐轮停稳/新源yaw停止/3秒ready/有限重试生效。tracking_correction=0，moving_recovery_enable=False；无新学习/solver/OTG依赖。
- 结论：22默认同输入各10次无物理碰撞到达，迎面两版仍碰撞；默认0次100ms miss、基线1次，仅本机有限样本。实验True三U83.14/87.44/82.78秒到达，0再入；开放multi回退是保持默认关闭的原因。
- 用户反馈：横移只有线速度，斜行为线速度+±45°。物理编号/方向/比例不完整，生产1/2仍禁用；m/s是软件模型单位，不擅称固件已标定。
- 完成范围：算法补丁、文献/条件到代码、对照/消融与失败、状态图、部署/验收/回滚以及交付索引与ZIP。未决实车条件已列报告，不以模型结果宣称症状全部解决。没有再开启算法迭代。
