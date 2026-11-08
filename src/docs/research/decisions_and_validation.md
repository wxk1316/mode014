# 最终方案、实现与验证

检索/核对截止2026-11-06。完成A感知、B规划、C控制/trap并行分工和两轮独立跨模块审查；共有笔记位于agent_notes。无Git，修改前202文件完整zip/hash已保存；40个定位及定位launch保护文件逐字节不变。旧review备份和旧validation JSON保持。当前入口仍是`ros2 launch car_launch run_rda_dwa.launch.py`，未启动ROS车辆或发送UDP。

## 路线选择及实际默认

|路线|条件与本车判断|最终选择|
|---|---|---|
|轻量DWA安全计算+非模型估计|保留ACK±可达、真实初态/转角估计、源年龄、时序足迹、制动与后验复查；已有NumPy，可同输入验证|默认实施batch/SAT下界、单plan nominal复用、80ms预算、KF异常unknown和真实source重确认|
|有界局部SE2搜索/记忆|只由确认静态数据指导，前后向弧满足曲率；有限观测/200扩展/15ms不完备|默认LocalStaticGuide与route锁；无解/超时无引导，DWA仍独立检查全部动态|
|移动局部极小多阶段恢复|6s真实source pose运动+至少1s定性wheel rolling、无goal/route进展、前路阻塞和两侧足迹受限、后方可退|实现且实验验证，但`moving_recovery_enable:false`默认只观测。三个U可到达，开放多块回退，未作为默认收益|
|完整MPC/MPPI/预测安全过滤|需要控制序列、可行终端refuge/backup、跟踪误差、真实转角/制动/数据失联模型；新solver/依赖和车机周期缺实测|比较后不替换当前控制；H6/提前旁路/desired-speed试验仍迎面失败，拒生产接口变更|
|Ruckig/jerk后处理|会改变已验证轨迹，需a/delta_dot状态及一致rollout/后验检查|本轮不启OTG和feedback correction；保留FF、既有斜率，完善真实停稳与模式时序|
|学习检测|权重、数据许可、Linux/CUDA/spconv、车机时延/低障碍召回均需验证|未部署；非模型优先，模型候选目录含失败回退与未验证项|

## 问题证据 → 文献/条件 → 本车适配 → 实际代码 → 性能/失败 → 对照

|问题/当前证据|来源和适用条件|实施文件/流程|性能、失败与验证|
|---|---|---|---|
|KF第二位置跳变被裁成known1.5m/s，反向CV迟滞|SimpleTrack/AB3DMOT组件与生命周期；物理可达集和source-window为独立工程条件，vbound/slack未标定|kf_tracker.py、lidar_track_node.py、obstacle_fusion.py：真实dt物理关联、位置/速度创新unknown、.2s源窗重确认、父子瓦片传播|位置仍保留、旧轨TTL；反向3s包络3/61→61/61。120轨p95增加3.528→3.839ms；4cm/20Hz及8cm高噪声保守性保留|
|每候选重复轨迹/逐瓦片NumPy计算，只有事后deadline|DWA可达/可停原则、SAT包络下界、批量计算；不改变原安全模型的适用范围|enhanced_dwa.py、collision_checker.py、params.py：本轮scene、64对象块、严格下界剔除、nominal复用、last_failure_reason/counters|16×12同输入batch/full命令一致；512远场2680.77→29.44ms，近场68.57ms；倒车压力118.14ms在80ms预算整轮False/0/0，不发部分结果|
|静止/让行/计算故障误作trap；侧移刷新预算、原地墙钟退出|APF/WF hit/leave记忆、混合guard专业结构；不继承单积分GAS/全局完备|vehicle_bridge.py：static反事实True/False/None，execution-stall与yield分离，episode/冷却/真实global重接；实际planner不再用旧subgoal覆盖global|±1.32m安全ACK但不动旧1次spin→0；±1.14真正blocked仍1。侧移.7m不刷新2次预算；无实际推进不能退出；数据/计算fault回滚计时但尝试保守消费|
|零指令/平均轮速/角容差不证明机械停止；ACK恢复可软件循环|本车3s/真实ready事实；RPP动力学边界和混合flow/jump工程guard|traj_tracker_core/node.py、mode_switch_guard.hpp、car_ctr.cpp：四轮max停稳、new-source yaw低速且仍对齐、原12s时钟重对齐、ACK超时锁存、最多2次补发|入前轮速missing/单轮动、yaw漂移/重复源/ready丢失、持续20s失败均功能验证；w=0旋转131 ticks完成、最终3.919°，前后机械等待均≥3s|
|unknown整旋转盘和未来ACK出口漏等待时间|可达集合包含/三角不等式，不称概率置信域|vehicle_bridge._spin_safe/_obstacles_after_delay：snapshot过去age不重复，完整停止/换轮/旋转/返回未来域重基准|6s未来delay半径增加9m，源snapshot不变；独立几何/时间等价检查通过；未知风险不会被当static|
|真实运动中往复、全局无进展；pose抖动亦可累计travel|C06记忆/进展启发，wheel rolling只为定性支持、不当独立纵速测量|_moving_local_minimum、confinement、RETREAT/ALIGN/ESCAPE/RETURN_ALIGN/REJOIN；全零/缺wheel不能靠1cm jitter触发|当前源码实验True：U83.14、厚U87.44、窄U82.78秒到达，无再入；旧自动开放多块90失败（基线36.34到达），所以默认False|

详细公式、状态/控制单位、论文章节、许可和代码映射见literature_catalog及recent_dynamic_tracking_focus。ACK模型为几何中心↔后轴bicycle，车1.2×.8m/L=.69m，rear=.345m为轴对称假设。当前配置速度/制动/转角/速率和unknown1.5m/s是条件，不是完整实车标定。定位生产/TF/补偿未动。

## 默认22场景对照与消融结论

原始与默认源码采用相同有界执行器、oracle几何、源时钟/目标/场景。两版本均11次到达，其中head_on发生物理模型碰撞，所以只有10次“到达且无物理碰撞”。11例未到达分别含期限制约、遮挡/unknown等待、后方阻塞/断流和静态局部失败；不把安全停车记成功。数据见validation/final_comparison/comparison.json，逐场景路径/净空/TTC/jerk/反号/模式/时延见validation_report。

默认没有消除5m/.7m/s非合作迎面碰撞。提前旁路、H6和完整desired-speed同输入试验仍失败。完整5s ACK转向单日程模型可行（.238m下界）并不证明每个中间状态可停车/失联backup可行；下一系统需要可验证的多段避让backup/terminal refuge，不能通过放宽硬约束制造成功。

固定快照batch-only/full及预算、控制模块相同依赖消融、KF门/源窗/20seeds噪声、默认/实验恢复开关、模式全日程适用/不适用与ready故障均有独立报告。对应JSON不互相当成同一整栈基线。闭环oracle绕过raw-cloud感知，fake ROS只验证构造/接口，不是DDS或车机验收。

## 冻结/部署/回滚

最终source/config/test清单与changes.patch、restore_baseline.py绑定；补丁先在src外解压本轮baseline核验apply与回滚，不创建第二套ROS包。完整回归248+12、独立C++旧/新guard、AST/参数/保护hash和补丁核验汇总到final_validation_summary.json。

已知边界：实车日志缺失；无真实转角、轮速纵向标定、完整制动/轮型动作时延、地面/遮挡真值、source同步和车机负载。CRAB/TRANSLATE仅部分语义确认（线速度、斜行±45°），IDs/方向/物理比例未确认，生产禁用。验收与回滚详deployment_and_rollback.md。
