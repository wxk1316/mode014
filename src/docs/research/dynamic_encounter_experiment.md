# 动态迎面独立方案实验：拒绝生产改动

2026-11-06。输入为根22场景中`head_on`：车从几何中心(0,0,0)静止，目标(4,0)，已知CV障碍中心`(5-.7t,0)`、物理.5×.5m、速度.7m/s。完整输入/实际50Hz有界执行器模型、原3秒机械ready、原watchdog沿用根harness。无真实日志、ROS2、DDS或车辆；不把reached覆盖先前碰撞。生产源码/参数/接口保持冻结。

## 方案与相同输入结果

| 独立方案 | 5m迎面首次no-safe / collision guard s | 最小circle净空m | 最小物理AABB分离下界m | plan p95 ms | 判断 |
|---|---|---:|---:|---:|---|
| 冻结集成基线 | 1.9 / 5.4 | -.3536 | 根后来补充物理oracle | 20.64 | 停车后外来目标撞击；失败 |
| 整个安全时域3→6s | 0.0 / 5.8 | -.3536 | -.6500 | 32.26 | 更早全部不可行；失败 |
| nominal6s、原reaction/braking/parking3s | 0.0 / 5.8 | -.3536 | -.6500 | 28.08 | 原初始±.05m/s恒target不能表达后续加速；失败 |
| 早期持久旁路目标(1.5,1.4) | 1.6 / 5.1 | -.3536 | -.6340 | 20.65 | 车仅横移约.15m即再次被backup拒绝；失败 |
| 更尖侧目标(.2,2.0) | 1.6 / 5.1 | -.3536 | -.6321 | 18.96 | 仍不能保持安全侧移；失败 |
| 完整desired-speed target，nominal3s | 1.7 / 5.2 | -.3536 | -.6472 | 13.63 | 表达加速改善但无迎面安全收益；失败 |
| 完整desired-speed target+nominal6s/backup3s | 1.4 / 5.1 | -.3536 | -.6257 | 19.35 | 中间态的安全停车备份仍不可行；失败 |
| 完整desired-speed target+原3s+旁路 | 1.6 / 5.2 | -.3536 | -.6333 | 13.49 | 失败 |
| 7m更早观测+旁路（改变条件，不是5m同输入收益） | 4.3 / 8.0 | -.2729 | -.1693 | 22.74 | 无成功宽初距案例，不冒充成功 |

所有方案动态场景mode switch=0、trap entry=0。维持动态让行与静态恢复分离；不因这些失败调用3s自转，等待期间障碍更近。full-target实验是**仅本进程**：采样±完整速度target→rollout从真实当前v按.5m/s²爬升→bridge传desired speed→tracker/执行器原加速度爬升→适配后同target重新command_safe。没有只在planner采.4m/s却实际传.05m/s，也没有跳过原硬安全复查。虽目标语义可表达更长加速动作，本轮不批准改变当前生产速度接口。

## 初始候选证据

独立`headon_initial_candidate_table`在JSON里保存全部18组。源时域3s时，初态完整速度target=.4m/s、转角±30°的nominal+reaction/braking/parking均通过；6s nominal转向通过而同6s的停车备份失败。初始固定target=.05m/s的6s转向甚至nominal也失败。说明有两个不同限制：有限常值低速度候选缺乏后续加速表达；立即反应/制动并保持停车的备份模型不能证明需要继续侧移的中间态。仅延长prediction不能把两者修成可靠递归安全。

## 交叉场景与保守性

| 方案 | crossing | overtaking | occlusion/unknown | false-velocity unknown |
|---|---|---|---|---|
| 冻结基线 | 到达，circle净空.4087m，12.58s | 到达但circle包络净空.1464m<.2配置；物理侧净空.25m | 20s未到达，净空.4644m | 保守停车，10s未到达，净空1.1331m |
| horizon6 | 同净空到达但20.24s | 更久25.54s，.1464m包络裕度仍不足 | 仍未到达且约相同净空 | 同保守停车 |
| full target+nominal6/backup3 | 到达10.98s、净空.5790m，但3次转向反号/模型jerk增大 | 包络裕度仍不足，不能称改善可靠性 | 未到达，净空.4227m | 同保守停车 |
| 原接口早期旁路guide | crossing原行为基本不变 | 包络裕度仍不足 | 同保守停车 | 同保守停车 |

crossing的单场景时间收益不足抵消迎面确证失败与轨迹改变后重新安全验证的需求。unknown可达域保持最大速度1.5m/s，无统计校准或可穿越未知的依据；不放宽碰撞阈值、源超时或unknown约束。

## 理论条件与代码对应

证据链：`EnhancedDWA._executable_window`目前只采下一dt可达速度并以该常值target贯穿nominal；`command_safe`检查nominal和响应至少.6s后制动、停车至sim_time。`VehicleBridgeCore.step`适配后复查；tracker各50Hz执行速度/转角限制。本轮实验保持制动硬约束，但停稳并不能阻止一个持续前进的非合作外来目标。

[DWA原论文](https://doi.org/10.1109/100.580977)的可达/可停控制原则是来源；不能把同步驱动/有限运动模型条件当作任意动态障碍安全证明。专业推导：通过当前有限时域检查，并不自动给出下一时刻安全问题仍可解的**递归可行性**；安全候选集合还可能是完整可行控制集合的严格子集。因此`no_safe_trajectory`不证明完整意义的Inevitable Collision State，也不能宣称本车物理上已无任何可避动作。

下一系统路线是显式多段控制序列与一个经过验证的避让backup，终端进入持续安全的refuge/terminal set，同时约束响应期的数据/执行器误差；参考[Wabersich与Zeilinger的predictive safety filter原文](https://arxiv.org/abs/1812.05506)，但其模型、约束、终端安全与扰动条件必须逐项建立。本车转角无实测反馈、速度未独立标定、观测遮挡/失联及障碍最大速度未标定，尚不满足直接套递归安全结论的前提。没有可靠证据支持一个本轮轻量动态补丁；保持已实施的算力/感知/trap改进并明确迎面失败和下一系统条件。

## 复现与源码状态

只运行`docs/research/dynamic_encounter_experiment.py`，变体由`--variant`选择。`horizon6`、`nominal6_brake3`、`early_side_guide`、`full_target_h3`、`full_target_nominal6`、`full_target_guided`均只monkeypatch当前Python进程；`--cases crossing overtaking occlusion false_velocity`提供交叉场景。图形/轨迹/每场景摘要位于dynamic_encounter_traces。全部输出/源hash与首次失败时间汇总在`dynamic_encounter_experiment.json`。

hash是汇总时当前源码，B planner源码与已冻结benchmark保持一致；根harness在实验期间补充了物理oracle/失败优先级，但物理运动常数和输入保持相同。逐trace hash保存实际输出，时延测量包含并行离线worker争用，不能拿这些少样本时延作公平车机性能排名。本分支所有CPU实验已完成并停止，根进行最终公平重跑。
