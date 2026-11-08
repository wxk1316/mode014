# 无人车算法改进交付索引

核对与检索截止：2026-11-06（Asia/Shanghai）。运行入口仍为 `ros2 launch car_launch run_rda_dwa.launch.py`。本轮完成感知、规划、控制／模式／trap 三个并行 agent 的研究、源码改进、跨模块审查及离线对照。交付包含实际补丁和回滚工具；没有启动实车、ROS2/DDS 或发送车辆 UDP。

## 默认补丁与效果

|模块|已实施内容|已验证效果与边界|
|---|---|---|
|非模型感知与动态输入|按真实源时间关联；异常速度进入 unknown；短源时间窗重确认；父轨到瓦片继承|突然反向的 3 秒模型可达覆盖由 3/61 提高到 61/61；120 轨 KF p95 由 3.528 增至 3.839 ms。高噪声可能明显增加保守停车，阈值仍需实车统计|
|DWA 与静态引导|批量时序足迹检查、保守几何下界、单次规划复用、80 ms 内部预算；有界 SE2 引导和持久路由|16×12 固定输入候选命令与原源码一致；512 有效近场 p50 为 68.57 ms。最坏倒车压力例超预算会整轮丢弃；没有全场景 10 Hz 保证|
|跟踪、模式与 trap|逐轮最大速度停稳检查；新源 yaw 停转且仍对齐才返回 ACK；至少 3 秒及真实 ready；有限重试／episode、让行与故障冻结、实际全局重接|修复平均轮速误停稳、错误模式回切、计算故障误作 trap、停车墙钟退出等反例。规划 w=0 的自转会话仍完成，模型最终误差 3.919°。默认前馈控制和 correction=0 保留，未证明实车抖动／jerk 已解决|

**默认 `moving_recovery_enable: false`。** 移动局部极小观测和多阶段恢复已实现；实验开启时 U／厚 U／窄 U 在 83.14／87.44／82.78 秒到达，均一次 trap、零再进入。开放多障碍场景曾从基线 36.34 秒到达回退为 90 秒未达，因此自动恢复保持关闭，实验结果不能算默认收益。

22 个同输入默认场景中，两版本各 11 次到达，但均有迎面物理模型碰撞，因此各 **10 次到达且无物理碰撞**。本轮默认观察到 0 次 100 ms deadline miss，基线 1 次；部分 U、多障碍、死胡同和未知／断流场景仍未到达。持续非合作迎面障碍可能撞上停止的本车，停车回退不构成任意动态环境安全保证。完整失败与净空口径见验证报告。

## 阅读与证据入口

|交付|入口|
|---|---|
|总体框架、运行链、坐标／时间／单位与数据契约|[framework_and_dataflow.md](framework_and_dataflow.md)、[parameter_inventory.md](parameter_inventory.md)|
|当前问题、已修复问题、日志待证假设|[problem_register.md](problem_register.md)|
|分类参考目录与许可、实验条件、影响证据|[literature_catalog.md](literature_catalog.md)|
|用户提醒的 8 篇近期重点及理论到代码对应|[recent_dynamic_tracking_focus.md](recent_dynamic_tracking_focus.md)|
|路线比较、优先选择及问题→来源→条件→代码→验证|[decisions_and_validation.md](decisions_and_validation.md)|
|模式／trap 状态图、转移表与失效场景|[control_design.md](control_design.md)|
|基线对照、消融、量化指标与未通过案例|[validation_report.md](validation_report.md)、[最终汇总 JSON](final_validation_summary.json)|
|实际 27 文件变更、hash 与可回滚补丁|[changes.patch](changes.patch)、[changes_manifest.json](changes_manifest.json)、[restore_baseline.py](restore_baseline.py)|
|车机部署、受控验收、完整回滚|[deployment_and_rollback.md](deployment_and_rollback.md)|
|源码基线与历史保留|[baseline_manifest.json](baseline_manifest.json)、[本轮基线 ZIP](baseline_sources.zip)|
|交付包与逐成员 hash|[delivery_20261106.zip](delivery_20261106.zip)、[delivery_manifest.json](delivery_manifest.json)|

8 篇重点覆盖 RPP、Ruckig、predictive safety filter、DRA-MPPI、SimpleTrack、Smac、GF-DWA、APF／wall-follower。较成熟论文的 Crossref 登记引用链接数已核查；新论文明确标注引用不足或未核实，未把所有近期论文称为高引用。每篇的适用条件与本车非完整运动、3 秒机械等待、源时预测和执行模型限制分开记录。没有引入神经权重、JAX、Ruckig、Nav2 或新求解器依赖。

## 验证范围与复现

最终回归 **248 项通过 + 12 个子测试**；C++17 原／扩展模式守卫通过，73 个 Python 源码解析无误，fake ROS 参数与接口检查通过；40 个定位／定位启动保护文件及 28 个历史审查／验证文件保持原字节。补丁在 src 外隔离应用、hash 核验、回滚、后续编辑拒绝覆盖和幂等回滚均已验证。

数据分为合成场景、模型 oracle、有限执行器模型、纯模块测量和 fake ROS 构造；没有完整 raw-cloud 闭环、DDS、车机周期、真实转角或实车安全结论。模型 yaw-to-goal、命令角速率与模型 jerk 分别报告，不能当作真实路径跟踪误差和实际 jerk。

在保留基线 ZIP 的完整工作区中，使用具备 NumPy、SciPy、PyYAML、pytest、pytest-subtests 的 Python 和 C++17 编译器执行：

```bash
RDA_TEST_PYTHON=/path/to/validated/python bash docs/research/validation/run_final_checks.sh
```

对照脚本为 [run_comparison.py](validation/run_comparison.py)、[planner_benchmark.py](planner_benchmark.py)、[run_perception_comparison.py](validation/run_perception_comparison.py)、[compare_control_policies.py](validation/compare_control_policies.py) 和 [compare_modes.py](validation/compare_modes.py)。脚本默认输出位置和实验开关各不相同；重跑前指定新的输出目录，保留已冻结 JSON 中的源码 hash、时限与 scope。当前默认 22 例的冻结记录位于 [final_comparison](validation/final_comparison/comparison.json)，实验恢复记录位于 [final_experimental_recovery](validation/final_experimental_recovery/comparison.json)。

## 交付包使用与剩余验收

交付 ZIP 按工作区相对路径存放最终变更源码／测试、研究文档、复现脚本及结果。它是面向已有本轮基线工作区的补丁交付，**不是完整 ROS2 工作区或 install**；不包含论文全文／网页缓存、字节码和 69 MB 基线 ZIP。基线 ZIP 单独保留；回滚与隔离补丁核验需把它保存在 `docs/research/baseline_sources.zip`。不要在 src 中解压出第二套 ROS 包，已有后续编辑时先核对 hash，不能盲目覆盖。

定位 FAST_LIO、global_localization、transform_fusion、发布、TF 和当前零补偿保持原样；后轴距离 0.345 m 仍是轴对称假设。横移仅线速度、斜行线速度及 ±45° 已依据用户反馈更新模型协议；命令／反馈编号、方向和固件比例未完整确认，生产 1／2 模式保持禁用。下一验收需要真实源时间／点云 bag、逐轮反馈、实际转角与刹停／滞后、车机时延及完整模式协议。方案和受控验收步骤已写入部署文档，实车启动由操作者执行。
