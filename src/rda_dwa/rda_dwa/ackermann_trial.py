"""
ackermann_trial.py — Ackermann 自行车模型闭环试验运行器。

与 trial.py 同构，但：
  - 运动学积分使用 AckermannModel（自行车模型），算法输出 (v,ω) 经适配层映射为 (v,δ)
  - 碰撞检测使用矩形轮廓（1.0×0.6m）与障碍圆的最小距离，而非点圆
  - 新增指标：curvature_violations、footprint_clearance、min_turning_radius、
    max_curvature、delta_series、kappa_series

[C1 重构] 主循环实现已收敛到 rda_dwa/trial_runner.py（单一试验引擎 +
VehicleAdapter/TrapController/MetricsCollector 接缝）；本文件保留
AckermannTrialMetrics 结构定义与两个入口（run_ackermann_trial /
run_ackermann_trial_with_planner），签名与行为逐位不变。
"""
from .trial_runner import (
    AckermannTrialMetrics,
    run_ackermann_trial,
    run_ackermann_trial_with_planner,
)

__all__ = ['AckermannTrialMetrics',
           'run_ackermann_trial', 'run_ackermann_trial_with_planner']
