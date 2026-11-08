"""
four_wheel_trial.py — 4WIS（四轮独立转向）闭环试验运行器。

与 ackermann_trial.py 同构，但：
  - 运动学积分使用 FourWheelSteerModel：算法输出 (v, ω) 直接驱动，
    v≈0 时 ω 依然有效（原地转向，绕车体几何中心旋转）
  - 碰撞检测使用几何中心矩形轮廓（±(L/2, W/2)，1.0×0.6m）与障碍圆 SAT 精确判定
  - 新增指标：spin_usage_ratio（原地转向控制周期占比）、max_spin_duration（最长连续旋转）、
    spin_cycles（原地转向事件次数）、四轮转向角系列

[C1 重构] 主循环实现已收敛到 rda_dwa/trial_runner.py（单一试验引擎 +
VehicleAdapter/TrapController/MetricsCollector 接缝）；本文件保留
FourWheelTrialMetrics 结构定义与两个入口（run_four_wheel_trial /
run_four_wheel_trial_with_planner），签名与行为逐位不变。
"""
from .trial_runner import (
    FourWheelTrialMetrics,
    run_four_wheel_trial,
    run_four_wheel_trial_with_planner,
)

__all__ = ['FourWheelTrialMetrics',
           'run_four_wheel_trial', 'run_four_wheel_trial_with_planner']
