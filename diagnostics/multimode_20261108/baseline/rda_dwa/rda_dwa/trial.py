"""
trial.py — 单次试验主循环（runTrialAblation.m 的 Python 移植）。

流程：障碍物状态更新 → 预测 P_obs(t+1) → 动态窗口采样筛选 → 代价 J(v,ω) →
      argmax (v*,ω*) → 运动学更新 → 碰撞/到达检测 → C3 逃逸状态机 → 指标统计。

[C1 重构] 主循环实现已收敛到 rda_dwa/trial_runner.py（单一试验引擎 +
VehicleAdapter/TrapController/MetricsCollector 接缝）；本文件保留
TrialResult / TrialMetrics 结构定义与三个入口（run_trial /
run_trial_simple / run_trial_with_planner），签名与行为逐位不变。
"""
from .trial_runner import (
    TrialResult,
    TrialMetrics,
    run_trial,
    run_trial_simple,
    run_trial_with_planner,
)

__all__ = ['TrialResult', 'TrialMetrics',
           'run_trial', 'run_trial_simple', 'run_trial_with_planner']
