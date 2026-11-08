"""
traj_predict.py — bridge 侧固定时域参考轨迹积分（纯 Python，无 ROS 依赖，可单测）。

任务书第 2 节："发布用参考轨迹按第 2 节自建积分"——与 collision_checker.predict_trajectory
（rectangle 模式从 δ=0、v=0 冷启动按加速度爬升，仅供评分）无关：
这里用规划最终 (v, ω)（已含 ack_model 约束）在当前位姿做 Ackermann 自行车模型
恒速固定时域积分，用于发布 /rda/traj 供 traj_tracker 纯跟踪。

模型（Ackermann 自行车，恒速，v<0 倒车同样成立——轨迹向后）：
    δ = clamp(atan(ω·L/v), ±δ_max)；|v| < v_eps → 路径点全为当前位姿
    x += v·cos(θ)·dt ;  y += v·sin(θ)·dt ;  θ += (v/L)·tan(δ)·dt
"""
from __future__ import annotations

import math


def predict_reference_traj(v: float, w: float, x0: float, y0: float, theta0: float,
                           wheelbase: float, delta_max_deg: float,
                           dt: float = 0.05, n_steps: int = 30,
                           v_eps: float = 0.05):
    """固定时域参考轨迹积分。

    参数:
      v/w:        规划最终指令（m/s / rad/s，已含 ack_model 约束）
      x0/y0/theta0: 当前位姿（map 系、base_link 口径）
      wheelbase:   轴距 L（实车 0.46）
      delta_max_deg: 前轮最大转角 deg（实车 40°）
      dt:          积分步长（默认 0.05s → 1.5s 时域 30 点）
      n_steps:     轨迹点数（默认 30）
      v_eps:       |v| 低于此视为静止 → 全点=当前位姿（轨迹长度≈0，仍照常返回）

    返回:
      list[(x, y, theta)]，长度 n_steps。
    """
    pts: list = []
    x, y, th = float(x0), float(y0), float(theta0)
    if abs(v) < v_eps:
        for _ in range(n_steps):
            pts.append((x, y, th))
        return pts
    delta = math.atan(w * wheelbase / v)
    ms = math.radians(delta_max_deg)
    delta = max(-ms, min(ms, delta))
    tan_d = math.tan(delta)
    for _ in range(n_steps):
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += (v / wheelbase) * tan_d * dt
        pts.append((x, y, th))
    return pts


def traj_point_count(sim_time: float, dt: float = 0.05) -> int:
    """任务书第 2 节：n = max(2, ceil(sim_time/dt))。sim_time=1.5 → 30 点。"""
    n = int(math.ceil(float(sim_time) / dt))
    return max(2, n)


def build_road_base(v: float, w: float, x0: float, y0: float, theta0: float,
                   wheelbase: float, delta_max_deg: float,
                   length: float = 2.5, ds: float = 0.1,
                   v_eps: float = 0.05):
    """按"当前位置 + 规划出的 (ω, v)"积分前方 length(m) 路基（弧长等距采样）。

    与 predict_reference_traj（定时域）的区别：路基按**里程**截断（用户口径 2.5m），
    步长按弧长 ds 等距，因此低速时点数自动变多、高速时变少，几何形状不受控制频率影响。

    返回 list[(x, y, theta, v, t)]：
      x/y/theta  map 系（base_link 口径）位姿；
      v          该点参考速度 m/s（恒速路基 = 下发 v）；
      t          相对当前时刻的参考到达时刻 s（= 弧长 i·ds / |v|），供下层做时间插值。
    返回 [] 表示本帧无路基（|v| < v_eps：停车/自转/SPIN_REQUEST 期间不发路径）。

    积分方式：恒曲率 κ=tan(δ)/L 的圆弧精确步进（中点和弦），比欧拉积分在
    ds=0.1m 下的航向误差低一个量级（κ 路径 2.5m 弧长累积误差 <0.5°）。
    """
    if length <= 0.0 or abs(v) < v_eps:
        return []
    delta = math.atan(w * wheelbase / v)
    ms = math.radians(delta_max_deg)
    delta = max(-ms, min(ms, delta))
    kappa = math.tan(delta) / wheelbase
    v_s = abs(v)
    sign = 1.0 if v >= 0.0 else -1.0
    n = max(2, int(math.ceil(length / max(ds, 1e-3))) + 1)
    step = length / (n - 1)
    x, y, th = float(x0), float(y0), float(theta0)
    pts = [(x, y, th, float(v), 0.0)]
    s = 0.0
    for _ in range(n - 1):
        th_next = th + sign * kappa * step
        th_mid = 0.5 * (th + th_next)
        x += sign * math.cos(th_mid) * step
        y += sign * math.sin(th_mid) * step
        th = th_next
        s += step
        pts.append((x, y, th, float(v), s / v_s))
    return pts
