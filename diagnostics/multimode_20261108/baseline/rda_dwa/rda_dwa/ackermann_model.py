"""
ackermann_model.py — Ackermann 自行车模型运动学 + v/ω→v/δ 适配层。

不修改 RDA-DWA 核心算法（enhanced_dwa.py / params.py / module_config.py）。
算法输出仍为 (v, ω)，本模块将其映射为 Ackermann 输入 (v, δ)，
并按自行车模型积分：
    x' = v·cos(θ)
    y' = v·sin(θ)
    θ' = (v/L)·tan(δ)

适配公式：δ = atan(ω·L / max(v, v_min))，并限制在 [-δ_max, δ_max]；
v≈0 时禁止转向（δ 保持上一帧值，不原地转向）。

车辆参数（用户指定）：
  L_body = 1.0 m, W = 0.6 m, wheelbase L = 0.65 m
  δ_max = 30°, δ_dot_max = 30°/s
  v_max = 1.5 m/s, a_max = 1.0 m/s², dec_max = 1.5 m/s²
  外接圆半径 r_c = sqrt((L_body/2)² + (W/2)²) ≈ 0.583 m
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .robot_model import wrap_angle


@dataclass
class AckermannParams:
    """Ackermann 车辆参数（与 params.py 解耦，不修改核心参数文件）。"""
    length: float = 1.0          # 车长 L_body (m)
    width: float = 0.6           # 车宽 W (m)
    wheelbase: float = 0.65      # 轴距 L (m)
    delta_max: float = math.radians(30.0)   # 最大前轮转角 (rad)
    delta_dot_max: float = math.radians(30.0)  # 最大转向角速度 (rad/s)
    v_max: float = 1.5           # 最大前进线速度 (m/s)
    v_min: float = -1.0          # 最大倒车速度 (m/s)（Reeds-Shepp 双向运动，真实汽车物理支持）
    a_max: float = 1.0           # 最大加速度 (m/s²)
    dec_max: float = 1.5         # 最大减速度 (m/s²)
    v_eps: float = 0.05          # |v| 低于此值视为近似静止，禁止转向

    @property
    def r_circum(self) -> float:
        """外接圆半径。"""
        return math.sqrt((self.length / 2.0) ** 2 + (self.width / 2.0) ** 2)

    @property
    def kappa_max(self) -> float:
        """最大曲率 κ_max = tan(δ_max)/L。"""
        return math.tan(self.delta_max) / self.wheelbase

    @property
    def r_min(self) -> float:
        """最小转弯半径 R_min = L/tan(δ_max)。"""
        return self.wheelbase / math.tan(self.delta_max)


class AckermannState:
    """Ackermann 车辆状态：位姿 + 速度 + 前轮转角。"""
    __slots__ = ('x', 'y', 'theta', 'v', 'delta')

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, delta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = float(v)
        self.delta = float(delta)

    def to_array(self):
        return [self.x, self.y, self.theta, self.v, self.delta]


class AckermannModel:
    """Ackermann 自行车模型：v/ω→v/δ 适配 + 运动学积分。"""

    def __init__(self, params: AckermannParams | None = None):
        self.p = params or AckermannParams()

    def adapt_velocity(self, v_cmd: float, omega_cmd: float,
                       current_v: float, current_delta: float, dt: float):
        """将算法输出 (v_cmd, omega_cmd) 适配为 Ackermann (v, δ)。

        步骤：
          1. v 限幅 [v_min, v_max]，并施加加速度约束
          2. δ = atan(ω·L / max(v, v_eps))，v≈0 时保持上一帧 δ
          3. δ 限幅 [-δ_max, δ_max]
          4. δ 变化率限幅 |Δδ| ≤ δ_dot_max·dt

        返回 (v, delta, curvature)。
        """
        p = self.p
        # --- v 加速度约束 ---
        dv_max = p.a_max * dt
        dv_min = -p.dec_max * dt
        v = current_v + max(dv_min, min(dv_max, v_cmd - current_v))
        v = max(p.v_min, min(p.v_max, v))

        # --- δ 适配 ---
        # [改进A] |v|<v_eps 时禁止转向（前后都一样）；v<0 时 atan 自动反号
        if abs(v) < p.v_eps:
            # 近似静止：禁止转向，保持上一帧 δ
            delta = current_delta
        else:
            delta_target = math.atan(omega_cmd * p.wheelbase / v)
            delta = max(-p.delta_max, min(p.delta_max, delta_target))

        # --- δ 变化率约束 ---
        ddelta_max = p.delta_dot_max * dt
        delta = current_delta + max(-ddelta_max, min(ddelta_max, delta - current_delta))
        delta = max(-p.delta_max, min(p.delta_max, delta))

        # --- 曲率 ---
        kappa = math.tan(delta) / p.wheelbase if abs(delta) > 1e-9 else 0.0

        return v, delta, kappa

    def integrate(self, s: AckermannState, v: float, delta: float, dt: float) -> AckermannState:
        """自行车模型前向积分（后轴中心为参考点）。"""
        nxt = AckermannState()
        nxt.x = s.x + v * math.cos(s.theta) * dt
        nxt.y = s.y + v * math.sin(s.theta) * dt
        nxt.theta = wrap_angle(s.theta + (v / self.p.wheelbase) * math.tan(delta) * dt)
        nxt.v = v
        nxt.delta = delta
        return nxt

    def curvature(self, delta: float) -> float:
        """当前前轮转角对应的轨迹曲率。"""
        return math.tan(delta) / self.p.wheelbase if abs(delta) > 1e-9 else 0.0

    def turning_radius(self, delta: float) -> float:
        """当前转弯半径（δ→0 时返回 inf）。"""
        if abs(delta) < 1e-9:
            return float('inf')
        return self.p.wheelbase / math.tan(abs(delta))

    def rectangle_corners(self, s: AckermannState):
        """计算车辆矩形四个角点（后轴中心为参考点，车长向前后延伸）。

        后轴中心距后悬 = (L_body - wheelbase)/2 = (1.0-0.65)/2 = 0.175 m
        后轴中心距前悬 = wheelbase + 后悬 = 0.65 + 0.175 = 0.825 m
        返回 [(x,y), ...] 4 个角点，顺序：FL, FR, RR, RL（世界系）。
        """
        p = self.p
        rear_overhang = (p.length - p.wheelbase) / 2.0   # 0.175
        front_extent = p.wheelbase + rear_overhang        # 0.825
        half_w = p.width / 2.0

        cos_t = math.cos(s.theta)
        sin_t = math.sin(s.theta)

        # 车体坐标系角点（后轴中心为原点，x 向前，y 向左）
        local = [
            (front_extent,  half_w),   # FL
            (front_extent, -half_w),   # FR
            (-rear_overhang, -half_w),  # RR
            (-rear_overhang,  half_w),  # RL
        ]
        corners = []
        for lx, ly in local:
            wx = s.x + lx * cos_t - ly * sin_t
            wy = s.y + lx * sin_t + ly * cos_t
            corners.append((wx, wy))
        return corners

    def center_to_rect_distance(self, s: AckermannState, px: float, py: float) -> float:
        """点 (px,py) 到车辆矩形的最小距离（点在矩形内返回 0）。"""
        corners = self.rectangle_corners(s)
        return point_to_rectangle_distance(px, py, corners)


def point_to_rectangle_distance(px: float, py: float, corners) -> float:
    """点到矩形（4 角点）的最小距离。点在矩形内返回 0。

    使用 SAT（分离轴定理）判断点是否在矩形内，否则计算到 4 条边的最小距离。
    """
    # 转换到矩形局部坐标系
    cx = sum(c[0] for c in corners) / 4.0
    cy = sum(c[1] for c in corners) / 4.0
    # 矩形 x 轴方向（FL - RL 的归一化）
    ax = corners[0][0] - corners[3][0]
    ay = corners[0][1] - corners[3][1]
    length = math.hypot(ax, ay)
    if length < 1e-12:
        return math.hypot(px - cx, py - cy)
    ax /= length
    ay /= length
    # y 轴（垂直）
    bx = -ay
    by = ax

    dx = px - cx
    dy = py - cy
    u = dx * ax + dy * ay   # 沿车长方向投影
    v = dx * bx + dy * by   # 沿车宽方向投影

    half_l = length / 2.0
    # 从角点计算半宽（FL - FR 的距离 / 2）
    half_w = math.hypot(corners[0][0] - corners[1][0],
                        corners[0][1] - corners[1][1]) / 2.0

    # 点在矩形内
    if abs(u) <= half_l and abs(v) <= half_w:
        return 0.0

    # 计算到矩形边界的最小距离
    du = max(abs(u) - half_l, 0.0)
    dv = max(abs(v) - half_w, 0.0)
    return math.hypot(du, dv)


def check_rect_circle_collision(corners, cx: float, cy: float, r: float) -> bool:
    """矩形（4 角点）与圆（中心 cx,cy，半径 r）是否碰撞。"""
    d = point_to_rectangle_distance(cx, cy, corners)
    return d < r
