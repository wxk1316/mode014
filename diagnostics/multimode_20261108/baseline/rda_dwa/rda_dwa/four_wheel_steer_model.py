"""
four_wheel_steer_model.py — 四轮独立转向（4WIS）车辆运动学模型。

真实车辆特性：
  - 四轮独立转向（每个轮可独立转向 ±δ_max），支持原地转向（v≈0 时 ω 有效）
  - 绕车体几何中心旋转（4WIS 全轮转向的标准旋转中心）
  - 本次不做斜行（β=0，仅沿车头方向平移 + 绕中心自转）

与 ackermann_model.py 的区别：
  - v≈0 时 ω 仍有效（原地转向），不再有 "v 过小禁止转向" 的 Ackermann 限制
  - 参考点为车体几何中心（矩形角点 ±(L/2, W/2)），而非后轴中心
  - ω 与 v 解耦：4WIS 的转向能力不依赖前进速度，只受 ω_max / ω_spin_max 限制

运动学（绕几何中心）：
    x' = x + v·cos(θ)·dt        (v>0 时)
    y' = y + v·sin(θ)·dt        (v>0 时)
    θ' = θ + ω·dt               (v≈0 时原地旋转，位置不变)

四轮逆运动学（ICR 全轮转向几何）：
    给定车体 (v, ω)，旋转中心 ICR = (0, -v/ω)（车体系，x 前正 y 左正）
    轮 i 位置 (l_i, w_i) → 轮速方向 v_i = ω × (r_i - ICR)，转向角 δ_i = atan2(v_i_y, v_i_x)
    归一化到 [-π/2, π/2]（超出表示轮速反向），并限幅 ±δ_max
    原地转向 v=0：ICR = 几何中心，四轮对称指向旋转切线（对角轮 ±atan(W/L) 方向）
"""
from __future__ import annotations

import math

from .robot_model import wrap_angle


class FourWheelSteerParams:
    """4WIS 车辆参数（用户确认值：δ_max=60°，其余沿用原 Ackermann 参数）。"""

    def __init__(self):
        self.length = 1.0          # 车长 L (m)
        self.width = 0.6           # 车宽 W (m)
        self.delta_max = math.radians(60.0)   # 每轮最大转向角 (rad)，用户实测 60°
        self.delta_dot_max = math.radians(90.0)  # 每轮最大转向角速度 (rad/s)，默认 90°/s
        self.v_max = 1.5           # 最大线速度 (m/s)
        self.v_min = -1.5          # 最小线速度 (m/s)（真实 4WIS 车双向驱动，支持倒车脱困）
        self.a_max = 1.0           # 最大加速度 (m/s²)
        self.dec_max = 1.5         # 最大减速度 (m/s²)
        self.omega_max = 1.0       # 行进中最大角速度 (rad/s)
        self.omega_spin_max = 0.8  # 原地转向最大角速度 (rad/s)，用户确认
        self.omega_acc = 1.0       # 角加速度上限 (rad/s²)
        self.v_eps = 0.05          # v 低于此值视为近似静止（允许原地转向）

    @property
    def r_circum(self) -> float:
        """外接圆半径（车体中心到角点）。"""
        return math.hypot(self.length / 2.0, self.width / 2.0)

    @property
    def spin_steer_angle(self) -> float:
        """原地转向所需轮转角（几何中心旋转时四角轮的最小转角需求）。"""
        return math.atan2(self.width, self.length)  # atan(0.3/0.5) ≈ 31°

    @property
    def spin_steer_angle_actual(self) -> float:
        """原地转向实际轮转角：90° - atan(W/L)（切线方向与车头夹角）。"""
        return math.pi / 2.0 - math.atan2(self.width, self.length)  # ≈ 59°


class FourWheelSteerState:
    """4WIS 车辆状态：位姿 + 速度 + 四轮转向角。"""

    __slots__ = ('x', 'y', 'theta', 'v', 'omega',
                 'd_FL', 'd_FR', 'd_RL', 'd_RR')

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, omega=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = float(v)
        self.omega = float(omega)
        self.d_FL = 0.0
        self.d_FR = 0.0
        self.d_RL = 0.0
        self.d_RR = 0.0

    def to_array(self):
        return [self.x, self.y, self.theta, self.v, self.omega,
                self.d_FL, self.d_FR, self.d_RL, self.d_RR]


_WHEEL_LOCAL = [
    (0.5, 0.3),    # FL（车体前左，x 前正 y 左正）
    (0.5, -0.3),   # FR
    (-0.5, 0.3),   # RL
    (-0.5, -0.3),  # RR
]


class FourWheelSteerModel:
    """4WIS 运动学：v/ω 解耦 + 原地转向 + ICR 四轮解算。"""

    def __init__(self, params: FourWheelSteerParams | None = None):
        self.p = params or FourWheelSteerParams()

    # ------------------------------------------------------------------
    # 顶层指令适配（v, ω 均为算法输出，直接可用；只做运动学约束）
    # ------------------------------------------------------------------
    def adapt_velocity(self, v_cmd: float, omega_cmd: float,
                       cur_v: float, cur_omega: float, dt: float):
        """将算法输出 (v_cmd, omega_cmd) 施加运动学约束后返回 (v, omega)。

        - v 施加加速度/减速度约束并限幅 [v_min, v_max]
        - ω 施加角加速度约束并限幅（v≈0 时上限为 omega_spin_max，否则 omega_max）
        """
        p = self.p
        dv_max = p.a_max * dt
        dv_min = -p.dec_max * dt
        v = cur_v + max(dv_min, min(dv_max, v_cmd - cur_v))
        v = max(p.v_min, min(p.v_max, v))

        w_lim = p.omega_spin_max if v < p.v_eps else p.omega_max
        w = max(-w_lim, min(w_lim, omega_cmd))
        dw_max = p.omega_acc * dt
        w = cur_omega + max(-dw_max, min(dw_max, w - cur_omega))
        w = max(-w_lim, min(w_lim, w))
        return v, w

    # ------------------------------------------------------------------
    # 前向积分（绕几何中心）
    # ------------------------------------------------------------------
    def integrate(self, s: FourWheelSteerState, v: float, omega: float,
                  dt: float) -> FourWheelSteerState:
        """运动学积分：v≈0 时原地旋转（位置不变），否则平移+旋转。"""
        nxt = FourWheelSteerState()
        if abs(v) < self.p.v_eps:
            nxt.x = s.x
            nxt.y = s.y
        else:
            nxt.x = s.x + v * math.cos(s.theta) * dt
            nxt.y = s.y + v * math.sin(s.theta) * dt
        nxt.theta = wrap_angle(s.theta + omega * dt)
        nxt.v = v
        nxt.omega = omega
        # 四轮转向角（ICR 解算 + 变化率约束 + 限幅）
        d_targets = self.wheel_steer_angles(v, omega)
        ddot_max = self.p.delta_dot_max * dt
        cur = [s.d_FL, s.d_FR, s.d_RL, s.d_RR]
        new_d = []
        for c, t in zip(cur, d_targets):
            d = c + max(-ddot_max, min(ddot_max, t - c))
            d = max(-self.p.delta_max, min(self.p.delta_max, d))
            new_d.append(d)
        nxt.d_FL, nxt.d_FR, nxt.d_RL, nxt.d_RR = new_d
        return nxt

    # ------------------------------------------------------------------
    # 四轮逆运动学（ICR 全轮转向）
    # ------------------------------------------------------------------
    def wheel_steer_angles(self, v: float, omega: float):
        """由车体 (v, ω) 解算四轮目标转向角（限幅 ±δ_max）。

        返回 (d_FL, d_FR, d_RL, d_RR)，弧度。
        纯直行 ω≈0 → 全 0；原地转向 v≈0 → 四轮对称切线方向（±59°）。
        """
        p = self.p
        if abs(omega) < 1e-9:
            return (0.0, 0.0, 0.0, 0.0)
        R = v / omega  # 有符号 ICR 距离（ICR = (0, -R) 车体系）
        out = []
        for (lx, ly) in _WHEEL_LOCAL:
            rx = lx
            ry = ly + R            # 轮→ICR 向量
            vx = -omega * ry       # ω × r 的 x 分量
            vy = omega * rx        # ω × r 的 y 分量
            d = math.atan2(vy, vx)
            # 归一化到 [-π/2, π/2]（超出 90° 表示轮速反向）
            if d > math.pi / 2.0:
                d -= math.pi
            elif d < -math.pi / 2.0:
                d += math.pi
            d = max(-p.delta_max, min(p.delta_max, d))
            out.append(d)
        return tuple(out)

    def wheel_speeds(self, v: float, omega: float):
        """四轮轮速（m/s）。返回 (s_FL, s_FR, s_RL, s_RR)。"""
        if abs(omega) < 1e-9:
            return (abs(v), abs(v), abs(v), abs(v))
        R = v / omega
        out = []
        for (lx, ly) in _WHEEL_LOCAL:
            rx = lx
            ry = ly + R
            sp = abs(omega) * math.hypot(rx, ry)
            out.append(sp)
        return tuple(out)

    # ------------------------------------------------------------------
    # 矩形几何（参考点 = 车体几何中心）
    # ------------------------------------------------------------------
    def rectangle_corners(self, s: FourWheelSteerState):
        """计算车辆矩形四个角点（几何中心参考，±(L/2, W/2)）。

        返回 [(x,y), ...] 4 个角点，顺序：FL, FR, RR, RL（世界系）。
        """
        p = self.p
        half_l = p.length / 2.0
        half_w = p.width / 2.0
        cos_t = math.cos(s.theta)
        sin_t = math.sin(s.theta)
        local = [
            (half_l, half_w),    # FL
            (half_l, -half_w),   # FR
            (-half_l, -half_w),  # RR
            (-half_l, half_w),   # RL
        ]
        corners = []
        for lx, ly in local:
            wx = s.x + lx * cos_t - ly * sin_t
            wy = s.y + lx * sin_t + ly * cos_t
            corners.append((wx, wy))
        return corners

    def center_to_rect_distance(self, s: FourWheelSteerState, px: float, py: float) -> float:
        """点 (px,py) 到车辆矩形的最小距离（点在矩形内返回 0）。"""
        from .ackermann_model import point_to_rectangle_distance
        corners = self.rectangle_corners(s)
        return point_to_rectangle_distance(px, py, corners)

    # ------------------------------------------------------------------
    # 原地旋转扫掠检查辅助
    # ------------------------------------------------------------------
    def spin_sweep_collides(self, s: FourWheelSteerState, omega: float,
                            dt: float, n_steps: int,
                            obstacles) -> bool:
        """原地旋转轨迹（位置不动，θ 线性旋转）逐位姿矩形 SAT 碰撞检查。

        返回 True 表示旋转过程中某位姿与障碍碰撞。
        """
        from .ackermann_model import check_rect_circle_collision
        th = s.theta
        for i in range(n_steps):
            th = wrap_angle(th + omega * dt)
            tmp = FourWheelSteerState(s.x, s.y, th, 0.0, omega)
            corners = self.rectangle_corners(tmp)
            for o in obstacles:
                if check_rect_circle_collision(corners, o.x, o.y, o.radius):
                    return True
        return False
