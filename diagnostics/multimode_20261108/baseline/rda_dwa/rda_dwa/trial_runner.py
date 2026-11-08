"""
trial_runner.py — 单一试验引擎（C1：7 份试验循环副本收拢）。

原先 trial.py / ackermann_trial.py / four_wheel_trial.py 中散布的 7 份试验循环
（run_trial / run_trial_simple / run_trial_with_planner / run_ackermann_trial /
run_ackermann_trial_with_planner / run_four_wheel_trial /
run_four_wheel_trial_with_planner）收敛为**一份主循环**，差异只经三个接缝注入：

  1. VehicleAdapter —— 车辆运动学/碰撞几何/专属统计（差速点模型 / Ackermann / 4WIS）
  2. TrapController —— 逃逸状态机（classic / simple / evolved / none）
  3. MetricsCollector —— 指标族（trial / ackermann / four_wheel；simple 只回路径）

硬约束：不改变任何算法原理/流程/逻辑与基线算法。所有载荷文本来自原各 trial
文件的循环体，逐字节搬移；行为经 C1 差分验证（7 入口 × 场景 × 配置 逐位一致）。
"""
from __future__ import annotations

import math
import time

import numpy as np

from .params import Params
from .robot_model import RobotModel, RobotState, new_robot, wrap_angle
from .obstacle import new_obstacle, circum_radius
from .predictor import ObstaclePredictor, IMMUKFPredictor
from .enhanced_dwa import EnhancedDWA
from .scenario import Scene
from .ackermann_model import (AckermannModel, AckermannState, AckermannParams,
                              point_to_rectangle_distance,
                              check_rect_circle_collision)
from .four_wheel_steer_model import (FourWheelSteerModel, FourWheelSteerParams,
                                     FourWheelSteerState)
from .collision_checker import (_rect_corners_vec, _rect_rect_gaps_vec,
                                _is_rect_obstacle)
from .global_guide import GlobalGuide


def _rect_hit(corners, o) -> bool:
    """闭环碰撞判定：车矩形（corners 四点）vs 障碍矩形（o：x/y/heading/length/width）。

    现实类型障碍用精确矩形 SAT 净距（<=0 即碰撞）；旧圆障碍回退
    check_rect_circle_collision（点-距 r）。返回是否碰撞。
    """
    if _is_rect_obstacle(o):
        c = np.asarray(corners, dtype=np.float64)
        cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
        e0 = c[1] - c[0]
        e1 = c[3] - c[0]
        th = math.atan2(e0[1], e0[0])
        L = float(math.hypot(e0[0], e0[1]))
        W = float(math.hypot(e1[0], e1[1]))
        A = _rect_corners_vec(np.array([float(o.x)]), np.array([float(o.y)]),
                              np.array([float(o.heading)]),
                              float(o.length), float(o.width), 0.0)
        B = _rect_corners_vec(np.array([cx]), np.array([cy]),
                              np.array([th]), max(L, 1e-9), max(W, 1e-9), 0.0)
        return bool(float(_rect_rect_gaps_vec(A, B[0])[0]) <= 0.0)
    return check_rect_circle_collision(corners, o.x, o.y, o.radius)


def _rect_footprint_gap(corners, o) -> float:
    """车矩形 vs 障碍的最近净距（矩形精确口径）。圆障碍回退 点-距 r。"""
    if _is_rect_obstacle(o):
        c = np.asarray(corners, dtype=np.float64)
        cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
        e0 = c[1] - c[0]
        e1 = c[3] - c[0]
        th = math.atan2(e0[1], e0[0])
        L = float(math.hypot(e0[0], e0[1]))
        W = float(math.hypot(e1[0], e1[1]))
        A = _rect_corners_vec(np.array([float(o.x)]), np.array([float(o.y)]),
                              np.array([float(o.heading)]),
                              float(o.length), float(o.width), 0.0)
        B = _rect_corners_vec(np.array([cx]), np.array([cy]),
                              np.array([th]), max(L, 1e-9), max(W, 1e-9), 0.0)
        return float(_rect_rect_gaps_vec(A, B[0])[0])
    return point_to_rectangle_distance(o.x, o.y, corners) - float(o.radius)


# ---------------------------------------------------------------------------
# 结果 / 指标结构（由原 trial 文件逐字迁移；原文件 re-export 保持兼容）
# ---------------------------------------------------------------------------
class TrialResult:
    """result 结构体等价。"""
    def __init__(self):
        self.reached = False
        self.collision = False
        self.timeout = False
        self.time = 0.0


class TrialMetrics:
    """metrics 结构体等价（差速点模型）。"""
    def __init__(self):
        self.path_length_raw = 0.0
        self.path_length_normalized = 0.0
        self.smoothness = 0.0
        self.avg_comp_time_ms = 0.0
        self.max_comp_time_ms = 0.0
        self.success_rate = 0.0
        self.min_obstacle_distance = 0.0
        self.oscillation_count = 0
        self.dynamic_collision = 0.0
        self.completion_time = 0.0
        self.reached = False
        self.collision = False
        self.timeout = False
        self.pred_rmse = float('nan')
        self.avg_candidates = float('nan')
        self.avg_feasible_ratio = float('nan')
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.max_stall_run = 0
        self.path = np.zeros((0, 4))
        self.state_series = np.zeros((0, 7))


class AckermannTrialMetrics:
    """Ackermann 试验指标（在原有指标基础上新增车辆可行性指标）。"""
    def __init__(self):
        # 原有指标
        self.path_length_raw = 0.0
        self.path_length_normalized = 0.0
        self.smoothness = 0.0
        self.avg_comp_time_ms = 0.0
        self.max_comp_time_ms = 0.0
        self.success_rate = 0.0
        self.min_obstacle_distance = 0.0   # 中心点最小距离（兼容旧字段）
        self.oscillation_count = 0
        self.dynamic_collision = 0.0
        self.completion_time = 0.0
        self.reached = False
        self.collision = False
        self.timeout = False
        self.pred_rmse = float('nan')
        self.avg_candidates = float('nan')
        self.avg_feasible_ratio = float('nan')
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.max_stall_run = 0
        self.path = np.zeros((0, 4))
        self.state_series = np.zeros((0, 7))
        # 新增 Ackermann 指标
        self.curvature_violations = 0      # |κ| > κ_max 的步数
        self.curvature_violation_ratio = 0.0
        self.footprint_clearance = float('inf')  # 矩形轮廓到最近障碍的最小距离
        self.min_turning_radius = float('inf')   # 实际轨迹最小转弯半径
        self.max_curvature = 0.0
        self.avg_curvature = 0.0
        self.delta_series = np.zeros(0)
        self.kappa_series = np.zeros(0)
        self.v_series_ack = np.zeros(0)
        # 车辆参数（写入 JSON 供验收）
        self.vehicle_length = 1.0
        self.vehicle_width = 0.6
        self.vehicle_wheelbase = 0.65
        self.vehicle_delta_max = math.radians(30.0)
        # v2: 算法内部碰撞模式
        self.internal_collision = "rect4wis"


class FourWheelTrialMetrics:
    """4WIS 试验指标（在原有指标基础上新增原地转向指标）。"""
    def __init__(self):
        # 原有指标
        self.path_length_raw = 0.0
        self.path_length_normalized = 0.0
        self.smoothness = 0.0
        self.avg_comp_time_ms = 0.0
        self.max_comp_time_ms = 0.0
        self.success_rate = 0.0
        self.min_obstacle_distance = 0.0   # 几何中心点最小距离（兼容旧字段）
        self.oscillation_count = 0
        self.dynamic_collision = 0.0
        self.completion_time = 0.0
        self.reached = False
        self.collision = False
        self.timeout = False
        self.pred_rmse = float('nan')
        self.avg_candidates = float('nan')
        self.avg_feasible_ratio = float('nan')
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.max_stall_run = 0
        self.path = np.zeros((0, 4))
        self.state_series = np.zeros((0, 7))
        # 矩形轮廓指标
        self.footprint_clearance = float('inf')  # 矩形轮廓到最近障碍的最小距离
        self.min_turning_radius = float('inf')   # 实际轨迹最小转弯半径（v>0 时）
        # 4WIS / 原地转向指标
        self.spin_usage_ratio = 0.0      # 原地转向控制周期占比（spin_steps / n_steps）
        self.max_spin_duration = 0.0     # 最长连续原地旋转时长 (s)
        self.spin_cycles = 0             # 原地转向事件次数
        self.total_spin_time = 0.0       # 累计原地转向时长 (s)
        self.delta_FL_series = np.zeros(0)
        self.delta_FR_series = np.zeros(0)
        self.delta_RL_series = np.zeros(0)
        self.delta_RR_series = np.zeros(0)
        self.wheel_speed_series = np.zeros(0)  # 四轮最大轮速
        self.v_series_4wis = np.zeros(0)
        self.w_series_4wis = np.zeros(0)
        # 车辆参数（写入 JSON 供验收）
        self.vehicle_length = 1.0
        self.vehicle_width = 0.6
        self.vehicle_delta_max = math.radians(60.0)
        self.vehicle_omega_spin_max = 0.8
        self.internal_collision = "rect4wis"


class DiffRectTrialMetrics:
    """真实尺寸矩形差速驱动试验指标（1.0×0.6m，几何中心参考，支持原地自转）。"""
    def __init__(self):
        self.path_length_raw = 0.0
        self.path_length_normalized = 0.0
        self.smoothness = 0.0
        self.avg_comp_time_ms = 0.0
        self.max_comp_time_ms = 0.0
        self.success_rate = 0.0
        self.min_obstacle_distance = 0.0
        self.oscillation_count = 0
        self.dynamic_collision = 0.0
        self.completion_time = 0.0
        self.reached = False
        self.collision = False
        self.timeout = False
        self.pred_rmse = float('nan')
        self.avg_candidates = float('nan')
        self.avg_feasible_ratio = float('nan')
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.max_stall_run = 0
        self.path = np.zeros((0, 4))
        self.state_series = np.zeros((0, 7))
        # 矩形轮廓指标
        self.footprint_clearance = float('inf')
        self.min_turning_radius = float('inf')
        # 原地自转指标（差速天然支持）
        self.spin_usage_ratio = 0.0
        self.max_spin_duration = 0.0
        self.spin_cycles = 0
        self.total_spin_time = 0.0
        self.v_series_diff = np.zeros(0)
        self.w_series_diff = np.zeros(0)
        # 车辆参数（写入 JSON 供验收）
        self.vehicle_length = 1.0
        self.vehicle_width = 0.6
        self.vehicle_omega_spin_max = 0.8
        self.internal_collision = "rect_diff"


# ---------------------------------------------------------------------------
# 接缝 1：VehicleAdapter —— 车辆运动学/碰撞几何/专属统计
# 载荷来自原各 trial 循环体，逐字节搬移（只换宿主）。
# ---------------------------------------------------------------------------
class VehicleAdapter:
    """车辆适配器接缝。子类承载：plan 输入构造、控制适配与积分、回写、
    碰撞几何、专属统计与指标收尾。"""
    footprint_collision = False   # 是否做矩形轮廓净空/碰撞

    def __init__(self, pp: Params):
        self.pp = pp
        self.min_turn_radius = float('inf')

    # -- 状态生命周期 --
    def init_state(self, sc: Scene):
        raise NotImplementedError

    def alloc_series(self, n_steps: int):
        """按需分配专属系列数组（delta/kappa、四轮转向角等）。"""
        pass

    # -- 每步钩子 --
    def make_plan_robot(self, robot: RobotState, state) -> RobotState:
        """算法 plan 输入状态（Ackermann 由 AckermannState 回写 RobotState）。
        state 为当前（本步积分前）物理状态；robot 为回写后的 RobotState。"""
        return robot

    def machine_robot(self, robot: RobotState, plan_robot: RobotState) -> RobotState:
        """逃逸状态机读取的状态（trial/fw 用积分后 robot，ackermann 用 plan 输入）。"""
        return robot

    def adapt_integrate(self, v_cmd: float, w_cmd: float, state, dt: float):
        """控制适配 + 运动学积分 → (v_实际, w_记录, 新 state)。"""
        raise NotImplementedError

    def writeback(self, robot: RobotState, state, v: float, w: float) -> RobotState:
        """把物理状态回写为 robot（供记录与下一次 plan）。"""
        raise NotImplementedError

    def record_series(self, n_rec: int, v: float, w: float, state):
        """专属系列记录（默认无）。"""
        pass

    def step_stats(self, v_cmd: float, w_cmd: float, v: float, w: float,
                   state, dt: float):
        """专属逐步统计（曲率 / 原地转向 / 最小转弯半径，默认无）。"""
        pass

    def finish_stats(self):
        """循环结束收尾统计（默认无）。"""
        pass

    def corners(self, state):
        """矩形轮廓角点（无 footprint 的模型返回 None）。"""
        return None

    def collision_hit(self, robot: RobotState, corners, obs) -> bool:
        """闭环碰撞判定（点圆 或 矩形 SAT）。"""
        raise NotImplementedError


class AckermannVehicle(VehicleAdapter):
    """Ackermann 自行车模型：plan 用 AckermannState 回写的 RobotState（ω 由 δ 反推），
    矩形轮廓碰撞；新增曲率/转弯半径/δ 系列指标。"""
    footprint_collision = True

    def __init__(self, pp: Params, ack_params: AckermannParams | None = None,
                 collision_model: str = "rectangle"):
        super().__init__(pp)
        self.ack = AckermannModel(ack_params)
        self.ap = self.ack.p
        self.collision_model = collision_model
        self.state = None
        self.last_delta = 0.0
        self.last_kappa = 0.0
        self.curvature_violations = 0
        self.max_kappa = 0.0
        self.sum_kappa = 0.0
        self.delta_series = None
        self.kappa_series = None

    def init_state(self, sc: Scene):
        self.state = AckermannState(sc.start_x, sc.start_y, sc.start_theta,
                                    v=0.0, delta=0.0)
        return self.state

    def alloc_series(self, n_steps: int):
        self.delta_series = np.zeros(n_steps)
        self.kappa_series = np.zeros(n_steps)

    def make_plan_robot(self, robot, state):
        rs = RobotState()
        rs.x = state.x
        rs.y = state.y
        rs.theta = state.theta
        rs.v = state.v
        # omega 由当前 v 和 δ 反推：ω = v·tan(δ)/L
        rs.omega = (state.v / self.ap.wheelbase) * math.tan(state.delta) \
            if abs(state.delta) > 1e-9 else 0.0
        return rs

    def machine_robot(self, robot, plan_robot):
        return plan_robot

    def adapt_integrate(self, v_cmd, w_cmd, state, dt):
        v, delta, kappa = self.ack.adapt_velocity(v_cmd, w_cmd, state.v,
                                                  state.delta, dt)
        self.last_delta = delta
        self.last_kappa = kappa
        state = self.ack.integrate(state, v, delta, dt)
        omega = (v / self.ap.wheelbase) * math.tan(delta) if abs(delta) > 1e-9 else 0.0
        return v, omega, state

    def writeback(self, robot, state, v, w):
        robot.x = state.x
        robot.y = state.y
        robot.theta = state.theta
        robot.v = state.v
        robot.omega = w
        return robot

    def record_series(self, n_rec, v, w, state):
        self.delta_series[n_rec] = self.last_delta
        self.kappa_series[n_rec] = self.last_kappa

    def step_stats(self, v_cmd, w_cmd, v, w, state, dt):
        kappa = self.last_kappa
        if abs(kappa) > self.ap.kappa_max + 1e-9:
            self.curvature_violations += 1
        if abs(kappa) > self.max_kappa:
            self.max_kappa = abs(kappa)
        self.sum_kappa += abs(kappa)
        if abs(self.last_delta) > 1e-9:
            r = self.ap.wheelbase / math.tan(abs(self.last_delta))
            if r < self.min_turn_radius:
                self.min_turn_radius = r

    def corners(self, state):
        return self.ack.rectangle_corners(state)

    def collision_hit(self, robot, corners, obs):
        for i in range(len(obs)):
            if _rect_hit(corners, obs[i]):
                return True
        return False


class FourWheelVehicle(VehicleAdapter):
    """4WIS（四轮独立转向）：v≈0 时 ω 依然有效（原地转向，绕几何中心）；
    矩形轮廓碰撞；新增原地转向/四轮转向角系列指标。"""
    footprint_collision = True

    def __init__(self, pp: Params, fws_params: FourWheelSteerParams | None = None,
                 collision_model: str = "rect4wis"):
        super().__init__(pp)
        self.fws = FourWheelSteerModel(fws_params)
        self.fp = self.fws.p
        self.collision_model = collision_model
        self.state = None
        self.spin_steps = 0
        self.spin_run = 0
        self.spin_run_max = 0
        self.spin_cycles = 0
        self.total_spin_time = 0.0
        self.d_FL_s = None
        self.d_FR_s = None
        self.d_RL_s = None
        self.d_RR_s = None
        self.wheel_spd_s = None

    @property
    def v_eps(self) -> float:
        return self.fp.v_eps

    def init_state(self, sc: Scene):
        self.state = FourWheelSteerState(sc.start_x, sc.start_y, sc.start_theta,
                                         v=0.0, omega=0.0)
        return self.state

    def alloc_series(self, n_steps: int):
        self.d_FL_s = np.zeros(n_steps)
        self.d_FR_s = np.zeros(n_steps)
        self.d_RL_s = np.zeros(n_steps)
        self.d_RR_s = np.zeros(n_steps)
        self.wheel_spd_s = np.zeros(n_steps)

    def adapt_integrate(self, v_cmd, w_cmd, state, dt):
        v, w = self.fws.adapt_velocity(v_cmd, w_cmd, state.v, state.omega, dt)
        state = self.fws.integrate(state, v, w, dt)
        return v, w, state

    def writeback(self, robot, state, v, w):
        robot.x = state.x
        robot.y = state.y
        robot.theta = state.theta
        robot.v = state.v
        robot.omega = state.omega
        return robot

    def record_series(self, n_rec, v, w, state):
        self.d_FL_s[n_rec] = state.d_FL
        self.d_FR_s[n_rec] = state.d_FR
        self.d_RL_s[n_rec] = state.d_RL
        self.d_RR_s[n_rec] = state.d_RR
        self.wheel_spd_s[n_rec] = max(self.fws.wheel_speeds(v, w))

    def step_stats(self, v_cmd, w_cmd, v, w, state, dt):
        # 原地转向统计（v≈0 且 ω 有效）
        is_spin = abs(v) < self.fp.v_eps and abs(w) > 0.05
        if is_spin:
            self.spin_steps += 1
            self.spin_run += 1
            self.total_spin_time += dt
            if self.spin_run > self.spin_run_max:
                self.spin_run_max = self.spin_run
        else:
            if self.spin_run > 0:
                self.spin_cycles += 1
            self.spin_run = 0
        # 最小转弯半径（仅行进段）
        if abs(v) >= self.fp.v_eps and abs(w) > 1e-6:
            r = abs(v) / abs(w)
            if r < self.min_turn_radius:
                self.min_turn_radius = r

    def finish_stats(self):
        # 结尾若仍在旋转中，计一个事件
        if self.spin_run > 0:
            self.spin_cycles += 1

    def corners(self, state):
        return self.fws.rectangle_corners(state)

    def collision_hit(self, robot, corners, obs):
        for i in range(len(obs)):
            if _rect_hit(corners, obs[i]):
                return True
        return False


# ---------------------------------------------------------------------------
# 真实尺寸矩形差速驱动车（DiffRectVehicle）
# ---------------------------------------------------------------------------
class DiffRectParams:
    """1.0×0.6m 真实尺寸矩形差速车参数（与 4WIS 同车身，运动学=纯差速）。

    - 参考点：车体几何中心（±(L/2, W/2)）
    - 双向驱动 v∈[-1.5, 1.5]（差速车可前后走）
    - v≈0 时 ω 有效（原地自转，绕几何中心）
    - 无 δ/轴距概念（与 Ackermann 本质区别）
    """
    def __init__(self):
        self.length = 1.0
        self.width = 0.6
        self.v_max = 1.5
        self.v_min = -1.5
        self.a_max = 1.0
        self.dec_max = 1.5
        self.omega_max = 1.0       # 行进中最大角速度
        self.omega_spin_max = 0.8  # 原地自转最大角速度（与 4WIS 一致）
        self.omega_acc = 1.0
        self.v_eps = 0.05

    @property
    def r_circum(self) -> float:
        return math.hypot(self.length / 2.0, self.width / 2.0)


class DiffRectState:
    """差速矩形车状态：位姿 + 线速度 + 角速度。"""
    __slots__ = ('x', 'y', 'theta', 'v', 'omega')

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, omega=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = float(v)
        self.omega = float(omega)


class DiffRectVehicle(VehicleAdapter):
    """真实尺寸矩形差速驱动车：v/ω 直接积分，支持原地自转。

    与 FourWheelVehicle 的区别：
      - 无四轮转向角（ICR 解算），就是纯差速运动学
      - v/ω 耦合：v≈0 时 ω 直接绕几何中心旋转（位置不动）
      - 矩形几何：±(L/2, W/2)，与 4WIS 同车身
    与 AckermannVehicle 的区别：
      - 无 δ/轴距概念，不做 v/ω→v/δ 适配
      - v≈0 时允许 ω 有效（阿克曼禁止原地转）
    """
    footprint_collision = True

    def __init__(self, pp: Params, params: DiffRectParams | None = None,
                 collision_model: str = "rect_diff"):
        super().__init__(pp)
        self.dp = params or DiffRectParams()
        self.collision_model = collision_model
        self.state = None
        self.spin_steps = 0
        self.spin_run = 0
        self.spin_run_max = 0
        self.spin_cycles = 0
        self.total_spin_time = 0.0

    @property
    def v_eps(self) -> float:
        return self.dp.v_eps

    def init_state(self, sc: Scene):
        self.state = DiffRectState(sc.start_x, sc.start_y, sc.start_theta,
                                    v=0.0, omega=0.0)
        return self.state

    def adapt_integrate(self, v_cmd, w_cmd, state, dt):
        """差速控制适配 + 积分。

        - v 加速度/减速度约束 + 限幅 [v_min, v_max]
        - ω 角加速度约束 + 限幅（v≈0 时上限 omega_spin_max，否则 omega_max）
        - 积分：v≈0 时位置不动、θ 更新（原地自转）；否则平移+旋转
        """
        dp = self.dp
        dv_max = dp.a_max * dt
        dv_min = -dp.dec_max * dt
        v = state.v + max(dv_min, min(dv_max, v_cmd - state.v))
        v = max(dp.v_min, min(dp.v_max, v))

        w_lim = dp.omega_spin_max if abs(v) < dp.v_eps else dp.omega_max
        w = max(-w_lim, min(w_lim, w_cmd))
        dw_max = dp.omega_acc * dt
        w = state.omega + max(-dw_max, min(dw_max, w - state.omega))
        w = max(-w_lim, min(w_lim, w))

        # 积分
        nxt = DiffRectState()
        if abs(v) < dp.v_eps:
            nxt.x = state.x
            nxt.y = state.y
        else:
            nxt.x = state.x + v * math.cos(state.theta) * dt
            nxt.y = state.y + v * math.sin(state.theta) * dt
        nxt.theta = wrap_angle(state.theta + w * dt)
        nxt.v = v
        nxt.omega = w
        return v, w, nxt

    def writeback(self, robot, state, v, w):
        robot.x = state.x
        robot.y = state.y
        robot.theta = state.theta
        robot.v = state.v
        robot.omega = state.omega
        return robot

    def step_stats(self, v_cmd, w_cmd, v, w, state, dt):
        is_spin = abs(v) < self.dp.v_eps and abs(w) > 0.05
        if is_spin:
            self.spin_steps += 1
            self.spin_run += 1
            self.total_spin_time += dt
            if self.spin_run > self.spin_run_max:
                self.spin_run_max = self.spin_run
        else:
            if self.spin_run > 0:
                self.spin_cycles += 1
            self.spin_run = 0
        if abs(v) >= self.dp.v_eps and abs(w) > 1e-6:
            r = abs(v) / abs(w)
            if r < self.min_turn_radius:
                self.min_turn_radius = r

    def finish_stats(self):
        if self.spin_run > 0:
            self.spin_cycles += 1

    def corners(self, state):
        """矩形 4 角点（几何中心参考，±(L/2, W/2)）。顺序：FL, FR, RR, RL。"""
        dp = self.dp
        half_l = dp.length / 2.0
        half_w = dp.width / 2.0
        ct = math.cos(state.theta)
        st = math.sin(state.theta)
        local = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
        ]
        out = []
        for lx, ly in local:
            wx = state.x + lx * ct - ly * st
            wy = state.y + lx * st + ly * ct
            out.append((wx, wy))
        return out

    def collision_hit(self, robot, corners, obs):
        for i in range(len(obs)):
            if _rect_hit(corners, obs[i]):
                return True
        return False


# ---------------------------------------------------------------------------
# ack_spin 执行器（阿克曼行进 + 原地 4WIS 自转）
# ---------------------------------------------------------------------------
# 规划/碰撞模型保持 rect4wis 原样（算法核心零改动）；本执行器只在"控制指令
# 下发与执行"层模拟真车 turnmode 双模：
#   - 自转判定：算法输出 (v,ω) 的曲率 |ω/v| 超过阿克曼能力 κ_max=tan(δ_max)/L
#     （即"阿克曼模型做不出来"：前轮转角需求 > δ_max），或 |v|<v_eps 且 ω≠0
#     （速度很小且进行转向）→ 一律转换为原地自转（v=0，ω 符号保持、限
#     omega_spin_max），不做低速行进转向；
#   - 其余（阿克曼能执行）→ 按阿克曼曲率约束执行：|ω| ≤ |v|·κ_max，
#     几何中心积分（与 rect4wis 参考点一致，保证闭环位姿口径不变）。
# 自转/行进判定阈值参数化（AckSpinParams），默认取真车实测 0.78m / 60°。
class AckSpinParams:
    """阿克曼行进 + 原地自转执行器参数（真车实测口径）。"""

    def __init__(self):
        self.length = 1.0                    # 车长（几何中心矩形，与 4WIS 一致）
        self.width = 0.6                     # 车宽
        self.wheelbase = 0.78                # 轴距 L（真车实测）
        self.delta_max = math.radians(60.0)  # 最大前轮转角（真车实测）
        self.delta_dot_max = math.radians(90.0)  # 转向角速度上限（与 fws 一致）
        self.v_max = 1.5
        self.v_min = -1.5
        self.a_max = 1.0
        self.dec_max = 1.5
        self.omega_max = 1.0                 # 行进最大角速度（保守上限）
        self.omega_spin_max = 0.8            # 原地自转最大角速度
        self.omega_acc = 1.0
        self.v_eps = 0.05                    # 静止判定

    @property
    def kappa_max(self) -> float:
        """阿克曼最大曲率 κ_max = tan(δ_max)/L。"""
        return math.tan(self.delta_max) / self.wheelbase

    @property
    def r_circum(self) -> float:
        return math.hypot(self.length / 2.0, self.width / 2.0)


class AckSpinState:
    """执行器状态：位姿 + 线速度 + 角速度（参考点=车体几何中心）。"""
    __slots__ = ('x', 'y', 'theta', 'v', 'omega')

    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, omega=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = float(v)
        self.omega = float(omega)


class AckermannSpinVehicle(VehicleAdapter):
    """阿克曼行进 + 原地自转执行器（真车 turnmode 0/4 双模）。

    与 FourWheelVehicle 的差异仅在 adapt_integrate：把算法输出的 (v_cmd, ω_cmd)
    按真车执行能力转换后积分；collision_model 仍传 'rect4wis'（规划/碰撞不动）。
    metrics 契约与 four_wheel 家族兼容（spin_usage_ratio / max_spin_duration /
    四轮转角系列 / min_turn_radius）。
    """

    footprint_collision = True

    def __init__(self, pp: Params,
                 ack_params: AckSpinParams | None = None,
                 fws_params: FourWheelSteerParams | None = None,
                 collision_model: str = "rect4wis"):
        super().__init__(pp)
        self.ap = ack_params or AckSpinParams()
        self.fp = fws_params or FourWheelSteerParams()
        self.fws = FourWheelSteerModel(self.fp)
        self.collision_model = collision_model
        self.state = None
        self.spin_steps = 0
        self.spin_run = 0
        self.spin_run_max = 0
        self.spin_cycles = 0
        self.total_spin_time = 0.0
        self.min_turn_radius = float('inf')
        self.d_FL_s = None
        self.d_FR_s = None
        self.d_RL_s = None
        self.d_RR_s = None
        self.wheel_spd_s = None

    @property
    def v_eps(self) -> float:
        return self.ap.v_eps

    def init_state(self, sc: Scene):
        self.state = AckSpinState(sc.start_x, sc.start_y, sc.start_theta,
                                  v=0.0, omega=0.0)
        return self.state

    def alloc_series(self, n_steps: int):
        self.d_FL_s = np.zeros(n_steps)
        self.d_FR_s = np.zeros(n_steps)
        self.d_RL_s = np.zeros(n_steps)
        self.d_RR_s = np.zeros(n_steps)
        self.wheel_spd_s = np.zeros(n_steps)

    def _should_spin(self, v: float, w_cmd: float) -> bool:
        """自转判定：曲率超限（阿克曼做不出来）或 低速且转向意图。"""
        p = self.ap
        if abs(w_cmd) > abs(v) * p.kappa_max + 1e-9:
            return True
        if abs(v) < p.v_eps and abs(w_cmd) > 1e-6:
            return True
        return False

    def adapt_integrate(self, v_cmd: float, w_cmd: float, state, dt: float):
        """控制指令转换 + 运动学积分 → (v_实际, w_记录, 新 state)。"""
        p = self.ap
        # 1) v 加速度/限幅（与 4WIS 同约束）
        v = state.v + max(-p.dec_max * dt, min(p.a_max * dt, v_cmd - state.v))
        v = max(p.v_min, min(p.v_max, v))
        # 2) 自转 / 阿克曼行进分支
        if self._should_spin(v, w_cmd):
            # --- 原地自转（turnmode=4）：v=0，位置不动，θ 旋转 ---
            w_t = math.copysign(min(abs(w_cmd), p.omega_spin_max), w_cmd) \
                if w_cmd != 0.0 else 0.0
            dw = p.omega_acc * dt
            w = state.omega + max(-dw, min(dw, w_t - state.omega))
            w = max(-p.omega_spin_max, min(p.omega_spin_max, w))
            state.x = state.x
            state.y = state.y
            state.theta = wrap_angle(state.theta + w * dt)
            state.v = 0.0
            state.omega = w
            return 0.0, w, state
        # --- 阿克曼行进（turnmode=0）：ω 受曲率约束，几何中心积分 ---
        w_lim = min(p.omega_max, abs(v) * p.kappa_max)
        w_t = max(-w_lim, min(w_lim, w_cmd))
        dw = p.omega_acc * dt
        w = state.omega + max(-dw, min(dw, w_t - state.omega))
        w = max(-w_lim, min(w_lim, w))
        state.x = state.x + v * math.cos(state.theta) * dt
        state.y = state.y + v * math.sin(state.theta) * dt
        state.theta = wrap_angle(state.theta + w * dt)
        state.v = v
        state.omega = w
        return v, w, state

    def writeback(self, robot, state, v, w):
        robot.x = state.x
        robot.y = state.y
        robot.theta = state.theta
        robot.v = state.v
        robot.omega = state.omega
        return robot

    def record_series(self, n_rec: int, v: float, w: float, state):
        """四轮转角系列（监控口径）：
        阿克曼行进 → 前轮 ±δ（δ=atan(ωL/v) 限 ±δ_max），后轮 0；
        原地自转 → 四轮 ICR 切线角（与 4WIS 自转一致 ±59°）。"""
        p = self.ap
        if abs(state.v) < p.v_eps and abs(state.omega) > 0.05:
            d = self.fws.wheel_steer_angles(0.0, state.omega)
            sp = max(self.fws.wheel_speeds(0.0, state.omega))
        else:
            if abs(state.v) > 1e-9 and abs(state.omega) > 1e-9:
                delta = math.atan(state.omega * p.wheelbase / state.v)
                delta = max(-p.delta_max, min(p.delta_max, delta))
            else:
                delta = 0.0
            d = (delta, delta, 0.0, 0.0)
            sp = abs(state.v)
        self.d_FL_s[n_rec] = d[0]
        self.d_FR_s[n_rec] = d[1]
        self.d_RL_s[n_rec] = d[2]
        self.d_RR_s[n_rec] = d[3]
        self.wheel_spd_s[n_rec] = sp

    def step_stats(self, v_cmd, w_cmd, v, w, state, dt):
        is_spin = abs(v) < self.ap.v_eps and abs(w) > 0.05
        if is_spin:
            self.spin_steps += 1
            self.spin_run += 1
            self.total_spin_time += dt
            if self.spin_run > self.spin_run_max:
                self.spin_run_max = self.spin_run
        else:
            if self.spin_run > 0:
                self.spin_cycles += 1
            self.spin_run = 0
        if abs(v) >= self.ap.v_eps and abs(w) > 1e-6:
            r = abs(v) / abs(w)
            if r < self.min_turn_radius:
                self.min_turn_radius = r

    def finish_stats(self):
        if self.spin_run > 0:
            self.spin_cycles += 1

    def corners(self, state):
        return self.fws.rectangle_corners(state)

    def collision_hit(self, robot, corners, obs):
        for i in range(len(obs)):
            if _rect_hit(corners, obs[i]):
                return True
        return False


# ---------------------------------------------------------------------------
# 接缝 2：TrapController —— 逃逸状态机
# 载荷来自原各 trial 文件机器块，逐字节搬移；控制器独占全部状态变量。
# ---------------------------------------------------------------------------
class TrapController:
    """逃逸状态机接缝。子类：Classic（trial/ackermann 内部）、
    Simple（run_trial_simple）、Evolved（four_wheel 内部）。外部基线模式为 None。"""
    def __init__(self, planner: EnhancedDWA, pp: Params):
        self.planner = planner
        self.pp = pp

    def reset(self, robot: RobotState):
        self.prev_pos = np.array([robot.x, robot.y])
        self.stall_theta_ref = None

    def step(self, robot, machine_robot, obs, goal, t=None, trace=False):
        raise NotImplementedError


class ClassicTrapController(TrapController):
    """经典 C3 状态机（run_trial / run_ackermann_trial 内部，含 stall_run_max /
    trap_total_steps / n_trap_activations 全量统计）。"""
    def __init__(self, planner, pp):
        super().__init__(planner, pp)
        self.trap = {'active': False, 'sub_goal': None, 'escape_heading': 0.0}
        self.stall = 0
        self.stall_run_max = 0
        self.stall_run = 0
        self.trap_steps = 0
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.prev_pos = None
        self.stall_theta_ref = None
        self._trap_active_disp = 0.0   # ★P0-4(U4)：trap_active 期间定位累计位移 m（退出"推进"门用）
        self._stall_time = 0.0          # ★2026-11-04：连续真停累计时长 s（dt 归一化，替代帧计数）

    def step(self, robot, machine_robot, obs, goal, t=None, trace=False, freeze_stall=False,
             v_cmd=None, dt=None):
        """robot=积分后位姿（disp_step 用）；machine_robot=逃逸判定用状态
        （trial/fw 同 robot，ackermann 为 plan 输入）。与原各 trial 文件一致。
        freeze_stall: True 时冻结 stall 位移通道（模式切换/自转期间车有意静止，不误判被困）。
        v_cmd: ★P0-4(U3) 上一周期最终下发线速度（=vehicle_bridge planner._last_cmd_v，= /cmd_vel）。
               None（仿真等旧调用方未传）→ 不施加"指令意图"门，保持旧行为；
               非 None 时：|v_cmd|≤0.05（命令停车）视为上游停车，stall 清零不计被困。
        dt: ★2026-11-04 规划周期 s（墙钟实测，5~7Hz 漂移 0.1~0.9s）。None 时回退
            pp.trap_nominal_dt；用于把"位移/帧"判据归一化为"速度"判据、把帧计数改时间触发。"""
        pp = self.pp
        planner = self.planner
        trap = self.trap
        # 2026-11-04：dt 归一化。规划墙钟 dt 漂移 0.1~0.9s，原"0.01m/帧"随 dt 差 10 倍；
        # 此处统一为速度判据 disp < trap_stall_speed·_dt、时间触发 stall_time ≥ trap_stall_time。
        _dt = dt if dt is not None else pp.trap_nominal_dt
        _stall_speed = getattr(pp, 'trap_stall_speed', 0.05)
        _stall_time_thr = getattr(pp, 'trap_stall_time', 1.5)
        disp_step = math.hypot(robot.x - self.prev_pos[0],
                               robot.y - self.prev_pos[1])
        self.prev_pos = np.array([robot.x, robot.y])
        if trap['active']:
            self.trap_steps += 1
            self.trap_total_steps += 1
            # P0-4c (U4)：累计 trap_active 期间定位位移（退出"推进"门判据用）。
            self._trap_active_disp += disp_step
            trap['sub_goal'], trap['escape_heading'] = planner.generate_local_goal(
                machine_robot, obs, goal)
            # P0-4c (U4)：trap 退出加"推进"门。trap_steps>=trap_min_steps 时，
            # 仅当 detect_trap()==False 且 trap_active 期间累计位移 > 0.5m（逃逸指令确实推动了车）
            # 才退出；否则继续逃逸，trap_max_steps=120 超时兜底保留。
            # ★第四轮 改动2：位移门 0.05→0.5m（用户指定，要求逃逸真的把车推离才退出，
            #   避免刚动一点就误退）。配合 trap_max_steps=120：规划 5~15Hz→120 帧=8~24s，
            #   escape 平均 v≈0.05~0.2m/s，0.5m 需 2.5~10s，在超时窗内可达；真停则超时兜底。
            if (self.trap_steps >= pp.trap_min_steps
                    and not planner.detect_trap(machine_robot, obs, goal)
                    and self._trap_active_disp > 0.5) \
                    or self.trap_steps > pp.trap_max_steps:
                trap['active'] = False
                self.trap_steps = 0
                self.stall = 0
                self._stall_time = 0.0   # P0-4c/2026-11-04：退出后累计停顿时长清零
                self._trap_active_disp = 0.0   # 退出后累计位移清零
        else:
            # freeze_stall=True 时冻结「检测/触发」侧（模式切换空窗 HANGUP / 自转执行
            # SPIN_RUN 期间车静止是有意的）：
            #   ① stall 清零（stall/stall_run/stall_theta_ref）——解除冻结后从 0 重新累加，
            #     不允许残留旧计数（上轮 W1 残留触发风险彻底消除）；
            #   ② 几何 detect_trap 通道同帧跳过调用与 trap 激活赋值（环节F冻结）。
            # trap 已 active（逃逸进行中）时：trap_steps 累加/sub_goal 更新照常，不冻结——
            #   理由：冻结逃逸会使 trap_max_steps=120 超时兜底失效（trap_steps 不再增长），
            #   反而增加逃逸卡死风险；用户"不触发/不累加/不判定"语义指向检测侧，非逃逸推进侧。
            if freeze_stall:
                self.stall = 0
                self.stall_run = 0
                self.stall_theta_ref = None
                self._stall_time = 0.0
            elif v_cmd is not None and abs(v_cmd) <= 0.05:
                # P0-4a (U3)：stall 累计加"指令意图"门。命令停车（|v_cmd|≤0.05，含 C 分支
                # ack_cmd(0,0)/到达/急停）时 stall 清零并冻结 stall_theta_ref——上游停车是
                # 有意静止，不应被误读为"被困"（ACK 模式 bridge_state=0 不受 freeze_stall 保护，
                # 此前 C 分支停车会照常累计 stall→误触发 trap）。
                self.stall = 0
                self.stall_run = 0
                self.stall_theta_ref = None
                self._stall_time = 0.0
            elif disp_step < _stall_speed * _dt:
                # 2026-11-04：速度判据（dt 归一化）。位移 < trap_stall_speed·dt ⇒ 速度<0.05m/s。
                # 与条件 1：朝向变化小于阈值才计被困。
                # 记录 stall 窗口起始朝向；窗口内朝向变化超 trap_heading_eps 视为原地转向
                # （自转/摆尾），不是真被困——清零 stall 不计。
                # ★第五轮 改动3：trap_heading_eps 0.3→0.533(=ω_spin_max0.8×2/3)，
                #   0.533/1.5s窗 ⇒ 仅 ω>≈0.36rad/s 的持续转向才清零 stall；慢摆尾计入被困。
                if self.stall == 0:
                    self.stall_theta_ref = robot.theta
                dtheta_win = abs(wrap_angle(robot.theta - self.stall_theta_ref))
                if dtheta_win >= pp.trap_heading_eps:
                    self.stall = 0
                    self.stall_run = 0
                    self.stall_theta_ref = None
                    self._stall_time = 0.0
                else:
                    self.stall += 1
                    self.stall_run += 1
                    self._stall_time += _dt          # 累计真停时长（时间触发，与 dt 解耦）
                    if self.stall_run > self.stall_run_max:
                        self.stall_run_max = self.stall_run
            else:
                self.stall = 0
                self.stall_run = 0
                self.stall_theta_ref = None
                self._stall_time = 0.0
            # freeze 期间不触发 trap（检测/触发侧冻结；trap 已 active 时逃逸推进照常）
            # P0-4b (U5)：stall 触发路径加近目标闸门。detect_trap 几何通道自带 d_goal<1.0m
            # 闸门（escape.py detect_trap），stall 路径此前没有——距目标 1.0m 内蠕行 1.5s 仍误激活。
            # 此处对【stall 触发】单独加 d_goal < trap_near_goal_stall_m(默认1.5m，留裕度) 跳过；
            # detect_trap 通道不重复加门（其内部已有 1.0m 闸门）。
            _d_goal = math.hypot(goal[0] - machine_robot.x, goal[1] - machine_robot.y)
            _near_goal_stall = _d_goal < getattr(pp, 'trap_near_goal_stall_m', 1.5)
            # 2026-11-04：触发改时间语义——连续真停 ≥ trap_stall_time(1.5s) 才触发，
            # 替代旧帧计数 trap_stall_steps=15（随 dt 漂移）。
            _stall_trig = self._stall_time >= _stall_time_thr and not _near_goal_stall
            if not freeze_stall and (_stall_trig \
                    or planner.detect_trap(machine_robot, obs, goal)):
                trap['active'] = True
                self.trap_steps = 0
                self._stall_time = 0.0   # 2026-11-04：激活即清零累计停顿时长
                self._trap_active_disp = 0.0   # P0-4c：激活即清零累计位移
                self.n_trap_activations += 1
                trap['sub_goal'], trap['escape_heading'] = planner.generate_local_goal(
                    machine_robot, obs, goal)


class SimpleTrapController(TrapController):
    """runTrial.m 兼容版状态机（run_trial_simple：无 trap_total_steps /
    n_trap_activations / stall_run_max 统计）。"""
    def __init__(self, planner, pp):
        super().__init__(planner, pp)
        self.trap = {'active': False, 'sub_goal': None, 'escape_heading': 0.0}
        self.stall = 0
        self.trap_steps = 0
        self.prev_pos = None

    def step(self, robot, machine_robot, obs, goal, t=None, trace=False):
        pp = self.pp
        planner = self.planner
        trap = self.trap
        disp_step = math.hypot(robot.x - self.prev_pos[0],
                               robot.y - self.prev_pos[1])
        self.prev_pos = np.array([robot.x, robot.y])
        if trap['active']:
            self.trap_steps += 1
            trap['sub_goal'], trap['escape_heading'] = planner.generate_local_goal(
                machine_robot, obs, goal)
            if (self.trap_steps >= pp.trap_min_steps
                    and not planner.detect_trap(machine_robot, obs, goal)) \
                    or self.trap_steps > pp.trap_max_steps:
                trap['active'] = False
                self.trap_steps = 0
                self.stall = 0
        else:
            if disp_step < pp.trap_progress_eps:
                self.stall += 1
            else:
                self.stall = 0
            if self.stall >= pp.trap_stall_steps \
                    or planner.detect_trap(machine_robot, obs, goal):
                trap['active'] = True
                self.trap_steps = 0
                trap['sub_goal'], trap['escape_heading'] = planner.generate_local_goal(
                    machine_robot, obs, goal)


class EvolvedTrapController(TrapController):
    """4WIS 演进状态机（run_four_wheel_trial 内部）：C3 + C5 让行等待 +
    C6 周期性重校验 + C12 对向威胁 persist + escape_subgoal（v15 覆盖）。"""
    def __init__(self, planner, pp):
        super().__init__(planner, pp)
        self.trap = {'active': False, 'sub_goal': None, 'escape_heading': 0.0,
                     'from_c12': False, 'from_ped': False, 'from_lateral': False,
                     'c12_band': 'none',
                     'hp_pos': None,           # [P0-1] HP(hit point) 逃逸激活时机器人位置
                     'force_cleared': False}   # [P0-1] 硬兜底标记（trap_steps>300 强制清除）
        self.stall = 0
        self.stall_run_max = 0
        self.stall_run = 0
        self.trap_steps = 0
        self.n_trap_activations = 0
        self.trap_total_steps = 0
        self.prev_pos = None

    def step(self, robot, machine_robot, obs, goal, t=None, trace=False):
        """Evolved 仅用于 4WIS（machine_robot 恒等于积分后 robot）；接口与
        Classic/Simple 统一：disp_step 用 robot，planner 判定用 machine_robot。"""
        pp = self.pp
        planner = self.planner
        trap = self.trap
        disp_step = math.hypot(robot.x - self.prev_pos[0],
                               robot.y - self.prev_pos[1])
        self.prev_pos = np.array([robot.x, robot.y])
        if trap['active']:
            self.trap_steps += 1
            self.trap_total_steps += 1
            # [P0-1] HP 闸门（方向确认版）：只有当前到目标距离 < HP 到目标距离时，
            # 才允许更新逃逸子目标方向。如果当前位置比 HP 更远，说明逃逸方向在把车
            # 往远离目标的方向拽，不更新（保持旧子目标），避免死循环。
            _hp_gate_ok = True
            if getattr(pp, 'enable_p0_1_hp_gate', False) and trap.get('hp_pos') is not None:
                _d_cur = math.hypot(machine_robot.x - goal[0], machine_robot.y - goal[1])
                _d_hp = math.hypot(trap['hp_pos'][0] - goal[0], trap['hp_pos'][1] - goal[1])
                if _d_cur >= _d_hp:
                    _hp_gate_ok = False
            # [MTDWA P2] 追尾带超车：c12_persist 期间每 c6_recheck_every 步重生成
            # 超车子目标（威胁在动，冻结的目标会落后），方向平滑前移不抖转。
            # [MTDWA P3] 行人让行：from_ped 期间同步频重生成让行子目标（机器人位置在变）。
            _regenerate = False
            if (pp.enable_c12_overtake and trap.get('c12_band') == 'rear'
                    and self.trap_steps % max(1, pp.c6_recheck_every) == 0):
                _regenerate = True
            elif (trap.get('from_ped', False) and pp.enable_ped_cross_evade
                    and self.trap_steps % max(1, pp.c6_recheck_every) == 0):
                _regenerate = True
            if _regenerate and _hp_gate_ok:
                trap['sub_goal'], trap['escape_heading'] = planner.escape_subgoal(
                    machine_robot, obs, goal, trap,
                    planner.headon_threat(machine_robot, obs))
            # C6：逃逸期间周期性用更长时距重校验冻结的逃逸方向（动态场景下过期→重生成）
            # C12：对向威胁存续期间每 2 步强制重校验（防止陈旧"沿车道向前"方向撞向对向车）
            elif _hp_gate_ok and ((pp.enable_c6_escape_recheck
                 and self.trap_steps % max(1, pp.c6_recheck_every) == 0)
                    or (pp.enable_c12_headon_evade
                        and planner.headon_threat(machine_robot, obs)
                        and self.trap_steps % 2 == 0)):
                planner.revalidate_escape(machine_robot, obs, goal, trap)
            # C12：对向威胁持续期间保持逃逸激活（防止反复放弃横向避让 → 再撞）
            # v14_s9lock：仅对已由 C12 激活的逃逸（trap['from_c12']）在 persist 阶段
            # 跳过角度门，方向锁存；初始触发仍保留角度门保护 S10。
            c12_persist = (pp.enable_c12_headon_evade
                           and planner.headon_threat(
                               machine_robot, obs,
                               skip_angle_gate=(pp.c12_persist_skip_angle
                                                and trap.get('from_c12', False))))
            # [MTDWA 融合] g3/g4：切入意图威胁存续 / 对向流间隙阻塞 → 维持逃逸
            # （g3：S9 切入车同速 CPA 失效，用横向合拢意图维持 persist；
            #  g4：S13 对向车流间隙内禁止回归车道中心，间隙开放才允许释放）
            _c12_hold = (c12_persist is not None
                         or (getattr(pp, 'enable_s9_lateral', False)
                             and planner.lateral_intent_threat(machine_robot, obs) is not None)
                         or (getattr(pp, 'enable_s13_gap', False)
                             and planner.gap_blocked(machine_robot, obs)))
            # [MTDWA P3] 行人让行 persist：横穿威胁存续期间保持激活，不中途放弃
            ped_persist = (trap.get('from_ped', False)
                           and pp.enable_ped_cross_evade
                           and planner.ped_cross_threat(machine_robot, obs) is not None)
            # [P0-1] Bug2 M-line 退出（带保护）：机器人回到 HP→goal 连线 ±band 内时
            # 清 escape state。保护：① |goal-hp| < p0_1_mline_min_len 时跳过；
            # ② trap_steps < trap_min_steps 时跳过（刚激活时机器人就在 HP→goal 线上，
            #   perp=0 会立即退出，逃逸动作根本没执行）。
            _mline_exit = False
            if (getattr(pp, 'enable_p0_1_hp_gate', False)
                    and trap.get('hp_pos') is not None
                    and self.trap_steps >= pp.trap_min_steps):
                _hpx = goal[0] - trap['hp_pos'][0]
                _hpy = goal[1] - trap['hp_pos'][1]
                _len_hp = math.hypot(_hpx, _hpy)
                if _len_hp > getattr(pp, 'p0_1_mline_min_len', 0.5):
                    _hx = goal[0] - machine_robot.x
                    _hy = goal[1] - machine_robot.y
                    _perp = abs(_hx * _hpy - _hy * _hpx) / _len_hp
                    if _perp < getattr(pp, 'p0_1_mline_band_m', 0.1):
                        _mline_exit = True
            # [P0-1] 硬兜底：trap_steps 超 p0_1_force_clear_steps 强制清 escape，
            # 不管 M-line/C12 persist 条件（防止 HP 闸门+M-line 互相锁死导致全超时）。
            _force_clear = (getattr(pp, 'enable_p0_1_hp_gate', False)
                            and self.trap_steps > getattr(pp, 'p0_1_force_clear_steps', 300))
            # 4WIS：冻结逃逸方向（激活时已生成），避免每步重算
            # escape_heading 导致原地转向目标不断漂移 → 抖转空转
            # g6：from_overtake 逃逸由 OvertakeStateMachine 显式释放（RETURN），
            # 不参与 C12/C3 的自动释放判据。
            if not _c12_hold and not ped_persist and not trap.get('from_overtake', False) and (
                    _force_clear
                    or _mline_exit
                    or (self.trap_steps >= pp.trap_min_steps
                        and not planner.detect_trap(machine_robot, obs, goal))
                    or self.trap_steps > pp.trap_max_steps):
                trap['active'] = False
                trap['from_c12'] = False
                trap['from_ped'] = False
                trap['from_lateral'] = False
                trap['c12_band'] = 'none'
                trap['force_cleared'] = _force_clear
                self.trap_steps = 0
                self.stall = 0
        else:
            if disp_step < pp.trap_progress_eps:
                self.stall += 1
                self.stall_run += 1
                if self.stall_run > self.stall_run_max:
                    self.stall_run_max = self.stall_run
            else:
                self.stall = 0
                self.stall_run = 0
            # C5：上下文感知——动态障碍正在逼近/横穿时（合法让行等待）不触发逃逸
            c5_block = planner.c5_wait_blocked(machine_robot, obs)
            # C12：同车道对向威胁（TTC 判定）→ 不等 stall 立即横向逃逸
            c12_threat = planner.headon_threat(machine_robot, obs)
            # [MTDWA 融合] g3：同向横向接近意图威胁（S9 切入，同速 CPA 失效的补丁）
            lat_threat = planner.lateral_intent_threat(machine_robot, obs)
            # [MTDWA P3] 行人横穿威胁 → 减速让行（独立于 C12，横穿带不满足其角度门）
            ped_threat = planner.ped_cross_threat(machine_robot, obs)
            # [MTDWA P3c] C5 抑制范围开关：默认 C5 抑制全部逃逸触发（p3b_c5 行为）；
            # enable_c5_stall_only=True 时仅抑制 stall 触发，detect_trap 静态陷阱 /
            # C12 对向 / 行人横穿仍可触发（治 S7 被全域抑制压崩，同时保留 S13 合法
            # 让行的 stall 抑制）。enable_c5_wait_aware_escape=False 时 c5_block=False，
            # 行为与 v15_latesc 逐位一致。
            _static_trig = (c12_threat is not None or lat_threat is not None
                            or ped_threat is not None
                            or planner.detect_trap(machine_robot, obs, goal))
            _stall_trig = self.stall >= pp.trap_stall_steps
            # [P0-1] 开阔场景抑制 stall-trap：d_goal > 5m 且本帧无动态障碍（所有 obs
            # speed≈0）时，stall-based trap 不触发（只允许 detect_trap 几何 U 形和
            # headon_threat C12 触发）。治 S1_sparse/S4_utrap/S14_longdist 开阔地误触发。
            if _stall_trig and getattr(pp, 'enable_p0_1_hp_gate', False):
                _d_goal_now = math.hypot(goal[0] - machine_robot.x, goal[1] - machine_robot.y)
                _has_dynamic = any(o.speed > 1e-6 for o in obs)
                if _d_goal_now > getattr(pp, 'p0_1_stall_suppress_dgoal', 5.0) and not _has_dynamic:
                    _stall_trig = False
            if getattr(pp, 'enable_c5_stall_only', False):
                _trig = _static_trig or (_stall_trig and not c5_block)
            else:
                _trig = not c5_block and (_static_trig or _stall_trig)
            if _trig:
                trap['active'] = True
                trap['from_c12'] = c12_threat is not None
                trap['from_lateral'] = lat_threat is not None and c12_threat is None
                trap['from_ped'] = (ped_threat is not None
                                    and c12_threat is None and lat_threat is None)
                trap['c12_band'] = 'none'
                # [P0-1] 记录 HP（hit point）：逃逸激活时机器人位置
                trap['hp_pos'] = [machine_robot.x, machine_robot.y]
                trap['force_cleared'] = False
                self.trap_steps = 0
                self.n_trap_activations += 1
                # v15_sigmoid / v15_latesc 逃逸子目标覆盖已收敛到
                # EscapeStrategy.escape_subgoal（rda_dwa/escape.py），语义逐字节不变；
                # g3 切入意图威胁以 from_lateral 分支在 escape_subgoal 内路由。
                _esc_threat = c12_threat if c12_threat is not None else lat_threat
                trap['sub_goal'], trap['escape_heading'] = planner.escape_subgoal(
                    machine_robot, obs, goal, trap, _esc_threat)


class OvertakeStateMachine:
    """[MTDWA 第二轮] g6：让行/超越/回归决策状态机（S13 超车，默认关）。

    卡 67（非合作动态博弈 G=<N,A_i,R_i>，Nash 均衡=(让行,切入)/(加速,让行)）的
    level-k 简化：策略枚举 {让行(跟随), 超越}，仅在 Gap Acceptance 窗口
    （对向车最快到达时间 t_arr > 超车完成时间 t_ovk + ovk_t_safe）时执行超越；
    回归由状态机显式触发（超车完成判据），替代 C12 persist 释放判据
    （解决 P2 persist 永锁 / g4 连续微调无改善的机制层缺口）。

    状态：OFF（未激活）/ FOLLOW（跟车限速）/ OVERTAKE（escape trap 注入）/
    RETURN（超车完成→回归）。
    触发条件（S13 专属）：同车道前方慢车（v<ovk_slow_ratio*v_max）∧ 对向流存在。
    其他场景不满足条件 → 状态机恒 OFF → mtdwa 行为不变。

    接口：step(robot, obs, goal) → (trap_or_None, vmax_or_None)
      trap 非 None → 调用方 trap.update（OVERTAKE 注入 / RETURN 清除）
      vmax 非 None → 调用方 v_cmd=min(v_cmd, vmax)（FOLLOW 跟车限速，只降不升）
    """

    def __init__(self, pp):
        self.p = pp
        self.state = 'OFF'
        self.slow = None
        self.side = None
        self.steps = 0

    def reset(self):
        self.state = 'OFF'
        self.slow = None
        self.side = None
        self.steps = 0

    def _find_slow(self, robot, obs):
        """同车道前方慢车（同向 |Δθ|<c12_rear_ang、同车道 |lat|<rss_lat_max、
        前方 ovk_detect_dist 内、v<ovk_slow_ratio*v_max）。返回最近者。"""
        p = self.p
        hx, hy = math.cos(robot.theta), math.sin(robot.theta)
        nx, ny = -hy, hx
        best = None
        for o in obs:
            if o.speed <= 1e-6:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang > p.c12_rear_ang:
                continue
            dx, dy = o.x - robot.x, o.y - robot.y
            along = dx * hx + dy * hy
            if along <= 0.0 or along > p.ovk_detect_dist:
                continue
            if abs(dx * nx + dy * ny) > p.rss_lat_max:
                continue
            if o.speed >= p.ovk_slow_ratio * p.max_linear_velocity:
                continue
            if best is None or along < best[0]:
                best = (along, o)
        return best

    def _find_oncoming(self, robot, obs):
        """对向流：rel_ang>c12_headon_ang 且在前方。返回最紧迫 (t_arr, o)。"""
        p = self.p
        hx, hy = math.cos(robot.theta), math.sin(robot.theta)
        worst = None
        for o in obs:
            if o.speed <= 1e-6:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang <= p.c12_headon_ang:
                continue
            dx, dy = o.x - robot.x, o.y - robot.y
            along = dx * hx + dy * hy
            if along <= 0.0:
                continue
            v_app = -(o.speed * math.cos(o.heading) * hx
                      + o.speed * math.sin(o.heading) * hy)
            if v_app <= 1e-3:
                continue
            t_arr = along / v_app
            if worst is None or t_arr < worst[0]:
                worst = (t_arr, o)
        return worst

    def _overtake_trap(self, robot, obs):
        """OVERTAKE 态 trap：横向偏移到远离对向流侧 + 沿慢车 heading 超前
        ovk_ahead 米（子目标放慢车前方，escape 路径接管；超车侧远离对向流
        平均 lat，规避 P2"邻道超前引向对向车流"的证伪点）。"""
        p = self.p
        slow = self.slow
        hx, hy = math.cos(robot.theta), math.sin(robot.theta)
        nx, ny = -hy, hx
        on_lats = []
        for o in obs:
            if o.speed <= 1e-6:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang <= p.c12_headon_ang:
                continue
            dx, dy = o.x - robot.x, o.y - robot.y
            if dx * hx + dy * hy <= 0.0:
                continue
            on_lats.append(dx * nx + dy * ny)
        side = -1.0
        if on_lats:
            side = -1.0 if sum(on_lats) / len(on_lats) >= 0.0 else 1.0
        self.side = side
        bx = slow.x + p.ovk_ahead * hx
        by = slow.y + p.ovk_ahead * hy
        sx = bx + side * p.ovk_lat_off * nx
        sy = by + side * p.ovk_lat_off * ny
        eh = math.atan2(sy - robot.y, sx - robot.x)
        return {'active': True, 'from_overtake': True, 'c12_band': 'none',
                'sub_goal': [sx, sy], 'escape_heading': eh}

    def step(self, robot, obs, goal):
        p = self.p
        if not p.enable_overtake_sm:
            return None, None
        self.steps += 1
        slow = self._find_slow(robot, obs)
        on = self._find_oncoming(robot, obs)
        if self.state == 'OFF':
            if slow is not None and on is not None:
                self.state = 'FOLLOW'
                self.slow = slow[1]
            return None, None

        if self.state == 'FOLLOW':
            if slow is None:
                self.state = 'OFF'
                return None, None
            if on is None:
                self.state = 'OFF'          # 对向流清空 → 正常行驶（无需决策层）
                return None, None
            self.slow = slow[1]
            # Gap Acceptance：对向最快到达时间 > 超车完成时间 + 裕量
            t_arr = on[0]
            d_slow = max(0.0, slow[0])
            v_e = min(p.max_linear_velocity, robot.v + 0.3)
            t_ovk = max(0.5, d_slow / max(0.2, v_e - self.slow.speed))
            if t_arr > t_ovk + p.ovk_t_safe:
                self.state = 'OVERTAKE'
            # 跟车限速（只降不升）
            return None, min(p.max_linear_velocity, self.slow.speed + 0.2)

        if self.state == 'OVERTAKE':
            if slow is None or on is None:
                self.state = 'RETURN'       # 慢车消失/对向清空 → 回归
                return {'active': False, 'from_overtake': False}, None
            self.slow = slow[1]
            # 超车完成判据：自车越过慢车（along 差 > 车长 1.0 + 裕量 1.0）
            hx, hy = math.cos(robot.theta), math.sin(robot.theta)
            d_slow = (slow[1].x - robot.x) * hx + (slow[1].y - robot.y) * hy
            if d_slow < -2.0 and on[0] > 2.0:
                self.state = 'RETURN'
                return {'active': False, 'from_overtake': False}, None
            return self._overtake_trap(robot, obs), None

        if self.state == 'RETURN':
            if slow is not None and on is not None:
                self.state = 'FOLLOW'       # 又遇新慢车+对向流 → 重新决策
            else:
                self.state = 'OFF'
            return None, None

        return None, None


# ---------------------------------------------------------------------------
# 单一主循环
# ---------------------------------------------------------------------------
def _run(pp: Params, sc: Scene, traj, *, vehicle: VehicleAdapter,
         trap_mode: str | None = None, metrics_family: str = 'trial',
         mode: str = 'ENHANCED_DWA', planner=None, collision_model: str = 'rect4wis',
         ack_params: AckermannParams | None = None,
         fws_params: FourWheelSteerParams | None = None,
         external: bool = False, init_mode: str = 'standard',
         sim_cap: bool = True, truncate: str | None = None,
         spin_timeout: bool = False, use_guide: bool = False,
         trace: bool = False, collect_pred_err: bool = True,
         collect_cand: str | None = None, collect_osc: bool = True,
         measure_time: bool = True, reset_planner: str | None = None,
         plan_safe: bool = False, record_wheel_series: bool = True,
         on_step=None):
    """单一试验主循环（7 份副本的并集，按配置逐位复现每份原语义）。"""
    # ---- 预测器（外部基线：α-β 直通；内部：IMM / α-β + C9 机动标记）----
    if external:
        pred = ObstaclePredictor(pp.dt, pp.obstacle_max_speed)
        pred.set_smoothing(1.0, 1.0)
    else:
        if pp.enable_m7_imm_ukf:
            pred = IMMUKFPredictor(pp)
        else:
            pred = ObstaclePredictor(pp.dt, pp.obstacle_max_speed)
            if pp.enable_m5_filter:
                pred.set_smoothing(pp.obs_filter_alpha, pp.obs_filter_beta)
            else:
                pred.set_smoothing(1.0, 1.0)
        spin_families = ('four_wheel', 'diff_rect')
        if metrics_family in spin_families and pp.enable_c9_uncertainty_inflate:
            pred.enable_maneuver_detect(pp.c9_resid_thresh, pp.c9_hold_steps,
                                        pp.c9_heading_jump)
        if metrics_family in spin_families and getattr(pp, 'enable_maneuver_arc', False):
            pred.enable_maneuver_arc(pp.arc_w_thresh, pp.arc_hold, pp.arc_min_dist)

    # ---- 规划器 / 逃逸状态机 ----
    if not external:
        if metrics_family == 'ackermann':
            planner = EnhancedDWA(pp, mode, collision_model=collision_model,
                                  ack_params=ack_params)
        elif metrics_family == 'four_wheel':
            planner = EnhancedDWA(pp, mode, collision_model=collision_model,
                                  fws_params=fws_params)
        elif metrics_family == 'diff_rect':
            # 矩形差速车：collision_model='rect_diff'；fws_params 占位（不使用，
            # collision_checker 走 diff_* 分支）；ack_params 占位同理。
            planner = EnhancedDWA(pp, mode, collision_model=collision_model,
                                  ack_params=ack_params, fws_params=fws_params)
        else:
            planner = EnhancedDWA(pp, mode, collision_model=collision_model)
        if trap_mode == 'classic':
            trap_ctrl = ClassicTrapController(planner, pp)
        elif trap_mode == 'evolved':
            trap_ctrl = EvolvedTrapController(planner, pp)
        elif trap_mode == 'simple':
            trap_ctrl = SimpleTrapController(planner, pp)
        else:
            trap_ctrl = None
        trap = trap_ctrl.trap if trap_ctrl is not None \
            else {'active': False, 'sub_goal': None, 'escape_heading': 0.0}
        # [MTDWA 第二轮] g6：让行/超越决策状态机（S13 超车，默认关）
        ovk_sm = OvertakeStateMachine(pp) if (
            not external and pp.enable_overtake_sm) else None
    else:
        if reset_planner is not None and hasattr(planner, 'reset'):
            if reset_planner == 'safe':
                try:
                    planner.reset()
                except Exception:
                    pass
            else:
                planner.reset()
        trap_ctrl = None
        trap = None

    c3 = pp.enable_c3_trap_escape

    # ---- C8 全局引导（4WIS 内部版）----
    guide = None
    if use_guide and pp.enable_c8_global_guide:
        try:
            guide = GlobalGuide(sc, pp)
        except Exception:
            guide = None

    trace_rec = [] if trace else None

    # ---- 初始化障碍物 ----
    robot = new_robot(sc.start_x, sc.start_y, sc.start_theta)
    state = vehicle.init_state(sc)
    goal = np.array([sc.goal_x, sc.goal_y], dtype=np.float64)

    n_obs = len(traj[0].x)
    obs = []
    # 静态障碍现实类型规格（与 static_obstacles 行一一对应；旧场景无 specs → 圆障碍）
    stat_specs = getattr(sc, 'static_obstacle_specs', None) or []
    dyn_specs = sc.dynamic_obstacles or []
    if init_mode == 'simple':
        for i in range(n_obs):
            o = new_obstacle()
            o.radius = pp.obstacle_radius
            o = pred.initialize(o, traj[0].x[i], traj[0].y[i],
                                traj[0].heading[i], sc.obstacle_speed)
            obs.append(o)
    else:
        n_stat = 0
        if hasattr(sc, 'static_obstacles') and sc.static_obstacles is not None:
            n_stat = int(np.asarray(sc.static_obstacles).shape[0])
        for i in range(n_obs):
            o = new_obstacle()
            if i < n_stat:
                # 静态：位置固定；若场景提供现实类型规格则携带 type/L/W/等效半径
                if i < len(stat_specs):
                    sp = stat_specs[i]
                    o.obstacle_type = sp.get('type', 'static_large')
                    o.length = float(sp.get('length', 1.0))
                    o.width = float(sp.get('width', 1.0))
                    o.radius = circum_radius(o.length, o.width)
                else:
                    o.radius = pp.obstacle_radius
                o = pred.initialize(o, traj[0].x[i], traj[0].y[i], 0.0, 0.0)
            else:
                # 动态：场景 dict 携带 type/L/W（运动学字段由预演提供）
                di = dyn_specs[i - n_stat] if (i - n_stat) < len(dyn_specs) else {}
                if 'type' in di:
                    o.obstacle_type = di['type']
                    o.length = float(di.get('length', 1.0))
                    o.width = float(di.get('width', 1.0))
                    o.radius = circum_radius(o.length, o.width)
                else:
                    o.radius = pp.obstacle_radius
                o = pred.initialize(o, traj[0].x[i], traj[0].y[i],
                                    traj[0].heading[i],
                                    float(di.get('speed', sc.obstacle_speed)))
            obs.append(o)

    result = TrialResult()
    t = 0.0
    dt = pp.dt
    if sim_cap:
        n_steps = min(len(traj), int(round(pp.max_sim_time / dt)))
    else:
        n_steps = len(traj)

    need_series = metrics_family != 'simple'
    path = np.zeros((n_steps, 4))
    if need_series:
        v_series = np.zeros(n_steps)
        w_series = np.zeros(n_steps)
        nearest_series = np.zeros(n_steps)
        comp_times = np.zeros(n_steps)
        vehicle.alloc_series(n_steps)
    else:
        v_series = w_series = nearest_series = comp_times = None
    cand_counts = []
    feas_ratios = []
    pred_err_sq = []
    min_clearance_center = math.inf
    min_footprint_clearance = math.inf
    prev_theta = robot.theta
    total_theta_change = 0.0
    prev_omega = 0.0
    omega_sign_changes = 0
    n_rec = 0

    if trap_ctrl is not None:
        trap_ctrl.reset(robot)
    ovk_vmax = None   # g6：FOLLOW 跟车限速（上帧状态机输出，本帧 v_cmd 应用）

    d0_goal = math.hypot(robot.x - goal[0], robot.y - goal[1])
    prog_check_done = False
    spin_consec = 0

    for k in range(n_steps):
        step = traj[k]
        for i in range(n_obs):
            obs[i] = pred.update(obs[i], step.x[i], step.y[i])
            if collect_pred_err and k < n_steps - 1:
                pred_err_sq.append((obs[i].fx - traj[k + 1].x[i]) ** 2
                                   + (obs[i].fy - traj[k + 1].y[i]) ** 2)

        if measure_time and need_series:
            t0 = time.perf_counter()

        # C8：以 A* 全局路径子目标代替终点目标（引导逃逸/采样方向）
        plan_goal = goal
        if guide is not None and guide.path is not None:
            plan_goal = np.asarray(guide.next_subgoal(robot), dtype=np.float64)

        plan_robot = vehicle.make_plan_robot(robot, state)
        if external:
            if plan_safe:
                try:
                    v_cmd, w_cmd = planner.plan(plan_robot, plan_goal, obs)
                except Exception:
                    # baseline 不收敛 → 记为该步失败停车，不中断 trial
                    v_cmd, w_cmd = 0.0, 0.0
            else:
                v_cmd, w_cmd = planner.plan(plan_robot, plan_goal, obs)
        else:
            _, v_cmd, w_cmd = planner.plan(plan_robot, plan_goal, obs, trap)
        if measure_time and need_series:
            comp_times[k] = (time.perf_counter() - t0) * 1000.0

        # 候选数采样
        if collect_cand == 'window' and (k + 1) % 30 == 1:
            wdw = planner.build_dynamic_window(plan_robot, obs, trap, goal)
            cand_counts.append(len(wdw))
            if not wdw:
                feas_ratios.append(0.0)
            else:
                feas_ratios.append(float(np.mean([s.feasible for s in wdw])))
        elif collect_cand == 'planner' and (k + 1) % 30 == 1:
            if hasattr(planner, 'last_n_candidates'):
                cand_counts.append(getattr(planner, 'last_n_candidates'))
            elif hasattr(planner, 'grid_size'):
                cand_counts.append(getattr(planner, 'grid_size'))

        # 4WIS 防死锁：连续原地转向超过 spin_timeout → 强制停车一帧，
        # 使 stall 计数增长，C3 重新生成逃逸方向（打破贴墙空转循环）
        if spin_timeout:
            if abs(v_cmd) < vehicle.v_eps and abs(w_cmd) > 0.05:
                spin_consec += 1
            else:
                spin_consec = 0
            if spin_consec * dt > 4.0:
                # v14_spinfree：C12 对向威胁存续的主动横向逃逸中（trap.active 且
                # headon_threat 存续）豁免强制停车，避免逃逸中途被 4s 超时打断。
                # v13_c12 因 spin_timeout_evade_exempt=False 行为完全不变。
                evade_exempt = (getattr(pp, 'spin_timeout_evade_exempt', False)
                                and trap['active']
                                and planner.headon_threat(robot, obs) is not None)
                if not evade_exempt:
                    v_cmd, w_cmd = 0.0, 0.0
                spin_consec = 0

        # [MTDWA 第二轮] g6：FOLLOW 态跟车限速（只降不升，来自上帧状态机输出）
        if ovk_vmax is not None:
            v_cmd = min(v_cmd, ovk_vmax)

        # 控制适配 + 运动学积分 + 回写
        v_act, w_rec, state = vehicle.adapt_integrate(v_cmd, w_cmd, state, dt)
        robot = vehicle.writeback(robot, state, v_act, w_rec)
        t = t + dt

        path[n_rec, :] = [t, robot.x, robot.y, robot.theta]
        if need_series:
            v_series[n_rec] = v_act
            w_series[n_rec] = w_rec
            vehicle.record_series(n_rec, v_act, w_rec, state)
        vehicle.step_stats(v_cmd, w_cmd, v_act, w_rec, state, dt)

        # 中心点最近距离（兼容字段）
        nd_ = math.inf
        for i in range(n_obs):
            d0_ = math.hypot(obs[i].x - robot.x, obs[i].y - robot.y)
            if d0_ < nd_:
                nd_ = d0_
        if need_series:
            nearest_series[n_rec] = nd_
        if nd_ < min_clearance_center:
            min_clearance_center = nd_

        # 矩形轮廓最近距离（footprint 模型，矩形-矩形精确净距）
        corners = vehicle.corners(state)
        if vehicle.footprint_collision:
            for i in range(n_obs):
                d_rect = _rect_footprint_gap(corners, obs[i])
                if d_rect < min_footprint_clearance:
                    min_footprint_clearance = d_rect

        n_rec += 1

        if collect_osc:
            if prev_omega * w_cmd < 0 and abs(prev_omega) > 0.05 and abs(w_cmd) > 0.05:
                omega_sign_changes += 1
            prev_omega = w_cmd

        dtheta = abs(wrap_angle(robot.theta - prev_theta))
        total_theta_change += dtheta
        prev_theta = robot.theta

        if trace_rec is not None:
            trace_rec.append(dict(
                t=round(t, 2), x=round(robot.x, 3), y=round(robot.y, 3),
                th=round(robot.theta, 3), v=round(robot.v, 3), w=round(robot.omega, 3),
                trap=bool(trap.get('active', False)),
                eh=round(trap.get('escape_heading', float('nan')), 3),
                obs=[dict(x=round(o.x, 3), y=round(o.y, 3), th=round(o.heading, 3),
                          sp=round(o.speed, 3)) for o in obs]))

        # 闭环碰撞判定
        hit = vehicle.collision_hit(robot, corners, obs)
        if hit:
            result.collision = True
            result.time = t
            break

        if math.hypot(robot.x - goal[0], robot.y - goal[1]) < pp.goal_tolerance:
            result.reached = True
            result.time = t
            break

        # 进度截断
        if truncate == 'once':
            # 非 C3 时 8s 后若无实际进展提前终止
            if not c3 and not prog_check_done and k + 1 >= 80:
                prog_check_done = True
                d_now = math.hypot(robot.x - goal[0], robot.y - goal[1])
                if d_now > 0.90 * d0_goal:
                    break
        elif truncate == 'always':
            # 与原 trial 一致：80 步后若比起点还远 10% 即放弃（逐帧检查）
            if k + 1 >= 80:
                d_now = math.hypot(robot.x - goal[0], robot.y - goal[1])
                if d_now > 0.90 * d0_goal:
                    break

        # 逃逸状态机（disp_step 用积分后 robot；detect/generate 用 family 专属
        # machine_robot——ackermann 为 plan 输入，与原各 trial 文件一致）
        if c3 and trap_ctrl is not None:
            machine_robot = vehicle.machine_robot(robot, plan_robot)
            trap_ctrl.step(robot, machine_robot, obs, plan_goal, t, trace)
            # [MTDWA 第二轮] g6：让行/超越状态机（S13，默认关）
            # trap 注入供下一帧 plan 使用；FOLLOW 限速经 ovk_vmax 于下一帧 v_cmd 应用
            if ovk_sm is not None:
                ovk_trap, ovk_vmax = ovk_sm.step(machine_robot, obs, plan_goal)
                if ovk_trap is not None:
                    for _k, _v in ovk_trap.items():
                        trap[_k] = _v

        # ---- 可选每步渲染钩子（默认 None；任何后端不得影响算法/指标）----
        # try/except 包裹：渲染失败只丢画面，绝不中断或改写闭环。
        if on_step is not None:
            try:
                on_step(k=k, t=t, robot=robot, state=state, obs=obs,
                        trap=trap, goal=goal, corners=corners,
                        v_cmd=v_cmd, w_cmd=w_cmd, v_act=v_act,
                        w_rec=w_rec, plan_robot=plan_robot,
                        plan_goal=plan_goal)
            except Exception:
                pass

    # ---- 收尾 ----
    vehicle.finish_stats()

    if not result.reached and not result.collision:
        result.timeout = True
        if metrics_family in ('four_wheel', 'diff_rect'):
            result.time = pp.max_sim_time
        else:
            result.time = t

    path = path[0:n_rec, :]

    if metrics_family == 'simple':
        return result, path

    if need_series:
        comp_times = comp_times[0:n_rec]

    # ---- 指标组装（与原各 trial 文件逐块一致） ----
    if metrics_family == 'ackermann':
        m = AckermannTrialMetrics()
    elif metrics_family == 'four_wheel':
        m = FourWheelTrialMetrics()
    elif metrics_family == 'diff_rect':
        m = DiffRectTrialMetrics()
    else:
        m = TrialMetrics()

    if metrics_family in ('four_wheel', 'diff_rect') and trace_rec is not None:
        m.trace = trace_rec

    if path.shape[0] > 1:
        if metrics_family == 'four_wheel' and external:
            # fw_wp 原实现用 sum(genexpr)（CPython float 快路径，末位与 += 循环不同）
            path_len = sum(math.hypot(path[i, 1] - path[i - 1, 1],
                                      path[i, 2] - path[i - 1, 2])
                           for i in range(1, path.shape[0]))
        else:
            path_len = 0.0
            for i in range(1, path.shape[0]):
                path_len += math.hypot(path[i, 1] - path[i - 1, 1],
                                       path[i, 2] - path[i - 1, 2])
    else:
        path_len = 0.0
    straight_dist = math.hypot(sc.goal_x - sc.start_x, sc.goal_y - sc.start_y)
    m.path_length_raw = path_len
    m.path_length_normalized = path_len / max(straight_dist, 1e-6)

    if n_rec > 1:
        m.smoothness = total_theta_change / n_rec
    else:
        m.smoothness = 0.0

    if metrics_family == 'trial' and not external:
        # runTrialAblation.m 原语义（n_rec==0 不可能，空数组均值即 nan）
        m.avg_comp_time_ms = float(np.mean(comp_times))
        m.max_comp_time_ms = float(np.max(comp_times))
    else:
        m.avg_comp_time_ms = float(np.mean(comp_times)) if n_rec > 0 else 0.0
        m.max_comp_time_ms = float(np.max(comp_times)) if n_rec > 0 else 0.0

    m.success_rate = float(result.reached)
    if math.isinf(min_clearance_center):
        min_clearance_center = 1e3
    m.min_obstacle_distance = min_clearance_center
    if collect_osc:
        m.oscillation_count = omega_sign_changes
    m.dynamic_collision = float(result.collision)
    m.completion_time = result.time
    m.reached = result.reached
    m.collision = result.collision
    m.timeout = result.timeout

    if pred_err_sq:
        m.pred_rmse = float(np.sqrt(np.mean(pred_err_sq)))
    if cand_counts:
        m.avg_candidates = float(np.mean(cand_counts))
        if feas_ratios:
            m.avg_feasible_ratio = float(np.mean(feas_ratios))

    m.n_trap_activations = getattr(trap_ctrl, 'n_trap_activations', 0) \
        if trap_ctrl is not None else 0
    m.trap_total_steps = getattr(trap_ctrl, 'trap_total_steps', 0) \
        if trap_ctrl is not None else 0
    m.max_stall_run = getattr(trap_ctrl, 'stall_run_max', 0) \
        if trap_ctrl is not None else 0

    m.path = path
    # fw_wp 原实现不组装 state_series（保持 __init__ 的空 0×7）
    if not (metrics_family == 'four_wheel' and external):
        m.state_series = np.column_stack([
            path[:, 0], path[:, 1], path[:, 2], path[:, 3],
            v_series[0:n_rec], w_series[0:n_rec], nearest_series[0:n_rec],
        ])

    # ---- 车辆专属指标 ----
    if metrics_family == 'ackermann':
        m.curvature_violations = vehicle.curvature_violations
        m.curvature_violation_ratio = vehicle.curvature_violations / max(n_rec, 1)
        if math.isinf(min_footprint_clearance):
            min_footprint_clearance = 1e3
        m.footprint_clearance = min_footprint_clearance
        m.min_turning_radius = vehicle.min_turn_radius
        m.max_curvature = vehicle.max_kappa
        m.avg_curvature = vehicle.sum_kappa / max(n_rec, 1)
        m.delta_series = vehicle.delta_series[0:n_rec]
        m.kappa_series = vehicle.kappa_series[0:n_rec]
        m.v_series_ack = v_series[0:n_rec]
        m.vehicle_length = vehicle.ap.length
        m.vehicle_width = vehicle.ap.width
        m.vehicle_wheelbase = vehicle.ap.wheelbase
        m.vehicle_delta_max = vehicle.ap.delta_max
        if external:
            m.internal_collision = getattr(planner, 'collision_model', 'rect4wis')
        else:
            m.internal_collision = collision_model
    elif metrics_family == 'four_wheel':
        if math.isinf(min_footprint_clearance):
            min_footprint_clearance = 1e3
        m.footprint_clearance = min_footprint_clearance
        m.min_turning_radius = vehicle.min_turn_radius
        m.spin_usage_ratio = vehicle.spin_steps / max(n_rec, 1)
        m.max_spin_duration = vehicle.spin_run_max * dt
        m.spin_cycles = vehicle.spin_cycles
        m.total_spin_time = vehicle.total_spin_time
        if record_wheel_series:
            m.delta_FL_series = vehicle.d_FL_s[0:n_rec]
            m.delta_FR_series = vehicle.d_FR_s[0:n_rec]
            m.delta_RL_series = vehicle.d_RL_s[0:n_rec]
            m.delta_RR_series = vehicle.d_RR_s[0:n_rec]
            m.wheel_speed_series = vehicle.wheel_spd_s[0:n_rec]
        m.v_series_4wis = v_series[0:n_rec]
        m.w_series_4wis = w_series[0:n_rec]
        m.vehicle_length = vehicle.fp.length
        m.vehicle_width = vehicle.fp.width
        m.vehicle_delta_max = vehicle.fp.delta_max
        m.vehicle_omega_spin_max = vehicle.fp.omega_spin_max
        m.internal_collision = collision_model
    elif metrics_family == 'diff_rect':
        if math.isinf(min_footprint_clearance):
            min_footprint_clearance = 1e3
        m.footprint_clearance = min_footprint_clearance
        m.min_turning_radius = vehicle.min_turn_radius
        m.spin_usage_ratio = vehicle.spin_steps / max(n_rec, 1)
        m.max_spin_duration = vehicle.spin_run_max * dt
        m.spin_cycles = vehicle.spin_cycles
        m.total_spin_time = vehicle.total_spin_time
        m.v_series_diff = v_series[0:n_rec]
        m.w_series_diff = w_series[0:n_rec]
        m.vehicle_length = vehicle.dp.length
        m.vehicle_width = vehicle.dp.width
        m.vehicle_omega_spin_max = vehicle.dp.omega_spin_max
        m.internal_collision = collision_model

    return result, m


# ---------------------------------------------------------------------------
# 7 个公开入口（原签名不变，行为由单一引擎复现）
# [C8 公平化 2026-10-16] run_ackermann_trial 与三个 run_*_trial_with_planner
# 新增可选参数 use_guide（默认 False，既有调用方行为不变），供全因子 driver
# 统一为全部算法开启 C8 全局子目标引导。
# ---------------------------------------------------------------------------
def run_trial(pp: Params, sc: Scene, mode: str, traj, measure_time: bool = True,
              on_step=None):
    """runTrialAblation.m 等价：矩形差速车 + classic C3 状态机 + TrialMetrics。"""
    return _run(pp, sc, traj, vehicle=DiffRectVehicle(pp, None, 'rect_diff'),
                trap_mode='classic', metrics_family='trial', mode=mode,
                collision_model='rect_diff',
                collect_cand='window', truncate='once',
                collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, on_step=on_step)


def run_trial_simple(pp: Params, sc: Scene, mode: str, traj):
    """runTrial.m 等价：全动态初始化、跑满轨迹、simple 状态机、只回 path。"""
    return _run(pp, sc, traj, vehicle=DiffRectVehicle(pp, None, 'rect_diff'),
                trap_mode='simple', metrics_family='simple', mode=mode,
                collision_model='rect_diff',
                init_mode='simple', sim_cap=False,
                collect_pred_err=False, collect_cand=None, collect_osc=False)


def run_trial_with_planner(pp: Params, sc: Scene, planner, traj,
                           measure_time: bool = True, on_step=None):
    """外部基线规划器闭环试验（矩形差速车，无 C3/截断，指标与 run_trial 一致）。"""
    return _run(pp, sc, traj, vehicle=DiffRectVehicle(pp, None, 'rect_diff'),
                metrics_family='trial', planner=planner, external=True,
                collision_model='rect_diff',
                collect_cand='planner', collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, reset_planner='safe', on_step=on_step)


def run_ackermann_trial(pp: Params, sc: Scene, mode: str, traj,
                        ack_params: AckermannParams | None = None,
                        measure_time: bool = True,
                        collision_model: str = "rectangle",
                        use_guide: bool = False,
                        on_step=None):
    """Ackermann 闭环试验（RDA-DWA 系列）：矩形碰撞 + 曲率指标。"""
    return _run(pp, sc, traj,
                vehicle=AckermannVehicle(pp, ack_params, collision_model),
                trap_mode='classic', metrics_family='ackermann', mode=mode,
                collision_model=collision_model, ack_params=ack_params,
                collect_cand='window', truncate='once',
                collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, use_guide=use_guide, on_step=on_step)


def run_ackermann_trial_with_planner(pp: Params, sc: Scene, planner, traj,
                                     ack_params: AckermannParams | None = None,
                                     measure_time: bool = True,
                                     collision_model: str = "rectangle",
                                     use_guide: bool = False,
                                     on_step=None):
    """Ackermann 闭环试验（外部基线规划器）。"""
    return _run(pp, sc, traj,
                vehicle=AckermannVehicle(pp, ack_params),
                metrics_family='ackermann', planner=planner, external=True,
                collision_model=collision_model, ack_params=ack_params,
                collect_cand='planner', collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, reset_planner='safe',
                use_guide=use_guide, on_step=on_step)


def run_four_wheel_trial(pp: Params, sc: Scene, mode: str, traj,
                         fws_params: FourWheelSteerParams | None = None,
                         measure_time: bool = True,
                         collision_model: str = "rect4wis",
                         executor: str = 'fws',
                         ack_params: AckSpinParams | None = None,
                         trace: bool = False, on_step=None):
    """4WIS 闭环试验（RDA-DWA 系列）：evolved 状态机 + 原地转向指标。

    executor：
      'fws'      — 既有全程 4WIS 执行（默认，行为不变）；
      'ack_spin' — 阿克曼行进 + 原地自转执行（真车 turnmode 0/4 双模；
                   规划/碰撞模型仍为 collision_model，默认 rect4wis 不动）。
    ack_params：executor='ack_spin' 时的执行器参数（默认 AckSpinParams 真车口径）。
    """
    if executor == 'ack_spin':
        vehicle = AckermannSpinVehicle(pp, ack_params, fws_params, collision_model)
    else:
        vehicle = FourWheelVehicle(pp, fws_params, collision_model)
    return _run(pp, sc, traj,
                vehicle=vehicle,
                trap_mode='evolved', metrics_family='four_wheel', mode=mode,
                collision_model=collision_model, fws_params=fws_params,
                collect_cand='window', truncate='once', spin_timeout=True,
                use_guide=True, trace=trace, record_wheel_series=True,
                collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, on_step=on_step)


def run_four_wheel_trial_with_planner(pp: Params, sc: Scene, planner, traj,
                                      fws_params: FourWheelSteerParams | None = None,
                                      measure_time: bool = True,
                                      collision_model: str = "rect4wis",
                                      executor: str = 'fws',
                                      ack_params: AckSpinParams | None = None,
                                      use_guide: bool = False,
                                      on_step=None):
    """4WIS 闭环试验（外部 baseline 规划器版）。executor 语义同 run_four_wheel_trial。"""
    if executor == 'ack_spin':
        vehicle = AckermannSpinVehicle(pp, ack_params, fws_params, collision_model)
    else:
        vehicle = FourWheelVehicle(pp, fws_params, collision_model)
    return _run(pp, sc, traj,
                vehicle=vehicle,
                metrics_family='four_wheel', planner=planner, external=True,
                collision_model=collision_model, fws_params=fws_params,
                truncate='always', collect_cand=None, collect_osc=False,
                collect_pred_err=False, record_wheel_series=False,
                measure_time=measure_time, reset_planner='bare',
                plan_safe=True, use_guide=use_guide, on_step=on_step)


def run_diff_rect_trial(pp: Params, sc: Scene, mode: str, traj,
                        diff_params: DiffRectParams | None = None,
                        measure_time: bool = True,
                        collision_model: str = "rect_diff",
                        trace: bool = False, on_step=None):
    """真实尺寸矩形差速驱动闭环试验（RDA-DWA 系列）。

    - 1.0×0.6m 矩形车身，几何中心参考
    - 差速运动学（v/ω 直接积分），支持原地自转
    - evolved 逃逸状态机（与 4WIS 同：支持 spin 兜底 / C12 / C5/C6）
    - 矩形 SAT 碰撞
    - v17：启用 RSS 限速 / S9横向意图 / S13间隙检测
    """
    # v17: 不额外启用特征，使用 v15_latesc 默认配置
    return _run(pp, sc, traj,
                vehicle=DiffRectVehicle(pp, diff_params, collision_model),
                trap_mode='evolved', metrics_family='diff_rect', mode=mode,
                collision_model=collision_model,
                collect_cand='window', truncate='once', spin_timeout=True,
                use_guide=True, trace=trace, record_wheel_series=False,
                collect_pred_err=True, collect_osc=True,
                measure_time=measure_time, on_step=on_step)


def run_diff_rect_trial_with_planner(pp: Params, sc: Scene, planner, traj,
                                     diff_params: DiffRectParams | None = None,
                                     measure_time: bool = True,
                                     collision_model: str = "rect_diff",
                                     use_guide: bool = False,
                                     on_step=None):
    """真实尺寸矩形差速闭环试验（外部 baseline 规划器版）。

    - 1.0×0.6m 矩形车身（rect_diff 矩形 SAT 精确碰撞，非点模型）
    - 差速运动学（v/ω 直接积分），支持原地自转
    - 与 run_four_wheel_trial_with_planner 同构：truncate='always'、
      外部 planner 闭环、reset_planner='bare'、plan_safe=True
    """
    return _run(pp, sc, traj,
                vehicle=DiffRectVehicle(pp, diff_params, collision_model),
                metrics_family='diff_rect', planner=planner, external=True,
                collision_model=collision_model,
                truncate='always', collect_cand=None, collect_osc=False,
                collect_pred_err=False, record_wheel_series=False,
                measure_time=measure_time, reset_planner='bare',
                plan_safe=True, use_guide=use_guide, on_step=on_step)


__all__ = [
    'TrialResult', 'TrialMetrics', 'AckermannTrialMetrics', 'FourWheelTrialMetrics',
    'DiffRectTrialMetrics', 'DiffRectParams', 'DiffRectState', 'DiffRectVehicle',
    'VehicleAdapter', 'AckermannVehicle', 'FourWheelVehicle',
    'TrapController', 'ClassicTrapController', 'SimpleTrapController',
    'EvolvedTrapController',
    'run_trial', 'run_trial_simple', 'run_trial_with_planner',
    'run_ackermann_trial', 'run_ackermann_trial_with_planner',
    'run_four_wheel_trial', 'run_four_wheel_trial_with_planner',
    'run_diff_rect_trial', 'run_diff_rect_trial_with_planner',
]
