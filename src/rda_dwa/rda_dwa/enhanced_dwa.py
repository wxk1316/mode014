"""
enhanced_dwa.py — EnhancedDWA.m 的 Python 移植（核心规划器）。

模块：M1 动态窗口 / M2 多步时距 / M3 高斯风险场 / M4 min-max 归一化 /
      M5 前置 α-β（在 predictor 中）/ M6 兜底受限减速 /
      C1 截断归一化 / C2 IMM-UKF（在 predictor 中）/ C3 前瞻+U形逃逸 / C4 APF 缩窄。

算法逻辑与参数严格对应 MATLAB 源码，禁止改动。
"""
from __future__ import annotations

import copy
import math

import numpy as np

from .params import Params
from .robot_model import RobotModel, RobotState, wrap_angle
from .obstacle import Obstacle
from .collision_checker import CollisionChecker
from .ackermann_model import AckermannParams
from .four_wheel_steer_model import FourWheelSteerParams
from .escape import EscapeStrategy


def matlab_colon(start: float, step: float, stop: float):
    """MATLAB a:step:b 语义（含端点，按算术序列累加）。"""
    out = []
    v = start
    if step > 0:
        while v <= stop:
            out.append(v)
            v = start + len(out) * step
    elif step < 0:
        while v >= stop:
            out.append(v)
            v = start + len(out) * step
    else:
        raise ValueError('step must be nonzero')
    return out


class VelocitySample:
    """struct {v, omega, score, feasible} 等价。"""
    __slots__ = ('v', 'omega', 'score', 'feasible', 'relaxed')

    def __init__(self, v=0.0, omega=0.0, score=0.0, feasible=True, relaxed=False):
        self.v = v
        self.omega = omega
        self.score = score
        self.feasible = feasible
        self.relaxed = relaxed


class CostTerms:
    """struct {head, dist, clear, future, risk, count, headon, unc, smooth} 偏置值（越大越好）。

    headon/unc 为 MTDWA 第四轮新增（g11 迎面风险 / g12 模型分歧惩罚，默认 0）；
    smooth 为代价平滑项（惩罚相对上周期指令的速度跳变，默认 0，仅内部 evaluate_window 使用）。
    """
    __slots__ = ('head', 'dist', 'clear', 'future', 'risk', 'count',
                 'headon', 'unc', 'smooth')

    def __init__(self, head=0.0, dist=0.0, clear=0.0, future=0.0, risk=0.0,
                 count=0.0, headon=0.0, unc=0.0, smooth=0.0):
        self.head = head
        self.dist = dist
        self.clear = clear
        self.future = future
        self.risk = risk
        self.count = count
        self.headon = headon
        self.unc = unc
        self.smooth = smooth

    def to_list(self):
        return [self.head, self.dist, self.clear, self.future, self.risk,
                self.count, self.headon, self.unc, self.smooth]


class EnhancedDWA:
    """EnhancedDWA.m 等价。mode: 'CONVENTIONAL_DWA' / 'ENHANCED_DWA'。"""

    def __init__(self, params: Params, mode: str, collision_model: str = "rect4wis",
                 ack_params: AckermannParams | None = None,
                 fws_params: FourWheelSteerParams | None = None):
        self.params = params
        self.mode = mode
        self.model = RobotModel(params.robot_radius)
        self.collision_model = collision_model
        # 4WIS / 矩形差速模式：v≈0 时允许原地转向（算法顶层仍输出 (v, ω)）。
        # Ackermann（"rectangle"）不支持原地自转：v<v_eps 时禁止转向。
        self.spin_enabled = collision_model in ("rect4wis", "rect_diff")
        self.cc = CollisionChecker(
            collision_model=collision_model,
            robot_radius=params.robot_radius,
            obstacle_radius=params.obstacle_radius,
            ack_params=ack_params,
            fws_params=fws_params,
        )
        # 轨迹预测模式：'ramp0'（v=0 爬升，默认）| 'const_v'（恒速外推）|
        # 'adaptive'（规则自适应：近密→长）| 'dqn'（DQN 学习自适应）
        self.cc.traj_mode = getattr(params, 'traj_predict_mode', 'ramp0')
        # DQN hook：traj_mode='dqn' 时，plan() 调用 dqn_policy(robot, obstacles, local_goal)
        # 返回 float（预测时域秒）。训练脚本注入。
        self.dqn_policy = None
        # DQN 权重乘子：(m_h, m_d, m_c) 分别乘 alpha_h/alpha_d/alpha_c
        # None 时用默认权重；训练脚本每步注入。
        self.dqn_weight_mult = None
        # DQN transition recorder：每步 (s_t, a_t) 暂存，on_step 回调里补 reward/s_{t+1}
        self.dqn_recorder = None
        # C2：逃逸策略接缝（判定/子目标/打分收敛到 escape.py，本类保留薄委托）
        self.escape = EscapeStrategy(self)
        # g13：RA-DWA 低通状态（Shead/Sclear/Sspeed 三通道上一时刻值）
        self._ra_w = None
        # [v16_kinematic] 多点转向 (3-point turn) 开放环状态机
        # 用于 Ackermann (rectangle) 在 DWA 全不可行时执行 forward→reverse→forward
        # 的 Reeds-Shepp 式摆尾；也用于 rect_diff 旋转被堵时先倒车腾空间。
        self._mpt_phase = None       # None / 'fwd_lock' / 'rev_lock' / 'fwd_lock2'
        self._mpt_timer = 0.0        # 当前阶段已执行秒数
        self._mpt_turn_sign = 1.0    # 左转 +1 / 右转 -1
        self._mpt_target_heading = None

        # ---- 代价平滑项 + 死区粘性（2026-11-03 第二轮） ----
        # _last_cmd_v/_last_cmd_w：上一周期最终下发指令（由 vehicle_bridge 在
        # adapt_velocity 之后经 set_last_cmd 注入，与 /cmd_vel 一致）；
        # None 表示尚无历史指令（平滑回退 robot.v/robot.omega，粘性不生效）。
        self._last_cmd_v = None
        self._last_cmd_w = None
        # 本周期是否已触发过一次粘性（plan() 开头复位，每周期最多粘一次）
        self._sticky_applied = False
        # 本帧平滑项是否生效（plan() 开头计算：trap 激活 / 自转停车期门控为 False）
        self._smooth_enabled = False
        # ★2026-11-06 局部绕行子目标侧向决策（+1 左绕 / -1 右绕 / 0 未决策）
        # 侧向一旦选定就保持（只在走廊不再被挡时清零），避免左右反复摇摆。
        self._detour_side = 0
        self._detour_goal_cur = None
        self._detour_obs_id = None  # 正在规避的障碍 id

    # ------------------------------------------------------------------
    # 代价平滑项 + 死区粘性：上周期指令注入
    # ------------------------------------------------------------------
    def set_last_cmd(self, v: float, omega: float):
        """注入上一周期最终下发指令（vehicle_bridge 在 adapt_velocity 之后调用，
        与发布到 /cmd_vel 的 (v, ω) 一致）。供代价平滑项参考基准与死区粘性匹配使用。"""
        self._last_cmd_v = float(v)
        self._last_cmd_w = float(omega)

    # ------------------------------------------------------------------
    # plan(): 公式(13) (v*,ω*) = argmax J，含兜底链
    # ------------------------------------------------------------------
    def plan(self, robot: RobotState, goal, obstacles, trap=None):
        """返回 (ok, v_out, omega_out)。trap: dict 或 None。"""
        v_out = 0.0
        omega_out = 0.0
        if trap is None:
            trap = {}
        trap_active = bool(trap) and trap.get('active', False)

        # [代价平滑 + 死区粘性] 本周期状态复位与门控（2026-11-03 第二轮）：
        #   _sticky_applied 每周期复位，每周期至多粘一次；
        #   平滑 gate：trap 激活 / 上周期近似静止（自转/停车/急停，|v|<spin_v_eps）
        #   → 有效权重 0（自转期 |v|≈0，平滑基准无意义）。上周期 |v| 以 set_last_cmd
        #   注入的指令为准，未设置时回退本帧 robot.v。
        self._sticky_applied = False
        _last_v_ref = self._last_cmd_v if self._last_cmd_v is not None else robot.v
        self._smooth_enabled = (not trap_active) and (
            abs(_last_v_ref) >= self.params.spin_v_eps)

        # 自适应预测时域：障碍越近/越密 → 时域越长（看得远好绕障）；
        # 开阔 → 时域短（贪婪前进）。仅在 traj_mode='adaptive' 时启用。
        _sim_time_bak = self.params.sim_time
        _n_samp_bak = self.params.n_trajectory_samples

        # [v16_kinematic] trap 释放时重置多点转向状态机
        if not trap_active and self._mpt_phase is not None:
            self._mpt_reset()

        local_goal = goal
        if trap_active and trap.get('sub_goal') is not None:
            local_goal = trap['sub_goal']

        # ★2026-11-06 局部绕行子目标：走廊被挡 → 把局部目标侧向挪到障碍侧面，
        # 让 head/dist 项把车"引"出去（trap 激活时让位给 trap 自己的子目标）。
        self._detour_goal_cur = None
        if (not trap_active) and self.enhanced_feature(
                getattr(self.params, 'enable_detour', False)):
            dg = self._detour_goal(robot, local_goal, obstacles)
            if dg is not None:
                self._detour_goal_cur = dg
                local_goal = dg

        if self.cc.traj_mode == 'adaptive':
            T_ad = self._adaptive_horizon(robot, obstacles)
            self.params.sim_time = T_ad
        elif self.cc.traj_mode == 'dqn' and self.dqn_policy is not None:
            result = self.dqn_policy(
                robot, obstacles, local_goal,
                trap_active and trap.get('sub_goal') is not None)
            # 返回 (tau, m_h, m_d, m_c) 或仅 tau（向后兼容）
            if isinstance(result, (tuple, list)):
                T_ad, m_h, m_d, m_c = result
                self.dqn_weight_mult = (m_h, m_d, m_c)
            else:
                T_ad = float(result)
            self.params.sim_time = T_ad
            self.params.n_trajectory_samples = max(3, int(round(T_ad / self.params.dt)))

        # 4WIS + C3：U 形陷阱需要大转角（> trap_block_angle）时，直接原地掉头
        # （原地转向不受前进曲率约束，比边转边走更快且不挤占通道）
        # ① 动态守卫：动态障碍近时禁用原地掉头（车不动 → 动态障碍逼近 → 碰撞）
        if (self.spin_enabled and trap_active
                and trap.get('escape_heading') is not None
                and not self._spin_blocked_by_dynamic(robot, obstacles)
                and not (self.params.enable_c12_headon_evade
                         and self.params.c12_evade_nospin
                         and trap.get('from_c12', False))):
            err = abs(wrap_angle(trap['escape_heading'] - robot.theta))
            if err > self.params.trap_block_angle:
                sign = 1.0 if wrap_angle(trap['escape_heading'] - robot.theta) > 0.0 else -1.0
                ok_spin, w_spin = self.try_spin(robot, obstacles,
                                                prefer_sign=sign)
                if ok_spin:
                    self.params.sim_time = _sim_time_bak
                    self.params.n_trajectory_samples = _n_samp_bak
                    return True, 0.0, w_spin

        window = self.build_dynamic_window(robot, obstacles, trap, local_goal)
        use_norm = self.enhanced_feature(self.params.enable_m4_normalize)

        found, window, v_out, omega_out = self.select_best(
            window, robot, local_goal, obstacles, use_norm, trap)
        if found:
            self.params.sim_time = _sim_time_bak
            self.params.n_trajectory_samples = _n_samp_bak
            return True, v_out, omega_out

        # C4 安全兜底：APF 缩窄后的候选不足时，重建未缩窄窗口
        if self.enhanced_feature(self.params.enable_c4_apf_omega):
            window = self.build_dynamic_window(robot, obstacles, trap, local_goal, skip_c4=True)
            found, window, v_out, omega_out = self.select_best(
                window, robot, local_goal, obstacles, use_norm, trap)
            if found:
                self.params.sim_time = _sim_time_bak
                self.params.n_trajectory_samples = _n_samp_bak
                return True, v_out, omega_out

        # 兜底 1：增强模式重新按”当前位置”碰撞评判
        if not found and self.mode == 'ENHANCED_DWA':
            if self._batch_feasible_supported():
                feas = self.is_feasible_batch(window, robot, obstacles, False)
                for i in range(len(window)):
                    window[i].feasible = bool(feas[i])
            else:
                for i in range(len(window)):
                    # 保留 4WIS 原地旋转样本排除（兜底重判不得重新启用）
                    if (self.spin_enabled
                            and abs(window[i].v) <= self.params.spin_v_eps
                            and abs(window[i].omega) > 0.05):
                        window[i].feasible = False
                    else:
                        window[i].feasible = self.is_feasible(window[i], robot, obstacles, False)
            found, _, v_out, omega_out = self.select_best(
                window, robot, local_goal, obstacles, use_norm, trap)
            if found:
                self.params.sim_time = _sim_time_bak
                self.params.n_trajectory_samples = _n_samp_bak
                return True, v_out, omega_out

        # C7 采样饿死软松弛（P4）：正常/C4/当前位置重判均零可行样本
        # （全判死→停车等死）时，以放宽后的 M2 危险半径与 M1 制动判据重建窗口，
        # 选"最不坏"样本并重惩罚（ISA-MPC slack ρ / 软约束思想）。
        if not found and self.enhanced_feature(self.params.enable_c7_soft_relax):
            window = self.build_dynamic_window(robot, obstacles, trap, local_goal,
                                               relax=True)
            found, window, v_out, omega_out = self.select_best(
                window, robot, local_goal, obstacles, use_norm, trap)
            if found:
                self.params.sim_time = _sim_time_bak
                self.params.n_trajectory_samples = _n_samp_bak
                return True, v_out, omega_out

        # 4WIS 兜底：所有前进样本不可行 → 原地转向脱困
        # （v=0 时 ω 依然有效；在可原地旋转的净空内朝目标方向最大化 |ω|）
        # ① 动态守卫：动态障碍近时跳过原地转向（车不动会被撞），直接走后退/停车
        if self.spin_enabled:
            if not self._spin_blocked_by_dynamic(robot, obstacles):
                ok_spin, w_spin = self.try_spin(robot, obstacles, goal=goal)
                if ok_spin:
                    self.params.sim_time = _sim_time_bak
                    self.params.n_trajectory_samples = _n_samp_bak
                    return True, 0.0, w_spin
            # 4WIS 兜底2：贴墙死锁（旋转净空不足，如车距墙 < r_circum）
            # → 缓慢后退脱困（真实车双向驱动；不进正常窗口，仅此兜底路径）
            # [v16_kinematic] rect_diff 旋转被堵时：先倒车腾空间，再重试旋转
            # （U 型陷阱中车距后墙太近导致原地旋转扫到墙，倒车 0.5~1m 后即可旋转）
            if self.params.backup_enabled:
                back_v = self.params.backup_v
                step_dt = self.params.sim_time / max(1, self.params.n_trajectory_samples)
                back_traj = self.cc.predict_trajectory(
                    -back_v, 0.0, robot.x, robot.y, robot.theta,
                    step_dt, self.params.n_trajectory_samples)
                if self.cc.is_collision_free(back_traj, obstacles):
                    self.params.sim_time = _sim_time_bak
                    self.params.n_trajectory_samples = _n_samp_bak
                    return True, -back_v, 0.0

        # [v16_kinematic] Ackermann 多点转向 (3-point turn) 逃逸原语
        # 当 DWA 全不可行且 trap 激活时，执行 forward-max-lock → reverse-opposite-lock
        # → forward 的 Reeds-Shepp 式摆尾。这是非完整约束车辆在 U 型陷阱/窄巷掉头的
        # 标准运动原语，不依赖具体场景几何。
        if (self.cc.model == "rectangle" and trap_active
                and self.params.enable_mpt_escape):
            mpt_result = self._multi_point_turn(robot, obstacles, trap)
            if mpt_result is not None:
                self.params.sim_time = _sim_time_bak
                self.params.n_trajectory_samples = _n_samp_bak
                return mpt_result

        # 兜底 2：M6 受限减速（M6 关闭时回退 (0,0) 停车）
        if self.enhanced_feature(self.params.enable_m6_fallback):
            v_out = max(0.0, robot.v - self.params.dec_lin_max * self.params.dt)
            omega_out = robot.omega
        else:
            v_out = 0.0
            omega_out = 0.0
        self.params.sim_time = _sim_time_bak
        self.params.n_trajectory_samples = _n_samp_bak
        return False, v_out, omega_out

    def _adaptive_horizon(self, robot: RobotState, obstacles) -> float:
        """自适应预测时域（秒）。
        最近障碍越近 / 局部障碍越多 → 时域越长（看得远好绕障）；
        开阔 → 时域短（贪婪前进）。范围 [t_min, t_max]。
        参考 ADWA (PMC10781240) 公式(9)，但方向反转为"近密→长"。
        """
        t_min = getattr(self.params, 'adaptive_t_min', 0.8)
        t_max = getattr(self.params, 'adaptive_t_max', 1.5)
        d_ref = getattr(self.params, 'adaptive_d_ref', 3.0)   # 距离≥此值→距离分=0
        n_ref = getattr(self.params, 'adaptive_n_ref', 6)    # 障碍数≥此值→密度分=1
        r_perc = getattr(self.params, 'adaptive_perception_r', 3.0)  # 局部半径

        d_min = math.inf
        n_local = 0
        for obs in obstacles:
            d = math.hypot(obs.fx - robot.x, obs.fy - robot.y)
            if d < d_min:
                d_min = d
            if d < r_perc:
                n_local += 1
        if d_min == math.inf:
            d_min = d_ref  # 无障碍 → 距离分=0

        # 距离分：d_min=0 → 1；d_min≥d_ref → 0
        d_score = max(0.0, min(1.0, 1.0 - d_min / d_ref))
        # 密度分：n_local=0 → 0；n_local≥n_ref → 1
        n_score = max(0.0, min(1.0, n_local / n_ref))
        # 等权混合
        s = 0.5 * d_score + 0.5 * n_score
        return t_min + (t_max - t_min) * s

    def _is_trajectory_safe(self, v, omega, robot, obstacles) -> bool:
        """[v17_safety] 精确轨迹-障碍预测碰撞检查。
        预测 (v,ω) 对应的完整轨迹，逐时刻检查是否与动态障碍预测位置碰撞。
        返回 True 表示安全。"""
        if not obstacles:
            return True
        step_dt = self.params.sim_time / max(1, self.params.n_trajectory_samples)
        # 构造临时 sample 用于 _eff_omega
        tmp_s = VelocitySample()
        tmp_s.v = v
        tmp_s.omega = omega
        eff_w = self._eff_omega(tmp_s)
        traj = self.cc.predict_trajectory(
            v, eff_w, robot.x, robot.y, robot.theta,
            step_dt, self.params.n_trajectory_samples)
        for i in range(traj.shape[0]):
            rx, ry = traj[i, 0], traj[i, 1]
            robot_pose = traj[i:i + 1]
            t_i = (i + 1) * step_dt
            for obs in obstacles:
                if getattr(obs, 'speed', 0.0) < 0.1:
                    # 静态障碍：用当前位置
                    ox, oy = obs.fx, obs.fy
                else:
                    # fx/fy are already one-step-ahead predictions at dt.
                    step = max(1, int(round(t_i / self.params.dt)))
                    ox, oy = self.predicted_obs_position(
                        obs, step, self.params.dt,
                        robot_position=(robot.x, robot.y))
                if self.collision_model in ("rectangle", "rect4wis", "rect_diff"):
                    oth = getattr(obs, 'heading', 0.0)
                    d = self.cc.min_clearance_to_obstacle_at(robot_pose, obs, oth=oth)
                    threshold = 0.0   # 统一净距口径：接触/重叠不可行
                else:
                    d = math.hypot(rx - ox, ry - oy)
                    threshold = self.params.robot_radius + getattr(
                        obs, 'radius', self.cc.obstacle_radius)
                if d < threshold:
                    return False
        return True

    # ------------------------------------------------------------------
    # ① 动态障碍原地转向守卫
    # ------------------------------------------------------------------
    def _spin_blocked_by_dynamic(self, robot: RobotState, obstacles) -> bool:
        """最近动态障碍（speed>0）进入 spin_dynamic_guard 距离时，
        原地转向兜底被禁用——车体不动期间动态障碍会持续逼近导致碰撞。
        返回 True 表示应禁用原地旋转（基线 spin_dynamic_guard=0 恒 False）。"""
        g = self.params.spin_dynamic_guard
        if g <= 0.0:
            return False
        for o in obstacles:
            if o.speed > 1e-6:
                if math.hypot(o.fx - robot.x, o.fy - robot.y) < g:
                    return True
        return False

    # ------------------------------------------------------------------
    # [v16_kinematic] Ackermann 多点转向 (3-point turn) 逃逸原语
    # ------------------------------------------------------------------
    def _multi_point_turn(self, robot: RobotState, obstacles, trap):
        """执行 Reeds-Shepp 式摆尾：forward-max-lock → reverse-opposite-lock → forward。

        这是非完整约束车辆（Ackermann）在 U 型陷阱/窄巷中掉头的标准运动原语。
        状态机由 self._mpt_phase / _mpt_timer / _mpt_turn_sign 驱动。
        返回 (True, v, omega) 或 None（退出机动，回到正常 DWA）。
        """
        pp = self.params
        kappa_max = self.cc.ack.kappa_max
        mpt_v = pp.mpt_velocity
        dt = pp.dt

        # 首次进入：决定转向方向（左/右哪边净空大）
        if self._mpt_phase is None:
            escape_h = trap.get('escape_heading')
            if escape_h is not None:
                err = wrap_angle(escape_h - robot.theta)
                self._mpt_turn_sign = 1.0 if err > 0 else -1.0
            else:
                # 无逃逸方向：扫描左右弧线净空
                left_clear = self._arc_clearance(robot, obstacles, +1.0, mpt_v, kappa_max)
                right_clear = self._arc_clearance(robot, obstacles, -1.0, mpt_v, kappa_max)
                self._mpt_turn_sign = 1.0 if left_clear >= right_clear else -1.0
            self._mpt_phase = 'fwd_lock'
            self._mpt_timer = 0.0
            self._mpt_target_heading = trap.get('escape_heading')

        # 状态机主循环：每帧推进当前阶段，碰撞/时间到则切换阶段并立即执行新阶段
        sign = self._mpt_turn_sign
        for _ in range(3):  # 最多切换 3 次（防死循环）
            if self._mpt_phase == 'fwd_lock':
                w_cmd = mpt_v * kappa_max * sign
                traj = self.cc.predict_trajectory(
                    mpt_v, w_cmd, robot.x, robot.y, robot.theta,
                    dt, pp.n_trajectory_samples)
                if not self.cc.is_collision_free(traj, obstacles):
                    self._mpt_phase = 'rev_lock'
                    self._mpt_timer = 0.0
                    continue  # 立即执行新阶段
                self._mpt_timer += dt
                if self._mpt_timer >= pp.mpt_fwd_time:
                    self._mpt_phase = 'rev_lock'
                    self._mpt_timer = 0.0
                    continue
                return True, mpt_v, w_cmd

            elif self._mpt_phase == 'rev_lock':
                w_cmd = -mpt_v * kappa_max * sign
                traj = self.cc.predict_trajectory(
                    -mpt_v, w_cmd, robot.x, robot.y, robot.theta,
                    dt, pp.n_trajectory_samples)
                if not self.cc.is_collision_free(traj, obstacles):
                    self._mpt_reset()
                    return None
                self._mpt_timer += dt
                if self._mpt_timer >= pp.mpt_rev_time:
                    self._mpt_phase = 'fwd_lock2'
                    self._mpt_timer = 0.0
                    continue
                return True, -mpt_v, w_cmd

            elif self._mpt_phase == 'fwd_lock2':
                w_cmd = mpt_v * kappa_max * sign
                traj = self.cc.predict_trajectory(
                    mpt_v, w_cmd, robot.x, robot.y, robot.theta,
                    dt, pp.n_trajectory_samples)
                if not self.cc.is_collision_free(traj, obstacles):
                    self._mpt_reset()
                    return None
                if self._mpt_target_heading is not None:
                    err = abs(wrap_angle(self._mpt_target_heading - robot.theta))
                    if err < pp.mpt_heading_tol:
                        self._mpt_reset()
                        return None
                self._mpt_timer += dt
                if self._mpt_timer >= pp.mpt_fwd_time * 2:
                    self._mpt_reset()
                    return None
                return True, mpt_v, w_cmd

        return None

    def _arc_clearance(self, robot, obstacles, turn_sign, v, kappa_max):
        """估计前进弧线方向的净空距离。"""
        w = v * kappa_max * turn_sign
        traj = self.cc.predict_trajectory(
            v, w, robot.x, robot.y, robot.theta,
            self.params.dt, self.params.n_trajectory_samples)
        if self.cc.is_collision_free(traj, obstacles):
            return 10.0
        # 找到碰撞前的步数
        for i in range(1, len(traj)):
            if not self.cc.is_collision_free(traj[:i], obstacles):
                return i * self.params.dt * v
        return 0.0

    def _mpt_reset(self):
        """重置多点转向状态机。"""
        self._mpt_phase = None
        self._mpt_timer = 0.0
        self._mpt_turn_sign = 1.0
        self._mpt_target_heading = None

    # ------------------------------------------------------------------
    # C5 上下文感知逃逸判定（P3）
    # ------------------------------------------------------------------
    def c5_wait_blocked(self, robot: RobotState, obstacles) -> bool:
        """C5 合法让行等待判定（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.c5_wait_blocked(robot, obstacles)

    # ------------------------------------------------------------------
    # C12 对向威胁主动横向逃逸（P1/P3，S9）
    # ------------------------------------------------------------------
    def headon_threat(self, robot: RobotState, obstacles, skip_angle_gate=False):
        """C12 对向威胁判定（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.headon_threat(robot, obstacles, skip_angle_gate)

    # ------------------------------------------------------------------
    # C6 逃逸方向动态重校验（P3）
    # ------------------------------------------------------------------
    def revalidate_escape(self, robot: RobotState, obstacles, goal, trap) -> dict:
        """C6 逃逸方向动态重校验（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.revalidate_escape(robot, obstacles, goal, trap)

    def _escape_heading_safe(self, robot: RobotState, obstacles, heading: float) -> bool:
        """C6 逃逸方向安全性滚动校验（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape._escape_heading_safe(robot, obstacles, heading)

    # ------------------------------------------------------------------
    # 4WIS 原地转向兜底
    # ------------------------------------------------------------------
    def try_spin(self, robot: RobotState, obstacles, goal=None,
                 prefer_sign: float = 0.0):
        """原地转向尝试：在可旋转净空内朝目标方向选最大可行 |ω|。

        返回 (ok, omega_out)。旋转轨迹逐位姿矩形 SAT 检查（位置不动，θ 线性变化）。
        优先旋转方向：goal 给定 → 朝把车头转向目标的方向；
        否则用 prefer_sign（>0 逆时针，<0 顺时针）；都为 0 时先逆时针。
        候选 |ω| 从 omega_spin_max 向下按 omega_resolution 递减。
        """
        p = self.params
        w_lim = p.omega_spin_max
        step_dt = p.sim_time / max(1, p.n_trajectory_samples)
        n_steps = p.n_trajectory_samples

        def spin_free(w: float) -> bool:
            traj = self.cc.predict_trajectory(
                0.0, w, robot.x, robot.y, robot.theta, step_dt, n_steps)
            return self.cc.is_collision_free(traj, obstacles)

        if goal is not None:
            goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
            err = wrap_angle(goal_ang - robot.theta)
            if err >= 0.0:
                signs = [1.0, -1.0]      # 车头需逆时针转
            else:
                signs = [-1.0, 1.0]      # 车头需顺时针转
        elif prefer_sign > 0.0:
            signs = [1.0, -1.0]
        elif prefer_sign < 0.0:
            signs = [-1.0, 1.0]
        else:
            signs = [1.0, -1.0]
        for sign in signs:
            w = sign * w_lim
            if spin_free(w):
                return True, w
        # 降级：减小的 |ω| 网格（贴近障碍时用慢速原地转向）
        ws = matlab_colon(0.0, p.omega_resolution, w_lim + 1e-9)
        for sign in signs:
            for w in ws:
                if w < 1e-9:
                    continue
                ww = sign * w
                if spin_free(ww):
                    return True, ww
        return False, 0.0

    # ------------------------------------------------------------------
    # 动态窗口（公式(4)(5)）
    # ------------------------------------------------------------------
    def build_dynamic_window(self, robot: RobotState, obstacles, trap=None,
                             goal=None, skip_c4: bool = False, relax: bool = False):
        """buildDynamicWindow.m 等价。relax=True 时（C7 软松弛）放宽可行性阈值。"""
        use_pred = (self.mode == 'ENHANCED_DWA')
        vmin = self.params.min_linear_velocity
        vmax = self.params.max_linear_velocity
        wmin = self.params.min_angular_velocity
        wmax = self.params.max_angular_velocity

        # Ackermann 车型原把窗口下限覆盖为 ack.v_min(负值)允许倒车；
        # ★第五轮 改动4：实车不支持负 v（第4轮已在末道闸钳 v≥0、backup_enabled=False），
        #   采样窗负值候选只浪费样本且"被选后钳 0"造成假停滞 → vmin 只取正值下限。
        if self.cc.model == "rectangle":
            vmin = max(0.0, self.cc.ack.v_min)

        trap_active = trap is not None and bool(trap) and trap.get('active', False)

        if self.enhanced_feature(self.params.enable_m1_dynamic_window):
            if trap_active and trap.get('escape_heading') is not None:
                # 逃逸模式：以逃逸方向的角速度为窗口中心
                w_des = wrap_angle(trap['escape_heading'] - robot.theta)
                w_center = min(max(w_des / self.params.dt, wmin), wmax)
                wlo = max(wmin, w_center - self.params.acc_ang_max * self.params.dt)
                whi = min(wmax, w_center + self.params.acc_ang_max * self.params.dt)
                wmin = wlo
                wmax = whi
            else:
                wmin = max(wmin, robot.omega - self.params.dec_ang_max * self.params.dt)
                wmax = min(wmax, robot.omega + self.params.acc_ang_max * self.params.dt)
            vmin = max(vmin, robot.v - self.params.dec_lin_max * self.params.dt)
            vmax = min(vmax, robot.v + self.params.acc_lin_max * self.params.dt)

            if vmin > vmax:
                vmin = min(max(robot.v, self.params.min_linear_velocity),
                           self.params.max_linear_velocity)
                vmax = vmin
            if wmin > wmax:
                wmin = min(max(robot.omega, self.params.min_angular_velocity),
                           self.params.max_angular_velocity)
                wmax = wmin

        # [MTDWA 第二轮] g5/g7：速度窗口安全约束（默认关，mtdwa 行为不变）
        # g5 LC-RSS 纵向安全距离限速（S9 终点区迎头时间换空间）；g7 横向运动带
        # 低速让行（S11 行人横穿 / S8 横切车通道保持）。逃逸激活期间不受限
        # （避免干扰 C12 横向逃逸动作）。vmax 压到 vmin 以下时保持 vmin
        # （渐进减速语义：本帧至少能减一档）。
        if not trap_active:
            v_lim = self.escape.rss_speed_limit(robot, obstacles)
            if v_lim is not None:
                vmax = min(vmax, v_lim)
            v_lim2 = self.escape.brake_wait_limit(robot, obstacles)
            if v_lim2 is not None:
                vmax = min(vmax, v_lim2)
            # [MTDWA 第三轮] g9 目标区对向 TTC 让行 / g10 高残差保守收缩（默认关）
            v_lim3 = self.escape.goal_ttc_limit(robot, obstacles, goal)
            if v_lim3 is not None:
                vmax = min(vmax, v_lim3)
            v_lim4 = self.escape.unc_vmax_limit(robot, obstacles)
            if v_lim4 is not None:
                vmax = min(vmax, v_lim4)
            if vmax < vmin:
                vmax = vmin

        # C4：APF 横向力缩窄 ω 窗口（仅当最近障碍 > apf_narrow_dist）
        if (self.enhanced_feature(self.params.enable_c4_apf_omega)
                and not skip_c4 and goal is not None):
            dmin = math.inf
            for k in range(len(obstacles)):
                dmin = min(dmin, math.hypot(obstacles[k].fx - robot.x,
                                            obstacles[k].fy - robot.y))
            if dmin > self.params.apf_narrow_dist:
                wmid = 0.5 * (wmin + wmax)
                wlo0 = wmin
                whi0 = wmax
                Fl = self.apf_lateral_force(robot, goal, obstacles)
                band = self.params.apf_omega_band
                if Fl > self.params.apf_lateral_threshold:
                    wmin = max(wmin, wmid + band)
                elif Fl < -self.params.apf_lateral_threshold:
                    wmax = min(wmax, wmid - band)
                else:
                    sb = self.params.apf_straight_band
                    wmin = max(wmin, wmid - sb)
                    wmax = min(wmax, wmid + sb)
                if wmin > wmax:
                    wmin = wlo0
                    wmax = whi0

        window = []
        vs = matlab_colon(vmin, self.params.v_resolution, vmax + 1e-9)
        ws = matlab_colon(wmin, self.params.omega_resolution, wmax + 1e-9)
        # [v16_kinematic] Ackermann 运动学楔形约束——已禁用：
        # 代价函数内部多处直接使用 s.omega（非 _eff_omega），过滤候选会破坏
        # M4 归一化。_eff_omega 在轨迹预测时已正确钳位，此处不再额外过滤。
        if self._batch_feasible_supported():
            # 全关路径：窗口级批量可行性（逐位一致，纯提速）
            for v in vs:
                for w in ws:
                    s = VelocitySample()
                    s.v = min(v, vmax)
                    s.omega = min(w, wmax)
                    s.score = 0.0
                    window.append(s)
            feas = self.is_feasible_batch(window, robot, obstacles, use_pred)
            for i in range(len(window)):
                window[i].feasible = bool(feas[i])
        else:
            for v in vs:
                for w in ws:
                    s = VelocitySample()
                    s.v = min(v, vmax)
                    s.omega = min(w, wmax)
                    s.score = 0.0
                    # 4WIS：v≈0 的大幅旋转样本仅在 fallback try_spin 中出现，
                    # 正常/逃逸窗口排除，避免"原地空转"以高净空被归一化选中、
                    # 劫持正常前进决策（原地转向是兜底能力，不是常规机动）
                    if (self.spin_enabled
                            and abs(s.v) <= self.params.spin_v_eps
                            and abs(s.omega) > 0.05):
                        s.feasible = False
                    else:
                        s.feasible = self.is_feasible(s, robot, obstacles, use_pred, relax)
                    window.append(s)
        return window

    # ------------------------------------------------------------------
    # 目标函数（公式(6)~(12)）
    # ------------------------------------------------------------------
    def evaluate(self, s: VelocitySample, robot: RobotState, goal, obstacles) -> float:
        if self.mode == 'CONVENTIONAL_DWA':
            c = self.heading_cost(s, robot, goal) + self.velocity_cost(s) \
                + self.clearance_cost(s, robot, obstacles)
        else:
            c = self.heading_cost(s, robot, goal) + self.distance_cost(s, robot, goal) \
                + self.clearance_cost(s, robot, obstacles) + self.future_cost(s, robot, obstacles) \
                + self.risk_cost(s, robot, obstacles) + self.count_cost(s, robot, obstacles)
        return c

    # M4：min-max 归一化扫描
    def evaluate_window(self, window, robot: RobotState, goal, obstacles):
        """返回 (ok, window)。"""
        m3 = self.enhanced_feature(self.params.enable_m3_gaussian_risk)
        k_terms = 9
        eps = self.params.normalize_epsilon

        min_p = np.full(k_terms, math.inf)
        max_p = np.full(k_terms, -math.inf)
        feasible_idx = []
        terms = []

        use_la = self.enhanced_feature(self.params.enable_c3_lookahead)
        for i in range(len(window)):
            if not window[i].feasible:
                continue
            if use_la:
                t = self.rollout_terms(window[i], robot, goal, obstacles,
                                       self.params.lookahead_steps,
                                       self.params.lookahead_gamma)
            else:
                t = self.compute_cost_terms(window[i], robot, goal, obstacles)
            vals = t.to_list()
            for j in range(k_terms):
                min_p[j] = min(min_p[j], vals[j])
                max_p[j] = max(max_p[j], vals[j])
            feasible_idx.append(i)
            terms.append(t)

        if not feasible_idx:
            return False, window

        if m3:
            w_clear = self.params.alpha_c + self.params.alpha_f
            w_future = 0.0
        else:
            w_clear = self.params.alpha_c
            w_future = self.params.alpha_f

        # g13：RA-DWA 风险自适应权重——风险指标→通道→乘性缩放（band 限幅 + 低通）
        if self.enhanced_feature(self.params.enable_ra_weight):
            wh_, wc_, ws_ = self._ra_channels(robot, obstacles)
            band = self.params.ra_band
            w_h = self.params.alpha_h * (1.0 - band * (1.0 - wh_))
            w_d = self.params.alpha_d * (1.0 - band * (1.0 - wh_))
            w_clear = w_clear * (1.0 - band * (1.0 - wc_))
            w_future = w_future * (1.0 - band * (1.0 - wc_))
            w_r = self.params.alpha_r * (1.0 - band * (1.0 - ws_))
        else:
            w_h = self.params.alpha_h
            w_d = self.params.alpha_d
            w_r = self.params.alpha_r

        # DQN 权重乘子覆盖（训练时注入，在 RA 之后）
        if self.dqn_weight_mult is not None:
            m_h, m_d, m_c = self.dqn_weight_mult
            w_h = w_h * m_h
            w_d = w_d * m_d
            w_clear = w_clear * m_c

        # [代价平滑] 第 9 项权重：平滑 gate 关闭（trap 激活 / 自转停车期）时有效权重 0，
        # 等效于不启用平滑项；归一化对该项照常 min-max（j==8 无截断特例，与其它项一致）。
        w_smooth = self.params.alpha_smooth if self._smooth_enabled else 0.0
        weights = [w_h, w_d, w_clear,
                   w_future, w_r, self.params.alpha_count,
                   self.params.hc_weight, self.params.unc_weight4,
                   w_smooth]

        for k in range(len(feasible_idx)):
            vals = terms[k].to_list()
            score = 0.0
            for j in range(k_terms):
                denom = (max_p[j] - min_p[j]) + eps
                pbar = 0.0
                if (j == 1 and self.enhanced_feature(self.params.enable_c1_trunc_norm)
                        and self.params.normalize_trunc_D > 0.0):
                    d_goal = -vals[j]   # t.dist = -||goal - P_robot(t+1)||
                    if d_goal < self.params.normalize_trunc_D:
                        pbar = max(0.0, min(1.0,
                                    (self.params.normalize_trunc_D - d_goal)
                                    / self.params.normalize_trunc_D))
                    else:
                        if max_p[j] > min_p[j]:
                            pbar = (vals[j] - min_p[j]) / denom
                        else:
                            pbar = 0.0
                else:
                    if max_p[j] > min_p[j]:
                        pbar = (vals[j] - min_p[j]) / denom
                    else:
                        pbar = 0.0
                score = score + weights[j] * pbar
            window[feasible_idx[k]].score = score
        return True, window

    # ---------------- g13：RA-DWA 风险通道 ----------------
    def _ra_channels(self, robot: RobotState, obstacles):
        """风险指标→三通道（Shead/Sclear/Sspeed，已归一化和为 1）+ 一阶低通。

        卡 56 RA-DWA 式(13)-(24) 的 4WIS/障碍感知版：
          RF = 1 - dF/Dmax（前向最近距离）
          A  = (dL-dR)/(dL+dR+ε)（左右不对称）
          RTTC = max(0, 1 - t_cpa/3)（前方逼近障碍的最近点时间风险）
        通道：Shead=1-RF、Sclear=RF+|A|、Sspeed=1-RTTC。
        低通 W_k+1 = e^{-λdt} W_k + (1-e^{-λdt}) W_new。
        """
        p = self.params
        hx, hy = math.cos(robot.theta), math.sin(robot.theta)
        dF = dL = dR = 1e6
        rttc = 0.0
        for o in obstacles:
            dx, dy = o.fx - robot.x, o.fy - robot.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            ang = math.atan2(dy, dx) - robot.theta
            ang = math.atan2(math.sin(ang), math.cos(ang))
            front_gate = -math.pi / 6.0 <= ang <= math.pi / 6.0
            left_gate = math.pi / 6.0 < ang <= 2.0 * math.pi / 3.0
            right_gate = -2.0 * math.pi / 3.0 <= ang < -math.pi / 6.0
            if front_gate and d < dF:
                dF = d
                # RTTC：相对速度最近点时间（逼近才计）
                if o.speed > 1e-6:
                    vx = o.speed * math.cos(o.heading)
                    vy = o.speed * math.sin(o.heading)
                    rvx = robot.v * hx
                    rvy = robot.v * hy
                    vrx, vry = vx - rvx, vy - rvy
                    v2 = vrx * vrx + vry * vry
                    if v2 > 1e-9:
                        tc = -((vrx * dx + vry * dy) / v2)
                        if 0.0 < tc < 3.0:
                            rttc = max(rttc, 1.0 - tc / 3.0)
            if left_gate and d < dL:
                dL = d
            if right_gate and d < dR:
                dR = d
        dF = min(dF, p.ra_d_max)
        dL = min(dL, p.ra_d_max)
        dR = min(dR, p.ra_d_max)
        rf = max(0.0, 1.0 - dF / p.ra_d_max)
        asym = (dL - dR) / (dL + dR + 1e-6)
        s_head = 1.0 - rf
        s_clear = rf + abs(asym)
        s_speed = 1.0 - rttc
        tot = s_head + s_clear + s_speed + 1e-6
        w_new = (s_head / tot, s_clear / tot, s_speed / tot)
        if self._ra_w is None:
            self._ra_w = w_new
        else:
            a = 1.0 - math.exp(-p.ra_lp_lambda * p.dt)
            self._ra_w = tuple(a * w_new[i] + (1.0 - a) * self._ra_w[i]
                               for i in range(3))
        return self._ra_w

    # ---------------- 代价项 ----------------
    def _vehicle_clearance_radius(self) -> float:
        """代价口径中的车辆等效半径：矩形模式用外接圆 r_c（车-障碍距离）。
        实现委托 CollisionChecker 公开方法。"""
        return self.cc.vehicle_clearance_radius()

    def heading_cost(self, s: VelocitySample, robot: RobotState, goal) -> float:
        goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        heading_err = wrap_angle(goal_ang - self.motion_heading(s, robot))
        return -self.params.alpha_h * abs(heading_err)

    def distance_cost(self, s: VelocitySample, robot: RobotState, goal) -> float:
        pro = self.model.predict_position(robot, s.v, self._eff_omega(s), self.params.dt)
        d = math.hypot(goal[0] - pro[0], goal[1] - pro[1])
        return -self.params.alpha_d * d

    def clearance_cost(self, s: VelocitySample, robot: RobotState, obstacles) -> float:
        pro = self.model.predict_position(robot, s.v, self._eff_omega(s), self.params.dt)
        th = self.motion_heading(s, robot)
        clearance = math.inf
        for k in range(len(obstacles)):
            o = obstacles[k]
            # 矩形现实障碍 → 矩形-矩形精确净距；旧圆形 → 圆心距-(rc+r_obs)
            d = self.cc.net_gap_at(pro[0], pro[1], th,
                                   o.fx, o.fy, o.heading,
                                   float(getattr(o, 'length', 0.0) or 0.0),
                                   float(getattr(o, 'width', 0.0) or 0.0),
                                   float(getattr(o, 'radius', self.cc.obstacle_radius)))
            clearance = min(clearance, d)
        if not obstacles:
            clearance = 1e3
        return math.exp(self.params.alpha_c * (self.params.d_safe - clearance))

    def future_cost(self, s: VelocitySample, robot: RobotState, obstacles) -> float:
        pro = self.model.predict_position(robot, s.v, self._eff_omega(s), self.params.dt)
        th = self.motion_heading(s, robot)
        dmin = math.inf
        for k in range(len(obstacles)):
            o = obstacles[k]
            d = self.cc.net_gap_at(pro[0], pro[1], th,
                                   o.fx, o.fy, o.heading,
                                   float(getattr(o, 'length', 0.0) or 0.0),
                                   float(getattr(o, 'width', 0.0) or 0.0),
                                   float(getattr(o, 'radius', self.cc.obstacle_radius)))
            dmin = min(dmin, d)
        if not obstacles:
            dmin = 1e3
        return math.exp(self.params.alpha_f * (self.params.d_safe - dmin))

    def risk_cost(self, s: VelocitySample, robot: RobotState, obstacles) -> float:
        sumv = 0.0
        theta_r = self.motion_heading(s, robot)
        rc = self._vehicle_clearance_radius()
        for k in range(len(obstacles)):
            obs = obstacles[k]
            thetaH = wrap_angle(math.atan2(obs.fy - robot.y, obs.fx - robot.x) - theta_r)
            # 风险场：继续用各障碍等效半径（外接圆）口径
            r_obs_i = float(getattr(obs, 'radius', self.cc.obstacle_radius))
            d = math.hypot(obs.fx - robot.x, obs.fy - robot.y) - (rc + r_obs_i)
            sumv = sumv + self.risk_piecewise(thetaH, max(d, 0.0))
        return sumv

    def count_cost(self, s: VelocitySample, robot: RobotState, obstacles) -> float:
        theta_r = self.motion_heading(s, robot)
        countL = 0
        countR = 0
        for k in range(len(obstacles)):
            obs = obstacles[k]
            thetaH = wrap_angle(math.atan2(obs.fy - robot.y, obs.fx - robot.x) - theta_r)
            if -math.pi / 9.0 < thetaH <= 0.0:
                countL += 1
            if 0.0 <= thetaH < math.pi / 9.0:
                countR += 1
        if countR > countL:
            return -self.params.alpha_count * float(countR - countL)
        elif countL > countR:
            return -self.params.alpha_count * float(countL - countR)
        else:
            return 0.0

    def velocity_cost(self, s: VelocitySample) -> float:
        return self.params.alpha_v * s.v

    # ------------------------------------------------------------------
    # 碰撞筛选（公式(5)）
    # ------------------------------------------------------------------
    def _eff_omega(self, s: VelocitySample) -> float:
        """rectangle 模式的"可执行角速度"：把 ω 钳制到 Ackermann 曲率约束
        |ω| ≤ v·κ_max 内（v≤v_eps 时禁止转向）。规划用钳制后的轨迹做碰撞/
        代价评估，保证"计划轨迹 = 执行轨迹"（执行适配层同样限幅）。
        rect4wis / rect_diff：v/ω 解耦，ω 不受前进曲率约束（v≈0 时原地转向仍有效），
        原样返回 s.omega。
        """
        if self.collision_model == "rectangle":
            ap = self.cc.ack
            if s.v > ap.v_eps:
                wmax = s.v * ap.kappa_max
                return max(-wmax, min(s.omega, wmax))
            return 0.0
        return s.omega

    def is_feasible(self, s: VelocitySample, robot: RobotState, obstacles,
                    use_predicted_obstacle: bool, relax: bool = False) -> bool:
        use_rect = self.collision_model in ("rectangle", "rect4wis", "rect_diff")
        # 统一净距口径：base_gap=0（接触即不可行）；矩形现实障碍按矩形-矩形精确净距，
        # 旧圆形障碍按 圆心距-r_obs（兼容）。C9 机动膨胀在净距上叠加 c9_infl。
        base_gap = 0.0
        m1 = self.enhanced_feature(self.params.enable_m1_dynamic_window)
        m2 = self.enhanced_feature(self.params.enable_m2_multistep)
        dmin_all = math.inf

        # C7：松弛模式下调低碰撞/制动阈值（样本被标记 relaxed，打分时重惩罚）
        relax_ratio = self.params.c7_relax_ratio if (
            relax and self.enhanced_feature(self.params.enable_c7_soft_relax)) else 1.0
        vel_relax = self.params.c7_vel_relax if (
            relax and self.enhanced_feature(self.params.enable_c7_soft_relax)) else 1.0
        c9_on = self.enhanced_feature(self.params.enable_c9_uncertainty_inflate)
        c9_infl = self.params.c9_inflate_radius
        aabb_on = self.enhanced_feature(self.params.enable_c11_aabb_filter)

        # 矩形模式：用"可执行角速度"预计算轨迹
        #   rectangle → Ackermann 轨迹（曲率约束内）；rect4wis → 4WIS 轨迹（v≈0 原地旋转）
        rect_traj = None
        if use_rect:
            step_dt = self.params.sim_time / max(1, self.params.n_trajectory_samples)
            rect_traj = self.cc.predict_trajectory(
                s.v, self._eff_omega(s), robot.x, robot.y, robot.theta,
                step_dt, self.params.n_trajectory_samples)
            # C11：轨迹包围盒预筛（纯提速，跳过不可能相交的远障碍）
            if aabb_on:
                tx_min = float(np.min(rect_traj[:, 0]))
                tx_max = float(np.max(rect_traj[:, 0]))
                ty_min = float(np.min(rect_traj[:, 1]))
                ty_max = float(np.max(rect_traj[:, 1]))
                r_c = self.cc.vehicle_clearance_radius()
                margin = r_c + self.cc.obstacle_radius + 0.05
                # 障碍在预测视界内的最大可达距离也纳入包围盒
                trav = 0.0
                if use_predicted_obstacle and m2:
                    trav = self.params.obstacle_max_speed * self.params.n_future_steps * self.params.dt
                margin += trav
        else:
            tx_min = tx_max = ty_min = ty_max = 0.0
            margin = 0.0

        relaxed_used = False
        for k in range(len(obstacles)):
            obs = obstacles[k]
            # 软松弛（c7）许可的重叠容差（米制）：等价旧"d >= r*relax_ratio"语义
            relax_tol_i = ((1.0 - relax_ratio)
                           * float(getattr(obs, 'radius', self.cc.obstacle_radius))
                           if relax_ratio < 1.0 else 0.0)
            # C11 预筛：障碍当前位置圆与（轨迹包围盒 + 障碍半径 + 预测可达）不相交 → 跳过
            # 语义安全修正：预测可达按各障碍实际速度计算（S12 对向障碍 1.1~1.3m/s >
            # 全局 obstacle_max_speed=1.05，全局上界低估会漏判 → 可能放过本应碰撞的样本）。
            # 恒速预测下 per-obstacle 上界精确覆盖预测轨迹，与全量检查语义一致。
            if use_rect and aabb_on:
                trav_o = 0.0
                if use_predicted_obstacle and m2:
                    trav_o = obs.speed * self.params.n_future_steps * self.params.dt
                r_obs_i = float(getattr(obs, 'radius', self.cc.obstacle_radius))
                m_o = margin + r_obs_i + trav_o
                if (obs.fx + r_obs_i + m_o < tx_min
                        or obs.fx - r_obs_i - m_o > tx_max
                        or obs.fy + r_obs_i + m_o < ty_min
                        or obs.fy - r_obs_i - m_o > ty_max):
                    continue
            # C9：处于"机动/不可预测"状态的障碍临时膨胀有效净距
            obs_clear = base_gap
            if c9_on and obs.maneuver_hold > 0:
                obs_clear = base_gap + c9_infl

            if use_predicted_obstacle and m2:
                n_steps = max(1, self.params.n_future_steps)
                vox = obs.speed * math.cos(obs.heading)
                voy = obs.speed * math.sin(obs.heading)
                for i in range(1, n_steps + 1):
                    t_i = float(i) * self.params.dt
                    step_dt = self.params.sim_time / max(
                        1, self.params.n_trajectory_samples)
                    if use_rect:
                        if obs.speed >= 0.1:
                            idx = min(max(int(round(t_i / step_dt)) - 1, 0),
                                      rect_traj.shape[0] - 1)
                        else:
                            # Keep the legacy static-obstacle headway sample unchanged.
                            idx = min(int(round(t_i / step_dt)),
                                      rect_traj.shape[0] - 1)
                        rob_i = [rect_traj[idx, 0], rect_traj[idx, 1]]
                        rob_th = rect_traj[idx, 2]
                    else:
                        if abs(s.omega) < 1e-6:
                            rob_i = [robot.x + s.v * math.cos(robot.theta) * t_i,
                                     robot.y + s.v * math.sin(robot.theta) * t_i]
                        else:
                            r = s.v / s.omega
                            xc = robot.x - r * math.sin(robot.theta)
                            yc = robot.y + r * math.cos(robot.theta)
                            th = robot.theta + s.omega * t_i
                            rob_i = [xc + r * math.sin(th), yc - r * math.cos(th)]
                        rob_th = robot.theta + s.omega * t_i
                    p = self.predicted_obs_position(
                        obs, max(1, int(round(t_i / self.params.dt))), self.params.dt,
                        robot_position=(robot.x, robot.y))
                    vrx = s.v * math.cos(rob_th)
                    vry = s.v * math.sin(rob_th)
                    dx = rob_i[0] - p[0]
                    dy = rob_i[1] - p[1]
                    dline = math.hypot(dx, dy)
                    v_rel = 0.0
                    if dline > 1e-9:
                        v_rel = max(((vox - vrx) * dx + (voy - vry) * dy) / dline, 0.0)
                    if use_rect:
                        check_traj = (rect_traj[idx:idx + 1]
                                      if obs.speed >= 0.1 else rect_traj)
                        # 弧线外推时预测朝向随角速度旋转
                        oth = obs.heading
                        if float(getattr(obs, 'arc_mode_hold', 0)) > 0:
                            oth = obs.heading + float(getattr(obs, 'heading_rate', 0.0)) * t_i
                        d = self.cc.min_clearance_to_obstacle_at(
                            check_traj, obs, oth=oth)
                    else:
                        if obs.speed >= 0.1:
                            d = math.hypot(rob_i[0] - p[0], rob_i[1] - p[1])
                        else:
                            d = self.min_trajectory_distance(s, robot, p)
                    dmin_all = min(dmin_all, d)
                    rho = obs_clear + v_rel * self.params.headway_time
                    if d < rho:
                        if relax_ratio < 1.0 and d >= -relax_tol_i:
                            relaxed_used = True
                        else:
                            return False
            else:
                if use_predicted_obstacle:
                    p = [obs.fx, obs.fy]
                else:
                    p = [obs.x, obs.y]
                if use_rect:
                    # [P0-2] early termination：逐点检查轨迹位姿，发现碰撞即 break
                    if self.enhanced_feature(self.params.enable_p0_2_early_term):
                        d = math.inf
                        for _i in range(rect_traj.shape[0]):
                            _d_i = self.cc.min_clearance_to_obstacle_at(
                                rect_traj[_i:_i + 1], obs)
                            if _d_i < d:
                                d = _d_i
                            if _d_i < obs_clear:
                                break  # early exit
                    else:
                        d = self.cc.min_clearance_to_obstacle_at(rect_traj, obs)
                else:
                    d = self.min_trajectory_distance(s, robot, p)
                dmin_all = min(dmin_all, d)
                if d < obs_clear:
                    if relax_ratio < 1.0 and d >= -relax_tol_i:
                        relaxed_used = True
                    else:
                        return False

        # [P0-2] capsule swept volume 检查（修复版 v2）：
        # 半径用车宽半宽 W/2（非外接圆 0.583m），仅对动态障碍（speed>1e-6）做。
        # 静态墙继续走现有逐位姿 SAT（不动）。相邻轨迹点 p_i→p_{i+1} 线段，
        # 障碍圆心到线段最短距离 < W/2 + r_obs + margin 即碰撞。
        if (use_rect and self.enhanced_feature(self.params.enable_p0_2_capsule)
                and rect_traj is not None and len(obstacles) > 0):
            _L, _W, _rc = self.cc._rect_dims()
            r_safe_base = _W / 2.0 + self.params.p0_2_safety_margin
            for k in range(len(obstacles)):
                obs = obstacles[k]
                # 只对动态障碍做 capsule 检查
                if obs.speed <= 1e-6:
                    continue
                # 矩形现实类型障碍：主路径已做逐位姿矩形-矩形精确判定（含扫掠间隙），
                # capsule 圆扫掠仅保留给旧圆形障碍（避免外接圆过保守）
                if (float(getattr(obs, 'length', 0.0) or 0.0) > 1e-9
                        and float(getattr(obs, 'width', 0.0) or 0.0) > 1e-9
                        and getattr(obs, 'obstacle_type', None) in
                        ('pedestrian', 'bicycle', 'vehicle', 'static_large', 'block')):
                    continue
                r_obs = getattr(obs, 'radius', self.cc.obstacle_radius)
                thresh = r_safe_base + r_obs
                ox, oy = obs.fx, obs.fy
                # 遍历相邻轨迹段
                for _si in range(rect_traj.shape[0] - 1):
                    ax = rect_traj[_si, 0]
                    ay = rect_traj[_si, 1]
                    bx = rect_traj[_si + 1, 0]
                    by = rect_traj[_si + 1, 1]
                    seg_dx = bx - ax
                    seg_dy = by - ay
                    seg_l2 = seg_dx * seg_dx + seg_dy * seg_dy
                    if seg_l2 < 1e-12:
                        d_seg = math.hypot(ox - ax, oy - ay)
                    else:
                        _t = ((ox - ax) * seg_dx + (oy - ay) * seg_dy) / seg_l2
                        _t = max(0.0, min(1.0, _t))
                        d_seg = math.hypot(ox - (ax + _t * seg_dx),
                                           oy - (ay + _t * seg_dy))
                    if d_seg < thresh:
                        return False

        if m1 and math.isfinite(dmin_all):
            v_stop = math.sqrt(2.0 * dmin_all * self.params.dec_lin_max)
            if s.v > v_stop * vel_relax:
                return False
            if s.v > v_stop and vel_relax > 1.0:
                relaxed_used = True
            # 4WIS 原地旋转（v≈0）：旋转不前进，停止距离判据不适用；
            # 旋转扫掠碰撞已由 rect_traj 精确检查，跳过 ω 判据
            is_spin = self.spin_enabled and abs(s.v) < self.params.spin_v_eps
            if not is_spin:
                w_stop = math.sqrt(2.0 * dmin_all * self.params.dec_ang_max)
                if abs(self._eff_omega(s)) > w_stop * vel_relax:
                    return False
                if abs(self._eff_omega(s)) > w_stop and vel_relax > 1.0:
                    relaxed_used = True
        if relaxed_used:
            s.relaxed = True
        return True

    def _batch_feasible_supported(self) -> bool:
        """批量可行性仅适用于"全关路径"（M1/M2/C9/C11 全关、无预测障碍、非逃逸）：
        该路径下判定 = min(轨迹-障碍 SAT 净空) >= obstacle_radius，可窗口级批量且逐位一致。
        其他配置（v15 等含 M1/M2/C9/C11）继续逐采样 is_feasible，行为不变。"""
        return not (
            self.enhanced_feature(self.params.enable_m1_dynamic_window)
            or self.enhanced_feature(self.params.enable_m2_multistep)
            or self.enhanced_feature(self.params.enable_c9_uncertainty_inflate)
            or self.enhanced_feature(self.params.enable_c11_aabb_filter)
        )

    def is_feasible_batch(self, window, robot: RobotState, obstacles,
                          use_predicted_obstacle: bool = False) -> np.ndarray:
        """窗口级批量可行性（全关路径）。与逐采样 is_feasible 逐位一致：
        轨迹 = predict_trajectory（标量，与逐采样相同），净空 = min_clearance_to_points
        （逐元素同运算、min 归约精确、np.hypot 1D/2D 同路径已验证），
        spin 排除逻辑与 build_dynamic_window 相同，障碍位置与 is_feasible 一致
        （use_predicted_obstacle 且 m2 关 → 用 fx/fy）。"""
        n_steps = max(1, self.params.n_trajectory_samples)
        step_dt = self.params.sim_time / n_steps
        N = len(window)
        if self.collision_model == "rect4wis":
            vs_arr = np.array([s.v for s in window], dtype=np.float64)
            om_arr = np.array([self._eff_omega(s) for s in window], dtype=np.float64)
            feas = np.ones(N, dtype=bool)
            if self.spin_enabled:
                mask_spin = (np.abs(vs_arr) <= self.params.spin_v_eps) & (np.abs(om_arr) > 0.05)
                feas[mask_spin] = False
            if feas.any() and obstacles:
                idx = np.nonzero(feas)[0]
                ds = np.empty((idx.size, len(obstacles)), dtype=np.float64)
                for k, i in enumerate(idx):
                    traj = self.cc.predict_trajectory(
                        vs_arr[i], om_arr[i], robot.x, robot.y, robot.theta,
                        step_dt, n_steps)
                    for j, o in enumerate(obstacles):
                        if use_predicted_obstacle:
                            # 预测障碍位姿：位置 fx/fy（无 M2 外推时仅 1 步）
                            ox0, oy0 = o.x, o.y
                            o.x, o.y = o.fx, o.fy
                            try:
                                ds[k, j] = self.cc.min_clearance_to_obstacle(traj, o)
                            finally:
                                o.x, o.y = ox0, oy0
                        else:
                            ds[k, j] = self.cc.min_clearance_to_obstacle(traj, o)
                dmin = np.min(ds, axis=1)
                sub = feas.copy()
                # 统一净距口径：分离（净距>0）即可行；接触/重叠不可行
                sub[idx] = (dmin > 0.0)
                feas = sub
            return feas
        # 非 rect4wis：回退逐采样（保持原逻辑）
        feas = np.empty(N, dtype=bool)
        for i in range(N):
            feas[i] = self.is_feasible(window[i], robot, obstacles, use_predicted_obstacle)
        return feas

    def min_trajectory_distance(self, s: VelocitySample, robot: RobotState, point) -> float:
        n = max(1, self.params.n_trajectory_samples)
        ts = np.array([self.params.sim_time * (i + 1) / n for i in range(n)])
        if abs(s.omega) < 1e-6:
            px = robot.x + s.v * math.cos(robot.theta) * ts
            py = robot.y + s.v * math.sin(robot.theta) * ts
        else:
            r = s.v / s.omega
            xc = robot.x - r * math.sin(robot.theta)
            yc = robot.y + r * math.cos(robot.theta)
            th = robot.theta + s.omega * ts
            px = xc + r * np.sin(th)
            py = yc - r * np.cos(th)
        return float(np.min(np.sqrt((px - point[0]) ** 2 + (py - point[1]) ** 2)))

    # ---------------- 局部绕行子目标（2026-11-06 阿克曼绕障） ----------------
    def _detour_goal(self, robot: RobotState, goal, obstacles):
        """前方走廊被障碍挡住时，返回一个"侧向让开"的局部目标；否则 None。

        为什么必须有这一层：DWA 的可行域只覆盖 v*sim_time ≈ 2.4m，而阿克曼
        (R_min≈1.2m) 横向让出 0.8m 需要 5~6m 的提前量。等障碍进到 2.4m 内，
        任何转向候选的 3s 预测弧都会撞上（全不可行）→ 只能 M6 减速停车。
        把局部目标挪到障碍侧面后，head/dist 两项（唯一能区分转向的项）就会
        在 6m 外开始把车往侧面引，可行域内的 ±0.1/±0.2 rad/s 弧线足以完成绕行。
        """
        p = self.params
        if not obstacles:
            self._detour_reset()
            return None

        # 侧向偏移量相对**障碍当前位置**在**当前车头系**下定义：
        #   lat_g = lat_obs + side*req,  lg = along_obs + pass_ahead
        # 这样车一旦向左偏出去，lat_obs 变负、目标点的侧向量随之收敛 → 车自然
        # 回正并从障碍侧旁平行通过（"平行偏移"引导）。若改成冻结车头的世界锚点，
        # 目标点不收敛，车会在障碍旁边停死（实测：正前方障碍场景由"通过"退化为"停住"）。
        c, s = math.cos(robot.theta), math.sin(robot.theta)
        gdx, gdy = goal[0] - robot.x, goal[1] - robot.y
        goal_along = gdx * c + gdy * s
        r_veh = self._vehicle_clearance_radius()
        # 侧向让开量：车体外接半径 + 碰撞安全距离 d_safe + 余量。
        # 只用外接半径不够——可行域检查里障碍还带 d_safe 膨胀(默认 0.375)，
        # 让开 0.9m 时仍被判"贴住"，DWA 会再次全不可行而停车（实测复现）。
        d_safe = float(getattr(p, 'd_safe', 0.375))
        req = r_veh + d_safe + float(getattr(p, 'detour_clear_margin', 0.20))
        look = float(getattr(p, 'detour_look_dist', 6.5))
        amin = float(getattr(p, 'detour_min_along', 2.0))
        holding = self._detour_side != 0   # 绕行中：不再用 amin 门槛，否则半途释放
        cands = []
        for idx, o in enumerate(obstacles):
            dx, dy = o.fx - robot.x, o.fy - robot.y
            along = dx * c + dy * s          # 车头方向投影
            lat = -dx * s + dy * c           # 左侧为正
            if along < -0.5 or along > look: # 已越过车尾 / 太远
                continue
            if along > goal_along:           # 障碍在目标之后 → 不必绕
                continue
            r_obs = float(getattr(o, 'radius', 0.15))
            if abs(lat) > req + r_obs:       # 不在本车走廊内，或已经让开了
                continue
            if (not holding) and along < amin:   # 已经贴脸才开始绕 → 来不及
                continue
            cands.append((along, lat, r_obs, idx))
        if not cands:
            self._detour_reset()             # 走出走廊/已通过 → 交回真实目标
            return None
        along, lat, r_obs, tgt = min(cands, key=lambda z: z[0])
        side = self._detour_side
        if side == 0:
            # 障碍偏左 → 从右侧绕（side=-1）；偏右 → 从左侧绕
            side = -1 if lat > 0.0 else 1
            if self._detour_side_blocked(robot, obstacles, along, lat, side, req, tgt):
                side = -side
                if self._detour_side_blocked(robot, obstacles, along, lat, side, req,
                                             tgt):
                    # 两侧都被占：绕不过去 → 返回 None，交给 trap/自转/停车
                    return None
            self._detour_side = side
        lat_g = lat + side * req
        lmax = float(getattr(p, 'detour_max_lateral', 2.5))
        lat_g = max(-lmax, min(lmax, lat_g))
        lg = along + float(getattr(p, 'detour_pass_ahead', 0.80))
        lg = min(lg, max(goal_along, 1.0))    # 不要越过真实目标
        tx = robot.x + c * lg - s * lat_g
        ty = robot.y + s * lg + c * lat_g
        self._detour_obs_id = obstacles[tgt].id
        return (tx, ty)

    def _detour_reset(self):
        self._detour_side = 0
        self._detour_obs_id = None


    def _detour_side_blocked(self, robot, obstacles, along, lat, side, req,
                             skip=None) -> bool:
        """检查在 (along, lat+side*req) 的绕行通道上是否还有别的障碍。

        skip：正在被规避的那个障碍的下标（必须排除，否则它自己正好落在
        req 边界上，会被误判为"堵住"）。"""
        c, s = math.cos(robot.theta), math.sin(robot.theta)
        lat_g = lat + side * req
        for idx, o in enumerate(obstacles):
            if skip is not None and idx == skip:
                continue
            dx, dy = o.fx - robot.x, o.fy - robot.y
            a = dx * c + dy * s
            b = -dx * s + dy * c
            if abs(a - along) < 2.0 and abs(b - lat_g) < req + float(
                    getattr(o, 'radius', 0.15)):
                return True
        return False

    # ---------------- RDA-DWA 内部辅助 ----------------
    def enhanced_feature(self, sw: bool) -> bool:
        return (self.mode == 'ENHANCED_DWA') and sw

    def motion_heading(self, s: VelocitySample, robot: RobotState) -> float:
        # [改进A] 倒车时（v<0）实际运动方向为车尾方向 robot.theta+π。
        # 这样 heading_cost 惩罚"运动方向不朝目标"而非"车头不朝目标"，
        # DWA 才会真正选择倒车轨迹绕出 U 型陷阱。
        base = robot.theta
        if self.cc.model == "rectangle" and s.v < -self.cc.ack.v_eps:
            base = base + math.pi
        if self.params.use_predicted_heading:
            return self.model.predict_heading(robot, self._eff_omega(s), self.params.dt)
        return base

    def closing_rate(self, s: VelocitySample, robot: RobotState, obs: Obstacle) -> float:
        dcur = math.hypot(robot.x - obs.fx, robot.y - obs.fy)
        if dcur < 1e-9:
            return 0.0
        vox = obs.speed * math.cos(obs.heading)
        voy = obs.speed * math.sin(obs.heading)
        vrx = s.v * math.cos(robot.theta)
        vry = s.v * math.sin(robot.theta)
        dotp = (vox - vrx) * (robot.x - obs.fx) + (voy - vry) * (robot.y - obs.fy)
        return max(dotp / dcur, 0.0)

    def predicted_obs_position(self, obs: Obstacle, step: int, dt: float,
                               robot_position=None):
        t = float(step) * dt
        # [MTDWA 第二轮] g8：换向检测→恒角速度弧线外推（默认关，mtdwa 行为不变）
        # 障碍处于弧线模式（IMM 检测到换向）→ θ(t)=θ0+ω·t 逐段累积积分，
        # 修正 M2 恒速直线外推在换向期的折线失真（S12 随机换向障碍）。
        if (self.params.enable_maneuver_arc and obs.arc_mode_hold > 0
                and abs(obs.heading_rate) > 1e-9 and robot_position is not None
                and math.hypot(obs.x - robot_position[0],
                               obs.y - robot_position[1]) < self.params.arc_min_dist):
            w = obs.heading_rate
            th0 = obs.heading
            px = obs.x
            py = obs.y
            n = max(1, int(round(step)))
            for i in range(1, n + 1):
                th_i = th0 + w * (float(i) - 0.5) * dt
                px += obs.speed * math.cos(th_i) * dt
                py += obs.speed * math.sin(th_i) * dt
            return [px, py]
        # fx/fy represent t+dt; only extrapolate the remaining (step-1) intervals.
        remaining = max(0.0, t - dt)
        return [obs.fx + obs.speed * math.cos(obs.heading) * remaining,
                obs.fy + obs.speed * math.sin(obs.heading) * remaining]

    def gaussian_risk_cost(self, s: VelocitySample, robot: RobotState, obstacles, pro) -> float:
        sumv = 0.0
        rc = self._vehicle_clearance_radius()
        for k in range(len(obstacles)):
            obs = obstacles[k]
            dx = pro[0] - obs.fx
            dy = pro[1] - obs.fy
            v_rel = self.closing_rate(s, robot, obs)
            sigma_x = self.params.risk_sigma_x0 \
                + self.params.risk_kappa * v_rel * self.params.risk_tau_s
            sigma_y = self.params.risk_sigma_y0
            c = math.cos(obs.heading)
            st = math.sin(obs.heading)
            dxr = c * dx + st * dy
            dyr = -st * dx + c * dy
            # 扣除车辆等效半径（外接圆），使风险场作用在车-障碍距离上
            r_obs_frame = math.hypot(dxr, dyr)
            if rc > 0.0 and r_obs_frame > 1e-9:
                scale = max(r_obs_frame - rc, 0.0) / r_obs_frame
                dxr *= scale
                dyr *= scale
            sx = max(sigma_x, 1e-3)
            sy = max(sigma_y, 1e-3)
            q = (dxr * dxr) / (sx * sx) + (dyr * dyr) / (sy * sy)
            sumv = sumv + self.params.risk_amp * math.exp(-0.5 * q)
        return -sumv

    def headon_risk_cost(self, s: VelocitySample, robot: RobotState,
                         obstacles, pro) -> float:
        """g11：前方锥内迎面逼近风险（MTDWA 第四轮，默认关返回 0）。

        对样本轨迹方向 θ = motion_heading(s, robot)（每个样本不同）：
          - 障碍方位相对 θ 的角度 |θH| < hc_cone_rad（正前方锥内）
          - 迎面沿向速度 a = max(0, −v_obs·t̂) > hc_along_thr（障碍迎着自车来）
          - 预测障碍到样本 1 步预测位置距离 d < hc_dist_gate（区分样本 → min-max 有效）
        风险 = Σ w(d)·a，w(d)=max(0, 1−d/gate)；返回负值（越大越好惯例）。
        论文依据：卡 38（θ_r 相向/同向风险）、卡 20（MTTC 相对速度差）、卡 12（概率碰撞时间）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_headon_cost):
            return 0.0
        theta = self.motion_heading(s, robot)
        ct, st = math.cos(theta), math.sin(theta)
        total = 0.0
        for k in range(len(obstacles)):
            o = obstacles[k]
            thetaH = wrap_angle(math.atan2(o.fy - robot.y, o.fx - robot.x) - theta)
            if abs(thetaH) > p.hc_cone_rad:
                continue
            a = -(o.speed * math.cos(o.heading) * ct
                  + o.speed * math.sin(o.heading) * st)
            if a < p.hc_along_thr:
                continue
            d = math.hypot(o.fx - pro[0], o.fy - pro[1])
            if d > p.hc_dist_gate:
                continue
            total += max(0.0, 1.0 - d / p.hc_dist_gate) * a
        return -total

    def imm_risk_cost(self, s: VelocitySample, robot: RobotState,
                      obstacles, pro) -> float:
        """g12：近距高模型分歧惩罚（MTDWA 第四轮，默认关返回 0）。

        IMM 三模型（CV/CA/CT）1 步预测位置两两距离最大 div_err（predictor 已存档）
        ——换向/变速瞬间 CT 弧线与 CV 直线预测分歧 → div_err 跳变（前瞻指标，先于残差）。
        判据：div_err > unc_div_thr 且预测障碍到样本 1 步位置 d < unc_d_gate4。
        惩罚 = Σ w(d)·frac(div_err)；返回负值。
        论文依据：卡 8（ε_k 保守度旋钮）、卡 12（概率加权风险）、卡 57（信念→保守化）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_imm_cost):
            return 0.0
        total = 0.0
        for k in range(len(obstacles)):
            o = obstacles[k]
            if o.div_err < p.unc_div_thr:
                continue
            d = math.hypot(o.fx - pro[0], o.fy - pro[1])
            if d > p.unc_d_gate4:
                continue
            frac = min(1.0, max(0.0, (o.div_err - p.unc_div_thr)
                                / max(1e-6, p.unc_div_max - p.unc_div_thr)))
            total += max(0.0, 1.0 - d / p.unc_d_gate4) * frac
        return -total

    def risk_piecewise(self, thetaH: float, d: float) -> float:
        th = abs(thetaH)
        if th <= math.pi / 12.0 and d > self.params.d_safe:
            return self.params.beta[0]
        elif th <= math.pi / 12.0 and d <= self.params.d_safe:
            return self.params.beta[1]
        elif th <= math.pi / 6.0 and d <= self.params.d_safe:
            return self.params.beta[2]
        elif th <= math.pi / 6.0 and d <= 2.0 * self.params.d_safe:
            return self.params.beta[3]
        elif th <= math.pi / 6.0 and d <= 3.0 * self.params.d_safe:
            return self.params.beta[4]
        else:
            return 0.0

    def compute_cost_terms(self, s: VelocitySample, robot: RobotState,
                           goal, obstacles) -> CostTerms:
        t = CostTerms()
        m3 = self.enhanced_feature(self.params.enable_m3_gaussian_risk)

        goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        # ★2026-11-06 蛇形修复：head/dist 按 cost_horizon 预测（默认 0.4s），而非单步 dt。
        # 单步 dt 使航向代价隐含增益 = 1/dt = 10 rad/s per rad，δ 每帧打满 → 极限环。
        ch = float(getattr(self.params, 'cost_horizon', 0.0) or 0.0)
        if ch > 0.0:
            ew = self._eff_omega(s)
            t.head = -abs(wrap_angle(goal_ang - self.model.predict_heading(robot, ew, ch)))
            pro = self.model.predict_position(robot, s.v, ew, ch)
        else:
            t.head = -abs(wrap_angle(goal_ang - self.motion_heading(s, robot)))
            pro = self.model.predict_position(robot, s.v, self._eff_omega(s), self.params.dt)

        t.dist = -math.hypot(goal[0] - pro[0], goal[1] - pro[1])

        if m3:
            dmin = math.inf
            rc = self._vehicle_clearance_radius()
            for k in range(len(obstacles)):
                dmin = min(dmin, math.hypot(obstacles[k].fx - pro[0],
                                            obstacles[k].fy - pro[1]) - rc)
            if not obstacles:
                dmin = 1e3
            if self.params.normalize_trunc_D > 0.0 and dmin > self.params.normalize_trunc_D:
                dmin = self.params.normalize_trunc_D
            t.clear = dmin
            t.future = 0.0
            t.risk = self.gaussian_risk_cost(s, robot, obstacles, pro)
            # [MTDWA 第四轮] g11/g12：条件触发代价项（默认关 → 恒 0，mtdwa 行为不变）
            t.headon = self.headon_risk_cost(s, robot, obstacles, pro)
            t.unc = self.imm_risk_cost(s, robot, obstacles, pro)
        else:
            rc = self._vehicle_clearance_radius()
            clearance = math.inf
            dmin = math.inf
            for k in range(len(obstacles)):
                clearance = min(clearance, math.hypot(obstacles[k].fx - pro[0],
                                                      obstacles[k].fy - pro[1]) - rc)
                dmin = min(dmin, math.hypot(obstacles[k].fx - pro[0],
                                            obstacles[k].fy - pro[1]) - rc)
            if not obstacles:
                clearance = 1e3
                dmin = 1e3
            t.clear = -math.exp(self.params.alpha_c * (self.params.d_safe - clearance))
            t.future = -math.exp(self.params.alpha_f * (self.params.d_safe - dmin))
            t.risk = -self.risk_cost(s, robot, obstacles)

        theta_r = self.motion_heading(s, robot)
        countL = 0
        countR = 0
        for k in range(len(obstacles)):
            obs = obstacles[k]
            thetaH = wrap_angle(math.atan2(obs.fy - robot.y, obs.fx - robot.x) - theta_r)
            if -math.pi / 9.0 < thetaH <= 0.0:
                countL += 1
            if 0.0 <= thetaH < math.pi / 9.0:
                countR += 1
        t.count = -abs(float(countR - countL))

        # [代价平滑] 惩罚本帧候选速度相对上一周期指令的跳变
        # t.smooth = -((v-v_last)² + (ω-ω_last)²)（越大越好惯例，跳变越大越负）；
        # v_last/w_last 取 set_last_cmd 注入的内部值，为 None 时回退 robot.v/robot.omega。
        # 门控（_smooth_enabled）只影响 evaluate_window 中参与加权的有效权重，此处恒按公式计算。
        v_last = self._last_cmd_v if self._last_cmd_v is not None else robot.v
        w_last = self._last_cmd_w if self._last_cmd_w is not None else robot.omega
        t.smooth = -((s.v - v_last) ** 2 + (s.omega - w_last) ** 2)
        return t

    # ------------------------------------------------------------------
    # selectBest：候选 argmax
    # ------------------------------------------------------------------
    def select_best(self, window, robot: RobotState, goal, obstacles,
                    use_norm: bool, trap=None):
        """返回 (found, window, v_out, omega_out)。"""
        v_out = 0.0
        omega_out = 0.0
        trap_active = trap is not None and bool(trap) and trap.get('active', False)
        relax_pen = (self.params.c7_relax_penalty
                     if self.enhanced_feature(self.params.enable_c7_soft_relax) else 0.0)
        if trap_active:
            best = -math.inf
            found = False
            for i in range(len(window)):
                if not window[i].feasible:
                    continue
                window[i].score = self.escape_score(window[i], robot, goal, obstacles)
                if window[i].relaxed and relax_pen > 0.0:
                    window[i].score -= relax_pen
                if window[i].score > best:
                    best = window[i].score
                    v_out = window[i].v
                    omega_out = window[i].omega
                    found = True
            return found, window, v_out, omega_out

        if use_norm:
            ok_w, window = self.evaluate_window(window, robot, goal, obstacles)
            if not ok_w:
                return False, window, v_out, omega_out
            best = -math.inf
            found = False
            for i in range(len(window)):
                if not window[i].feasible:
                    continue
                if window[i].relaxed and relax_pen > 0.0:
                    window[i].score -= relax_pen
                if window[i].score > best:
                    best = window[i].score
                    v_out = window[i].v
                    omega_out = window[i].omega
                    found = True
        else:
            best = -math.inf
            found = False
            for i in range(len(window)):
                if not window[i].feasible:
                    continue
                window[i].score = self.evaluate(window[i], robot, goal, obstacles)
                if window[i].relaxed and relax_pen > 0.0:
                    window[i].score -= relax_pen
                if window[i].score > best:
                    best = window[i].score
                    v_out = window[i].v
                    omega_out = window[i].omega
                    found = True
        # [死区粘性] 归一化/非归一化两条正常路径共用：最优样本找到后尝试沿用上周期指令
        # （trap 分支已在上方提前 return，不会走到这里）。
        if found:
            v_out, omega_out = self._apply_sticky(window, v_out, omega_out, best)
        return found, window, v_out, omega_out

    def _apply_sticky(self, window, v_best: float, w_best: float,
                      score_best: float):
        """[死区粘性] 抑制最优样本边界抖动（2026-11-03 第二轮）。

        前提（全部满足才粘）：
          - 上周期指令已注入（set_last_cmd 未调用时不粘）；
          - 上周期非静止：|v_last| >= spin_v_eps（自转/停车/急停期不粘）；
          - 本周期尚未粘过（_sticky_applied，plan() 开头复位，每周期至多粘一次）。
        在 window 中按容差（≈v_resolution/2、omega_resolution/2）找与 (v_last, w_last)
        匹配且 feasible 的样本；找到且 |score_best - score_last| < sticky_eps
        （归一化加权和 score 空间）→ 输出上周期指令 (v_last, w_last) 并置粘性标记。
        返回 (v_out, omega_out)。
        """
        if self._sticky_applied:
            return v_best, w_best
        if self._last_cmd_v is None or self._last_cmd_w is None:
            return v_best, w_best
        if abs(self._last_cmd_v) < self.params.spin_v_eps:
            return v_best, w_best
        v_tol = self.params.v_resolution / 2.0
        w_tol = self.params.omega_resolution / 2.0
        v_last, w_last = self._last_cmd_v, self._last_cmd_w
        score_last = None
        for i in range(len(window)):
            if not window[i].feasible:
                continue
            if (abs(window[i].v - v_last) <= v_tol
                    and abs(window[i].omega - w_last) <= w_tol):
                score_last = window[i].score
                break
        if score_last is None:
            return v_best, w_best
        if abs(score_best - score_last) < self.params.sticky_eps:
            self._sticky_applied = True
            return v_last, w_last
        return v_best, w_best

    # ------------------------------------------------------------------
    # C3：DWA* 前瞻 + U 形逃逸
    # ------------------------------------------------------------------
    def rollout_terms(self, s: VelocitySample, robot: RobotState, goal,
                      obstacles, N: int, gamma: float) -> CostTerms:
        t = CostTerms()
        r = robot
        for i in range(1, N + 1):
            r = self.model.integrate(r, s.v, self._eff_omega(s), self.params.dt)
            obs_i = []
            for k in range(len(obstacles)):
                pk = self.predicted_obs_position(
                    obstacles[k], i, self.params.dt,
                    robot_position=(robot.x, robot.y))
                pn = self.predicted_obs_position(
                    obstacles[k], i + 1, self.params.dt,
                    robot_position=(robot.x, robot.y))
                o2 = obstacles[k]
                # 浅拷贝修改位置
                o3 = copy.copy(o2)
                o3.x = pk[0]
                o3.y = pk[1]
                o3.fx = pn[0]
                o3.fy = pn[1]
                obs_i.append(o3)
            ti = self.compute_cost_terms(s, r, goal, obs_i)
            w = gamma ** (float(i) - 1.0)
            t.head += w * ti.head
            t.dist += w * ti.dist
            t.clear += w * ti.clear
            t.future += w * ti.future
            t.risk += w * ti.risk
            t.count += w * ti.count
            t.smooth += w * ti.smooth
        return t

    def detect_trap(self, robot: RobotState, obstacles, goal) -> bool:
        """C3 U 形陷阱检测（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.detect_trap(robot, obstacles, goal)

    def generate_local_goal(self, robot: RobotState, obstacles, goal):
        """C3 局部子目标（实现见 rda_dwa/escape.py EscapeStrategy）。返回 (sub_goal, escape_heading)。"""
        return self.escape.generate_local_goal(robot, obstacles, goal)

    def generate_local_goal_safe(self, robot: RobotState, obstacles, goal):
        """C6 安全重校验局部子目标（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.generate_local_goal_safe(robot, obstacles, goal)

    def generate_evasion_goal(self, robot: RobotState, obstacles, goal):
        """C12 横向避让子目标（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.generate_evasion_goal(robot, obstacles, goal)

    def escape_subgoal(self, robot: RobotState, obstacles, goal, trap, threat):
        """C3/C12 逃逸子目标生成（v15 perp/sigmoid 覆盖；实现见 rda_dwa/escape.py）。"""
        return self.escape.escape_subgoal(robot, obstacles, goal, trap, threat)

    def ped_cross_threat(self, robot: RobotState, obstacles):
        """[MTDWA P3] 行人横穿威胁判定（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.ped_cross_threat(robot, obstacles)

    def free_distance(self, robot: RobotState, obstacles, ang: float, max_range: float) -> float:
        """沿角方向自由净空（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.free_distance(robot, obstacles, ang, max_range)

    def lateral_intent_threat(self, robot: RobotState, obstacles):
        """[MTDWA 融合 g3] 同向横向接近意图威胁判定（实现见 escape.py）。"""
        return self.escape.lateral_intent_threat(robot, obstacles)

    def gap_blocked(self, robot: RobotState, obstacles):
        """[MTDWA 融合 g4] 对向流间隙阻塞判定（实现见 escape.py）。"""
        return self.escape.gap_blocked(robot, obstacles)

    def rss_speed_limit(self, robot: RobotState, obstacles):
        """[MTDWA 第二轮 g5] LC-RSS 纵向安全距离限速（实现见 escape.py）。"""
        return self.escape.rss_speed_limit(robot, obstacles)

    def brake_wait_limit(self, robot: RobotState, obstacles):
        """[MTDWA 第二轮 g7] 横向运动带低速让行限速（实现见 escape.py）。"""
        return self.escape.brake_wait_limit(robot, obstacles)

    # ------------------------------------------------------------------
    # C4：人工势场横向力
    # ------------------------------------------------------------------
    def apf_lateral_force(self, robot: RobotState, goal, obstacles) -> float:
        F = self.apf_force([robot.x, robot.y], goal, obstacles)
        return F[0] * (-math.sin(robot.theta)) + F[1] * math.cos(robot.theta)

    def apf_force(self, pos, goal, obstacles):
        Fatt = [self.params.apf_k_att * (goal[0] - pos[0]),
                self.params.apf_k_att * (goal[1] - pos[1])]
        Frep = [0.0, 0.0]
        d0 = self.params.apf_d0
        krep = self.params.apf_k_rep
        rc = self._vehicle_clearance_radius()
        for k in range(len(obstacles)):
            obs = obstacles[k]
            d = max(math.hypot(obs.fx - pos[0], obs.fy - pos[1]) - rc, 0.0)
            if d < d0 and d > 1e-6:
                ux = (pos[0] - obs.fx) / d
                uy = (pos[1] - obs.fy) / d
                f = krep * (1.0 / d - 1.0 / d0) * (1.0 / (d * d))
                Frep[0] += f * ux
                Frep[1] += f * uy
        return [Fatt[0] + Frep[0], Fatt[1] + Frep[1]]

    # ------------------------------------------------------------------
    # C3 逃逸专用打分
    # ------------------------------------------------------------------
    def escape_score(self, s: VelocitySample, robot: RobotState, sub_goal, obstacles) -> float:
        """C3 逃逸专用打分（实现见 rda_dwa/escape.py EscapeStrategy）。"""
        return self.escape.escape_score(s, robot, sub_goal, obstacles)
