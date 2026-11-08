"""
predictor.py — 障碍物运动预测模块。

包含：
  ObstaclePredictor —— M5 前置 α-β 滤波 + 差分速度/航向（公式(3)）
  IMMUKFPredictor   —— C2 IMM-UKF 三模型交互滤波（CV/CA/CT，7 维状态）

接口与 MATLAB 一致：initialize(obs, x, y, heading, speed) -> obs
                       update(obs, nx, ny) -> obs
"""
from __future__ import annotations

import math

import numpy as np

from .robot_model import wrap_angle
from .obstacle import Obstacle, new_obstacle


class ObstaclePredictor:
    """ObstaclePredictor.m 等价。"""

    def __init__(self, dt: float = 0.1, max_speed: float = 1.05):
        self.dt = dt
        self.max_speed = max_speed
        self.alpha = 1.0
        self.beta = 1.0
        # C9 预测残差机动检测（默认关闭）
        self.maneuver_detect = False
        self.resid_thresh = 0.15
        self.hold_steps = 8

    def set_smoothing(self, alpha: float, beta: float):
        """M5：配置 α-β 系数，(0,1]；1.0 = 不滤波。"""
        self.alpha = min(max(alpha, 0.0), 1.0)
        self.beta = min(max(beta, 0.0), 1.0)

    def enable_maneuver_detect(self, resid_thresh: float, hold_steps: int,
                               heading_jump: float = 0.8):
        """C9：开启预测残差/航向跳变驱动的机动标记（方向突变/反弹/换向时膨胀有效半径）。"""
        self.maneuver_detect = True
        self.resid_thresh = resid_thresh
        self.hold_steps = int(hold_steps)
        self.heading_jump_thresh = heading_jump

    def initialize(self, obs: Obstacle, x: float, y: float,
                   heading: float, speed: float) -> Obstacle:
        """首帧初始化：构造虚拟上一位置 P_obs(t-1)=P_obs(t)-V·[cosθ,sinθ]·Δt。"""
        obs.x = x
        obs.y = y
        obs.px = x - speed * math.cos(heading) * self.dt
        obs.py = y - speed * math.sin(heading) * self.dt
        obs.speed = speed
        obs.heading = heading
        obs.has_history = True
        # M5：以给定初值（仿真真值）作为滤波先验
        obs.filtered_speed = speed
        obs.filtered_heading = heading
        obs.has_filter_state = True
        self.predict_next(obs)
        return obs

    def update(self, obs: Obstacle, nx: float, ny: float) -> Obstacle:
        """每步接收新位置：差分求 V_obs/θ_obs，α-β 平滑后预测 P_obs(t+1)。"""
        obs.px = obs.x
        obs.py = obs.y
        obs.x = nx
        obs.y = ny
        obs.has_history = True

        dx = obs.x - obs.px
        dy = obs.y - obs.py
        raw_heading = math.atan2(dy, dx)
        raw_speed = math.hypot(dx, dy) / self.dt

        if raw_speed > self.max_speed:
            raw_speed = self.max_speed

        if not obs.has_filter_state:
            obs.filtered_speed = raw_speed
            obs.filtered_heading = raw_heading
            obs.has_filter_state = True
        else:
            obs.filtered_speed = self.alpha * raw_speed + (1.0 - self.alpha) * obs.filtered_speed
            d_theta = wrap_angle(raw_heading - obs.filtered_heading)
            obs.filtered_heading = wrap_angle(obs.filtered_heading + self.beta * d_theta)
        if obs.filtered_speed > self.max_speed:
            obs.filtered_speed = self.max_speed
        obs.speed = obs.filtered_speed
        obs.heading = obs.filtered_heading
        # 静态障碍（现实类型场景）为轴对齐固定物体：航向恒 0，
        # 冻结差分 atan2 产生的幻影航向（静态目标速度≈0 → atan2 任意角）。
        if getattr(obs, 'obstacle_type', None) in ('static_large', 'block'):
            obs.heading = 0.0

        # C9：上一步预测（fx/fy）对当前实际位置的残差 + 航向跳变 → 机动标记
        if self.maneuver_detect:
            res_err = math.hypot(obs.fx - obs.x, obs.fy - obs.y)
            dth = abs(wrap_angle(obs.heading - obs.prev_heading))
            if res_err > self.resid_thresh or dth > self.heading_jump_thresh:
                obs.maneuver_hold = self.hold_steps
            elif obs.maneuver_hold > 0:
                obs.maneuver_hold -= 1
        # g10：残差存档（MTDWA 第三轮，规划层保守收缩用；默认关不影响行为）
        obs.res_err = math.hypot(obs.fx - obs.x, obs.fy - obs.y)
        obs.prev_heading = obs.heading

        self.predict_next(obs)
        return obs

    def predict_next(self, obs: Obstacle) -> Obstacle:
        """公式(3) 第一式：P_obs(t+1) = P_obs(t) + V·[cosθ, sinθ]·Δt。"""
        obs.fx = obs.x + obs.speed * math.cos(obs.heading) * self.dt
        obs.fy = obs.y + obs.speed * math.sin(obs.heading) * self.dt
        return obs


# ============================================================================
# C2: IMM-UKF 多模型交互滤波
# ============================================================================

class IMMUKFPredictor:
    """IMMUKFPredictor.m 等价（C2）。三模型 CV/CA/CT，7 维状态 [px,py,vx,vy,ax,ay,ω]。"""

    def __init__(self, pp):
        self.dt = pp.dt
        self.max_speed = pp.obstacle_max_speed
        self.models = ['CV', 'CA', 'CT']
        self.trans = np.asarray(pp.imm_transition, dtype=np.float64)     # 3×3
        self.mu = np.asarray(pp.imm_init_prob, dtype=np.float64).reshape(3, 1)
        qp = pp.imm_q_p if hasattr(pp, 'imm_q_p') else 1e-4
        qv = pp.imm_q_v if hasattr(pp, 'imm_q_v') else 0.10
        qa = pp.imm_q_a if hasattr(pp, 'imm_q_a') else 1.00
        qw = pp.imm_q_w if hasattr(pp, 'imm_q_w') else 0.80
        self.Q = np.diag([qp, qp, qv, qv, qa, qa, qw])
        sr = pp.imm_meas_noise
        self.R = np.diag([sr * sr, sr * sr])
        self.alpha_u = 1e-3
        self.beta_u = 2.0
        self.kappa_u = 0.0
        # C9 预测残差机动检测（默认关闭）
        self.maneuver_detect = False
        self.resid_thresh = 0.15
        self.hold_steps = 8
        self.heading_jump_thresh = 0.8
        # g8 换向检测→弧线外推（MTDWA 第二轮，默认关闭）
        self.maneuver_arc = False
        self.arc_w_thresh = 0.4
        self.arc_hold = 5
        self.arc_min_dist = 1.5
        self.arc_w_cap = 1.2          # rad/s：角速度写入上限（防噪声过冲）

    def enable_maneuver_detect(self, resid_thresh: float, hold_steps: int,
                               heading_jump: float = 0.8):
        """C9：开启预测残差/航向跳变驱动的机动标记。"""
        self.maneuver_detect = True
        self.resid_thresh = resid_thresh
        self.hold_steps = int(hold_steps)
        self.heading_jump_thresh = heading_jump

    def enable_maneuver_arc(self, w_thresh: float, hold: int, min_dist: float):
        """g8：开启换向检测→弧线外推标记（卡 8 多模态预测的简化：换向→切换运动模型）。"""
        self.maneuver_arc = True
        self.arc_w_thresh = w_thresh
        self.arc_hold = int(hold)
        self.arc_min_dist = min_dist

    # ---------------- 公共接口 ----------------

    def initialize(self, obs: Obstacle, x: float, y: float,
                   heading: float, speed: float) -> Obstacle:
        obs.x = x
        obs.y = y
        obs.px = x - speed * math.cos(heading) * self.dt
        obs.py = y - speed * math.sin(heading) * self.dt
        obs.speed = speed
        obs.heading = heading
        obs.has_history = True

        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        x0 = np.array([x, y, vx, vy, 0.0, 0.0, 0.0], dtype=np.float64)
        sp, sv, sa, sw = 0.05, 0.2, 0.5, 0.2
        P0 = np.diag([sp**2, sp**2, sv**2, sv**2, sa**2, sa**2, sw**2])

        obs.imm = {
            'x': [x0.copy(), x0.copy(), x0.copy()],
            'P': [P0.copy(), P0.copy(), P0.copy()],
            'mu': self.mu.copy(),
            'P_fused': P0[0:2, 0:2].copy(),
        }
        self.predict_next(obs)
        return obs

    def update(self, obs: Obstacle, nx: float, ny: float) -> Obstacle:
        obs.px = obs.x
        obs.py = obs.y
        obs.x = nx
        obs.y = ny
        obs.has_history = True
        z = np.array([nx, ny], dtype=np.float64)

        mu_old = obs.imm['mu'].reshape(3, 1)

        # ---- 1) 模型概率交互（mixing） ----
        cj = self.trans.T @ mu_old                       # 3×1
        mu_mix = (self.trans * mu_old) / cj.T            # 3×3，列和为 1

        x0 = np.zeros((7, 3))
        P0 = np.zeros((7, 7, 3))
        for j in range(3):
            x0j = np.zeros(7)
            for i in range(3):
                x0j = x0j + mu_mix[i, j] * obs.imm['x'][i]
            x0[:, j] = x0j
            P0j = np.zeros((7, 7))
            for i in range(3):
                dxi = obs.imm['x'][i] - x0j
                P0j = P0j + mu_mix[i, j] * (obs.imm['P'][i] + np.outer(dxi, dxi))
            P0[:, :, j] = P0j

        # ---- 2) 各模型 UKF predict + update ----
        L = np.zeros(3)
        xup = np.zeros((7, 3))
        Pup = np.zeros((7, 7, 3))
        for j in range(3):
            xp, Pp = self.ukf_predict(x0[:, j], P0[:, :, j], self.models[j])
            xu, Pu, nu, S, lk = self.ukf_update(xp, Pp, z, self.models[j])
            xup[:, j] = xu
            Pup[:, :, j] = Pu
            L[j] = lk

        # ---- 3) 模型概率更新 ----
        mu_new = L * cj.ravel()
        s = mu_new.sum()
        mu_new = mu_new / s if s > 0 else np.array([1.0, 0.0, 0.0])
        obs.imm['mu'] = mu_new.reshape(3, 1)

        # ---- 4) 概率加权融合 ----
        xf = np.zeros(7)
        for j in range(3):
            xf = xf + mu_new[j] * xup[:, j]
        Pf = np.zeros((7, 7))
        for j in range(3):
            dxj = xup[:, j] - xf
            Pf = Pf + mu_new[j] * (Pup[:, :, j] + np.outer(dxj, dxj))

        # ---- 5) 输出兼容字段 ----
        for j in range(3):
            obs.imm['x'][j] = xup[:, j]
            obs.imm['P'][j] = Pup[:, :, j]
        obs.imm['P_fused'] = Pf[0:2, 0:2]
        vx = xf[2]      # 状态序 [px,py,vx,vy,ax,ay,ω]
        vy = xf[3]
        spd = min(math.hypot(vx, vy), self.max_speed)
        obs.speed = spd
        obs.heading = math.atan2(vy, vx)
        # 静态障碍为轴对齐固定物体：航向恒 0（冻结 IMM 对静态目标的幻影航向）
        if getattr(obs, 'obstacle_type', None) in ('static_large', 'block'):
            obs.heading = 0.0

        # C9：上一步预测对当前实际位置的残差 + 航向跳变 → 机动标记
        if self.maneuver_detect:
            res_err = math.hypot(obs.fx - obs.x, obs.fy - obs.y)
            dth = abs(wrap_angle(obs.heading - obs.prev_heading))
            if res_err > self.resid_thresh or dth > self.heading_jump_thresh:
                obs.maneuver_hold = self.hold_steps
            elif obs.maneuver_hold > 0:
                obs.maneuver_hold -= 1
        # g10：残差存档（MTDWA 第三轮，规划层保守收缩用；默认关不影响行为）
        obs.res_err = math.hypot(obs.fx - obs.x, obs.fy - obs.y)

        # g12：IMM 三模型 1 步预测位置两两距离最大（MTDWA 第四轮，预测分歧存档）
        # 换向/变速瞬间 CT（弧线）与 CV（直线）预测分歧 → div_err 跳变（前瞻指标，
        # 先于残差反映"预测不可靠"）；恒速时三模型预测接近 → div_err 小。
        pos1 = self.model_propagate(obs.imm['x'][0], 'CV')[0:2]
        pos2 = self.model_propagate(obs.imm['x'][1], 'CA')[0:2]
        pos3 = self.model_propagate(obs.imm['x'][2], 'CT')[0:2]
        d12 = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
        d13 = math.hypot(pos1[0] - pos3[0], pos1[1] - pos3[1])
        d23 = math.hypot(pos2[0] - pos3[0], pos2[1] - pos3[1])
        obs.div_err = max(d12, d13, d23)

        # g8：换向检测 → 弧线外推标记（MTDWA 第二轮）
        # 检测滤波航向变化率 |Δθ|/Δt 超阈值 → 障碍进入弧线模式（M2 外推改用
        # 恒角速度弧线积分，见 enhanced_dwa.predicted_obs_position）。与 C9 的
        # 区别：C9 是"判死膨胀"（可行性层），g8 是"预测基准修正"（预测层）。
        if self.maneuver_arc:
            dth_now = wrap_angle(obs.heading - obs.prev_heading)
            dth_dt = abs(dth_now) / self.dt
            if dth_dt > self.arc_w_thresh:
                obs.arc_mode_hold = self.arc_hold
                w_sign = 1.0 if dth_now >= 0.0 else -1.0
                obs.heading_rate = w_sign * min(dth_dt, self.arc_w_cap)
            elif obs.arc_mode_hold > 0:
                obs.arc_mode_hold -= 1
                if obs.arc_mode_hold == 0:
                    obs.heading_rate = 0.0

        obs.prev_heading = obs.heading

        self.predict_next(obs)
        return obs

    def predict_next(self, obs: Obstacle) -> Obstacle:
        """按模型概率加权的 1 步外推 P_obs(t+1)。"""
        mu = obs.imm['mu'].ravel()
        p1 = np.zeros(2)
        for j in range(3):
            xj_next = self.model_propagate(obs.imm['x'][j], self.models[j])
            p1 = p1 + mu[j] * xj_next[0:2]
        obs.fx = float(p1[0])
        obs.fy = float(p1[1])
        return obs

    # ---------------- 内部：模型运动学 / UKF ----------------

    def model_propagate(self, x: np.ndarray, model: str) -> np.ndarray:
        """状态序 [px, py, vx, vy, ax, ay, ω]（索引 0..6）。"""
        dt = self.dt
        if model == 'CV':
            xn = x + dt * np.array([x[2], x[3], 0, 0, 0, 0, 0])
        elif model == 'CA':
            xn = x + dt * np.array([x[2], x[3], x[4], x[5], 0, 0, 0]) \
                 + 0.5 * dt * dt * np.array([x[4], x[5], 0, 0, 0, 0, 0])
        elif model == 'CT':
            w = x[6]   # ω = 第 7 个状态分量
            if abs(w) < 1e-6:
                xn = x + dt * np.array([x[2], x[3], 0, 0, 0, 0, 0])
            else:
                sw = math.sin(w * dt)
                cw = math.cos(w * dt)
                vx = x[2]
                vy = x[3]
                xn = x.copy()
                xn[0] = x[0] + (vx / w) * sw - (vy / w) * (1 - cw)
                xn[1] = x[1] + (vy / w) * sw + (vx / w) * (1 - cw)
                xn[2] = vx * cw - vy * sw
                xn[3] = vy * cw + vx * sw
        else:
            raise ValueError(f'unknown model {model}')
        return xn

    def sigma_points(self, x: np.ndarray, P: np.ndarray):
        n = x.size
        lam = self.alpha_u**2 * (n + self.kappa_u) - n
        c = math.sqrt(n + lam)
        try:
            A = np.linalg.cholesky((P + P.T) / 2.0)
        except np.linalg.LinAlgError:
            vals, vecs = np.linalg.eigh((P + P.T) / 2.0)
            vals = np.clip(vals, 0.0, None)
            A = vecs @ np.diag(np.sqrt(vals))
        chi = np.zeros((n, 2 * n + 1))
        chi[:, 0] = x
        for i in range(n):
            chi[:, i + 1] = x + c * A[:, i]
            chi[:, n + i + 1] = x - c * A[:, i]
        Wm = np.zeros(2 * n + 1)
        Wc = np.zeros(2 * n + 1)
        Wm[0] = lam / (n + lam)
        Wc[0] = Wm[0] + (1.0 - self.alpha_u**2 + self.beta_u)
        Wm[1:] = 1.0 / (2.0 * (n + lam))
        Wc[1:] = Wm[1:]
        return chi, Wm, Wc

    def ukf_predict(self, x: np.ndarray, P: np.ndarray, model: str):
        n = 7
        chi, Wm, Wc = self.sigma_points(x, P)
        Xn = np.zeros((n, 2 * n + 1))
        for i in range(2 * n + 1):
            Xn[:, i] = self.model_propagate(chi[:, i], model)
        xp = Xn @ Wm
        Pp = np.zeros((n, n))
        for i in range(2 * n + 1):
            d = Xn[:, i] - xp
            Pp = Pp + Wc[i] * np.outer(d, d)
        Pp = Pp + self.Q
        return xp, Pp

    def ukf_update(self, xp: np.ndarray, Pp: np.ndarray, z: np.ndarray, model: str):
        n = 7
        chi, Wm, Wc = self.sigma_points(xp, Pp)
        npts = 2 * n + 1
        Z = np.zeros((2, npts))
        for i in range(npts):
            xc = chi[:, i]
            Z[:, i] = [xc[0], xc[1]]
        zp = Z @ Wm
        S = self.R.copy()
        for i in range(npts):
            d = Z[:, i] - zp
            S = S + Wc[i] * np.outer(d, d)
        Pxz = np.zeros((n, 2))
        for i in range(npts):
            dx = chi[:, i] - xp
            dz = Z[:, i] - zp
            Pxz = Pxz + Wc[i] * np.outer(dx, dz)
        K = Pxz @ np.linalg.inv(S)
        nu = z - zp
        xu = xp + K @ nu
        Pu = Pp - K @ S @ K.T
        # 高斯量测似然 L = N(z; zp, S)
        lk = math.exp(-0.5 * float(nu @ np.linalg.solve(S, nu))) / math.sqrt(2 * math.pi * np.linalg.det(S))
        if not math.isfinite(lk) or lk < 1e-300:
            lk = 1e-300
        return xu, Pu, nu, S, lk
