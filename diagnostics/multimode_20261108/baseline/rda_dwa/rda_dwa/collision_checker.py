"""
collision_checker.py — 可切换的内部碰撞/可行性检查器。

三种模式：
  - "rectangle"：Ackermann 自行车模型轨迹预测 + 矩形轮廓碰撞（1.0×0.6m，
    参考点=后轴中心；v<v_eps 时禁止转向，不支持原地自转）
  - "rect4wis"：四轮独立转向（4WIS）模型轨迹预测 + 矩形轮廓碰撞
    （参考点 = 车体几何中心；v≈0 时支持原地旋转轨迹）
  - "rect_diff"：真实尺寸矩形差速驱动模型轨迹预测 + 矩形轮廓碰撞
    （参考点 = 车体几何中心 ±(L/2,W/2)；v≈0 时支持原地旋转轨迹；
    运动学 = 纯差速 v/ω 直接积分，无 δ/轴距概念；双向驱动 v∈[-1.5,1.5]）

rectangle / rect4wis / rect_diff 模式采用分离轴定理（SAT）精确检测：
  对轨迹每个采样位姿计算车体矩形，将障碍圆心投影到矩形两条轴向
  （纵向=车头方向、横向=垂直方向），得到圆心到矩形的精确最近距离，
  d < r_obs 即碰撞。外接圆仅用作远障碍快速预筛（提速），最终判定以 SAT 精确值为准。

[2026-10-27 场景改造] 障碍物矩形化（行人/自行车/车辆/静态方块）：
  - 新增矩形-矩形精确净距（SAT 多轴投影 + 顶点-边最近距离，向量化）；
  - 带 type/length/width 的障碍按矩形精确判定；旧圆形障碍回退 point-r 语义；
  - 统一"净距"口径：>0 分离、=0 相切、<0 重叠（碰撞）；
  - radius（外接圆）仍用于远障碍预筛与 DQN/风险场/APF 等"半径使用逻辑"。

供 EnhancedDWA、ClassicDWA、MPC、A*+DWA、APF 等规划器内部调用。
不改变规划器的决策机制、代价函数、采样语义——仅替换轨迹预测与碰撞口径。
"""
from __future__ import annotations

import math
import numpy as np

from .ackermann_model import AckermannParams, AckermannState, point_to_rectangle_distance
from .four_wheel_steer_model import FourWheelSteerParams, FourWheelSteerState

_RECT_TYPES = ('pedestrian', 'bicycle', 'vehicle', 'static_large', 'block')


def _is_rect_obstacle(o) -> bool:
    """障碍是否为矩形现实类型（带 length/width）。"""
    t = getattr(o, 'obstacle_type', None)
    L = float(getattr(o, 'length', 0.0) or 0.0)
    W = float(getattr(o, 'width', 0.0) or 0.0)
    return t in _RECT_TYPES and L > 1e-9 and W > 1e-9


def _rect_corners_vec(x, y, th, L, W, hb=0.0):
    """矩形角点（向量化 batch）：输入 (N,) 数组，输出 (N,4,2)。

    参考点偏移 hb（rectangle 模式后轴→中心）。角点顺序：前右、前左、后左、后右。
    """
    c, s = np.cos(th), np.sin(th)
    hl, hw = L / 2.0, W / 2.0
    cx = x + hb * c
    cy = y + hb * s
    out = np.empty((len(c), 4, 2))
    out[:, 0, 0] = cx + hl * c - hw * s
    out[:, 0, 1] = cy + hl * s + hw * c
    out[:, 1, 0] = cx + hl * c + hw * s
    out[:, 1, 1] = cy + hl * s - hw * c
    out[:, 2, 0] = cx - hl * c + hw * s
    out[:, 2, 1] = cy - hl * s - hw * c
    out[:, 3, 0] = cx - hl * c - hw * s
    out[:, 3, 1] = cy - hl * s + hw * c
    return out


def _vertex_edge_min(A, B):
    """两凸多边形（矩形）顶点-边最近距离，返回 shape (S,)（A: (S,4,2), B: (4,2)）。"""
    S = A.shape[0]
    d = np.full((S,), np.inf)
    # A 顶点 → B 各边
    for j in range(4):
        p0 = B[j]
        p1 = B[(j + 1) % 4]
        seg = p1 - p0
        l2 = max(float(seg.dot(seg)), 1e-12)
        rel = A - p0
        t = (rel @ seg) / l2
        t = np.clip(t, 0.0, 1.0)
        proj = p0[None, None, :] + t[..., None] * seg[None, None, :]
        dist = np.linalg.norm(A - proj, axis=-1)
        d = np.minimum(d, dist.min(axis=1))
    # B 顶点 → A 各边（S 相关）
    for i in range(4):
        p0 = A[:, i]
        p1 = A[:, (i + 1) % 4]
        seg = p1 - p0
        l2 = np.maximum((seg * seg).sum(axis=1), 1e-12)
        rel = B[None, :, :] - p0[:, None, :]
        t = (rel * seg[:, None, :]).sum(axis=-1) / l2[:, None]
        t = np.clip(t, 0.0, 1.0)
        proj = p0[:, None, :] + t[..., None] * seg[:, None, :]
        dist = np.linalg.norm(B[None, :, :] - proj, axis=-1)
        d = np.minimum(d, dist.min(axis=1))
    return d


def _rect_rect_gaps_vec(A, B):
    """两矩形批量净距（SAT + 顶点-边），A: (S,4,2)，B: (4,2) → (S,)。

    >0 分离（精确最近距离）、=0 相切、<0 重叠（取最小穿透深度 MTV 负值）。
    """
    S = A.shape[0]
    dA = np.roll(A, -1, axis=1) - A          # (S,4,2)
    nA = np.stack([dA[:, :, 1], -dA[:, :, 0]], axis=-1)
    dB = np.roll(B, -1, axis=0) - B          # (4,2)
    nB = np.stack([dB[:, 1], -dB[:, 0]], axis=-1)
    axes = np.concatenate([nA, np.tile(nB[None], (S, 1, 1))], axis=1)  # (S,8,2)
    # 归一化轴向：各轴投影/重叠量统一到米、跨轴可比较
    # （未归一化时投影按边长缩放，max 取出的"最大重叠"可能不是真实 MTV）
    nrm = np.linalg.norm(axes, axis=-1, keepdims=True)
    axes = axes / np.maximum(nrm, 1e-12)
    aA = np.einsum('spd,sqd->spq', A, axes)   # (S,4,8)
    pA0 = aA.min(axis=1)
    pA1 = aA.max(axis=1)
    aB = np.einsum('pd,sqd->spq', B, axes)    # (S,4,8)
    pB0 = aB.min(axis=1)
    pB1 = aB.max(axis=1)
    sep = np.maximum(pB0 - pA1, pA0 - pB1)    # 各轴分离间隙（两个方向的较大者）
    ov = np.minimum(pA1, pB1) - np.maximum(pA0, pB0)  # 各轴重叠（米）
    any_sep = (sep > 0.0).any(axis=1)
    min_overlap = ov.min(axis=1)              # MTV：最小平移分离量
    gaps = np.where(any_sep, _vertex_edge_min(A, B), -min_overlap)
    return gaps


class CollisionChecker:
    """可切换碰撞模式的轨迹预测 + 可行性检查器。"""

    def __init__(self, collision_model: str = "rect4wis",
                 robot_radius: float = 0.1,
                 obstacle_radius: float = 0.15,
                 ack_params: AckermannParams | None = None,
                 fws_params: FourWheelSteerParams | None = None):
        assert collision_model in ("rectangle", "rect4wis", "rect_diff"), \
            f"unknown collision_model: {collision_model}"
        self.model = collision_model
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.ack = ack_params or AckermannParams()
        self.fws = fws_params or FourWheelSteerParams()
        # rectangle 模式轨迹检查用外接圆（保守、快速；用户明确允许 r_c≈0.583m）
        self.rect_traj_clearance = self.ack.r_circum + obstacle_radius  # 0.733m
        # rect4wis 外接圆（几何中心到角点，与 ackermann 外接圆一致）
        self.fws_rect_traj_clearance = self.fws.r_circum + obstacle_radius  # 0.733m
        # rect_diff 矩形差速车：真实尺寸 1.0×0.6m（与 4WIS 同车身），几何中心参考
        self.diff_length = 1.0
        self.diff_width = 0.6
        self.diff_v_max = 1.5
        self.diff_v_min = -1.5
        self.diff_a_max = 1.0
        self.diff_dec_max = 1.5
        self.diff_v_eps = 0.05
        self.diff_r_circum = math.hypot(self.diff_length / 2.0, self.diff_width / 2.0)
        # 轨迹预测模式：'ramp0'（原 v=0 爬升，默认）| 'const_v'（恒速外推）
        self.traj_mode = 'ramp0'

    # ------------------------------------------------------------------
    # 轨迹预测
    # ------------------------------------------------------------------
    def predict_trajectory(self, v: float, omega: float, x0: float, y0: float,
                           theta0: float, dt: float, n_steps: int):
        """预测候选 (v, ω) 对应的轨迹点。

        rectangle 模式：Ackermann 自行车模型（v,ω→v,δ 适配后积分）。

        返回 np.ndarray shape (n_steps, 3)：每行 [x, y, theta]。
        """
        traj = np.zeros((n_steps, 3))
        if self.model == "rectangle":
            # Ackermann：从 δ=0 开始适配
            ap = self.ack
            delta = 0.0
            v_cur = 0.0  # 从上一帧速度开始加速
            x, y, th = x0, y0, theta0
            for i in range(n_steps):
                # 适配 v,ω → v,δ
                v_cur, delta, _ = self._adapt(v, omega, v_cur, delta, dt)
                x += v_cur * math.cos(th) * dt
                y += v_cur * math.sin(th) * dt
                th = th + (v_cur / ap.wheelbase) * math.tan(delta) * dt
                traj[i] = [x, y, th]
        elif self.model == "rect4wis":
            # rect4wis：v/ω 解耦，v≈0 时原地旋转（位置不动，θ 线性变化）
            # 注意：保持标量 math.cos/sin（数组 np.cos 多元素路径与 libm 存在 ulp
            # 差异，会改变行为；此处向量化会违反"行为逐位一致"）
            fp = self.fws
            x, y, th = x0, y0, theta0
            if self.traj_mode == 'const_v':
                # 恒速外推：从当前位姿直接以采样 (v,ω) 恒定积分 T 秒，
                # 不从 0 爬升（标准 DWA 做法）。
                v_use = max(fp.v_min, min(fp.v_max, v))
                for i in range(n_steps):
                    if abs(v_use) < fp.v_eps:
                        th = th + omega * dt
                    else:
                        x += v_use * math.cos(th) * dt
                        y += v_use * math.sin(th) * dt
                        th = th + omega * dt
                    traj[i] = [x, y, th]
            else:
                v_cur = 0.0  # 从静止开始按加速度爬升
                for i in range(n_steps):
                    dv_max = fp.a_max * dt
                    dv_min = -fp.dec_max * dt
                    v_cur = v_cur + max(dv_min, min(dv_max, v - v_cur))
                    v_cur = max(fp.v_min, min(fp.v_max, v_cur))
                    if abs(v_cur) < fp.v_eps:
                        # 原地旋转：位置不动，仅 θ 更新
                        th = th + omega * dt
                    else:
                        x += v_cur * math.cos(th) * dt
                        y += v_cur * math.sin(th) * dt
                        th = th + omega * dt
                    traj[i] = [x, y, th]
        else:
            # rect_diff：真实尺寸矩形差速驱动。运动学 = 纯差速（v/ω 直接积分），
            # 但 v≈0 时允许原地旋转（位置不动，θ 线性变化），与 rect4wis 一致。
            # 双向驱动 v∈[-1.5,1.5]；矩形参考点 = 车体几何中心。
            v_cur = 0.0
            x, y, th = x0, y0, theta0
            for i in range(n_steps):
                dv_max = self.diff_a_max * dt
                dv_min = -self.diff_dec_max * dt
                v_cur = v_cur + max(dv_min, min(dv_max, v - v_cur))
                v_cur = max(self.diff_v_min, min(self.diff_v_max, v_cur))
                if abs(v_cur) < self.diff_v_eps:
                    th = th + omega * dt
                else:
                    x += v_cur * math.cos(th) * dt
                    y += v_cur * math.sin(th) * dt
                    th = th + omega * dt
                traj[i] = [x, y, th]
        return traj

    def _adapt(self, v_cmd, omega_cmd, v_cur, delta_cur, dt):
        """v/ω→v/δ 适配（与 ackermann_model.AckermannModel.adapt_velocity 一致）。

        [改进A] 支持倒车（v<0）：|v|<v_eps 时禁止转向；v<0 时 atan(ωL/v) 自动
        反号（倒车打右转向车向左后转，符合真实汽车 Reeds-Shepp 运动学）。
        """
        ap = self.ack
        dv_max = ap.a_max * dt
        dv_min = -ap.dec_max * dt
        v = v_cur + max(dv_min, min(dv_max, v_cmd - v_cur))
        v = max(ap.v_min, min(ap.v_max, v))

        if abs(v) < ap.v_eps:
            delta = delta_cur
        else:
            delta_target = math.atan(omega_cmd * ap.wheelbase / v)
            delta = max(-ap.delta_max, min(ap.delta_max, delta_target))

        ddelta_max = ap.delta_dot_max * dt
        delta = delta_cur + max(-ddelta_max, min(ddelta_max, delta - delta_cur))
        delta = max(-ap.delta_max, min(ap.delta_max, delta))

        kappa = math.tan(delta) / ap.wheelbase if abs(delta) > 1e-9 else 0.0
        return v, delta, kappa

    # ------------------------------------------------------------------
    # 碰撞检查
    # ------------------------------------------------------------------
    def _rect_hb(self) -> float:
        """矩形中心相对轨迹参考点的纵向偏移。

        rectangle（Ackermann）：参考点=后轴中心，矩形中心前移 wheelbase/2。
        rect4wis / rect_diff：参考点=车体几何中心，偏移 0。
        """
        if self.model == "rectangle":
            return self.ack.wheelbase / 2.0
        return 0.0

    def _rect_dims(self):
        """返回 (length, width, r_circum)。"""
        if self.model == "rectangle":
            return self.ack.length, self.ack.width, self.ack.r_circum
        if self.model == "rect_diff":
            return self.diff_length, self.diff_width, self.diff_r_circum
        return self.fws.length, self.fws.width, self.fws.r_circum

    def vehicle_clearance_radius(self) -> float:
        """车辆等效半径（代价/APF/预筛口径）：矩形模式用外接圆 r_c。
        公开访问点，取代跨模块取 _rect_dims 私有成员。"""
        if self.model in ("rectangle", "rect4wis", "rect_diff"):
            return self._rect_dims()[2]
        return 0.0

    def _rect_traj_clearance_to_point(self, traj: np.ndarray, px: float, py: float) -> float:
        """矩形模式：轨迹逐位姿矩形轮廓到单个点的最小距离（SAT 精确）。

        参考点（轨迹坐标）到矩形中心的偏移：rectangle=wheelbase/2，rect4wis=0。
        对每个位姿：
          u = 圆心投影到纵向轴（车头方向）的带符号距离
          v = 圆心投影到横向轴（垂直方向）的带符号距离
          d = hypot(max(|u|-L/2,0), max(|v|-W/2,0))，点在矩形内为 0。
        返回整个轨迹上的最小距离。
        """
        L, W, _ = self._rect_dims()
        hb = self._rect_hb()
        xs = traj[:, 0]
        ys = traj[:, 1]
        ths = traj[:, 2]
        ct = np.cos(ths)
        st = np.sin(ths)
        # 矩形中心
        cx = xs + hb * ct
        cy = ys + hb * st
        dx = px - cx
        dy = py - cy
        # 投影到纵向/横向轴
        u = dx * ct + dy * st
        v = -dx * st + dy * ct
        du = np.maximum(np.abs(u) - L / 2.0, 0.0)
        dv = np.maximum(np.abs(v) - W / 2.0, 0.0)
        d = np.hypot(du, dv)
        return float(np.min(d))

    def is_collision_free(self, traj: np.ndarray, obstacles) -> bool:
        """轨迹是否无碰撞。

        rectangle / rect4wis / rect_diff：逐位姿矩形轮廓精确判定。
        矩形现实类型障碍 → 矩形-矩形净距 < 0（重叠）即碰撞；
        旧圆形障碍 → 圆心到矩形 SAT 距离 < r_obs 即碰撞（兼容）。
        先用矩形中心距做外接圆预筛跳过远处障碍（仅提速，不改变判定）。
        """
        if not obstacles:
            return True
        ox = np.array([o.x for o in obstacles], dtype=np.float64)
        oy = np.array([o.y for o in obstacles], dtype=np.float64)
        L, W, r_c = self._rect_dims()
        hb = self._rect_hb()
        xs = traj[:, 0]
        ys = traj[:, 1]
        ths = traj[:, 2]
        ct = np.cos(ths)
        st = np.sin(ths)
        cx = xs + hb * ct
        cy = ys + hb * st
        A = _rect_corners_vec(cx, cy, ths, L, W, 0.0)
        for j in range(len(obstacles)):
            o = obstacles[j]
            r_obs = getattr(o, 'radius', self.obstacle_radius)
            d_c = np.hypot(cx - ox[j], cy - oy[j])
            # 外接圆预筛：矩形中心距 ≥ r_c + r_obs 则必无碰撞（跳过）
            if float(np.min(d_c)) >= r_c + r_obs:
                continue
            if _is_rect_obstacle(o):
                B = _rect_corners_vec(np.array([o.x]), np.array([o.y]),
                                      np.array([o.heading]), o.length, o.width, 0.0)[0]
                gaps = _rect_rect_gaps_vec(A, B)
                if float(np.min(gaps)) < 0.0:
                    return False
            else:
                # 旧圆形障碍：SAT 精确判定（圆心到自车矩形距离 < r_obs 碰撞）
                dx = ox[j] - cx
                dy = oy[j] - cy
                u = dx * ct + dy * st
                v = -dx * st + dy * ct
                du = np.maximum(np.abs(u) - L / 2.0, 0.0)
                dv = np.maximum(np.abs(v) - W / 2.0, 0.0)
                d_sat = np.hypot(du, dv)
                if float(np.min(d_sat)) < r_obs:
                    return False
        return True

    def obstacle_rect_corners(self, o):
        """障碍矩形角点（现实类型）；非矩形障碍返回 None。"""
        if not _is_rect_obstacle(o):
            return None
        return _rect_corners_vec(np.array([o.x]), np.array([o.y]),
                                 np.array([o.heading]), o.length, o.width, 0.0)[0]

    def net_gap_at(self, x: float, y: float, theta: float,
                   ox: float, oy: float, oth: float,
                   L: float, W: float, r: float | None = None) -> float:
        """单姿态净距：自车矩形(x,y,theta) vs 障碍(ox,oy,oth,L,W)。

        矩形障碍（L/W 有效）→ 矩形-矩形精确净距；L/W 无效 → 圆心距 - r。
        供代价函数（clearance/future）在预测位姿处计算精确净距。
        """
        if L > 1e-9 and W > 1e-9:
            hb = self._rect_hb()
            eL, eW, _ = self._rect_dims()
            A = _rect_corners_vec(np.array([x + hb * math.cos(theta)]),
                                  np.array([y + hb * math.sin(theta)]),
                                  np.array([theta]), eL, eW, 0.0)
            B = _rect_corners_vec(np.array([ox]), np.array([oy]),
                                  np.array([oth]), L, W, 0.0)[0]
            return float(_rect_rect_gaps_vec(A, B)[0])
        d = self._rect_traj_clearance_to_point(np.array([[x, y, theta]]), ox, oy)
        return d - float(r if r is not None else self.obstacle_radius)

    def rect_gap_at(self, x: float, y: float, theta: float, o) -> float:
        """自车矩形（位姿 x,y,theta）到单个障碍的净距。

        矩形现实类型 → 矩形-矩形精确净距；旧圆形 → 圆心距 - r_obs。
        统一口径：>0 分离、≤0 接触/重叠。
        """
        return self.net_gap_at(x, y, theta, o.x, o.y, o.heading,
                               float(getattr(o, 'length', 0.0) or 0.0),
                               float(getattr(o, 'width', 0.0) or 0.0),
                               float(getattr(o, 'radius', self.obstacle_radius)))

    def min_clearance_to_obstacle(self, traj: np.ndarray, o) -> float:
        """轨迹到单个障碍的最小净距（矩形-矩形精确；旧圆形回退 point-r）。"""
        if _is_rect_obstacle(o):
            L, W, _ = self._rect_dims()
            hb = self._rect_hb()
            A = _rect_corners_vec(traj[:, 0] + hb * np.cos(traj[:, 2]),
                                  traj[:, 1] + hb * np.sin(traj[:, 2]),
                                  traj[:, 2], L, W, 0.0)
            B = _rect_corners_vec(np.array([o.x]), np.array([o.y]),
                                  np.array([o.heading]), o.length, o.width, 0.0)[0]
            return float(np.min(_rect_rect_gaps_vec(A, B)))
        d = self.min_clearance_to_point(traj, o.x, o.y)
        return d - float(getattr(o, 'radius', self.obstacle_radius))

    def min_clearance_to_obstacle_at(self, traj: np.ndarray, o,
                                     oth: float | None = None) -> float:
        """轨迹到"预测位姿障碍"的最小净距（M2 多步外推用）。

        oth：障碍预测朝向（默认 o.heading；弧线外推时传入 o.heading+rate*t）。
        """
        if _is_rect_obstacle(o):
            L, W, _ = self._rect_dims()
            hb = self._rect_hb()
            A = _rect_corners_vec(traj[:, 0] + hb * np.cos(traj[:, 2]),
                                  traj[:, 1] + hb * np.sin(traj[:, 2]),
                                  traj[:, 2], L, W, 0.0)
            th = o.heading if oth is None else float(oth)
            B = _rect_corners_vec(np.array([o.x]), np.array([o.y]),
                                  np.array([th]), o.length, o.width, 0.0)[0]
            return float(np.min(_rect_rect_gaps_vec(A, B)))
        d = self.min_clearance_to_point(traj, o.x, o.y)
        return d - float(getattr(o, 'radius', self.obstacle_radius))

    def min_clearance(self, traj: np.ndarray, obstacles) -> float:
        """轨迹到最近障碍的最小轮廓净空（统一净距口径）。

        rectangle / rect4wis / rect_diff：逐位姿矩形轮廓精确判定。
        矩形现实类型 → 矩形-矩形净距；旧圆形 → 圆心距 - r_obs。
        """
        if not obstacles:
            return 1e3
        dmin = math.inf
        for o in obstacles:
            d = self.min_clearance_to_obstacle(traj, o)
            if d < dmin:
                dmin = d
        return dmin

    def min_clearance_to_point(self, traj: np.ndarray, px: float, py: float) -> float:
        """轨迹到单个点的最小净空（SAT 精确轮廓距离）。"""
        return self._rect_traj_clearance_to_point(traj, px, py)

    def _rect_traj_clearance_to_points(self, traj: np.ndarray, px, py) -> np.ndarray:
        """批量版：轨迹到多个点的最小净空（SAT 精确），返回 shape (..., M) 数组。

        支持 traj 为 (S,3)（单轨迹→(M,)）或 (K,S,3)（批量→(K,M)）。
        与逐点 _rect_traj_clearance_to_point 逐元素同运算（同一批 numpy 调用），
        min 归约对浮点精确（min 无舍入），因此逐位一致。
        """
        L, W, _ = self._rect_dims()
        hb = self._rect_hb()
        xs = traj[..., 0]
        ys = traj[..., 1]
        ths = traj[..., 2]
        ct = np.cos(ths)
        st = np.sin(ths)
        cx = xs + hb * ct
        cy = ys + hb * st
        PX = np.asarray(px, dtype=np.float64)
        PY = np.asarray(py, dtype=np.float64)
        dx = PX[..., None, :] - cx[..., :, None]
        dy = PY[..., None, :] - cy[..., :, None]
        u = dx * ct[..., :, None] + dy * st[..., :, None]
        v = -dx * st[..., :, None] + dy * ct[..., :, None]
        du = np.maximum(np.abs(u) - L / 2.0, 0.0)
        dv = np.maximum(np.abs(v) - W / 2.0, 0.0)
        d = np.hypot(du, dv)
        return np.min(d, axis=-2)

    def min_clearance_to_points(self, traj: np.ndarray, px, py) -> np.ndarray:
        """批量版：轨迹到多个点的最小净空，返回 shape (..., M) 数组（逐位一致）。"""
        return self._rect_traj_clearance_to_points(traj, px, py)

    def point_clearance_at(self, x: float, y: float, theta: float, obstacles) -> float:
        """当前位姿到最近障碍的净距（用于 APF 等当前位置检查）。

        矩形现实类型 → 矩形-矩形净距；旧圆形 → 圆心到自车矩形距离 - r_obs。
        """
        if not obstacles:
            return 1e3
        dmin = math.inf
        for o in obstacles:
            d = self.rect_gap_at(x, y, theta, o)
            if d < dmin:
                dmin = d
        return dmin

    # ------------------------------------------------------------------
    # 矩形几何
    # ------------------------------------------------------------------
    def _rect_corners(self, x, y, theta):
        """计算车辆矩形 4 角点。

        rectangle：与 ackermann_model.rectangle_corners 一致（后轴中心参考）。
        rect4wis / rect_diff：车体几何中心参考（±(L/2, W/2)）。
        """
        if self.model in ("rect4wis", "rect_diff"):
            half_l = self.diff_length / 2.0 if self.model == "rect_diff" else self.fws.length / 2.0
            half_w = self.diff_width / 2.0 if self.model == "rect_diff" else self.fws.width / 2.0
            local = [
                (half_l, half_w),
                (half_l, -half_w),
                (-half_l, -half_w),
                (-half_l, half_w),
            ]
        else:
            ap = self.ack
            rear_oh = (ap.length - ap.wheelbase) / 2.0
            front_ext = ap.wheelbase + rear_oh
            half_w = ap.width / 2.0
            local = [
                (front_ext, half_w),
                (front_ext, -half_w),
                (-rear_oh, -half_w),
                (-rear_oh, half_w),
            ]
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        corners = []
        for lx, ly in local:
            wx = x + lx * cos_t - ly * sin_t
            wy = y + lx * sin_t + ly * cos_t
            corners.append((wx, wy))
        return corners

    @property
    def inflation_radius(self) -> float:
        """用于 A* 网格膨胀的半径，统一用矩形外接圆 r_c。"""
        return self._rect_dims()[2]

    @property
    def is_spin_capable(self) -> bool:
        """该车型是否支持原地自转（v≈0 时 ω 有效）。

        rectangle（Ackermann）：False（v<v_eps 时禁止转向）。
        rect4wis / rect_diff：True。
        """
        return self.model in ("rect4wis", "rect_diff")
