"""
escape.py — 逃逸策略模块（C2 架构深化：从 EnhancedDWA 单体中抽出）。

收纳全部"逃逸判定 / 子目标生成 / 逃逸打分"逻辑：
  C3  U 形陷阱检测 + 局部子目标（detect_trap / generate_local_goal / generate_local_goal_safe）
  C5  上下文感知合法让行等待（c5_wait_blocked）
  C6  逃逸方向动态重校验（revalidate_escape / _escape_heading_safe）
  C12 对向威胁主动横向逃逸（headon_threat / generate_evasion_goal / escape_subgoal）
  C3  逃逸专用打分（escape_score）

算法逻辑与参数严格对应原 enhanced_dwa.py 中的实现，逐字节搬移，禁止改动。
EnhancedDWA 保留同名公开方法作薄委托，外部调用（trial / ros / tests / 基线）零改动。

共享小助手（enhanced_feature / motion_heading / _eff_omega / predicted_obs_position /
_vehicle_clearance_radius）经 core 转发，使搬移方法体保持逐字节不变。
"""
from __future__ import annotations

import math

from .robot_model import wrap_angle


class EscapeStrategy:
    """逃逸策略：判定 / 子目标生成 / 打分。

    持有 core（EnhancedDWA）引用；params / cc / model 以同名属性直接引用，
    共享小助手以转发方法暴露。
    """

    def __init__(self, core):
        self.core = core
        self.params = core.params
        self.cc = core.cc
        self.model = core.model

    # ------------------------------------------------------------------
    # 共享小助手转发（保持搬移方法体逐字节不变）
    # ------------------------------------------------------------------
    def enhanced_feature(self, sw: bool) -> bool:
        return self.core.enhanced_feature(sw)

    def motion_heading(self, s, robot) -> float:
        return self.core.motion_heading(s, robot)

    def _eff_omega(self, s) -> float:
        return self.core._eff_omega(s)

    def predicted_obs_position(self, obs, step: int, dt: float, robot_position=None):
        return self.core.predicted_obs_position(obs, step, dt, robot_position)

    def _vehicle_clearance_radius(self) -> float:
        return self.core._vehicle_clearance_radius()

    # ------------------------------------------------------------------
    # C5 上下文感知逃逸判定（P3）
    # ------------------------------------------------------------------
    def c5_wait_blocked(self, robot, obstacles) -> bool:
        """判断当前停滞是否为"合法让行等待"（动态障碍主动逼近/横穿），
        返回 True 表示应抑制 C3 陷阱逃逸触发。

        规则（参数标定见 params.py，几何依据来自 S9/S11/S13 场景布局）：
          对每个动态障碍，若其恒速直线预测路径在 c5_wait_horizon 内接近自车
          当前位置（< c5_wait_threat_dist），则：
            - 若障碍运动方向与自车航向近似对向（夹角 > c5_headon_ang）
              且横向偏移 < c5_lateral_off（同车道对向逼近，如 S9 反弹车）→
              等待不安全，不抑制（允许逃逸避让）；
            - 否则（横穿如 S11 行人、相邻车道通过如 S13 对向车流、同向慢车）→
              等待合法，抑制逃逸。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_c5_wait_aware_escape):
            return False
        if p.c5_wait_horizon <= 0.0:
            return False
        robot_heading = robot.theta
        horizon = p.c5_wait_horizon
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            vx = o.speed * math.cos(o.heading)
            vy = o.speed * math.sin(o.heading)
            dx = robot.x - o.fx
            dy = robot.y - o.fy
            # 点到障碍直线路径的最近时间/距离（恒速假设）
            d2 = vx * vx + vy * vy
            tca = 0.0
            if d2 > 1e-12:
                tca = (dx * vx + dy * vy) / d2
                tca = max(0.0, tca)
            if tca > horizon:
                continue
            # 最近点位置
            cx = o.fx + vx * tca
            cy = o.fy + vy * tca
            lat_dist = math.hypot(robot.x - cx, robot.y - cy)
            if lat_dist > p.c5_wait_threat_dist:
                continue
            # 接近角：障碍运动方向 vs 自车航向（0..π）
            rel_ang = abs(wrap_angle(o.heading - robot_heading))
            # 同车道对向逼近 → 等待不安全，不抑制
            if rel_ang > p.c5_headon_ang and lat_dist < p.c5_lateral_off:
                continue
            return True
        return False

    # ------------------------------------------------------------------
    # C12 对向威胁主动横向逃逸（P1/P3，S9）
    # ------------------------------------------------------------------
    def headon_threat(self, robot, obstacles, skip_angle_gate=False):
        """同车道对向逼近威胁判定：动态障碍以相对速度逼近自车（TTC<c12_ttc_max）、
        其恒速路径横向偏移 < c12_lat_max、且运动方向与自车航向近似对向
        （夹角 > c12_headon_ang）→ 需要立即横向逃逸（不等 stall 15 步）。

        skip_angle_gate=True 时跳过角度门（仅用于已由 C12 激活的逃逸在 persist
        阶段方向锁存：逃逸中 robot.theta 旋转后 rel_ang 变化不应误释放已确认威胁）。
        初始触发阶段保持 False，以保护 S10 横穿/路口不被方向无关误触发。

        返回威胁障碍对象（供 generate_evasion_goal 生成横向避让目标）；
        无威胁返回 None。

        与 C5 的合法等待判定互补：横穿（S11 行人）/相邻车道（S13 对向车流）
        不满足"对向角"或"横向偏移"条件，不触发。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_c12_headon_evade):
            return None
        rvx = robot.v * math.cos(robot.theta)
        rvy = robot.v * math.sin(robot.theta)
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            vx = o.speed * math.cos(o.heading)
            vy = o.speed * math.sin(o.heading)
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            # 障碍必须在自车前方（航向 ±90° 内），排除"身后"误触发
            if dx * hx + dy * hy <= 0.0:
                continue
            # vrel = 障碍速度 - 自车速度（障碍坐标系相对速度）
            vrelx = vx - rvx
            vrely = vy - rvy
            vrel2 = vrelx * vrelx + vrely * vrely
            if vrel2 < 1e-9:
                continue
            # 相对速度最近点（CPA）：沿 vrel 方向两车最接近的距离与时刻
            # （借鉴第3篇 UUV"预测轨迹交点 P1 到达时间"与第5篇"预测位置裁剪"：
            #  用双方运动预测判是否真的会撞，而非只看当前位置垂距/瞬时 TTC）
            t_cpa = -((vrelx * dx + vrely * dy) / vrel2)
            if t_cpa < 0.0 or t_cpa > p.c12_ttc_max:
                continue
            cpa = math.hypot(dx + t_cpa * vrelx, dy + t_cpa * vrely)
            if cpa >= p.c12_cpa_max:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            # 双频带触发：对向带 rel_ang>c12_headon_ang（现状）∪ 追尾带 rel_ang<c12_rear_ang
            # （>0 时启用同向追近），排除 90° 横穿带。skip_angle_gate（persist 锁存）整门跳过。
            if not skip_angle_gate and not (
                    rel_ang > p.c12_headon_ang
                    or (p.c12_rear_ang > 0.0 and rel_ang < p.c12_rear_ang)):
                continue
            return o
        return None

    # ------------------------------------------------------------------
    # [MTDWA 融合] g3：同向横向接近意图触发（S9 切入，默认关）
    # ------------------------------------------------------------------
    def lateral_intent_threat(self, robot, obstacles):
        """同向横向接近（切入意图）威胁判定：障碍与自车同向、在前方、横向带内
        且朝自车路径横向合拢（TLC≤阈值）→ 判定为切入意图威胁。

        S9 切入车：与自车同速 → CPA vrel≈0 无定义 → headon_threat 永不触发；
        本谓词用"横向合拢速率×到车道线距离（TLC）"补上意图维度
        （论文卡 09/20/49：IMM/MMAE 意图估计的物理量纲）。
        横穿（S10/S11 |Δθ|≈π/2）与非同向（S6/S7 远角度）被同向带排除。

        返回威胁障碍对象；无则 None。默认 enable_s9_lateral=False → 恒 None。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_s9_lateral):
            return None
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            # 必须在自车前方（航向 ±90° 内），排除"身后"误触发
            if dx * hx + dy * hy <= 0.0:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang > p.s9_lateral_same_ang:
                continue                      # 非"同向带"（横穿/对向/远角度）不归切入
            # 横向偏移（航向系左法向投影，左为正）与横向合拢速度
            lat = dx * (-hy) + dy * hx
            lat_v = o.speed * math.sin(wrap_angle(o.heading - robot.theta))
            if abs(lat) > p.s9_lateral_lat_max:
                continue
            if lat * lat_v >= 0.0:
                continue                      # 未朝自车路径合拢（远离或平行）
            tlc = abs(lat) / max(abs(lat_v), 1e-6)
            if tlc > p.s9_lateral_ttc:
                continue
            return o
        return None

    def _lateral_escape_subgoal(self, robot, obstacles, goal, threat):
        """g3 切入意图逃逸子目标：与 c12_escape_perp 相同几何（垂直横移选净空大侧
        + 少量前进），但不受频带门约束（切入车 rel_ang≈0 不满足对向带门）。
        返回 (sub_goal, escape_heading)。"""
        p = self.params
        off = getattr(p, 'c12_evade_offset', 1.1)
        fwd = getattr(p, 'c12_evade_forward', 0.4)
        _hx = math.cos(robot.theta)
        _hy = math.sin(robot.theta)
        _nx, _ny = -_hy, _hx
        _cl_l = self.free_distance(robot, obstacles, robot.theta + math.pi / 2.0, 2.0)
        _cl_r = self.free_distance(robot, obstacles, robot.theta - math.pi / 2.0, 2.0)
        _sign = 1.0 if _cl_l >= _cl_r else -1.0
        _sx = robot.x + _sign * off * _nx + fwd * _hx
        _sy = robot.y + _sign * off * _ny + fwd * _hy
        return [_sx, _sy], math.atan2(_sy - robot.y, _sx - robot.x)

    # ------------------------------------------------------------------
    # [MTDWA 融合] g4：对向流间隙感知（S13，默认关）
    # ------------------------------------------------------------------
    def gap_blocked(self, robot, obstacles):
        """相邻车道对向流阻塞判定：存在"对向（|Δθ|>c12_headon_ang）且横向带
        [s13_gap_lat_min, s13_gap_lat_max] 内且沿自车航向在前 s13_gap_dist 内"的
        动态障碍 → 返回 True（间隙被阻塞，逃逸期间应维持横向偏移不回归）。

        默认 enable_s13_gap=False → 恒 False（mtdwa 行为不变）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_s13_gap):
            return False
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang <= p.c12_headon_ang:
                continue                      # 非对向（横穿/同向）不阻塞
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            along = dx * hx + dy * hy
            if along < 0.0 or along > p.s13_gap_dist:
                continue
            lat = dx * (-hy) + dy * hx
            if abs(lat) < p.s13_gap_lat_min or abs(lat) > p.s13_gap_lat_max:
                continue
            return True
        return False

    # ------------------------------------------------------------------
    # [MTDWA 第二轮] g5：LC-RSS 纵向安全距离限速（S9 终点区迎头，默认关）
    # ------------------------------------------------------------------
    def rss_speed_limit(self, robot, obstacles):
        """LC-RSS 纵向安全距离限速（卡 33 式 5.2 简化为相对速度形式）。

        同车道（|lat|<rss_lat_max）对向（rel_ang>c12_headon_ang）或追尾
        （rel_ang<c12_rear_ang）动态障碍，实际距离
            d < ρ·v_rel + v_rel²/(2·a_brake) + dlon
        判定"不可安全保持当前速度"→ 返回 proper response 安全速度
            v_safe = √(2·a_brake·(d − dlon))
        （时间换空间，不等 C12 几何最近点判据；解决 S9 cpa=0.501≥0.5 边缘失判）。
        返回 None 表示不限速（mtdwa 行为不变）。

        与已证伪的 cpa 阈值放宽（点值微调）区别：判据层是 worst-case 纵向
        安全距离不等式（反应时间+制动包络），动作层是速度窗口约束，且与 C12
        横向逃逸互补不互斥。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_rss_speed_limit):
            return None
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        v_safe = p.max_linear_velocity
        any_lim = False
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            along = dx * hx + dy * hy
            if along <= 0.0:
                continue                      # 后方障碍不触发
            lat = dx * (-hy) + dy * hx
            if abs(lat) > p.rss_lat_max:
                continue                      # 非同车道（S10/S11 横向不触发）
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if not (rel_ang > p.c12_headon_ang
                    or (p.c12_rear_ang > 0.0 and rel_ang < p.c12_rear_ang)):
                continue                      # 非对向/追尾（横穿带排除）
            # 沿连线方向的相对接近速度（正=相互接近）
            vx = o.speed * math.cos(o.heading)
            vy = o.speed * math.sin(o.heading)
            rvx = robot.v * math.cos(robot.theta)
            rvy = robot.v * math.sin(robot.theta)
            v_rel = -((vx - rvx) * (dx / d) + (vy - rvy) * (dy / d))
            if v_rel <= 0.0:
                continue
            d_safe = p.rss_rho * v_rel + v_rel * v_rel / (2.0 * p.rss_a_brake) + p.rss_dlon
            if d < d_safe:
                v_lim = math.sqrt(max(0.0, 2.0 * p.rss_a_brake * (d - p.rss_dlon)))
                v_safe = min(v_safe, v_lim)
                any_lim = True
        return v_safe if any_lim else None

    # ------------------------------------------------------------------
    # [MTDWA 第二轮] g7：横向运动带专用低速让行（S11/S8/S12，默认关）
    # ------------------------------------------------------------------
    def brake_wait_limit(self, robot, obstacles):
        """横向运动带专用低速让行（卡 34 刹车等待行为状态 + §11.2 方向三）。

        障碍夹角 rel_ang ∈ [bw_band_min, bw_band_max]（横向带 45°~135°）且在前方
        bw_dist 内（横向带内）→ 返回低速让行 vmax=bw_vmax（停车让其先过）。
        返回 None 表示不限速（mtdwa 行为不变）。

        区别于已证伪的 P3（行人分类器+让行子目标）/C5（全域抑制）/g3（横向意图
        触发逃逸）：本候选只在采样层限速（vmax 窗口约束），不触发、不抑制逃逸
        状态机；障碍类别无关（行人/车/换向皆适用，只要横向运动）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_brake_wait):
            return None
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            along = dx * hx + dy * hy
            if along <= 0.0 or along > p.bw_dist:
                continue
            lat = dx * (-hy) + dy * hx
            if abs(lat) > p.bw_dist:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if p.bw_band_min <= rel_ang <= p.bw_band_max:
                return p.bw_vmax
        return None

    # ------------------------------------------------------------------
    # [MTDWA 第三轮] g9：目标区对向 TTC 减速让行 / g10：高残差保守收缩
    # ------------------------------------------------------------------
    def goal_ttc_limit(self, robot, obstacles, goal):
        """目标区对向 TTC 减速让行（第一轮报告 §8 方向一官方首选；卡 01/07）。

        S9 终点区"慢车反弹迎头 + cpa≈0.501≥0.5 边缘失判"：C12 几何判据在
        cpa 恰超阈值 1mm 时失效。本判据在接近目标（d_goal<gtz_dist）时，对
        前方对向（rel_ang>c12_headon_ang）且对向接近时间
            TTC = along / v_rel < gtz_ttc
        的动态障碍（任意侧偏，无横向带门控）返回让行限速 vmax=gtz_vmax
        （时间换空间，不等 C12 触发）。返回 None 表示不限速（mtdwa 不变）。

        区别于：cpa 阈值放宽（几何点值微调，已证伪）；g5 RSS lat 门控
        （rss_lat_max=0.5 中心横向带被 S9 威胁侧偏击穿——本候选无 lat 门控，
        目标区任何侧偏对向车都让行）。仅在目标区生效（副作用局部化）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_goal_ttc):
            return None
        if goal is None:
            return None
        if hasattr(goal, 'x'):
            gx, gy = goal.x, goal.y
        else:
            gx, gy = goal[0], goal[1]
        d_goal = math.hypot(gx - robot.x, gy - robot.y)
        if d_goal > p.gtz_dist:
            return None
        hx = math.cos(robot.theta)
        hy = math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang <= p.c12_headon_ang:
                continue                      # 非对向
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            along = dx * hx + dy * hy
            if along <= 0.0:
                continue                      # 后方不触发
            vx = o.speed * math.cos(o.heading)
            vy = o.speed * math.sin(o.heading)
            rvx = robot.v * math.cos(robot.theta)
            rvy = robot.v * math.sin(robot.theta)
            v_rel = -((vx - rvx) * (dx / d) + (vy - rvy) * (dy / d))
            if v_rel <= 0.0:
                continue
            ttc = along / v_rel
            if ttc < p.gtz_ttc:
                return p.gtz_vmax
        return None

    def unc_vmax_limit(self, robot, obstacles):
        """高残差近距速度窗口保守收缩（卡 8 ISA-MPC ε_k / 卡 24 GMM 不确定性）。

        S12 随机换向障碍：IMM 恒速预测失真（残差超 unc_r_thr）→ M2 选中
        碰撞轨迹（18 撞）。g1 C9 判死（膨胀半径→样本剔除）在 S10 堵塞交叉口
        （0/15 红线破坏）；本候选不判死样本、不剔除轨迹，只对近距
        （d<unc_d_gate）高残差障碍收缩 vmax 窗口：
            scale = 1 - unc_k·min(1, (r-unc_r_thr)/(unc_r_max-unc_r_thr))
        （vmax≥vmin 保底由 build_dynamic_window 保证）→ 慢速通过不阻塞。
        返回 None 表示不限速（mtdwa 行为不变）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_unc_vmax):
            return None
        scale = 1.0
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            r = getattr(o, 'res_err', 0.0)
            if r <= p.unc_r_thr:
                continue
            d = math.hypot(o.fx - robot.x, o.fy - robot.y)
            if d > p.unc_d_gate:
                continue
            frac = min(1.0, (r - p.unc_r_thr) / max(1e-6, (p.unc_r_max - p.unc_r_thr)))
            scale = min(scale, 1.0 - p.unc_k * frac)
        if scale >= 1.0:
            return None
        return p.max_linear_velocity * scale

    # ------------------------------------------------------------------
    # C6 逃逸方向动态重校验（P3）
    # ------------------------------------------------------------------
    def revalidate_escape(self, robot, obstacles, goal, trap) -> dict:
        """逃逸激活期间用更长时距重校验冻结的逃逸方向；不安全则基于当前
        障碍状态重新生成逃逸子目标/方向（解决 S7 逃逸方向冻结过期）。

        返回更新后的 trap dict（含可能的 sub_goal / escape_heading 更新）。
        """
        if not self.enhanced_feature(self.params.enable_c6_escape_recheck):
            return trap
        eh = trap.get('escape_heading')
        if eh is None:
            return trap
        if self._escape_heading_safe(robot, obstacles, eh):
            return trap
        sg, eh2 = self.generate_local_goal_safe(robot, obstacles, goal)
        trap['sub_goal'] = sg
        trap['escape_heading'] = eh2
        return trap

    def _escape_heading_safe(self, robot, obstacles, heading: float) -> bool:
        """以 c6_escape_speed 沿 heading 滚动 c6_escape_horizon 秒，
        检查该方向对动态障碍预测位置的最小净空是否 >= c6_escape_clearance。"""
        p = self.params
        horizon = p.c6_escape_horizon
        n_steps = max(2, int(round(horizon / self.params.dt)))
        v = min(p.c6_escape_speed, p.max_linear_velocity)
        traj = self.cc.predict_trajectory(
            v, 0.0, robot.x, robot.y, robot.theta, self.params.dt, n_steps)
        th_min = p.c6_escape_clearance
        for o in obstacles:
            if o.speed <= 1e-6:
                continue
            for i in range(n_steps):
                t_i = (i + 1) * self.params.dt
                p_obs = self.predicted_obs_position(
                    o, i + 1, self.params.dt,
                    robot_position=(robot.x, robot.y))
                pose = traj[i:i + 1, :]
                d = self.cc.min_clearance_to_point(pose, p_obs[0], p_obs[1])
                if d < th_min:
                    return False
        return True

    # ------------------------------------------------------------------
    # C3：DWA* 前瞻 + U 形逃逸
    # ------------------------------------------------------------------
    def detect_trap(self, robot, obstacles, goal) -> bool:
        # 近目标闸门（几何通道，d_goal<trap_near_goal_m=1.0m 关闭几何 trap 检测）。
        # 注释一致性说明（P0-4b/U5）：stall 触发路径此前无近目标闸门，现已在
        # trial_runner.ClassicTrapController.step 中另加 d_goal<trap_near_goal_stall_m(默认1.5m)
        # 跳过 stall 触发；本函数几何闸门保持 1.0m 不变，两通道各自独立闸门、互不干扰。
        d_goal = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
        if d_goal < self.params.trap_near_goal_m:
            return False
        goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        th_block = self.params.trap_block_angle
        d_block = self.params.trap_block_range
        n_in = 0
        n_sector = 0
        for k in range(len(obstacles)):
            obs = obstacles[k]
            d = math.hypot(obs.fx - robot.x, obs.fy - robot.y)
            if d > d_block:
                continue
            n_in += 1
            ang = math.atan2(obs.fy - robot.y, obs.fx - robot.x)
            if abs(wrap_angle(ang - goal_ang)) < th_block:
                n_sector += 1
        return n_in >= self.params.trap_min_obs and n_sector == n_in

    def generate_local_goal(self, robot, obstacles, goal):
        """返回 (sub_goal, escape_heading)。"""
        nd = max(4, self.params.trap_scan_dirs)
        goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        kb = self.params.trap_goal_bias
        # [MTDWA P1] 近目标段：逃逸方向硬约束朝目标（治"接近→逃逸拉离→再接近"死循环）
        d_goal = math.hypot(goal[0] - robot.x, goal[1] - robot.y)
        near_goal = d_goal < self.params.trap_near_goal_constraint_m
        max_dev = self.params.trap_near_goal_ang
        best_d = -1.0
        best_val = -1.0
        best_ang = robot.theta
        for i in range(nd):
            ang = 2.0 * math.pi * float(i) / float(nd)
            if near_goal and abs(wrap_angle(ang - goal_ang)) > max_dev:
                continue
            d = self.free_distance(robot, obstacles, ang, self.params.trap_scan_range)
            align = max(math.cos(wrap_angle(ang - goal_ang)), 0.0)
            val = d * (1.0 + kb * align)
            if val > best_val:
                best_val = val
                best_d = d
                best_ang = ang
        if best_d < 0.0:
            # 近目标段全部候选被约束剔除 → 直接朝目标点推进
            dist_sub = max(0.2, min(0.6, self.params.trap_subgoal_dist))
            sub_goal = [robot.x + dist_sub * math.cos(goal_ang),
                        robot.y + dist_sub * math.sin(goal_ang)]
            return sub_goal, goal_ang
        dist_sub = max(0.2, min(best_d, self.params.trap_subgoal_dist))
        sub_goal = [robot.x + dist_sub * math.cos(best_ang),
                    robot.y + dist_sub * math.sin(best_ang)]
        return sub_goal, best_ang

    def generate_local_goal_safe(self, robot, obstacles, goal):
        """同 generate_local_goal，但候选方向按 val 降序逐个做 C6 动态预测
        安全性校验（_escape_heading_safe，2s 滚动净空 vs 预测障碍位置），返回
        第一个安全方向；全部不安全时回退最佳方向。

        解决 S9：反弹车同车道对向时，"沿车道向前"方向 free_distance 最大、
        但对向车 2s 内会占位 → 不安全 → 改选横向安全方向。
        """
        nd = max(4, self.params.trap_scan_dirs)
        goal_ang = math.atan2(goal[1] - robot.y, goal[0] - robot.x)
        # IFGM 效用评分（第8篇）：U = m1*d_gap + m2*(pi - |delta|)
        m1 = getattr(self.params, 'c6_ifgm_gap_w', 0.35)
        m2 = getattr(self.params, 'c6_ifgm_goal_w', 1.0)
        cands = []
        for i in range(nd):
            ang = 2.0 * math.pi * float(i) / float(nd)
            d = self.free_distance(robot, obstacles, ang, self.params.trap_scan_range)
            delta = abs(wrap_angle(ang - goal_ang))
            val = m1 * d + m2 * (math.pi - delta)
            cands.append((val, d, ang))
        cands.sort(key=lambda x: -x[0])
        for _val, d, ang in cands:
            if self._escape_heading_safe(robot, obstacles, ang):
                dist_sub = max(0.2, min(d, self.params.trap_subgoal_dist))
                sub = [robot.x + dist_sub * math.cos(ang),
                       robot.y + dist_sub * math.sin(ang)]
                return sub, ang
        _val, d, ang = cands[0]
        dist_sub = max(0.2, min(d, self.params.trap_subgoal_dist))
        sub = [robot.x + dist_sub * math.cos(ang),
               robot.y + dist_sub * math.sin(ang)]
        return sub, ang

    def generate_evasion_goal(self, robot, obstacles, goal):
        """C12 对向威胁专用横向避让子目标：沿威胁路径线的垂直方向平移
        c12_evade_offset 米（先取朝向目标一侧，空间不足再取另一侧），保留少量
        前进分量；返回 (sub_goal, escape_heading)。

        解决 S9：对向车同车道逼近时，自由空间扫描会选"远离目标的超长直道"
        （掉头向西）原地转圈被撞；横向平移直接"把车拉出对向车道"。
        """
        threat = self.headon_threat(robot, obstacles)
        if threat is None:
            return self.generate_local_goal(robot, obstacles, goal)
        off = getattr(self.params, 'c12_evade_offset', 1.1)
        fwd = getattr(self.params, 'c12_evade_forward', 0.4)
        # 威胁路径线法向（heading 垂直方向）
        nx = math.cos(threat.heading + math.pi / 2.0)
        ny = math.sin(threat.heading + math.pi / 2.0)
        sign = 1.0 if ((goal[1] - threat.fy) * ny >= 0.0) else -1.0
        ok = True
        for k in range(len(obstacles)):
            o = obstacles[k]
            d = math.hypot(o.fx - robot.x, o.fy - robot.y)
            if d < 1.5 * off + o.radius and d > 1e-6:
                if (o.fx - robot.x) * nx + (o.fy - robot.y) * ny > 0.0:
                    ok = False
                    break
        if not ok:
            sign = -sign
        sx = robot.x + sign * off * nx + fwd * math.cos(robot.theta)
        sy = robot.y + sign * off * ny + fwd * math.sin(robot.theta)
        eh = math.atan2(sy - robot.y, sx - robot.x)
        return [sx, sy], eh

    def free_distance(self, robot, obstacles, ang: float, max_range: float) -> float:
        d = max_range
        cos_a = math.cos(ang)
        sin_a = math.sin(ang)
        for k in range(len(obstacles)):
            obs = obstacles[k]
            dx = obs.fx - robot.x
            dy = obs.fy - robot.y
            proj = dx * cos_a + dy * sin_a
            if proj < 0.0:
                continue
            perp2 = dx * dx + dy * dy - proj * proj
            rsum = self.cc.inflation_radius + obs.radius + 0.05
            if perp2 > rsum * rsum:
                continue
            hit = proj - math.sqrt(max(rsum * rsum - perp2, 0.0))
            if hit < d:
                d = max(hit, 0.0)
        return d

    def escape_score(self, s, robot, sub_goal, obstacles) -> float:
        esc_ang = math.atan2(sub_goal[1] - robot.y, sub_goal[0] - robot.x)
        err = abs(wrap_angle(esc_ang - self.motion_heading(s, robot)))
        hp = 1.0 - err / math.pi

        pro = self.model.predict_position(robot, s.v, self._eff_omega(s), self.params.dt)
        dref = max(self.params.trap_subgoal_dist, 0.2)
        dp = max(0.0, 1.0 - math.hypot(sub_goal[0] - pro[0], sub_goal[1] - pro[1]) / dref)

        dmin = math.inf
        rc = self._vehicle_clearance_radius()
        for k in range(len(obstacles)):
            dmin = min(dmin, math.hypot(obstacles[k].fx - pro[0],
                                        obstacles[k].fy - pro[1]) - rc)
        if not obstacles:
            dmin = 1e3
        cref = self.cc.inflation_radius + self.params.obstacle_radius + 1.0
        cp = min(max(dmin, 0.0) / cref, 1.0)

        vp = min(s.v / max(self.params.max_linear_velocity, 1e-6), 1.0)

        return (self.params.alpha_h * hp + self.params.alpha_d * dp
                + self.params.alpha_c * cp + self.params.trap_v_weight * vp)

    # ------------------------------------------------------------------
    # [MTDWA P3] 行人横穿威胁分类器
    # ------------------------------------------------------------------
    def ped_cross_threat(self, robot, obstacles):
        """行人横穿威胁判定（P3）：慢速 + 横向运动带 + 路径紧迫交叉。

        返回应"减速让行"的行人障碍对象；无则 None。参数窄带设计（speed 上限、
        接近角 ∈ [ped_cross_ang_min, π-ped_cross_ang_min]、CPA 紧迫）使其只覆盖
        S11 式横穿行人，尽量不触碰 S10 路口横穿车（实验裁定回归与否）。
        """
        p = self.params
        if not self.enhanced_feature(p.enable_ped_cross_evade):
            return None
        rvx = robot.v * math.cos(robot.theta)
        rvy = robot.v * math.sin(robot.theta)
        hx, hy = math.cos(robot.theta), math.sin(robot.theta)
        for o in obstacles:
            if o.speed <= 1e-6 or o.speed > p.ped_cross_speed_max:
                continue
            vx = o.speed * math.cos(o.heading)
            vy = o.speed * math.sin(o.heading)
            dx = o.fx - robot.x
            dy = o.fy - robot.y
            if dx * hx + dy * hy <= 0.0:   # 必须在自车前方
                continue
            rel_ang = abs(wrap_angle(o.heading - robot.theta))
            if rel_ang < p.ped_cross_ang_min or rel_ang > math.pi - p.ped_cross_ang_min:
                continue                  # 非横向运动带（对向/同向）不归行人分类
            vrelx, vrely = vx - rvx, vy - rvy
            vrel2 = vrelx * vrelx + vrely * vrely
            if vrel2 < 1e-9:
                continue
            t_cpa = -((vrelx * dx + vrely * dy) / vrel2)
            if t_cpa < 0.0 or t_cpa > p.ped_cross_ttc:
                continue
            cpa = math.hypot(dx + t_cpa * vrelx, dy + t_cpa * vrely)
            if cpa >= p.ped_cross_cpa:
                continue
            return o
        return None

    def _yield_subgoal(self, robot, ped):
        """行人横穿让行子目标：向行人来向侧小偏移 + 少量前进（自然减速等待通过）。"""
        p = self.params
        nx, ny = -math.sin(robot.theta), math.cos(robot.theta)
        lat_obs = (ped.fx - robot.x) * nx + (ped.fy - robot.y) * ny
        sign = 1.0 if lat_obs >= 0.0 else -1.0   # 向来向侧（行人已离开的区域）偏移
        sx = robot.x + sign * p.ped_cross_offset * nx + p.ped_cross_fwd * math.cos(robot.theta)
        sy = robot.y + sign * p.ped_cross_offset * ny + p.ped_cross_fwd * math.sin(robot.theta)
        return [sx, sy], math.atan2(sy - robot.y, sx - robot.x)

    # ------------------------------------------------------------------
    # [MTDWA P2] 追尾带超车逃逸子目标
    # ------------------------------------------------------------------
    def _overtake_subgoal(self, robot, obstacles, threat):
        """追尾带（同向慢车堵前）超车逃逸：横向切到净空大的一侧 + 沿威胁 heading
        超前 c12_overtake_ahead 米（子目标放在威胁车辆前方，c12_persist 锁存
        直至超车完成）。被证伪的 v15_latesc_rear（纯垂直横移 299/420）证明
        追尾带不能只横移，必须"邻道超前"。"""
        p = self.params
        lat = p.c12_overtake_lateral
        ahead = p.c12_overtake_ahead
        cl_l = self.free_distance(robot, obstacles, robot.theta + math.pi / 2.0, 2.5)
        cl_r = self.free_distance(robot, obstacles, robot.theta - math.pi / 2.0, 2.5)
        sign = 1.0 if cl_l >= cl_r else -1.0
        nx, ny = -math.sin(robot.theta), math.cos(robot.theta)
        hx, hy = math.cos(threat.heading), math.sin(threat.heading)
        bx = threat.fx + ahead * hx
        by = threat.fy + ahead * hy
        sx = bx + sign * lat * nx
        sy = by + sign * lat * ny
        return [sx, sy], math.atan2(sy - robot.y, sx - robot.x)

    # ------------------------------------------------------------------
    # [MTDWA P5] 自适应横移量评估
    # ------------------------------------------------------------------
    def _adapt_evade_goal(self, robot, obstacles):
        """对向带垂直横移多候选评估：3 档 offset × 2 档 fwd × 2 侧，用未来
        c12_adapt_horizon 秒（直线滚动，含动态障碍预测位）的最小净空选优；
        净空平手取更小 offset / fwd（少挤邻道）。返回 (sx, sy)。"""
        p = self.params
        n_steps = max(2, int(round(p.c12_adapt_horizon / self.params.dt)))
        v_eval = min(p.c12_adapt_speed, p.max_linear_velocity)
        _hx, _hy = math.cos(robot.theta), math.sin(robot.theta)
        _nx, _ny = -_hy, _hx
        best_key = None
        best_sg = None
        for cand_sign in (1.0, -1.0):
            for off in p.c12_adapt_offsets:
                for fw in p.c12_adapt_fwds:
                    sx = robot.x + cand_sign * off * _nx + fw * _hx
                    sy = robot.y + cand_sign * off * _ny + fw * _hy
                    eh_c = math.atan2(sy - robot.y, sx - robot.x)
                    traj = self.cc.predict_trajectory(
                        v_eval, 0.0, robot.x, robot.y, eh_c, self.params.dt, n_steps)
                    clr = 1e9
                    for k in range(len(obstacles)):
                        o = obstacles[k]
                        if o.speed > 1e-6:
                            for i in range(n_steps):
                                p_o = self.predicted_obs_position(
                                    o, i + 1, self.params.dt,
                                    robot_position=(robot.x, robot.y))
                                d_i = self.cc.min_clearance_to_point(
                                    traj[i:i + 1, :], p_o[0], p_o[1])
                                if d_i < clr:
                                    clr = d_i
                        else:
                            clr = min(clr, self.cc.min_clearance(traj, [o]))
                    key = (clr, -off, -fw)
                    if best_key is None or key > best_key:
                        best_key = key
                        best_sg = (sx, sy)
        return best_sg

    # ------------------------------------------------------------------
    # C12 v15 逃逸子目标（原 four_wheel_trial.py 主循环内联块逐字节搬移）
    # ------------------------------------------------------------------
    def escape_subgoal(self, robot, obstacles, goal, trap, threat):
        """C3/C12 逃逸子目标生成：v15 perp/sigmoid 覆盖（C12 威胁激活且对应
        开关开时，用带 offset 的横向逃逸子目标替代 free_distance 自由方向扫描；
        开关默认 False → v21_s9band_c11 行为逐字节不变），否则回退 C6 安全
        重校验或经典自由方向扫描。返回 (sub_goal, escape_heading)。

        [MTDWA] P2：追尾带 → 邻道超前超车逃逸；P3：from_ped → 减速让行子目标；
        P5：perp 分支自适应横移候选评估。
        """
        p = self.params
        # [MTDWA 融合] g3：切入意图逃逸分支（优先于 P3/C12；切入车 rel_ang≈0
        # 不满足 C12 对向带门，需专用垂直横移）
        if trap.get('from_lateral', False) and getattr(p, 'enable_s9_lateral', False):
            lat_th = self.lateral_intent_threat(robot, obstacles)
            if lat_th is not None:
                trap['c12_band'] = 'lateral'
                return self._lateral_escape_subgoal(robot, obstacles, goal, lat_th)
            trap['from_lateral'] = False   # 切入威胁已离开 → 回退常规逃逸路径
        # [P3] 行人横穿让行分支（优先于 C12；横穿带本就不满足 C12 角度门）
        if trap.get('from_ped', False) and getattr(p, 'enable_ped_cross_evade', False):
            ped = self.ped_cross_threat(robot, obstacles)
            if ped is not None:
                return self._yield_subgoal(robot, ped)
            trap['from_ped'] = False   # 行人已离开 → 回退常规逃逸路径
        _v15_override = False
        if trap['from_c12'] and threat is not None and (
                getattr(p, 'c12_escape_perp', False)
                or getattr(p, 'c12_offset_sigmoid', False)
                or getattr(p, 'enable_c12_overtake', False)):
            _th_rel = abs(wrap_angle(threat.heading - robot.theta))
            if (getattr(p, 'enable_c12_overtake', False)
                    and p.c12_rear_ang > 0.0 and _th_rel < p.c12_rear_ang):
                # [P2] 追尾带：邻道超前超车逃逸（替代 perp 纯横移）
                trap['c12_band'] = 'rear'
                return self._overtake_subgoal(robot, obstacles, threat)
            trap['c12_band'] = 'headon'
            if getattr(p, 'c12_escape_perp', False):
                if (_th_rel > p.c12_headon_ang
                        or (getattr(p, 'c12_escape_perp_rear', False)
                            and p.c12_rear_ang > 0.0
                            and _th_rel < p.c12_rear_ang)):
                    _v15_override = True
            elif getattr(p, 'c12_offset_sigmoid', False):
                _v15_override = True
        if _v15_override:
            _hx = math.cos(robot.theta)
            _hy = math.sin(robot.theta)
            _nx, _ny = -_hy, _hx   # 左法向（heading + π/2）
            # 左右各扫 2.0m 静态净空，取净空大者
            _cl_l = self.free_distance(robot, obstacles, robot.theta + math.pi / 2.0, 2.0)
            _cl_r = self.free_distance(robot, obstacles, robot.theta - math.pi / 2.0, 2.0)
            _sign = 1.0 if _cl_l >= _cl_r else -1.0
            if getattr(p, 'c12_adapt_evade', False):
                # [P5] 自适应横移：多候选最小净空选优（含动态预测）
                _sx, _sy = self._adapt_evade_goal(robot, obstacles)
                return [_sx, _sy], math.atan2(_sy - robot.y, _sx - robot.x)
            if getattr(p, 'c12_offset_sigmoid', False):
                # h = 1 - cpa/c12_cpa_max（0=边缘0.5m，1=重合）
                _rvx = robot.v * math.cos(robot.theta)
                _rvy = robot.v * math.sin(robot.theta)
                _vx = threat.speed * math.cos(threat.heading)
                _vy = threat.speed * math.sin(threat.heading)
                _dx = threat.fx - robot.x
                _dy = threat.fy - robot.y
                _vrelx = _vx - _rvx
                _vrely = _vy - _rvy
                _vrel2 = _vrelx * _vrelx + _vrely * _vrely
                if _vrel2 > 1e-9:
                    _tcpa = -((_vrelx * _dx + _vrely * _dy) / _vrel2)
                    _cpa = math.hypot(_dx + _tcpa * _vrelx, _dy + _tcpa * _vrely)
                else:
                    _cpa = 0.0
                _h = max(0.0, min(1.0, 1.0 - _cpa / p.c12_cpa_max))
                _off = (p.c12_offset_min
                        + (p.c12_evade_offset - p.c12_offset_min)
                        / (1.0 + math.exp(-p.c12_offset_kappa * (_h - 0.5))))
            else:
                _off = p.c12_evade_offset   # latesc 仍 1.1m
            _fwd = p.c12_evade_forward
            _sx = robot.x + _sign * _off * _nx + _fwd * _hx
            _sy = robot.y + _sign * _off * _ny + _fwd * _hy
            return [_sx, _sy], math.atan2(_sy - robot.y, _sx - robot.x)
        elif p.enable_c6_escape_recheck:
            trap['c12_band'] = 'none'
            return self.generate_local_goal_safe(robot, obstacles, goal)
        else:
            trap['c12_band'] = 'none'
            return self.generate_local_goal(robot, obstacles, goal)
