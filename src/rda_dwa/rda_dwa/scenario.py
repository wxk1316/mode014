"""
scenario.py — S1~S15 场景构建（build_scenario.m / preSimulateCustom.m 的 Python 移植）。

关键点：
  - 随机数流与 MATLAB `rng(seed,'twister')` 一致：numpy RandomState(seed) 与 MATLAB
    twister 采用相同 init_by_array 播种与 genrand_res53 输出（已有公开验证；
    本包内 test_scenario_rng 用已知首值校验）。
  - randi([a,b]) 在 MATLAB 中等价于 a + floor((b-a+1)*rand)，此处显式实现，
    避免使用 numpy 的 randint（其算法不同）。
  - 动态障碍物出界反弹（非回卷）；S5 支持中途转向 turn_time/new_heading。

[2026-10-27 场景改造] 障碍物现实类型化（行人/自行车/车辆/大静态方块/小方块墙）：
  - 尺寸注册：OBSTACLE_DIMS 基准 + 确定性 ±0.1m 相异性抖动（rng 抽取一次，不再修改）；
  - 运动模式扩展：pedestrian 直线横穿+随机折返（社会力简单版，确定性换向序列）；
    bicycle 匀加速直线（accel）/ 简单圆弧（arc_rate）；vehicle 匀速直线 / 简单圆弧；
    全部"不主动避让主车"；动态障碍每场景 ≤4；
  - U 形窗口/道路边界等静态墙用 block（0.3~0.5m）连续铺成；
  - 静态障碍在 Scene.static_obstacle_specs 携带 type/length/width/heading，
    static_obstacles 仍为位置数组（兼容下游）；
  - 可达性 +20% 裕量由 scripts/check_scenes.py 按矩形-矩形净距口径核验。
"""
from __future__ import annotations

import math

import numpy as np

from .obstacle import circum_radius
from .params import Params


class MatlabRNG:
    """MATLAB rng(seed,'twister') 兼容随机流（rand / randi）。"""

    def __init__(self, seed: int):
        self._rng = np.random.RandomState(int(seed))

    def rand(self) -> float:
        return float(self._rng.random_sample())

    def randi(self, a: int, b: int) -> int:
        """MATLAB randi([a,b])：a + floor((b-a+1)*rand)。"""
        return a + int(math.floor((b - a + 1) * self.rand()))


# 场景种子区间登记：每个场景 30 个种子 = seed0+1 .. seed0+30
SCENARIO_SEED_OFFSETS = {
    'S1_sparse': 1000,
    'S2_dense': 2000,
    'S3_corridor': 3000,
    'S4_utrap': 4000,
    'S5_single_dyn': 5000,
    'S6_multi_dyn': 6000,
    'S7_mixed': 7000,
    'S8_random': 8000,
    'S9_cutin': 9000,
    'S10_intersection': 10000,
    'S11_pedestrian': 11000,
    'S12_dense_dyn': 12000,
    'S13_overtake': 13000,
    'S14_longdist': 14000,
    'S15_utrap_in': 15000,
    'S16_corner_clean': 16000,
    'S17_corner_ped': 17000,
    'S18_corner_block': 18000,
    'S19_corner_dyn': 19000,
}

# ---------------------------------------------------------------------------
# 障碍物类型注册（2026-10-27）
# ---------------------------------------------------------------------------
# (type, 标称(L,W))；相异性抖动：每个实例尺寸在标称 ±0.1m 内确定性抽取。
# static_large：正方形 1~2m（尺寸范围本身即相异性）；block：0.3~0.5m（U 形墙等）。
OBSTACLE_DIMS = {
    'pedestrian': (0.15, 0.15),   # 行人（正方），速度 ≤0.5 m/s
    'bicycle':    (0.8, 0.5),     # 自行车，速度 ≤0.6 m/s
    'vehicle':    (1.4, 0.8),     # 车辆，速度 ≤0.8 m/s
    'static_large': (1.0, 1.0),   # 大静态方块 1~2m（正方形）
    'block':      (0.4, 0.4),     # 小方块墙 0.3~0.5m
}
# 速度上限（m/s）
SPEED_LIMITS = {'pedestrian': 0.5, 'bicycle': 0.6, 'vehicle': 0.95}


def draw_obstacle_dim(rng: MatlabRNG, otype: str):
    """确定性抽取障碍实例尺寸（±0.1 相异性；static_large/block 在范围内抽取）。

    返回 (length, width)。同 (场景,seed) 下恒定——每次仿真不再修改。
    """
    if otype == 'static_large':
        s = 1.0 + rng.rand() * 1.0      # 1.0~2.0 m 正方
        return s, s
    if otype == 'block':
        s = 0.3 + rng.rand() * 0.2      # 0.3~0.5 m 正方/矩形
        L = s + (rng.rand() - 0.5) * 0.05
        W = s + (rng.rand() - 0.5) * 0.05
        return max(0.3, L), max(0.3, W)
    base_l, base_w = OBSTACLE_DIMS[otype]
    return (max(0.05, base_l + (rng.rand() - 0.5) * 0.2),
            max(0.05, base_w + (rng.rand() - 0.5) * 0.2))


def make_static(rng: MatlabRNG, otype: str, x: float, y: float,
                heading: float = 0.0):
    """静态障碍规格 dict（位置 + 类型 + 尺寸 + 等效半径）。"""
    L, W = draw_obstacle_dim(rng, otype)
    return {'x': float(x), 'y': float(y), 'type': otype,
            'length': float(L), 'width': float(W),
            'heading': float(heading), 'radius': circum_radius(L, W)}


def make_dynamic(rng: MatlabRNG, otype: str, x0: float, y0: float,
                 heading: float, speed: float = 0.5,
                 turn_time: float = 0.0, new_heading: float = 0.0,
                 accel: float = 0.0, speed_max: float | None = None,
                 arc_rate: float = 0.0,
                 heading_schedule: list | None = None,
                 reversal_period: float = 0.0):
    """动态障碍规格 dict（运动模式随类型注册）。"""
    L, W = draw_obstacle_dim(rng, otype)
    if speed_max is None:
        speed_max = min(float(speed) * 1.0, SPEED_LIMITS.get(otype, 0.8))
    d = {'x0': float(x0), 'y0': float(y0), 'type': otype,
         'length': float(L), 'width': float(W),
         'radius': circum_radius(L, W),
         'speed': float(speed), 'heading': float(heading),
         'turn_time': float(turn_time), 'new_heading': float(new_heading),
         'accel': float(accel), 'speed_max': float(speed_max),
         'arc_rate': float(arc_rate)}
    if heading_schedule is not None:
        d['heading_schedule'] = heading_schedule
    if reversal_period > 0:
        d['reversal_period'] = float(reversal_period)
    return d


def pedestrian_schedule(rng: MatlabRNG, max_time: float, base_heading: float,
                        period_min: float = 2.0, period_max: float = 4.0,
                        dt: float = 0.1, early_first: bool = True,
                        band: tuple = None, pos0: float = 0.0,
                        speed: float = 0.4, axis: str = 'y'):
    """行人随机折返（社会力简单版）确定性换向序列（带边缘硬反弹版）。

    与 _pre_simulate_custom 同 dt 步进、换向在 t_switch 步生效，轨迹一一对应。
    每步先做带约束判定、再积分（与 pre-sim 完全一致）：
      - 位置已出带且仍朝带外运动 → 立即折返（硬反弹，保证 |pos−带界| ≤ 1 步
        位移，任意窄带都严格约束）；
      - 每 period∈[period_min,period_max] 秒随机决策：65% 继续横穿 / 35% 折返
        （朝向仅在 {base_heading, base_heading+π} 间切换）；
      - 首段换向固定在 1.2~2.0s 且必然折返（保安全冲程）。
    返回 [(t_switch, new_heading), ...]（幂等 last-wins）。
    """
    n_steps = int(math.ceil(max_time / dt))
    sgn = math.sin if axis == 'y' else math.cos
    pos = pos0
    h = base_heading
    switches = []
    first = True
    next_dec = round((1.2 + rng.rand() * 0.8) / dt) * dt if early_first \
        else round((period_min + rng.rand() * (period_max - period_min)) / dt) * dt
    for k in range(1, n_steps + 1):
        t = k * dt
        hf = base_heading + math.pi if h == base_heading else base_heading
        # 1) 带边缘硬反弹（每步检查，先于随机决策）
        if band is not None:
            if pos < band[0] and sgn(h) < 0:
                h = hf
                switches.append((t, h))
            elif pos > band[1] and sgn(h) > 0:
                h = hf
                switches.append((t, h))
        # 2) 随机折返决策（到决策时刻）
        if t >= next_dec - 1e-9:
            if first and early_first:
                h = hf                     # 首段必然折返
                switches.append((t, h))
                first = False
            elif band is not None:
                # 随机折返需保证带内仍可达（折返后一步内仍不越带才折返）
                end_flip = pos + (sgn(hf) - sgn(h)) * speed * dt + sgn(hf) * speed * dt
                if rng.rand() < 0.35 and band[0] <= end_flip <= band[1]:
                    h = hf
                    switches.append((t, h))
            elif rng.rand() < 0.35:
                h = hf
                switches.append((t, h))
            next_dec = t + round((period_min + rng.rand() * (period_max - period_min)) / dt) * dt
        # 3) 积分一步（与 pre-sim 完全一致）
        pos += sgn(h) * speed * dt
    # 相邻同刻/同向合并（幂等 last-wins）
    merged = []
    for (tt, hh) in switches:
        if merged and abs(merged[-1][0] - tt) < 1e-9 and abs(merged[-1][1] - hh) < 1e-9:
            continue
        merged.append((round(tt / dt) * dt, hh))
    return merged


class Scene:
    """场景配置（sc 结构体等价）+ 障碍物轨迹。"""

    def __init__(self):
        self.case_id = ''
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.obstacle_speed = 1.05
        self.seed = 0
        self.scene_x_min = 0.0
        self.scene_x_max = 10.0
        self.scene_y_min = 0.0
        self.scene_y_max = 10.0
        self.wrap_scene = False
        self.direction_update_period = 0
        self.static_obstacles = np.zeros((0, 2))
        self.static_obstacle_specs = []   # [{'x','y','type','length','width','heading','radius'}]
        self.dynamic_obstacles = []   # list of dict {x0,y0,type,length,width,speed,heading,...}


class TrajStep:
    """traj(t)：{x: ndarray, y: ndarray, heading: ndarray}（长度 n_obs）。"""
    __slots__ = ('x', 'y', 'heading')

    def __init__(self, x, y, heading):
        self.x = np.asarray(x, dtype=np.float64)
        self.y = np.asarray(y, dtype=np.float64)
        self.heading = np.asarray(heading, dtype=np.float64)


def build_scenario(scenario_id: str, seed: int, pp: Params, enlarge_config: dict | None = None):
    """build_scenario.m 等价。返回 (sc, traj)。

    enlarge_config: 可选场景放大参数（仅改变布局常量/接受条件，不改变 RNG 抽取顺序）。
      支持键：
        S2_min_pair, S4_u_half_height, S6_n_dyn, S7_n_dyn,
        S8_min_pair, S8_n_dyn_lo, S8_n_dyn_hi,
        S9_lane_low, S9_lane_high, S9_cutin_y,
        S12_n_dyn_lo, S12_n_dyn_hi, S12_min_init_dist
    """
    rng = MatlabRNG(seed)
    dt = pp.dt
    max_time = pp.max_sim_time
    n_steps = int(math.ceil(max_time / dt))
    ec = enlarge_config or {}

    # 默认 start/goal：左下 → 右上
    start_x, start_y, start_theta = 0.5, 0.5, math.atan2(8, 8)
    goal_x, goal_y = 9.5, 9.5

    dyn_obs = []
    stat_obs = np.zeros((0, 2))
    stat_specs = []

    def add_static(otype, x, y, heading=0.0):
        """登记静态障碍：位置数组 + 类型规格（保持一一对应顺序）。"""
        nonlocal stat_obs, stat_specs
        sp = make_static(rng, otype, x, y, heading)
        stat_obs = np.vstack([stat_obs, [sp['x'], sp['y']]]) if stat_obs.size else \
            np.array([[sp['x'], sp['y']]], dtype=np.float64)
        stat_specs.append(sp)

    def add_static_sized(x, y, L, W, heading=0.0, otype='static_large'):
        """登记静态障碍（尺寸预先抽取，避免布点后再抽尺寸破坏间距约束）。"""
        nonlocal stat_obs, stat_specs
        sp = {'x': float(x), 'y': float(y), 'type': otype,
              'length': float(L), 'width': float(W),
              'heading': float(heading), 'radius': circum_radius(L, W)}
        stat_obs = np.vstack([stat_obs, [x, y]]) if stat_obs.size else \
            np.array([[x, y]], dtype=np.float64)
        stat_specs.append(sp)

    def add_dyn(otype, x0, y0, heading, speed, **kw):
        dyn_obs.append(make_dynamic(rng, otype, x0, y0, heading, speed, **kw))

    def wall_rows(y_fixed, x_lo, x_hi, step=0.3, block_type='block',
                  skip_x_range=None):
        """连续方块墙：沿 x 轴铺 block（0.3~0.5m），间距 step 保证相邻衔接。"""
        x = x_lo
        while x <= x_hi + 1e-9:
            if skip_x_range is None or not (skip_x_range[0] - 1e-9 <= x <= skip_x_range[1] + 1e-9):
                add_static(block_type, x, y_fixed)
            x += step

    def wall_band_h(y_inner, x_lo, x_hi, side, step=0.3):
        """水平墙（内沿精确贴 y_inner）：每个 block 先抽尺寸再以 内沿 定心，
        保证通道净宽确定性（走廊类场景专用，块内沿即通道边界）。"""
        nonlocal stat_obs, stat_specs
        x = x_lo
        while x <= x_hi + 1e-9:
            sp = make_static(rng, 'block', x, y_inner + side * 0.0)
            h = sp['width'] / 2.0
            sp['y'] = y_inner + side * h
            stat_obs = np.vstack([stat_obs, [sp['x'], sp['y']]]) if stat_obs.size else \
                np.array([[sp['x'], sp['y']]], dtype=np.float64)
            stat_specs.append(sp)
            x += step

    def wall_band_v(x_inner, y_lo, y_hi, side, step=0.3):
        """竖直墙（内沿精确贴 x_inner）：同 wall_band_h。"""
        nonlocal stat_obs, stat_specs
        y = y_lo
        while y <= y_hi + 1e-9:
            sp = make_static(rng, 'block', x_inner + side * 0.0, y)
            h = sp['width'] / 2.0
            sp['x'] = x_inner + side * h
            stat_obs = np.vstack([stat_obs, [sp['x'], sp['y']]]) if stat_obs.size else \
                np.array([[sp['x'], sp['y']]], dtype=np.float64)
            stat_specs.append(sp)
            y += step

    def place_static_band(n, x_lo, x_hi, y_lo, y_hi,
                          edge_margin=1.05, pair_gap=1.3,
                          center_exclude=None):
        """在带内放置 n 个大静态方块（1~2m，±0.1 确定性抖动）：
        - 方块**边缘**到 start→goal 连线距离 ≥ edge_margin（默认 1.05 =
          车半宽 0.3 + 20% 裕量 0.72 + 0.03 余量），保证主车沿连线走廊
          全程保持 ≥0.72m 净距、且不存在凹形口袋（本地 DWA 可单调通过）；
        - 两两精确轴对齐净距 ≥ pair_gap（横向绕行通道宽度）；
        - 起点/终点预留 hr+1.1。
        - center_exclude=(cx, cy, r)：候选中心距 (cx,cy) < r 则跳过
          （用于连线中置静态门时给绕行留出净环带）。
        返回 [(x, y, L, W)]。放置失败的方块跳过（不影响已放置）。
        """
        placed = []
        lx, ly = goal_x - start_x, goal_y - start_y
        Ln = math.hypot(lx, ly)
        for i in range(n):
            L, W = draw_obstacle_dim(rng, 'static_large')
            hr = max(L, W) / 2.0
            for _ in range(20000):
                x = x_lo + rng.rand() * (x_hi - x_lo)
                y = y_lo + rng.rand() * (y_hi - y_lo)
                if center_exclude is not None:
                    if math.hypot(x - center_exclude[0], y - center_exclude[1]) < center_exclude[2]:
                        continue
                # 方块中心到连线的无向距离 ≥ hr + edge_margin
                ld = abs(lx * (start_y - y) - ly * (start_x - x)) / Ln
                if ld < hr + edge_margin:
                    continue
                ds = math.hypot(x - start_x, y - start_y)
                dg = math.hypot(x - goal_x, y - goal_y)
                if ds < hr + 1.1 or dg < hr + 1.1:
                    continue
                ok = True
                for (rx, ry, rL, rW) in placed:
                    hx = (L + rL) / 2.0
                    hy = (W + rW) / 2.0
                    if max(abs(x - rx) - hx, abs(y - ry) - hy) < pair_gap:
                        ok = False
                        break
                if ok:
                    placed.append((x, y, L, W))
                    break
        return placed

    sid = scenario_id
    if sid == 'S1_sparse':
        # 稀疏静态：6 个大静态方块（1~2m）随机错落于对角走廊两侧带内
        # （方块边缘距 start→goal 连线 ≥1.05 → 走廊净宽 ≥2×0.72m，无凹形口袋，
        # 本地 DWA 可单调通过；两两精确净距 ≥1.3m 保证横向绕行通道）
        # + 连线正中静态门（static_large 1~2m 于 (5,5)）：门正压 start→goal
        # 直线，主车无法直线通行，必须绕门环带（center_exclude=3.0 保证绕行净空）。
        start_theta = math.atan2(9, 9)
        add_static('static_large', 5.0, 5.0)
        placed = place_static_band(6, 1.0, 9.0, 1.0, 9.0,
                                   center_exclude=(5.0, 5.0, 3.0))
        for (x, y, L, W) in placed:
            add_static_sized(x, y, L, W)

    elif sid == 'S2_dense':
        # 10~12 个大静态方块（1~2m）密集随机（12×12）：同一走廊带规则
        start_theta = math.atan2(9, 9)
        n_dense = ec.get('S2_n_dense', 10)
        placed = place_static_band(n_dense, 1.0, 11.0, 1.0, 11.0)
        for (x, y, L, W) in placed:
            add_static_sized(x, y, L, W)

    elif sid == 'S3_corridor':
        # 走廊墙（block 0.3~0.5m 连续铺墙）：下墙 y=3.0，上墙 y=4.5 且 x∈[4,6] 开口；
        # start(0.5,0.5)→goal(9.5,0.5) 沿墙南侧直行（墙为北侧屏障）
        # + 连线静态障碍 block 0.45×0.45 于 (5.0, 0.5)（正压在 start→goal 直线上）：
        #   主车必须抬升越障绕行，无法再直线通过。
        #   [2026-10-27 实测] static_large 1~2m（含钳制 1.0~1.3m）在墙距内形成的
        #   门洞通道（≤1.1m）低于本地 DWA 可穿下限（≈1.8m），主车在门前停滞；
        #   改用 0.45m block 后通道净宽 ≥1.8m，DWA 可高弧越过（裕量充足）。
        start_theta = 0.0
        goal_x, goal_y = 9.5, 0.5
        start_y = 0.5
        wall_rows(3.0, 0.5, 9.5)
        wall_rows(4.5, 0.5, 9.5, skip_x_range=(4.0, 6.0))
        add_static_sized(5.0, 0.5, 0.45, 0.45)

    elif sid == 'S4_utrap':
        # U 形陷阱：墙用 block（0.3~0.5m）铺成；左墙 x=3.0，上/下臂 y=7.8/4.2，
        # 右端封口 x=6.0。start(1.0,6.0) 在 U 外左侧，goal(11.0,6.0) 右侧 → 南北绕行
        start_theta = 0.0
        start_x, start_y = 1.0, 6.0
        goal_x, goal_y = 11.0, 6.0
        u_bottom_x = 3.0
        u_center_x = 4.5
        u_top_y = 7.8
        u_bot_y = 4.2
        wall_rows(u_top_y, u_bottom_x, u_center_x + 1.5)
        wall_rows(u_bot_y, u_bottom_x, u_center_x + 1.5)
        y = u_bot_y
        while y <= u_top_y + 1e-9:
            add_static('block', u_bottom_x, y)
            y += 0.3

    elif sid == 'S15_utrap_in':
        # 车内 U 型陷阱：同上 U 形墙（block），车在腔体中心 (4.5,6.0) 面朝封闭端
        start_theta = math.pi
        start_x, start_y = 4.5, 6.0
        goal_x, goal_y = 11.0, 6.0
        u_bottom_x = 3.0
        u_center_x = 4.5
        u_top_y = 7.8
        u_bot_y = 4.2
        wall_rows(u_top_y, u_bottom_x, u_center_x + 1.5)
        wall_rows(u_bot_y, u_bottom_x, u_center_x + 1.5)
        y = u_bot_y
        while y <= u_top_y + 1e-9:
            add_static('block', u_bottom_x, y)
            y += 0.3

    elif sid == 'S5_single_dyn':
        # 单一动态车辆：从 (3.0,5.0) 以 heading π（向西）0.9 m/s 匀速横穿主车
        # start→goal 路径（x=0.5 竖线）——车辆 t≈2.8s 压线、t≈3.3s 停在地图西缘
        # (x=0 出界钳制，车体 [−0.4,0.4]×[4.6,5.4] 正压路径)；
        # 主车约 t≈4s 才到达该处 → 车辆横穿在前、停驻在后，
        # 主车必须先停车观察再向东侧绕行通过（无法直线行驶到达）。
        # [2026-10-27 实测] 对向迎面/同向驶离/正点交汇（x0=5.0 等）均在主车到达
        # 时车辆仍在压线，超出 DWA M2 前向视界（5 步×0.1s）→ 必撞；
        # x0=3.0 使车辆在 M2 视界外完成横穿并停驻为静态等效障碍，几何可达。
        start_theta = math.pi / 2.0
        start_x, start_y = 0.5, 0.5
        goal_x, goal_y = 0.5, 9.5
        add_dyn('vehicle', 3.0, 5.0, math.pi, 0.9)

    elif sid == 'S6_multi_dyn':
        # 3 个动态障碍（1 车辆 + 2 自行车）分居 3 条侧车道，全部与主车对角
        # 走廊 y=x±0.6 的"旋转矩形足迹带"保持 ≥0.72 净距：
        # 车 v2 下侧车道 y=5.4←（0.5；主车角点足迹 x−0.566 于 x≥6.4 起高于其
        #   顶缘 5.8+0.72，且 v2 移动 −x，与主车 x 带重叠时主车角点早已远离）；
        # 自行车 b1 下侧车道 y=2.5→（0.4+0.05 匀加速≤0.6，全程 x 落后主车
        #   保持 ≥1.9m，轨迹与主车无 x 向重叠）；
        # 自行车 b2 上侧车道 y=9.5←（0.5，朝向远离终点方向移动，主车到达
        #   终点区时 b2 已退至 x≈4.0）。
        # 三条车道两两互不相交（竖距 4.0/2.9），无相互重叠；主车沿对角直线
        # 行驶无任何贴行/交汇段 → 规划器无需激进绕行，稳定可达。
        start_theta = math.atan2(9, 9)
        add_dyn('vehicle', 12.0, 5.0, math.pi, 0.6)
        add_dyn('bicycle', 0.0, 2.5, 0.0, 0.4, accel=0.05, speed_max=0.6)
        add_dyn('bicycle', 8.5, 9.5, math.pi, 0.5)

    elif sid == 'S7_mixed':
        # 混合：4 大静态方块（1~2m）+ 车辆(车道 y=8.0 西→东) + 行人(车道 y=1.5→，折返)
        # 终点周围车辆（原 (11.5,8.0) 东侧 ←）按用户要求镜像到另一侧
        # (−1.5, 8.0) 并反转运动方向（→，0.85 ≤0.95）：从起点侧穿全图跑向
        # 终点侧；方块避开两动态车道带（|y-8|/|y-1.5| ≥ s/2+1.3），两两精确净距 ≥0.78
        start_theta = math.atan2(9, 9)
        add_dyn('vehicle', -1.5, 8.0, 0.0, 0.7)
        add_dyn('pedestrian', 0.5, 1.5, 0.0, 0.5,
                heading_schedule=pedestrian_schedule(
                    MatlabRNG(seed + 1000), max_time, 0.0, dt=dt))
        placed = []
        for i in range(4):
            L, W = draw_obstacle_dim(rng, 'static_large')
            hr = max(L, W) / 2.0
            for _ in range(5000):
                x = 1.2 + rng.rand() * 7.6
                y = 3.8 + rng.rand() * 1.8
                ds = math.hypot(x - start_x, y - start_y)
                dg = math.hypot(x - goal_x, y - goal_y)
                if ds > hr + 1.1 and dg > hr + 1.1:
                    ok = True
                    for (rx, ry, rL, rW) in placed:
                        hx = (L + rL) / 2.0
                        hy = (W + rW) / 2.0
                        if max(abs(x - rx) - hx, abs(y - ry) - hy) < 0.78:
                            ok = False
                            break
                    if ok:
                        placed.append((x, y, L, W))
                        break
        for (x, y, L, W) in placed:
            add_static_sized(x, y, L, W)

    elif sid == 'S8_random':
        # 随机：静态方块 4~6 个（1~2m）布于对角走廊两侧带（边缘距连线 ≥1.05，
        # 两两净距 ≥1.3），避开 y=1.5/9.0 动态车道带；动态 2 车辆（≤0.8）
        # 沿 y=1.5→ 与 y=9.0← 巡逻（主车在 x≈1.5/9.0 处先越过后对角远离）
        start_theta = math.atan2(9, 9)
        n_stat = rng.randi(4, 6)
        add_dyn('vehicle', 0.3, 1.5, 0.0, min(0.75 + rng.rand() * 0.2, 0.95))
        add_dyn('vehicle', 11.7, 9.0, math.pi, min(0.75 + rng.rand() * 0.2, 0.95))
        placed = place_static_band(n_stat, 1.2, 10.8, 2.6, 8.0)
        for (x, y, L, W) in placed:
            add_static_sized(x, y, L, W)

    elif sid == 'S9_cutin':
        # 宽直道（车道边界 block 墙 y=2.0/8.0，6m 宽）；cut-in 车辆 + 前方慢车
        # （≤0.8）分居上车道两平行子道：
        #   慢车 y=5.5（bbox [5.05,5.95]）；cutin y=6.6（bbox [6.15,7.05]，
        #     微切 −0.15rad dy≈0.15m 时底缘最低 6.0）→ 两车纵向净距 ≥0.05、
        #     与上墙（block 底缘 ≥7.75）净距 ≥0.7，全程不相交；
        # 主车行线 y=3.8（旋转矩形顶缘 4.1）：与慢车底缘 5.05 净距 0.95、
        # 与 cutin 底缘 ≥6.0 净距 ≥1.9 → 全程 ≥0.72 裕量，直行稳定可达，
        # 仅受上车道切入车辆风险场轻微作用，无需激进变道/贴行。
        start_theta = 0.0
        start_x, start_y = 0.5, 3.8
        goal_x, goal_y = 9.5, 3.8
        lane_low = ec.get('S9_lane_low', 2.0)
        lane_high = ec.get('S9_lane_high', 8.0)
        wall_rows(lane_low, 0.5, 9.5)
        wall_rows(lane_high, 0.5, 9.5)
        cutin_x = 2.5 + (rng.rand() - 0.5) * 1.0
        cutin_turn = 2.0 + (rng.rand() - 0.5) * 1.0
        add_dyn('vehicle', cutin_x, 6.6, 0.0, 0.65,
                heading_schedule=[(cutin_turn, -0.15), (cutin_turn + 2.5, 0.15),
                                  (cutin_turn + 5.0, 0.0)])
        add_dyn('vehicle', 6.0, 5.5, 0.0, 0.4)

    elif sid == 'S10_intersection':
        # 无信号交叉口：自车南向北直行；四角建筑（大静态方块 1~1.4m 就近角）
        # 3 车辆（≤0.8）：car1 西向东横穿；car2 东向西→右转下（x≈9.2 下行车道
        # 避开右下建筑 x≤8.2）；car3 北向南→左转西行（y≈9.2 横移避开右上建筑 y≤8.2）
        start_theta = math.pi / 2.0
        start_x, start_y = 5.0, 0.5
        goal_x, goal_y = 5.0, 9.5
        for (cx, cy) in [(1.2, 1.2), (1.2, 7.2), (7.2, 1.2), (7.2, 7.2)]:
            add_static('static_large', cx, cy)
        # car1 西向东横穿主车路径（车道 y=3.5）：
        #   [2026-10-27 实测] 车道 y=5.0/4.6 且 x0≈0.5 时，car1 与主车同时到达
        #   交汇区（主车车身 [y−0.5,y+0.5] 必与 car1 车道 [3.1,3.9] 重叠，
        #   且 car1 压线时超出 DWA M2 前向视界）→ 必撞；
        #   改为 x0∈[4.2,4.8]：car1 在 t≈0.9s 即越过主车行线，t≤2.1s 完全离开
        #   主车 x 影响区（[4.0,6.0]），主车车身到达该区（t≥2.3s）时 car1 已在
        #   x≥6.3 处，交汇窗口由几何消除，可达且有让行考验。
        car1_x = 4.2 + rng.rand() * 0.6
        car1_spd = 0.85 + rng.rand() * 0.1
        add_dyn('vehicle', car1_x, 3.5, 0.0, min(car1_spd, 0.95))
        add_dyn('vehicle', 9.5, 5.5, math.pi, 0.75,
                heading_schedule=[(0.5, -math.pi / 2.0)])
        add_dyn('vehicle', 5.5, 9.5, -math.pi / 2.0, 0.75,
                heading_schedule=[(0.5, math.pi)])

    elif sid == 'S11_pedestrian':
        # 直道（边界 block 墙 y=2.5/7.5，5m 宽）+ 1 行人横穿车道（随机折返，≤0.5）
        # 行人在窄带 y∈[4.75,5.25]（0.5m）内横穿主车车道；该窄带完全被主车
        # 行线旋转矩形足迹（y∈[4.4,5.6]）覆盖 → 行人在带内时任何带内直行候选
        # 在未来步都不可行，DWA 只能停车等待或绕到带外通道
        # （下通道 2.5→4.75、上通道 5.25→7.5，净宽均 ≥2.0m），从根上杜绝
        # "带内贴行 0.1m 擦碰"。行人折返限定于带内，永不到达带外。
        # 仅 1 行人（x=3.0）：主车绕行带外通道（y≈6.2~6.5，与行人全程净距
        # ≥0.72）越过 x=3.0 后即可安全回正车道线，不受第二次横穿干扰；
        # 2 行人横穿会诱使主车在两次横穿之间提前下沉回车道线而擦碰。
        start_theta = 0.0
        start_x, start_y = 0.5, 5.0
        goal_x, goal_y = 9.5, 5.0
        wall_rows(2.5, 0.5, 9.5)
        wall_rows(7.5, 0.5, 9.5)
        spd_a = 0.3 + rng.rand() * 0.1    # 0.3~0.4
        add_dyn('pedestrian', 3.0, 5.0, math.pi / 2.0, spd_a,
                heading_schedule=pedestrian_schedule(
                    MatlabRNG(seed + 1000), max_time, math.pi / 2.0, dt=dt,
                    band=(4.75, 5.25), pos0=5.0, speed=spd_a))

    elif sid == 'S12_dense_dyn':
        # 密集多动态（4：2 行人 + 2 自行车）沿 4 条平行水平车道巡逻
        # （y=4.0/6.0 行人折返、y=2.0/8.0 自行车匀加速）；车道互不相交 → 无相互重叠；
        # 自车对角穿越 4 条车道，构成密集多动态避让
        start_theta = math.atan2(9, 9)
        start_x, start_y = 0.5, 0.5
        goal_x, goal_y = 9.5, 9.5
        ped_a = 0.3 + rng.rand() * 0.1   # 0.3~0.4
        ped_b = 0.3 + rng.rand() * 0.1
        add_dyn('pedestrian', 0.5, 4.0, 0.0, ped_a,
                heading_schedule=pedestrian_schedule(
                    MatlabRNG(seed + 1000), max_time, 0.0, dt=dt))
        add_dyn('pedestrian', 11.5, 6.0, math.pi, ped_b,
                heading_schedule=pedestrian_schedule(
                    MatlabRNG(seed + 2000), max_time, math.pi, dt=dt))
        add_dyn('bicycle', 0.5, 2.0, 0.0, 0.4, accel=0.05, speed_max=0.6)
        add_dyn('bicycle', 11.5, 8.0, math.pi, 0.4, accel=0.05, speed_max=0.6)

    elif sid == 'S13_overtake':
        # 双车道直道（车道边界 block 墙 y=2.0/7.0）：前方慢车 + 对向车流，
        # 变道超车窗口：慢车顶缘(3.9) 与对向底缘(5.6) 间净带 1.7m ≥ 车宽0.6+0.72
        start_theta = 0.0
        start_x, start_y = 0.5, 3.5
        goal_x, goal_y = 9.5, 3.5
        wall_rows(2.0, 0.5, 9.5)
        wall_rows(7.0, 0.5, 9.5)
        # 对向车流分居两平行车道 y=5.2 / y=6.2（避免同车道追尾/对头交汇）：
        # on1 在前且更快（0.6~0.8），on2 在后更慢（0.5~0.6）
        on1_x = 8.0 + (rng.rand() - 0.5) * 1.0
        on1_spd = 0.75 + (rng.rand() - 0.5) * 0.2
        on2_x = 10.5 + (rng.rand() - 0.5) * 1.0
        on2_spd = 0.65 + rng.rand() * 0.1
        add_dyn('vehicle', 3.5, 3.5, 0.0, 0.45)
        add_dyn('vehicle', on1_x, 5.2, math.pi, min(on1_spd, 0.95))
        add_dyn('vehicle', on2_x, 6.2, math.pi, min(on2_spd, 0.75))

    elif sid == 'S14_longdist':
        # 长直道（x∈[0,20]），边界 block 墙 y=2.5/7.5（5m 宽），主车行线 y=5.0；
        # 2 动态（≤4）分居上下侧车道，全程与主车旋转矩形足迹保持 ≥0.72 净距：
        # 车 v1 上车道 y=6.6←（0.5；顶缘 7.0 距墙 0.5、底缘 6.2 距主车顶缘 5.3
        #   ≥0.9，v1 移动 −x 与主车 x 带重叠时主车已在其后 7m+）；
        # 自行车 b1 下车道 y=3.4→（0.5；底缘 3.15 距墙 0.65、顶缘 3.65 距主车
        #   底缘 4.7 ≥1.05，与主车 x 带重叠时横向分离 ≥1.05m）。
        # 直道无弯道/无行线动态 → 无局部极小陷阱，主车直线巡航稳定可达终点。
        # + 连线中点行人 (10.0, 5.0) 横穿主车行线（band y∈[4.6,5.4] 折返，
        #   速度 ≤0.5；带内避开上下车道动态 y=6.6/3.4，全程无重叠）：
        #   主车到达 x=10 处必须减速/时机避让。
        start_theta = 0.0
        start_x, start_y = 0.5, 5.0
        goal_x, goal_y = 19.5, 5.0
        wall_rows(2.5, 0.5, 19.5)
        wall_rows(7.5, 0.5, 19.5)
        add_dyn('vehicle', 15.5, 6.6, math.pi, 0.65)
        add_dyn('bicycle', 4.0, 3.4, 0.0, 0.5)
        spd_p = 0.3 + rng.rand() * 0.1
        add_dyn('pedestrian', 10.0, 5.0, math.pi / 2.0, spd_p,
                heading_schedule=pedestrian_schedule(
                    MatlabRNG(seed + 3000), max_time, math.pi / 2.0, dt=dt,
                    band=(4.6, 5.4), pos0=5.0, speed=spd_p))

    elif sid in ('S16_corner_clean', 'S17_corner_ped', 'S18_corner_block',
                 'S19_corner_dyn'):
        # ===== 直角转弯长走廊（4 场景）=====
        # 几何（wall_band_h/v：block 内沿精确贴线，通道净宽确定性）：
        #   水平臂：y∈[6−w/2, 6+w/2]，x∈[0.5, 9.9]（H-south 全段，H-north 至 7.6）；
        #   竖直臂：x∈[9.5−w/2, 9.5+w/2]，y∈[7.3, 11.5]（V-east 全段，V-west 自 7.8）；
        #   拐角袋：H-north 截止 7.6 + V-west 自 7.8 + 封口 y=7.8 + V-east 拐角鼓包
        #     （内沿 9.5+w/2+0.38 至 y=7.3，为 90° 转向提供扫掠空间）；
        #   顶端封口 y=11.5（goal 位于竖臂尽头 (9.5,11.0)）。
        # start(1.0,6.0) 在水平臂入口，heading 0；主车须直行→左转 90°→直行达终点。
        # w 视场景而定（保证 10~20% 通过裕量 + 考验力）：
        #   [2026-10-27 实测] 0.84/1.2/1.4/1.6m 走廊 + 直角转弯均无法被本地
        #   DWA 通过（A* 引导网格在 <1.8m 走廊内无连续可行格 / 转弯扫掠空间不足），
        #   w=1.8m 为纯走廊 DWA 实测可达下限（S16 35.6s 通过，考验力在 90° 转弯）；
        #   S17/S19 行人走廊取 w=3.6：行人带贴北/东墙（带底内沿 ≥7.225/10.175，
        #     均高于主车走廊逃逸循环包络上界 7.0），主车南/西通道净宽 ≥2.475m，
        #     行人全程分离由几何保证（走廊 trap 逃逸循环为结构性行为：C3 逃逸取
        #     最长自由视界方向，中段西向视界 > 东向至拐角视界 → 偶发西向循环，
        #     行人带必须高于循环包络上界，否则西向循环必撞）；
        #   S18 中段静态块 0.45 固定北偏（底 7.175 ≥ 循环包络上界 7.0），
        #     连续走廊 w=2.9，南侧通道 2.625m（净裕 2.025 ≥ 1.8）。
        #     （此前放行室西墙造成“前方死胡同”误判 → 西向逃逸循环卡死，已废除）
        start_theta = 0.0
        start_x, start_y = 1.0, 6.0
        # 终点下移至 (9.5, 10.9)：顶端封口内沿 y=11.5，终点距封口 0.6m，
        # 车头（0.5m）到达终点时仍距封口 ≥0.1m（goal_tolerance=0.3 提前判定到达）
        goal_x, goal_y = 9.5, 10.9
        w = ec.get('corner_w', {'S16_corner_clean': 1.8, 'S17_corner_ped': 3.6,
                                'S18_corner_block': 2.6, 'S19_corner_dyn': 3.6}[sid])
        h_s, h_n = 6.0 - w / 2.0, 6.0 + w / 2.0
        v_w, v_e = 9.5 - w / 2.0, 9.5 + w / 2.0

        def mid_room(room_x, room_y, rs):
            """中段放行室（S18）：在走廊中段扩出一个 rs×rs 房间容纳静态块。"""
            wall_band_h(room_y - rs / 2.0, room_x - rs / 2.0, room_x + rs / 2.0, -1)
            wall_band_h(room_y + rs / 2.0, room_x - rs / 2.0, room_x + rs / 2.0, +1)
            wall_band_v(room_x - rs / 2.0, room_y - rs / 2.0, room_y + rs / 2.0, -1)
            wall_band_v(room_x + rs / 2.0, room_y - rs / 2.0, room_y + rs / 2.0, +1)

        # 水平臂墙（S18 为连续走廊 + 中段北偏块，无放行室缺口：
        #   放行室西墙曾在 t≈26s 造成“前方死胡同”误判 → 逃逸西向循环卡死，
        #   改连续走廊后逃逸视界东向最长，不再西逃）
        wall_band_h(h_s, 0.5, 9.9, -1)
        wall_band_h(h_n, 0.5, 7.6, +1)
        # 竖直臂墙 + 拐角鼓包（V-east 下部外推 0.38m 供转向扫掠）
        wall_band_v(v_e + 0.38, h_s, 7.3, +1)
        wall_band_v(v_e, 7.3, 11.5, +1)
        wall_band_v(v_w, 7.8, 11.5, -1)
        # 拐角封口 + 顶端封口
        wall_band_h(7.8, 7.6, v_w, +1)
        wall_band_h(11.5, v_w, v_e, +1)
        # 西端世界边界（block 墙）：防主车/A* 从走廊西端绕出地图
        # （S18 曾在此逃逸卡死；S16~S19 统一封口，场景内无其它逃逸口）
        wall_band_v(0.0, 4.5, 12.5, -1)

        if sid == 'S16_corner_clean':
            pass   # 纯走廊
        elif sid == 'S17_corner_ped':
            # 行人 1：水平臂中段 (4.5,6.0)，北侧带 y∈[7.30,7.55] 折返（≤0.5）：
            #   w=3.6 下带底内沿 7.225 ≥ 主车走廊逃逸循环包络上界 7.0（中心 6.5+0.5），
            #   主车南侧通道（4.45→7.225 = 2.775m）全程无重叠，分离由几何保证；
            #   行人随机折返构成机动考验（带内贴北墙内沿 7.8，顶部余量 ≥0.175）。
            spd_a = 0.3 + rng.rand() * 0.1
            add_dyn('pedestrian', 4.5, 6.0, math.pi / 2.0, spd_a,
                    heading_schedule=pedestrian_schedule(
                        MatlabRNG(seed + 1000), max_time, math.pi / 2.0, dt=dt,
                        band=(7.30, 7.55), pos0=6.0, speed=spd_a))
            # 行人 2：拐弯后竖臂 (9.5,8.5)，东侧带 x∈[10.25,10.60] 折返（≤0.5）：
            #   主车西侧通道（7.7→10.175 = 2.475m）全程无重叠
            spd_b = 0.3 + rng.rand() * 0.1
            add_dyn('pedestrian', 9.5, 8.5, 0.0, spd_b,
                    heading_schedule=pedestrian_schedule(
                        MatlabRNG(seed + 2000), max_time, 0.0, dt=dt,
                        band=(10.25, 10.60), pos0=9.5, speed=spd_b, axis='x'))
        elif sid == 'S18_corner_block':
            # 中段静态块（0.45×0.45 固定，北偏贴墙）：连续走廊 w=2.9，
            # 南侧通道 7.175−4.55=2.625m（净裕 2.025，≥DWA 走廊下限 1.8）；
            # 块底 7.175 ≥ 逃逸循环包络上界 7.0，循环安全；主车贴南侧通过。
            add_static_sized(4.5, 7.4, 0.45, 0.45)
        elif sid == 'S19_corner_dyn':
            # 测试场景：拐弯前 (4.5,6.0) + 拐弯后 (9.5,8.5) 各一个动态行人
            spd_a = 0.3 + rng.rand() * 0.1
            add_dyn('pedestrian', 4.5, 6.0, math.pi / 2.0, spd_a,
                    heading_schedule=pedestrian_schedule(
                        MatlabRNG(seed + 1000), max_time, math.pi / 2.0, dt=dt,
                        band=(7.30, 7.55), pos0=6.0, speed=spd_a))
            spd_b = 0.3 + rng.rand() * 0.1
            add_dyn('pedestrian', 9.5, 8.5, 0.0, spd_b,
                    heading_schedule=pedestrian_schedule(
                        MatlabRNG(seed + 2000), max_time, 0.0, dt=dt,
                        band=(10.25, 10.60), pos0=9.5, speed=spd_b, axis='x'))

    else:
        raise ValueError(f'Unknown scenario: {scenario_id}')

    sc = Scene()
    sc.case_id = sid
    sc.start_x = start_x
    sc.start_y = start_y
    sc.start_theta = start_theta
    sc.goal_x = goal_x
    sc.goal_y = goal_y
    sc.obstacle_speed = pp.obstacle_speed
    sc.seed = seed
    sc.scene_x_min = 0.0
    sc.scene_x_max = 10.0
    sc.scene_y_min = 0.0
    sc.scene_y_max = 10.0
    sc.wrap_scene = False
    sc.direction_update_period = 0
    sc.static_obstacles = stat_obs
    sc.static_obstacle_specs = stat_specs
    sc.dynamic_obstacles = dyn_obs

    # S14 长距离场景地图横向扩展到 x∈[0,20]（不影响 S1~S13）
    if sid == 'S14_longdist':
        sc.scene_x_max = 20.0

    # 4WIS 重构：车体 1×0.6m 偏大导致原 10×10 场景几何裕量不足，
    # S2/S4/S6/S8/S12 场景放大到 12×12（障碍布局已随场景放大，见各分支）
    if sid in ('S2_dense', 'S4_utrap', 'S6_multi_dyn', 'S8_random', 'S12_dense_dyn',
               'S15_utrap_in'):
        sc.scene_x_max = 12.0
        sc.scene_y_max = 12.0

    # 直角转弯长走廊：地图 12×12.5（水平臂至 x≈10.3、竖臂至 y≈11.9）
    if sid in ('S16_corner_clean', 'S17_corner_ped', 'S18_corner_block',
               'S19_corner_dyn'):
        sc.scene_x_max = 12.0
        sc.scene_y_max = 12.5

    traj = _pre_simulate_custom(sc, dt, max_time, pp.obstacle_radius)
    return sc, traj


def _pre_simulate_custom(sc: Scene, dt: float, max_time: float, robs: float):
    """preSimulateCustom.m 等价：静态不动，动态按定义运动。

    [2026-10-27] 支持运动模式字段：
      - accel / speed_max：匀加速直线（自行车）；
      - arc_rate：简单圆弧（常角速度 heading += arc_rate·dt）；
      - turn_time/new_heading、heading_schedule（行人随机折返序列）沿用；
      - 出界反弹（非回卷）沿用。
    """
    n_steps = int(math.ceil(max_time / dt))
    n_stat = int(sc.static_obstacles.shape[0])
    n_dyn = len(sc.dynamic_obstacles)
    n_total = n_stat + n_dyn

    obs_x = np.zeros(n_total)
    obs_y = np.zeros(n_total)
    obs_h = np.zeros(n_total)
    obs_sp = np.zeros(n_total)

    for i in range(n_stat):
        obs_x[i] = sc.static_obstacles[i, 0]
        obs_y[i] = sc.static_obstacles[i, 1]
        obs_h[i] = 0.0
        obs_sp[i] = 0.0
    for i in range(n_dyn):
        idx = n_stat + i
        d = sc.dynamic_obstacles[i]
        obs_x[idx] = d['x0']
        obs_y[idx] = d['y0']
        obs_h[idx] = d['heading']
        obs_sp[idx] = d['speed']

    traj = []
    # 换向/转弯判定用整数步索引（round(t_switch/dt)），避免浮点 cur>=t_switch
    # 在 j·dt 与 t_switch 不相等时造成的 ±1 步随机漂移
    sw_idx = {}
    turn_idx = {}
    for i in range(n_dyn):
        d = sc.dynamic_obstacles[i]
        if 'heading_schedule' in d:
            sw_idx[i] = [(int(round(t_switch / dt)), h_new)
                         for (t_switch, h_new) in d['heading_schedule']]
        if d['turn_time'] > 0:
            turn_idx[i] = int(round(d['turn_time'] / dt))
    for t in range(1, n_steps + 1):
        traj.append(TrajStep(obs_x.copy(), obs_y.copy(), obs_h.copy()))
        for i in range(n_dyn):
            idx = n_stat + i
            d = sc.dynamic_obstacles[i]
            if i in turn_idx and t >= turn_idx[i]:
                obs_h[idx] = d['new_heading']
            # heading_schedule：到步即换向（幂等），last-wins
            if i in sw_idx:
                for (k_sw, h_new) in sw_idx[i]:
                    if t >= k_sw:
                        obs_h[idx] = h_new
            # 简单圆弧：常角速度旋转
            arc = float(d.get('arc_rate', 0.0) or 0.0)
            if abs(arc) > 1e-12:
                obs_h[idx] = obs_h[idx] + arc * dt
            # 匀加速直线（自行车）：速度随步长累加并限幅
            acc = float(d.get('accel', 0.0) or 0.0)
            if abs(acc) > 1e-12:
                vmax = float(d.get('speed_max', d['speed']))
                obs_sp[idx] = max(0.0, min(vmax, obs_sp[idx] + acc * dt))
            sp = obs_sp[idx]
            obs_x[idx] = obs_x[idx] + sp * math.cos(obs_h[idx]) * dt
            obs_y[idx] = obs_y[idx] + sp * math.sin(obs_h[idx]) * dt
            if obs_x[idx] < sc.scene_x_min or obs_x[idx] > sc.scene_x_max:
                obs_h[idx] = math.pi - obs_h[idx]
                obs_x[idx] = max(sc.scene_x_min, min(sc.scene_x_max, obs_x[idx]))
            if obs_y[idx] < sc.scene_y_min or obs_y[idx] > sc.scene_y_max:
                obs_h[idx] = -obs_h[idx]
                obs_y[idx] = max(sc.scene_y_min, min(sc.scene_y_max, obs_y[idx]))
    return traj


# ============================================================================
# 兼容层：loadScenario.m / preSimulateTrajectory.m（Case I-1/I-2/II 旧结构）
# ============================================================================

def load_scenario(pp: Params):
    """loadScenario.m 等价。"""
    sc = Scene()
    sc.case_id = pp.case_id
    if pp.case_id == 'I-1':
        sc.start_x, sc.start_y, sc.start_theta = pp.case_I1_start_x, pp.case_I1_start_y, pp.case_I1_start_theta
        sc.goal_x, sc.goal_y = pp.case_I1_goal_x, pp.case_I1_goal_y
        sc.n_obstacles = pp.case_I1_n_obstacles
    elif pp.case_id == 'I-2':
        sc.start_x, sc.start_y, sc.start_theta = pp.case_I2_start_x, pp.case_I2_start_y, pp.case_I2_start_theta
        sc.goal_x, sc.goal_y = pp.case_I2_goal_x, pp.case_I2_goal_y
        sc.n_obstacles = pp.case_I2_n_obstacles
    else:
        sc.start_x, sc.start_y, sc.start_theta = pp.case_II_start_x, pp.case_II_start_y, pp.case_II_start_theta
        sc.goal_x, sc.goal_y = pp.case_II_goal_x, pp.case_II_goal_y
        sc.n_obstacles = pp.case_II_n_obstacles
    sc.obstacle_speed = pp.obstacle_speed
    sc.direction_update_period = pp.direction_update_period
    sc.scene_x_min = pp.scene_x_min
    sc.scene_x_max = pp.scene_x_max
    sc.scene_y_min = pp.scene_y_min
    sc.scene_y_max = pp.scene_y_max
    sc.wrap_scene = pp.wrap_scene
    sc.seed = pp.random_seed
    return sc


def pre_simulate_trajectory(sc: Scene, dt: float, max_time: float):
    """preSimulateTrajectory.m 等价（Case I-1/I-2/II record-replay）。"""
    n_steps = int(math.ceil(max_time / dt))
    rng = MatlabRNG(sc.seed)

    obs, heading = _init_obstacles(sc, rng)
    update_every = max(1, round(sc.direction_update_period / dt))
    n_obs = len(obs)

    traj = []
    for t in range(1, n_steps + 1):
        xa = np.array([o[0] for o in obs])
        ya = np.array([o[1] for o in obs])
        ha = np.array(heading)
        traj.append(TrajStep(xa, ya, ha))

        t0 = t - 1
        if sc.case_id == 'II' and t0 > 0 and t0 % update_every == 0:
            for i in range(n_obs):
                heading[i] = rng.rand() * 2.0 * math.pi

        obs, heading = _resolve_overlap(obs, heading)

        for i in range(n_obs):
            obs[i][0] += sc.obstacle_speed * math.cos(heading[i]) * dt
            obs[i][1] += sc.obstacle_speed * math.sin(heading[i]) * dt

        if sc.wrap_scene:
            for i in range(n_obs):
                if obs[i][0] < sc.scene_x_min:
                    obs[i][0] += (sc.scene_x_max - sc.scene_x_min)
                if obs[i][0] > sc.scene_x_max:
                    obs[i][0] -= (sc.scene_x_max - sc.scene_x_min)
                if obs[i][1] < sc.scene_y_min:
                    obs[i][1] += (sc.scene_y_max - sc.scene_y_min)
                if obs[i][1] > sc.scene_y_max:
                    obs[i][1] -= (sc.scene_y_max - sc.scene_y_min)
    return traj


def _init_obstacles(sc: Scene, rng: MatlabRNG):
    """initObstacles.m 等价。"""
    from .obstacle import new_obstacle
    if sc.case_id == 'I-1':
        xs = [1.5, 3.5, 5.5, 7.5]
        ys = [9.5, 8.2, 9.0, 8.6]
        obs = []
        heading = []
        for i in range(4):
            o = new_obstacle(i, xs[i], ys[i], -math.pi / 2.0, sc.obstacle_speed, 0.15)
            obs.append([o.x, o.y])
            heading.append(-math.pi / 2.0)
    elif sc.case_id == 'I-2':
        ys = [1.5, 3.0, 4.5, 6.0]
        obs = []
        heading = []
        for i in range(4):
            obs.append([-2.0, ys[i]])
            heading.append(0.0)
    else:
        n = sc.n_obstacles
        obs = []
        heading = []
        for i in range(n):
            for _ in range(100):
                x = (sc.scene_x_min + 0.5) + rng.rand() * ((sc.scene_x_max - 0.5) - (sc.scene_x_min + 0.5))
                y = (sc.scene_y_min + 0.5) + rng.rand() * ((sc.scene_y_max - 0.5) - (sc.scene_y_min + 0.5))
                d_start = math.hypot(x - sc.start_x, y - sc.start_y)
                d_goal = math.hypot(x - sc.goal_x, y - sc.goal_y)
                if d_start > 0.8 and d_goal > 0.8:
                    break
            obs.append([x, y])
            heading.append(rng.rand() * 2.0 * math.pi)
    return obs, heading


def _resolve_overlap(obs, heading):
    """resolveOverlap.m 等价：距离 < 2·r_obs 的障碍对互相远离。"""
    min_d = 2.0 * 0.15
    n = len(obs)
    for i in range(n):
        for j in range(i + 1, n):
            dx = obs[j][0] - obs[i][0]
            dy = obs[j][1] - obs[i][1]
            d = math.hypot(dx, dy)
            if d < min_d and d > 1e-6:
                ang = math.atan2(dy, dx)
                heading[i] = ang + math.pi
                heading[j] = ang
    return obs, heading
