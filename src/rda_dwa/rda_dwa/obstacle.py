"""
obstacle.py — 动态/静态障碍物状态（newObstacle.m 的 Python 移植）。

字段与 MATLAB struct 一一对应，含 M5 前置 α-β 滤波状态与 C2 IMM-UKF 状态。

[2026-10-27 场景改造] 障碍物由"统一圆柱"扩展为四种现实类型：
  - pedestrian 行人（正方，基准 0.15×0.15 m，±0.1 m 相异性抖动；速度 ≤0.5 m/s，
    直线横穿 + 随机折返（社会力简单版））
  - bicycle    自行车（矩形，基准 0.8×0.5 m；速度 ≤0.6 m/s，匀加速直线 / 简单圆弧，
    不主动避让主车）
  - vehicle    车辆（矩形，基准 1.4×0.8 m；速度 ≤0.8 m/s，匀速直线 / 简单圆弧，
    不主动避让主车）
  - static_large / block  静态（正方形 1~2 m / 小方块 0.3~0.5 m，用于 U 形墙等）
每个障碍携带 length/width/type；radius = 外接圆半径 r=hypot(L/2,W/2)，
继续供"半径使用逻辑"（DQN 扇区/风险场/APF/预筛）消费。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

# 障碍类型注册（基准尺寸；相异性抖动在 build_scenario 内确定性抽取）
OBSTACLE_TYPES = ('pedestrian', 'bicycle', 'vehicle', 'static_large', 'block')


def circum_radius(length: float, width: float) -> float:
    """障碍外接圆半径 r = hypot(L/2, W/2)（适配现有 radius 使用逻辑）。"""
    return math.hypot(length / 2.0, width / 2.0)


@dataclass
class Obstacle:
    """单个障碍物（§2.2：圆形、统一尺寸与速度；2026-10-27 起为矩形现实类型）。"""
    id: int = 0
    x: float = 0.0          # P_obs(t) 当前位置 x
    y: float = 0.0          # P_obs(t) 当前位置 y
    px: float = 0.0         # P_obs(t-1) 上一位置 x（用于差分）
    py: float = 0.0         # P_obs(t-1) 上一位置 y
    speed: float = 0.0      # V_obs(t) 障碍物速度（公式(3)）
    heading: float = 0.0    # θ_obs(t) 障碍物运动方向
    fx: float = 0.0         # P_obs(t+1) 预测位置 x（公式(3)）
    fy: float = 0.0         # P_obs(t+1) 预测位置 y
    radius: float = 0.15    # 等效半径 = 外接圆 r=hypot(L/2,W/2)（Table 1 原 0.15 仅兼容旧场景）
    obstacle_type: str = 'static_large'   # pedestrian/bicycle/vehicle/static_large/block
    length: float = 1.0     # 障碍矩形长（沿 heading 方向；静态默认 1.0m 正方）
    width: float = 1.0      # 障碍矩形宽（垂直 heading 方向）
    has_history: bool = False
    # M5 前置 α-β 滤波状态
    filtered_speed: float = 0.0
    filtered_heading: float = 0.0
    has_filter_state: bool = False
    # C2 IMM-UKF 状态（未启用时保持同构）
    imm: dict = field(default_factory=dict)
    # C9 预测残差机动标记：预测残差连续超阈值 → 行为不可预测 → 临时膨胀有效半径
    maneuver_hold: int = 0      # 剩余膨胀保持步数（>0 表示处于"机动/不可预测"状态）
    prev_heading: float = 0.0   # 上一步滤波航向（C9 航向跳变检测用）
    # g8：换向检测→恒角速度弧线外推（MTDWA 第二轮）
    heading_rate: float = 0.0   # 障碍当前角速度（rad/s，IMM 差分估计；>0 时 M2 弧线外推）
    arc_mode_hold: int = 0      # 弧线外推剩余保持步数（>0 表示处于换向弧线模式）
    # g10：IMM 1 步预测残差（MTDWA 第三轮，C9 已算，存档供规划层保守收缩）
    res_err: float = 0.0        # 上一步预测位置 vs 当前实际位置的欧氏残差（m）
    div_err: float = 0.0        # IMM 三模型 1 步预测位置两两距离最大值（m，预测分歧指标）


def new_obstacle(id_: int = 0, x: float = 0.0, y: float = 0.0,
                 heading: float = 0.0, speed: float = 0.0,
                 radius: float = 0.15,
                 obstacle_type: str = 'static_large',
                 length: float = 1.0, width: float = 1.0) -> Obstacle:
    """newObstacle.m 等价。

    2026-10-27：支持现实类型障碍。传入 obstacle_type/length/width 时，
    radius 自动取外接圆 r=hypot(L/2,W/2)；仍允许显式 radius 覆盖（旧圆形兼容）。
    """
    o = Obstacle()
    o.id = int(id_)
    o.x = x
    o.y = y
    o.px = x
    o.py = y
    o.speed = speed
    o.heading = heading
    o.fx = x
    o.fy = y
    o.obstacle_type = obstacle_type
    o.length = float(length)
    o.width = float(width)
    o.radius = circum_radius(o.length, o.width) if radius is None else float(radius)
    o.has_history = False
    o.filtered_speed = 0.0
    o.filtered_heading = 0.0
    o.has_filter_state = False
    o.imm = {}
    return o
