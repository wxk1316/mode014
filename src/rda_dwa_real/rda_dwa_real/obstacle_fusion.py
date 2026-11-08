"""
obstacle_fusion.py — 外部感知 → 算法 Obstacle 的融合层（纯 Python，可单测）。

输入形态（实车真实感知）:
  1) 结构化障碍物消息（推荐）：每个障碍已含 id/type/x/y/length/width/heading/speed
     → 直接构造/更新 Obstacle。
  2) sensor_msgs/LaserScan（2D 激光）：极坐标 → 传感器系点 → 距离聚类 →
     簇 → 矩形包围盒（L/W/heading）→ 类型推断 → Obstacle。
  3) sensor_msgs/PointCloud2（2D 点集）：直接聚类 → 同上。

类型推断（与场景四类现实障碍对应）:
  pedestrian 正方 0.15×0.15（≤0.35m）
  bicycle    矩形 0.8×0.5（长边 ≤0.95m）
  vehicle    矩形 1.4×0.8（长边 ≤2.0m）
  static_large 正方形 1~2m（更大）
尺寸保留聚类实测；聚类无法可靠估计时回退类型基准尺寸（保守）。

跨帧跟踪（TrackManager）:
  - 结构化消息带稳定 id：直接按 id 维护。
  - 聚类结果不带 id：与上帧 track 中心最近邻门控匹配（track_gate 默认 1.0m），
    新簇开新 track；连续 track_lost（默认 5 帧）未匹配则移除。
  - 每个 Obstacle 的 type/length/width/radius 在创建时锁定，后续每帧只更新位置
    （与场景改造口径一致："物体类型确定后每次仿真不再修改"）。
"""
from __future__ import annotations

import math
from typing import Optional

from rda_dwa.obstacle import Obstacle, new_obstacle, circum_radius
from rda_dwa.robot_model import wrap_angle


# ---------------------------------------------------------------------------
# 类型推断 / 尺寸基准
# ---------------------------------------------------------------------------
TYPE_BASE_SIZE = {
    'pedestrian': (0.15, 0.15),
    'bicycle': (0.8, 0.5),
    'vehicle': (1.4, 0.8),
    'static_large': (1.5, 1.5),
    'block': (0.4, 0.4),
}

# 类型判定阈值（按长边）：<=0.35 → pedestrian；<=0.95 → bicycle；
# <=2.0 → vehicle；否则 static_large。
_TYPE_MAX_LONG = {
    'pedestrian': 0.35,
    'bicycle': 0.95,
    'vehicle': 2.0,
}


def infer_type(L: float, W: float) -> str:
    """由实测包围盒推断四类现实障碍类型。"""
    a, b = max(L, W), min(L, W)
    if a <= _TYPE_MAX_LONG['pedestrian']:
        return 'pedestrian'
    if a <= _TYPE_MAX_LONG['bicycle']:
        return 'bicycle'
    if a <= _TYPE_MAX_LONG['vehicle'] and b <= 1.1:
        return 'vehicle'
    if 2.0<a<=6.0 and .8<=b<=2.5:return 'vehicle'
    return 'static_large'


def resolve_size(obs_type: str, L_meas: Optional[float], W_meas: Optional[float]):
    """实测尺寸可用则保留，否则回退类型基准尺寸（保守）。

    返回 (L, W)。对动态类型（行人/自行车/车辆）实测过小/不可信时
    至少给到基准尺寸下限，防止半径过小造成漏碰。
    """
    base = TYPE_BASE_SIZE.get(obs_type, (1.0, 1.0))
    if L_meas is None or W_meas is None or L_meas <= 0.02 or W_meas <= 0.02:
        return base
    L = max(L_meas, base[0] * 0.5)
    W = max(W_meas, base[1] * 0.5)
    return L, W


# ---------------------------------------------------------------------------
# 点集聚类 → 障碍物矩形
# ---------------------------------------------------------------------------
class Clusterer:
    """简单 2D 距离聚类（单遍近邻合并），用于激光/点云障碍物提取。

    参数:
      gate: 同一障碍物内两点最大间距（默认 0.6m，行人/车辆尺度适用）
      min_points: 簇最少点数（默认 2，过滤单点噪声）
      max_radius: 单簇质心距离原点最大范围（默认 30m，滤远点）
    """

    def __init__(self, gate: float = 0.6, min_points: int = 2, max_radius: float = 30.0):
        self.gate = float(gate)
        self.min_points = int(min_points)
        self.max_radius = float(max_radius)

    def cluster(self, points: list[tuple[float, float]]):
        """输入：算法系点集 [(x, y), ...]。输出：簇列表 [[(x,y), ...], ...]。"""
        pts = [p for p in points
               if math.hypot(p[0], p[1]) <= self.max_radius]
        if not pts:
            return []
        # Spatial hash + Euclidean connected components. Long car surfaces
        # must not fragment just because their centroid drifts during a scan.
        cells={};gate2=self.gate*self.gate
        for i,(x,y) in enumerate(pts):
            cell=(math.floor(x/self.gate),math.floor(y/self.gate))
            cells.setdefault(cell,[]).append(i)
        unseen=set(range(len(pts)));clusters=[]
        while unseen:
            seed=unseen.pop();stack=[seed];cl=[]
            while stack:
                i=stack.pop();x,y=pts[i];cl.append(pts[i])
                cx,cy=math.floor(x/self.gate),math.floor(y/self.gate)
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        for j in cells.get((cx+dx,cy+dy),()):
                            if j in unseen and (pts[j][0]-x)**2+(pts[j][1]-y)**2<=gate2:
                                unseen.remove(j);stack.append(j)
            if len(cl)>=self.min_points:clusters.append(cl)
        return clusters



def cluster_bounding_box(cluster: list[tuple[float, float]]):
    """簇 → 轴对齐包围盒 (cx, cy, L, W, heading)。

    简单实现：沿主轴（协方差最大特征向量）取旋转矩形；主轴无法可靠
    估计时退化为轴对齐包围盒。heading 为矩形长边方向（rad）。
    返回 (cx, cy, L, W, heading)；静态默认 heading=0 由调用方决定。
    """
    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    n = len(cluster)
    # 主轴（PCA）
    mxx = sum((p[0] - cx) ** 2 for p in cluster) / n
    myy = sum((p[1] - cy) ** 2 for p in cluster) / n
    mxy = sum((p[0] - cx) * (p[1] - cy) for p in cluster) / n
    if abs(mxy) < 1e-9 or math.hypot(mxx, myy) < 1e-12:
        theta = 0.0
    else:
        theta = 0.5 * math.atan2(2.0 * mxy, mxx - myy)
    c, s = math.cos(theta), math.sin(theta)
    proj_u = [p[0] * c + p[1] * s for p in cluster]
    proj_v = [-p[0] * s + p[1] * c for p in cluster]
    u_min, u_max = min(proj_u), max(proj_u)
    v_min, v_max = min(proj_v), max(proj_v)
    L = max(u_max - u_min, 0.05)
    W = max(v_max - v_min, 0.05)
    uc,vc=(u_min+u_max)/2,(v_min+v_max)/2
    return c*uc-s*vc, s*uc+c*vc, L, W, theta


# ---------------------------------------------------------------------------
# TrackManager：跨帧障碍物跟踪（无 id 感知时）
# ---------------------------------------------------------------------------
class TrackManager:
    """按最近邻门控匹配维护 Obstacle 列表（等价于场景障碍物的逐帧更新）。

    - tracks: dict[id] -> Obstacle（type/L/W/radius 创建时锁定）
    - predict: 外部传入的 predictor（ObstaclePredictor 或 IMMUKFPredictor）
      ，每帧调用 predict.update(obs, x, y) 更新真值位置并产出 fx/fy/speed/heading。
    """

    def __init__(self, predictor, gate: float = 1.0, lost_ttl: int = 5,
                 max_speed: float = 3.0):
        self.predict = predictor
        self.gate = float(gate)
        self.lost_ttl = int(lost_ttl)
        self.max_speed = float(max_speed)
        self.tracks: dict[int, Obstacle] = {}
        self._lost: dict[int, int] = {}
        self._next_id = 1

    def reset(self):
        self.tracks.clear()
        self._lost.clear()
        self._next_id = 1

    def update_measurements(self, detections):
        """detections: 列表，每项 (x, y, obs_type, L, W, heading, speed, id或None)。

        - 有 id：直接按 id 创建/更新。
        - 无 id：与上帧 track 最近邻门控匹配；匹配则更新位置，否则开新 track。
        返回更新后的 Obs 列表（供 DWA 消费，顺序稳定）。
        """
        matched: set[int] = set()
        new_tracks: dict[int, Obstacle] = {}
        # ---- 带 id 的检测：直接映射 ----
        for det in detections:
            x, y, otype, L, W, heading, speed = det[0], det[1], det[2], det[3], det[4], det[5], det[6]
            sid = det[7] if len(det) > 7 else None
            if sid is not None:
                sid = int(sid)
                if sid in self.tracks:
                    obs = self.tracks[sid]
                    if obs.obstacle_type != otype:
                        # 类型变化（实车场景：行人静止→移动 / 移动→静止，
                        # 由 lidar_track_node 运动判别触发）：更新锁定尺寸/半径，
                        # 重置预测状态，保证算法按新类型（动态预测或静态冻结）消费。
                        obs.obstacle_type = otype
                        L, W = resolve_size(otype, L, W)
                        obs.length, obs.width = L, W
                        obs.radius = circum_radius(L, W)
                        obs.imm = {}
                        obs.has_history = False
                        self.predict.initialize(obs, x, y, heading, speed)
                    else:
                        self.predict.update(obs, x, y)
                else:
                    obs = self._make_obstacle(sid, x, y, heading, speed, otype, L, W)
                    self.predict.initialize(obs, x, y, heading, speed)
                new_tracks[sid] = obs
                matched.add(sid)

        # ---- 无 id 的检测：最近邻匹配 ----
        remaining = [d for d in detections if (d[7] if len(d) > 7 else None) is None]
        for det in remaining:
            x, y, otype, L, W, heading, speed = det[0], det[1], det[2], det[3], det[4], det[5], det[6]
            best_id, best_d = None, self.gate
            for tid, obs in self.tracks.items():
                if tid in new_tracks or tid in matched:
                    continue
                d = math.hypot(obs.x - x, obs.y - y)
                if d < best_d:
                    best_d = d
                    best_id = tid
            if best_id is not None:
                obs = self.tracks[best_id]
                self.predict.update(obs, x, y)
                new_tracks[best_id] = obs
                matched.add(best_id)
            else:
                sid = self._next_id
                self._next_id += 1
                obs = self._make_obstacle(sid, x, y, heading, speed, otype, L, W)
                self.predict.initialize(obs, x, y, heading, speed)
                new_tracks[sid] = obs

        # ---- 失联清理 ----
        self._lost = {tid: (self._lost.get(tid, 0) + 1)
                      for tid in self.tracks if tid not in new_tracks}
        self.tracks = new_tracks
        # 静态障碍不应因短暂丢失被删（保持障碍地图）；动态障碍按 ttl 删除
        for tid in list(self.tracks):
            if self.tracks[tid].obstacle_type in ('static_large', 'block'):
                continue
            if self._lost.get(tid, 0) >= self.lost_ttl:
                del self.tracks[tid]
        # 对失联的动态障碍：用上一帧位置外推补帧（平滑消失），保证 DWA 持续避让
        # 实现：不再补帧，按 ttl 删（简单且安全，缺失帧 DWA 会放松约束，
        # 但实车感知正常 10Hz 下不会发生连续 5 帧漏检）。
        return sorted(self.tracks.values(), key=lambda o: o.id)

    def _make_obstacle(self, sid, x, y, heading, speed, otype, L, W) -> Obstacle:
        otype = otype if otype in TYPE_BASE_SIZE else 'static_large'
        L, W = resolve_size(otype, L, W)
        obs = new_obstacle(sid, x, y, heading, speed,
                           radius=None, obstacle_type=otype,
                           length=L, width=W)
        obs.has_history = False
        return obs
