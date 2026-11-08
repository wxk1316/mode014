"""
global_guide.py — C8 全局引导模块（P7：密集静态场无全局引导 → A*/D* 引导线思想）。

基于静态障碍地图（sc.static_obstacles，含场景墙体行）构建栅格 A* 路径，
输出等距路点序列；运行时按单调游标给出"下一个子目标"，供局部规划器
以子目标代替终点目标，使逃逸/采样偏好朝向全局路径方向（借鉴论文：
A*+DWA 融合 / 子目标序列，见《DWA 方向论文总结》第 20/22/24/26 篇）。

设计约束：
  - 仅使用静态障碍信息（不含动态障碍未来信息，不构成作弊）；
  - 膨胀半径 0.45m = 车宽半宽 0.3 + 障碍半径 0.15，为引导而非硬约束
    （若某间隙实际不可穿，局部规划器仍会自行处理）；
  - A* 不可达时回退直线引导（无引导），不影响基线行为。
"""
from __future__ import annotations

import heapq
import math

import numpy as np


class GlobalGuide:
    """静态地图 A* 引导线 + 单调子目标游标。"""

    def __init__(self, sc, pp):
        self.pp = pp
        self.res = pp.c8_grid_res
        self.inflate = pp.c8_inflate
        self.lookahead = pp.c8_lookahead_dist
        self.start = (float(sc.start_x), float(sc.start_y))
        self.goal = (float(sc.goal_x), float(sc.goal_y))
        self.path = None
        self.cursor = 0
        self.fallback = False
        self._build(sc.static_obstacles)

    # ------------------------------------------------------------------
    def _build(self, static_obs):
        """栅格化 + A*（8 邻接）。"""
        pp = self.pp
        x_max = max(static_obs[:, 0].max() if static_obs.shape[0] else 0.0,
                    self.goal[0], self.start[0]) + 1.0
        y_max = max(static_obs[:, 1].max() if static_obs.shape[0] else 0.0,
                    self.goal[1], self.start[1]) + 1.0
        res = self.res
        nx = int(math.ceil(x_max / res)) + 1
        ny = int(math.ceil(y_max / res)) + 1
        occ = np.zeros((nx, ny), dtype=bool)

        # 膨胀静态障碍
        r = self.inflate
        cells_r = int(math.ceil(r / res))
        for (ox, oy) in static_obs:
            ix = int(round(ox / res))
            iy = int(round(oy / res))
            x0 = max(0, ix - cells_r)
            x1 = min(nx - 1, ix + cells_r)
            y0 = max(0, iy - cells_r)
            y1 = min(ny - 1, iy + cells_r)
            for xi in range(x0, x1 + 1):
                for yi in range(y0, y1 + 1):
                    if math.hypot(xi * res - ox, yi * res - oy) <= r:
                        occ[xi, yi] = True

        # 边界外不可达
        occ[0, :] = True
        occ[-1, :] = True
        occ[:, 0] = True
        occ[:, -1] = True

        si = (int(round(self.start[0] / res)), int(round(self.start[1] / res)))
        gi = (int(round(self.goal[0] / res)), int(round(self.goal[1] / res)))
        si = (min(max(si[0], 1), nx - 2), min(max(si[1], 1), ny - 2))
        gi = (min(max(gi[0], 1), nx - 2), min(max(gi[1], 1), ny - 2))

        if occ[si] or occ[gi]:
            self.fallback = True
            return

        # A*（8 邻接，欧氏启发）
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (1, -1), (-1, 1), (1, 1)]
        costs = [1.0, 1.0, 1.0, 1.0, math.sqrt(2.0), math.sqrt(2.0),
                 math.sqrt(2.0), math.sqrt(2.0)]
        open_h = []
        heapq.heappush(open_h, (0.0, 0, si))
        g_score = {si: 0.0}
        came = {}
        found = False
        while open_h:
            _, _, cur = heapq.heappop(open_h)
            if cur == gi:
                found = True
                break
            cx, cy = cur
            for d, c in zip(dirs, costs):
                nx_, ny_ = cx + d[0], cy + d[1]
                if not (0 <= nx_ < nx and 0 <= ny_ < ny):
                    continue
                if occ[nx_, ny_]:
                    continue
                ng = g_score[cur] + c
                if (nx_, ny_) not in g_score or ng < g_score[(nx_, ny_)]:
                    g_score[(nx_, ny_)] = ng
                    came[(nx_, ny_)] = cur
                    h = math.hypot(nx_ - gi[0], ny_ - gi[1]) * res
                    heapq.heappush(open_h, (ng + h, len(g_score), (nx_, ny_)))
        if not found:
            self.fallback = True
            return

        # 回溯路径
        pts = []
        cur = gi
        while cur != si:
            pts.append((cur[0] * res, cur[1] * res))
            cur = came[cur]
        pts.append((self.start[0], self.start[1]))
        pts.reverse()
        # 等距化（间隔 ≈ res）
        way = [pts[0]]
        acc = 0.0
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            acc += d
            if acc >= res * 0.9:
                way.append(pts[i])
                acc = 0.0
        way.append(self.goal)
        self.path = way

    # ------------------------------------------------------------------
    def next_subgoal(self, robot):
        """返回当前子目标（单调游标，不回退）。"""
        if self.fallback or self.path is None or len(self.path) < 2:
            return self.goal
        p = self.path
        # 找到机器人前方最近路点（优先当前游标附近，单调推进）
        best_i = self.cursor
        best_d = math.inf
        for i in range(self.cursor, len(p)):
            d = math.hypot(p[i][0] - robot.x, p[i][1] - robot.y)
            if d < best_d:
                best_d = d
                best_i = i
        # 前进 lookahead 距离
        look = self.lookahead
        tgt = best_i
        while tgt + 1 < len(p) and math.hypot(p[tgt + 1][0] - robot.x,
                                              p[tgt + 1][1] - robot.y) < look:
            tgt += 1
        self.cursor = max(self.cursor, tgt)
        if tgt >= len(p) - 1:
            return self.goal
        return p[tgt]
