"""
traj_cache.py — 方案B：/rda/traj 轨迹时序缓存（纯 Python，无 ROS 依赖，可单测）。

背景：DWA 每个规划周期（墙钟实测 ~5~10Hz）基于当前位姿重规划，predict_reference_traj
输出的参考轨迹以"当前位姿"为根。每周期换根会让下层纯跟踪（traj_tracker，50Hz）
参考点跳动，导致打角震荡。本模块把多周期 DWA 预测轨迹按墙钟时间轴对齐缓存：

  add_cycle(now, pts)  每规划周期把恒速积分出的 n 个预测点附绝对时间
                        t_i = now + i*dt 后有序插入缓存，并按时间窗剪枝；
  query(now, dt, n)    发布时按时间轴对 tau_i = now + i*dt 逐点插值：
                        左右包围 → x/y 线性插值、theta 最短角 wrap 插值；
                        单侧包围 → 零阶保持（取最近一侧点）；
                        无点     → 调用方原始轨迹点兜底（缓存空档/首帧）。

多周期预测在同一时间轴上自然融合，发布轨迹平滑"换根"，且保持固定时域/点数/步长。
静止语义不受影响：调用方在 |v|<v_eps 时旁路直发原始轨迹（全点重合，弧长≈0），
不经过本缓存。
"""
from __future__ import annotations

import bisect
import math


def _wrap_angle(a: float) -> float:
    """把角度 wrap 到 [-π, π)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class TrajTimeCache:
    """参考轨迹时序缓存：多周期 DWA 预测按时间轴插值融合。

    内部采样点结构 (t, x, y, theta)，按 t 升序维护（bisect 有序插入）。

    参数:
      dt:             单步积分步长 s（add_cycle 缺省点间时间间隔）
      horizon:        预测时域 s（剪枝右界 = now+horizon+future_margin）
      past_margin:    保留的过去时间窗 s（剪枝下界 = now-past_margin）
      future_margin:  保留的未来余量 s（超出 horizon 的预测尾部，应对周期抖动）
    """

    def __init__(self, dt: float, horizon: float,
                 past_margin: float = 0.2, future_margin: float = 0.2):
        self.dt = float(dt)
        self.horizon = float(horizon)
        self.past_margin = float(past_margin)
        self.future_margin = float(future_margin)
        self._samples: list = []   # [(t, x, y, theta), ...]，按 t 升序

    # ------------------------------------------------------------------
    def add_cycle(self, now: float, pts, dt: float | None = None):
        """把一个规划周期的预测轨迹按绝对时间入缓存并剪枝。

        参数:
          now: 本周期墙钟时刻（time.monotonic()）
          pts: 预测轨迹点 list[(x, y, theta)]，与 predict_reference_traj 输出同构
          dt:  点间时间间隔 s（缺省用构造时 dt；bridge 侧显式传 traj_dt）
        """
        dt = self.dt if dt is None else float(dt)
        for i, (px, py, pth) in enumerate(pts):
            bisect.insort(self._samples, (now + i * dt,
                                          float(px), float(py), float(pth)))
        # 剪枝：只保留 [now-past_margin, now+horizon+future_margin] 内的点
        lo = now - self.past_margin
        hi = now + self.horizon + self.future_margin
        left = bisect.bisect_left(self._samples, (lo,))
        right = bisect.bisect_right(
            self._samples, (hi, float('inf'), float('inf'), float('inf')))
        self._samples = self._samples[left:right]

    # ------------------------------------------------------------------
    def query(self, now: float, dt: float, n: int, fallback_pts=None):
        """按时间轴插值出 n 个发布点（tau_i = now + i*dt）。

        参数:
          now:          本周期墙钟时刻（time.monotonic()）
          dt:           发布点步长 s
          n:            发布点数（与 predict_reference_traj 一致）
          fallback_pts: 缓存无覆盖时的兜底点 list[(x, y, theta)]
                        （一般传本周期原始 pts，保证首帧/空档不崩）

        返回:
          list[(x, y, theta)]，长度 n。
        """
        out = []
        for i in range(int(n)):
            tau = now + i * dt
            fb = fallback_pts[i] if fallback_pts is not None \
                and i < len(fallback_pts) else None
            out.append(self._interp(tau, fb))
        return out

    # ------------------------------------------------------------------
    def _interp(self, tau: float, fb):
        """单个 tau 的缓存插值：双包围→线性/最短角插值；单侧→零阶；无点→兜底。"""
        if not self._samples:
            return self._fallback(fb)
        idx = bisect.bisect_left(self._samples, (tau,))
        left = self._samples[idx - 1] if idx > 0 else None
        right = self._samples[idx] if idx < len(self._samples) else None
        if left is None and right is None:
            return self._fallback(fb)
        if left is None:    # 仅右侧包围 → 零阶保持
            return (right[1], right[2], right[3])
        if right is None:    # 仅左侧包围 → 零阶保持
            return (left[1], left[2], left[3])
        if right[0] == tau:
            return (right[1], right[2], right[3])
        if left[0] == tau:
            return (left[1], left[2], left[3])
        span = right[0] - left[0]
        if span <= 0.0:
            return (left[1], left[2], left[3])
        a = (tau - left[0]) / span
        x = left[1] + a * (right[1] - left[1])
        y = left[2] + a * (right[2] - left[2])
        theta = left[3] + a * _wrap_angle(right[3] - left[3])
        return (x, y, theta)

    # ------------------------------------------------------------------
    @staticmethod
    def _fallback(fb):
        """缓存无覆盖兜底：有调用方原始点则用之，否则 (0, 0, 0)。"""
        if fb is None:
            return (0.0, 0.0, 0.0)
        return (float(fb[0]), float(fb[1]), float(fb[2]))
