"""
dqn_infer.py — DQN 模型版实车推理（纯 numpy，车机免装 torch）。

用途：
  将 RL-DWA 训练好的 DuelingDQN checkpoint 导出为 .npz 权重（见 scripts/export_dqn_npz.py），
  在实车节点（car_ros/rda_dwa_real）用 numpy 实现前向，输出预测时域 τ∈T_CHOICES，
  交给 EnhancedDWA 规划 (v, ω)。与 eval_dqn_v5.py 的注入语义完全一致：
    policy(robot, obstacles, goal, trap_active) -> tau(float)

网络结构（与 scripts/driver_parallel.py 的 DuelingDQN 一致）：
  feature: Linear(in_dim,128) -> ReLU -> Linear(128,128) -> ReLU
  value:   Linear(128,1)
  adv:     Linear(128,4)
  q = value(f) + (adv(f) - mean(adv(f)))
  action = argmax(q)  →  T_CHOICES = [0.8, 1.0, 1.2, 1.5]

依赖：仅 numpy + 算法包 rda_dwa.dqn_state.make_state（car_ros 内算法拷贝）。
"""
from __future__ import annotations

import os

import numpy as np

T_CHOICES = [0.8, 1.0, 1.2, 1.5]


class DQNPolicy:
    """numpy 版 DuelingDQN 推理策略。"""

    def __init__(self, npz_path: str, state_dt: float = 0.1):
        if not os.path.isfile(npz_path):
            raise FileNotFoundError(f'DQN 权重文件不存在: {npz_path}')
        d = np.load(npz_path)
        self.in_dim = int(np.asarray(d['in_dim']).reshape(-1)[0])
        self.W0 = d['feature.0.weight']
        self.b0 = d['feature.0.bias']
        self.W2 = d['feature.2.weight']
        self.b2 = d['feature.2.bias']
        self.Wv = d['value.weight']
        self.bv = d['value.bias']
        self.Wa = d['adv.weight']
        self.ba = d['adv.bias']
        self.state_dt = float(state_dt)
        self.n_actions = self.Wa.shape[0]
        if self.n_actions != len(T_CHOICES):
            raise ValueError(f'checkpoint 动作数 {self.n_actions} != {len(T_CHOICES)}')

    # ------------------------------------------------------------------
    def _forward(self, s2d: np.ndarray) -> np.ndarray:
        """s2d: (N, in_dim) → q 值 (N, n_actions)。"""
        h = s2d @ self.W0.T + self.b0
        h = np.maximum(h, 0.0)
        h = h @ self.W2.T + self.b2
        h = np.maximum(h, 0.0)
        v = h @ self.Wv.T + self.bv                     # (N,1)
        a = h @ self.Wa.T + self.ba                     # (N,K)
        q = v + (a - a.mean(axis=1, keepdims=True))
        return q

    # ------------------------------------------------------------------
    def predict_tau(self, robot, obstacles, goal, trap_active: bool = False) -> float:
        """与 eval_dqn_v5.dqn_policy 同签名：返回预测时域（秒）。"""
        from rda_dwa.dqn_state import make_state

        s = make_state(robot, obstacles, goal,
                       trap_active=trap_active,
                       sub_goal=goal if trap_active else None,
                       input_dim=self.in_dim,
                       dt=self.state_dt,
                       use_prediction=(self.in_dim >= 21))
        q = self._forward(s[None, :])
        a = int(np.argmax(q, axis=1)[0])
        return float(T_CHOICES[a])
