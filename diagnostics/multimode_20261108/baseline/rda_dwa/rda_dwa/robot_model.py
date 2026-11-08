"""
robot_model.py — 两轮差分驱动机器人运动学模型（RobotModel.m 的 Python 移植）。
公式(14)：x(t+1)=x(t)+v·cosθ·Δt, y(t+1)=y(t)+v·sinθ·Δt, θ(t+1)=θ(t)+ω·Δt。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class RobotState:
    """机器人位姿（世界系）。v/omega 为当前执行速度（v_c, ω_c）。"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0


def wrap_angle(a: float) -> float:
    """角度归一化到 [-π, π)。对应 RobotModel.wrapAngle：
    mod(a+π, 2π) - π（MATLAB mod 为 floor 语义，非 fmod）。"""
    return (a + math.pi) - 2.0 * math.pi * math.floor((a + math.pi) / (2.0 * math.pi)) - math.pi


def new_robot(x: float, y: float, theta: float) -> RobotState:
    """newRobot.m 等价。"""
    return RobotState(x=x, y=y, theta=theta, v=0.0, omega=0.0)


class RobotModel:
    """RobotModel.m 等价：运动学积分与预测。"""

    def __init__(self, robot_radius: float = 0.1):
        self.radius = robot_radius

    def integrate(self, s: RobotState, v: float, omega: float, dt: float) -> RobotState:
        """公式(14)：运动学更新，返回新状态（v/omega 保留旧值，由调用方回写）。"""
        nxt = RobotState()
        nxt.x = s.x + v * math.cos(s.theta) * dt
        nxt.y = s.y + v * math.sin(s.theta) * dt
        nxt.theta = wrap_angle(s.theta + omega * dt)
        return nxt

    def predict_position(self, s: RobotState, v: float, omega: float, dt: float):
        """公式(14)前两式：一步预测位置 P_robot(t+1)（2×1 向量）。"""
        return [
            s.x + v * math.cos(s.theta) * dt,
            s.y + v * math.sin(s.theta) * dt,
        ]

    def predict_heading(self, s: RobotState, omega: float, dt: float) -> float:
        """公式(14)第三式：候选速度对应的一步预测朝向。"""
        return wrap_angle(s.theta + omega * dt)

    def wheel_to_robot_velocity(self, wheel_radius, wheel_base, vL, vR):
        """公式(2)：由左右轮角速度求 v/ω。"""
        v = 0.5 * wheel_radius * (vL + vR)
        omega = wheel_radius / wheel_base * (vR - vL)
        return v, omega
