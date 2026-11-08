"""Shared DQN observation construction for training and inference."""
from __future__ import annotations

import math

import numpy as np


SECTOR_COUNT = 8
FOV = math.radians(240.0)
D_MAX = 3.5
DG_MAX = 6.0
V_MAX = 1.0
OMEGA_MAX = 1.5
VREL_MAX = 1.5
RISK_TTC_MAX = 6.0
RISK_UNCERTAINTY_MAX = 3.0


def _trap_goal(trap_active, sub_goal):
    return bool(trap_active and sub_goal is not None)


def _predicted_obstacle_position(obs, t, dt):
    if getattr(obs, 'arc_mode_hold', 0) > 0 and abs(getattr(obs, 'heading_rate', 0.0)) > 1e-9:
        n = max(1, int(round(t / dt)))
        step_dt = t / n
        x, y = obs.x, obs.y
        for i in range(1, n + 1):
            heading = obs.heading + obs.heading_rate * (i - 0.5) * step_dt
            x += obs.speed * math.cos(heading) * step_dt
            y += obs.speed * math.sin(heading) * step_dt
        return x, y
    return (obs.x + obs.speed * math.cos(obs.heading) * t,
            obs.y + obs.speed * math.sin(obs.heading) * t)


def _risk_features(robot, obstacles, dt):
    """Return TTC risk and three bounded uncertainty summaries.

    TTC uses discrete 4WIS rectangle-vs-obstacle-circle clearance under constant
    robot velocity and the predictor's constant-velocity/arc obstacle forecast.
    The robot heading is held fixed during this observation-only estimate.
    """
    min_ttc = RISK_TTC_MAX
    max_sigma = max_residual = max_divergence = 0.0
    count = max(1, int(math.ceil(RISK_TTC_MAX / dt)))
    c, s = math.cos(robot.theta), math.sin(robot.theta)
    for obs in obstacles:
        imm = getattr(obs, 'imm', {}) or {}
        covariance = imm.get('P_fused')
        if covariance is not None:
            try:
                cov = np.asarray(covariance, dtype=np.float64)
                sigma = math.sqrt(max(float(np.trace(cov)), 0.0))
                if math.isfinite(sigma):
                    max_sigma = max(max_sigma, sigma)
            except (TypeError, ValueError, OverflowError):
                pass
        residual = float(getattr(obs, 'res_err', 0.0) or 0.0)
        divergence = float(getattr(obs, 'div_err', 0.0) or 0.0)
        if math.isfinite(residual):
            max_residual = max(max_residual, max(residual, 0.0))
        if math.isfinite(divergence):
            max_divergence = max(max_divergence, max(divergence, 0.0))

        for i in range(1, count + 1):
            t = min(i * dt, RISK_TTC_MAX)
            rx = robot.x + robot.v * c * t
            ry = robot.y + robot.v * s * t
            ox, oy = _predicted_obstacle_position(obs, t, dt)
            dx, dy = ox - rx, oy - ry
            local_x = dx * c + dy * s
            local_y = -dx * s + dy * c
            gap_x = max(abs(local_x) - 0.5, 0.0)
            gap_y = max(abs(local_y) - 0.3, 0.0)
            if math.hypot(gap_x, gap_y) <= max(float(getattr(obs, 'radius', 0.15)), 0.0):
                min_ttc = min(min_ttc, t)
                break

    return (
        1.0 - min(min_ttc, RISK_TTC_MAX) / RISK_TTC_MAX,
        min(max_sigma / RISK_UNCERTAINTY_MAX, 1.0),
        min(max_residual / RISK_UNCERTAINTY_MAX, 1.0),
        min(max_divergence / RISK_UNCERTAINTY_MAX, 1.0),
    )


def make_state(robot, obstacles, goal, *, trap_active=False, sub_goal=None,
               input_dim=25, dt=0.1, use_prediction=False):
    """Build old 20D, target-aligned 21D, or risk-aware 25D observations.

    use_prediction=True: sector range/bearing use the predictor's 1-step
    predicted position (obs.fx/fy), i.e. model input is fully estimator-based
    (position/speed/risk) with no per-frame ground-truth position.
    use_prediction=False: measured position (obs.x/y) — legacy semantics for
    old 20D checkpoints trained on measured positions.
    """
    sectors_d = [D_MAX] * SECTOR_COUNT
    sectors_v = [0.0] * SECTOR_COUNT
    for obs in obstacles:
        ox = obs.fx if use_prediction else obs.x
        oy = obs.fy if use_prediction else obs.y
        dx, dy = ox - robot.x, oy - robot.y
        d = max(math.hypot(dx, dy), 0.05)
        bearing = math.atan2(dy, dx) - robot.theta
        wrapped = (bearing + math.pi) % (2 * math.pi) - math.pi
        if abs(wrapped) > FOV / 2:
            continue
        idx = max(0, min(SECTOR_COUNT - 1, int((bearing + FOV / 2) / FOV * SECTOR_COUNT)))
        if d < sectors_d[idx]:
            sectors_d[idx] = d
            world_bearing = robot.theta + bearing
            ovx = obs.speed * math.cos(obs.heading)
            ovy = obs.speed * math.sin(obs.heading)
            sectors_v[idx] = (ovx * math.cos(world_bearing) + ovy * math.sin(world_bearing)
                              - robot.v * math.cos(bearing))

    s_dist = [1.0 - min(d / D_MAX, 1.0) for d in sectors_d]
    s_vrel = [max(-1.0, min(1.0, v / VREL_MAX)) for v in sectors_v]
    dxg, dyg = goal[0] - robot.x, goal[1] - robot.y
    dg = math.hypot(dxg, dyg)
    bearing = math.atan2(dyg, dxg) - robot.theta
    bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
    state = s_dist + s_vrel + [min(dg / DG_MAX, 1.0), bearing / math.pi,
                                max(-1.0, min(1.0, robot.v / V_MAX)),
                                max(-1.0, min(1.0, robot.omega / OMEGA_MAX))]
    if input_dim >= 21:
        state.append(float(_trap_goal(trap_active, sub_goal)))
    if input_dim == 25:
        state.extend(_risk_features(robot, obstacles, dt))
    if input_dim >= 26:
        nearest = None
        nearest_distance = float('inf')
        for obs in obstacles:
            ox = obs.fx if use_prediction else obs.x
            oy = obs.fy if use_prediction else obs.y
            dx, dy = ox - robot.x, oy - robot.y
            distance = math.hypot(dx, dy)
            if distance < nearest_distance:
                nearest, nearest_distance = obs, distance
                nearest_dx, nearest_dy = dx, dy
        if nearest is None:
            state.extend([0.0, 1.0, 0.0])
        else:
            c, s = math.cos(robot.theta), math.sin(robot.theta)
            robot_vx, robot_vy = robot.v * c, robot.v * s
            obstacle_vx = nearest.speed * math.cos(nearest.heading)
            obstacle_vy = nearest.speed * math.sin(nearest.heading)
            lateral_relative_speed = (obstacle_vx - robot_vx) * (-s) + (obstacle_vy - robot_vy) * c
            bearing = math.atan2(nearest_dy, nearest_dx) - robot.theta
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            state.extend([
                max(-1.0, min(1.0, lateral_relative_speed / VREL_MAX)),
                min(nearest_distance / D_MAX, 1.0),
                bearing / math.pi,
            ])
        # robot.v/omega are the realized velocities at this decision, i.e. the
        # previous control cycle's state after vehicle dynamics.
        state.extend([
            max(-1.0, min(1.0, robot.v / V_MAX)),
            max(-1.0, min(1.0, robot.omega / OMEGA_MAX)),
        ])
    if input_dim not in (20, 21, 25, 26):
        raise ValueError(f'unsupported DQN input dimension: {input_dim}')
    return np.asarray(state, dtype=np.float32)


def effective_goal(goal, trap_active, sub_goal):
    return sub_goal if _trap_goal(trap_active, sub_goal) else goal
