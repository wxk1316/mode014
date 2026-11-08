"""Offline planner → tracker → bounded actuator scenarios using the active YAML.

The in-memory ROS doubles only construct configured objects. This simulation is
an engineering regression, not a vehicle model identification or DDS test.
"""
from __future__ import annotations

import importlib
import json
import math
import time
from pathlib import Path

from check_ros_adapters import ROOT, install_doubles


def clip_change(current, target, maximum):
    return current + max(-maximum, min(maximum, target-current))


def obstacle(case, t):
    # Return world centre, geometry, velocity heading and speed.
    if case in ('open', 'reverse_goal'):
        return []
    if case == 'static_block':
        return [(1, 2.5, 0., .6, .6, 0., 0., 'block')]
    if case == 'crossing':
        return [(1, 2., 1.8-t, .5, .5, -math.pi/2, 1., 'pedestrian')]
    if case == 'rear_blocked':
        return [(1, -1., 0., .4, .4, 0., 0., 'block')]
    raise ValueError(case)


def independent_circle_gap(x, y, yaw, ob, length, width, offset):
    # Independent point-to-rectangle calculation. The static test block is
    # evaluated with its enclosing circle, making this metric conservative.
    _, ox, oy, ol, ow, _, _, _ = ob
    centre_x, centre_y = x+offset*math.cos(yaw), y+offset*math.sin(yaw)
    dx, dy = ox-centre_x, oy-centre_y
    u = dx*math.cos(yaw)+dy*math.sin(yaw)
    v = -dx*math.sin(yaw)+dy*math.cos(yaw)
    return math.hypot(max(abs(u)-length/2, 0.), max(abs(v)-width/2, 0.))-math.hypot(ol/2, ow/2)


def run(case, duration=40.):
    bridge = importlib.import_module('rda_dwa_real.bridge_node').RealVehicleBridgeNode().core
    tracker = importlib.import_module('rda_dwa_real.traj_tracker_node').TrajTrackerNode().core
    dt = .02
    goal = (-2., 0.) if case in ('reverse_goal', 'rear_blocked') else (4., 0.)
    bridge.set_goal(*goal, frame='map')
    x = y = yaw = speed = steer = spin_speed = 0.
    target_mode = actual_mode = 0
    switch_started = -10.
    info = {'v_cmd': 0., 'w_cmd': 0., 'traj_mode': 0, 'spin_target': 0.}
    trajectory = [(0., 0., 0.)] * 60
    reference_seq = 0
    transitions = []
    steering_flips = 0
    last_sign = 0
    min_gap = math.inf
    maximum_steer_rate = 0.
    plan_durations = []
    reasons = {}
    previous_angle = 0.
    rows = []
    reached = False
    for tick in range(int(duration/dt)):
        t = tick*dt
        if actual_mode != target_mode and t-switch_started >= 3.:
            actual_mode = target_mode
        ready = actual_mode if actual_mode == target_mode else -1
        tracker.set_mode_ready(ready, t)
        tracker.set_speed_feedback(speed, t)
        bridge.set_mode_ready(ready, arrival=t)
        bridge.set_bridge_state(tracker.bridge_state())
        bridge.set_wheel_feedback([speed]*4, stamp=t)
        if tick % 5 == 0:
            bridge.set_localization(x, y, yaw, stamp=1000.+t, arrival=t)
            detections = [(ox, oy, typ, length, width, heading, velocity, ident)
                          for ident, ox, oy, length, width, heading, velocity, typ in obstacle(case, t)]
            bridge.set_obstacles(detections, stamp=1000.+t, frame='map', arrival=t)
            start = time.perf_counter()
            info = bridge.step(dt=.1, now=t)
            plan_durations.append(time.perf_counter()-start)
            reason = info.get('reason', 'motion')
            reasons[reason] = reasons.get(reason, 0)+1
            tracker.set_traj_mode(info.get('traj_mode', 0))
            trajectory = list(bridge._reference) or [(x, y, yaw)]*60
            reference_seq += 1
            if info.get('reached'):
                reached = True
        tracker.set_pose(x, y)
        # Synthetic source-rate observation for the production yaw-stop gate.
        # This is model feedback, not measured vehicle angular velocity.
        if tick % 5 == 0 and hasattr(tracker, 'set_yaw_feedback'):
            tracker.set_yaw_feedback(yaw, 1000.+t, t)
        command = tracker.step(info['v_cmd'], info['w_cmd'], yaw, trajectory,
                               info.get('spin_target', yaw), t, dt, reference_seq)
        bridge.set_tracker_cmd_speed(command['speed'], command['angle'],
                                     command['turnmode'], arrival=t)
        if command['turnmode'] != target_mode and ready != -1:
            target_mode = command['turnmode']
            switch_started = t
            transitions.append({'t': round(t, 3), 'target_mode': target_mode})
        live = actual_mode == target_mode
        target_v = command['speed'] if live and actual_mode == 0 else 0.
        accelerating = abs(target_v) > abs(speed) and target_v*speed >= 0.
        speed = clip_change(speed, target_v, (.5 if accelerating else 1.5)*dt)
        target_w = command['speed'] if live and actual_mode == 4 else 0.
        spin_speed = clip_change(spin_speed, target_w, .8*dt)
        steer = clip_change(steer, math.radians(command['angle']), math.radians(45)*dt)
        if actual_mode == 4:
            yaw += spin_speed*dt
        else:
            # The user confirmed the compensated localization point is the
            # geometric centre. Integrate the bicycle rear axle, then transform
            # back to that centre, assuming symmetric axles until calibrated.
            rear_x = x-.345*math.cos(yaw)
            rear_y = y-.345*math.sin(yaw)
            rear_x += speed*math.cos(yaw)*dt
            rear_y += speed*math.sin(yaw)*dt
            yaw += speed*math.tan(steer)/.69*dt
            x = rear_x+.345*math.cos(yaw)
            y = rear_y+.345*math.sin(yaw)
        if command['turnmode'] == 0 and abs(speed) > .05:
            sign = 1 if command['angle'] > .5 else -1 if command['angle'] < -.5 else 0
            if sign and last_sign and sign != last_sign:
                steering_flips += 1
            if sign:
                last_sign = sign
            maximum_steer_rate = max(maximum_steer_rate, abs(command['angle']-previous_angle)/dt)
        previous_angle = command['angle']
        for ob in obstacle(case, t+dt):
            gap = independent_circle_gap(x, y, yaw, ob, bridge.ack_model.p.length,
                                         bridge.ack_model.p.width, bridge.ack_model.p.footprint_center_offset)
            min_gap = min(min_gap, gap)
        if tick % 5 == 0:
            rows.append({'t': round(t, 3), 'x': x, 'y': y, 'yaw': yaw,
                         'speed': speed, 'angle': command['angle'],
                         'controller': tracker.mode, 'arbitration': bridge._arbit,
                         'reason': info.get('reason', ''), 'mode': actual_mode})
        if reached and speed < .001:
            break
    distances = sorted(plan_durations)
    summary = {
        'case': case, 'duration_s': round(t+dt, 3), 'reached': reached,
        'distance_to_goal_m': math.hypot(x-goal[0], y-goal[1]),
        'minimum_conservative_circle_gap_m': min_gap if math.isfinite(min_gap) else None,
        'mode_requests': transitions, 'steering_sign_flips_over_half_degree': steering_flips,
        'maximum_command_steer_rate_deg_s': maximum_steer_rate,
        'plan_p95_ms': distances[min(len(distances)-1, int(.95*len(distances)))]*1000,
        'plan_max_ms': max(distances)*1000, 'stop_reasons': reasons,
    }
    return summary, rows


def main():
    install_doubles()
    output = ROOT/'docs/validation'
    summaries = []
    for case in ('open', 'static_block', 'crossing', 'reverse_goal', 'rear_blocked'):
        summary, rows = run(case, duration=3. if case == 'rear_blocked' else 40.)
        summaries.append(summary)
        (output/f'{case}_trace.json').write_text(json.dumps(rows, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    (output/'closed_loop_results.json').write_text(json.dumps(summaries, indent=2)+'\n')
    for result in summaries[:4]:
        assert result['reached'], f"{result['case']}: failed to reach the goal"
    assert not summaries[0]['mode_requests'], 'Open space triggered a wheel-mode switch'
    assert summaries[0]['steering_sign_flips_over_half_degree'] == 0
    assert not summaries[2]['mode_requests'], 'A single crossing pedestrian triggered recovery'
    assert summaries[4]['distance_to_goal_m'] >= 1.98, 'Rear blockage allowed unsafe reverse motion'
    for result in summaries:
        gap = result['minimum_conservative_circle_gap_m']
        assert gap is None or gap > 0., f"{result['case']}: collision envelope violated"
        assert result['maximum_command_steer_rate_deg_s'] <= 45.01
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
