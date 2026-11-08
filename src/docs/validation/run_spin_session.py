"""Closed-loop yaw session with w=0 and a simulated 3-second mode actuator.

Uses the actual YAML tracker configuration. No ROS/DDS/UDP is started.
"""
from __future__ import annotations
import importlib
import json
import math
from check_ros_adapters import ROOT, install_doubles


def main():
    install_doubles()
    core = importlib.import_module('rda_dwa_real.traj_tracker_node').TrajTrackerNode().core
    dt, yaw, target = .02, 0., math.pi/2
    actual_mode = requested_mode = 0
    request_at = -3.
    core.set_traj_mode(1)
    requests, transitions, trace = [], [], []
    first_rotation = None
    spin_requested = ack_requested = None
    first_ack = None
    moved_with_zero_planner_w = 0
    reached = False
    previous_state = None
    for tick in range(1000):
        t = tick*dt
        if actual_mode != requested_mode and t-request_at >= 3.:
            actual_mode = requested_mode
        ready = actual_mode if actual_mode == requested_mode else -1
        core.set_mode_ready(ready, t)
        core.set_speed_feedback(0., t)
        core.set_pose(0., 0.)
        # Inject a new 10Hz synthetic localization observation. Repeated control
        # ticks do not count as new source feedback for physical stop readiness.
        if tick % 5 == 0 and hasattr(core, 'set_yaw_feedback'):
            core.set_yaw_feedback(yaw, 1000.+t, t)
        # A normal ACK intent during RUN is not a physical yaw completion.
        if 4. <= t < 4.2:
            core.set_traj_mode(0)
        elif not reached:
            core.set_traj_mode(1)
        command = core.step(0., 0., yaw, [(0.,0.,yaw)]*60, target, t, dt, tick//5)
        state = (core.mode, core._spin_stage)
        if state != previous_state:
            transitions.append({'t': t, 'mode': state[0], 'stage': state[1]})
            previous_state = state
        if core.bridge_state() == 3:
            reached = True
            core.set_traj_mode(0)
        if command['turnmode'] != requested_mode and ready != -1:
            requested_mode = command['turnmode']
            request_at = t
            requests.append({'t': t, 'mode': requested_mode})
            if requested_mode == 4:
                spin_requested = t
            else:
                ack_requested = t
        speed = command['speed'] if ready == 4 and command['turnmode'] == 4 else 0.
        if abs(speed) > 1e-9:
            if first_rotation is None:
                first_rotation = t
            moved_with_zero_planner_w += 1
        yaw += speed*dt
        trace.append({'t': t, 'yaw': yaw, 'yaw_error_deg': math.degrees(target-yaw),
                      'physical_mode': actual_mode, 'request_mode': requested_mode,
                      'controller': core.mode, 'stage': core._spin_stage,
                      'spin_speed': speed, 'planner_w': 0., 'bridge_state': core.bridge_state()})
        if reached and core.mode == core.MODE_ACK and actual_mode == 0:
            first_ack = t
            break
    result = {
        'passed': bool(reached and first_ack is not None),
        'mode_requests': requests, 'transitions': transitions,
        'first_actual_rotation_s': first_rotation,
        'spin_request_s': spin_requested, 'ack_request_s': ack_requested,
        'ack_resumed_s': first_ack, 'final_error_deg': math.degrees(target-yaw),
        'motion_ticks_with_planner_w_zero': moved_with_zero_planner_w,
        'scope': 'Configured tracker + simulated yaw/mode actuator; no ROS/DDS/UDP',
    }
    folder = ROOT/'docs/validation'
    (folder/'spin_session_results.json').write_text(json.dumps(result,indent=2)+'\n')
    (folder/'spin_session_trace.json').write_text(json.dumps(trace,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    assert result['passed']
    assert first_rotation-spin_requested >= 3.-1e-6
    assert first_ack-ack_requested >= 3.-1e-6
    assert abs(result['final_error_deg']) <= core.p.rotate_precision_deg+1e-6
    assert moved_with_zero_planner_w > 0
    assert len(requests) == 2 and [r['mode'] for r in requests] == [4,0]
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
