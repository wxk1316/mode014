"""Offline multi-mode action schedules with physical waiting and footprint risk.

Only ACK command/feedback=0 and SPIN=4 have user-confirmed production meaning.
CRAB/TRANSLATE are parameterized simulation hypotheses, disabled for deployment.
This module does not publish ROS messages, contact hardware, or alter localization.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class ModeProtocol:
    command_mode: int | None
    feedback_mode: int | None
    angle_unit: str | None
    speed_unit: str | None
    enabled: bool = False
    verified: bool = False
    max_angle_rad: float = math.pi/2


DEFAULT_PROTOCOLS = {
    'ACK': ModeProtocol(0,0,'degree','m/s',True,True,math.radians(30)),
    'SPIN': ModeProtocol(4,4,'degree','rad/s',True,True,0.),
    # User confirmed linear speed and +/-45 degree crab steering on 2026-11-06.
    # Command/ready IDs and positive directions remain unverified; keep disabled.
    'CRAB': ModeProtocol(None,None,'degree','m/s',False,False,math.radians(45)),
    # Translation has only a speed command. The simulated +/-pi/2 direction is
    # a body-frame hypothesis, not a hardware angle field or confirmed sign.
    'TRANSLATE': ModeProtocol(None,None,'unused','m/s',False,False,math.pi/2),
}


@dataclass
class ManeuverResult:
    status: str
    safe: bool
    trajectory: np.ndarray
    times: np.ndarray
    phases: list[str]
    minimum_gap_m: float | None
    nominal_duration_s: float
    protocol_verified: bool


def evaluate_maneuver(cc, robot, obstacles, mode='ACK', speed=.2, steering=0.,
                      yaw_change=0., motion_time=2., translation_angle=None,
                      protocols=None, simulation=False, ready_at_s=3.,
                      ready_lost_at_s=None, wait_min_s=3., ready_timeout_s=8.,
                      dt=.05, safety_margin=.2, uncertainty_rate=.05):
    """Stop → >=3s wheel switch + real ready → action → brake → recovery wait.

    Source obstacle positions are map positions at schedule time zero. All waits
    participate in the same future reachable-set collision check. A real-ready
    loss truncates the action into a bounded braking tail, never an instant ACK
    command. ACK reversal includes a stationary .2s confirmation hypothesis;
    deployment still uses the actual tracker measured-stop gate.
    """
    protocols = protocols or DEFAULT_PROTOCOLS
    protocol = protocols.get(mode)
    if translation_angle is None:
        translation_angle = math.pi/4 if mode == 'CRAB' else math.pi/2
    empty = np.empty((0,3)); zero = np.empty(0)
    if protocol is None or (not simulation and not (protocol.enabled and protocol.verified)):
        return ManeuverResult('protocol_unverified',False,empty,zero,[],None,0.,False)
    if mode not in DEFAULT_PROTOCOLS or dt<=0. or speed < cc.ack.v_min or speed > cc.ack.v_max:
        return ManeuverResult('invalid_request',False,empty,zero,[],None,0.,protocol.verified)
    if mode=='ACK' and abs(steering)>cc.ack.delta_max+1e-9:
        return ManeuverResult('steering_limit',False,empty,zero,[],None,0.,protocol.verified)
    if mode in ('CRAB','TRANSLATE') and abs(translation_angle)>protocol.max_angle_rad+1e-9:
        return ManeuverResult('angle_limit',False,empty,zero,[],None,0.,protocol.verified)
    pts = [[float(robot.x),float(robot.y),float(robot.theta)]]
    times = [0.]; phases = ['STOP']
    speed_cur = float(robot.v)
    delta = float(getattr(cc,'initial_delta',0.) or 0.)
    t=0.
    def append(pose,phase):
        nonlocal t
        t += dt; pts.append(list(pose)); times.append(t); phases.append(phase)
    def ack_tick(target_v,target_delta,phase):
        nonlocal speed_cur,delta
        from .ackermann_model import limit_signed_speed
        speed_cur = limit_signed_speed(target_v,speed_cur,cc.ack.a_max,cc.ack.dec_max,dt)
        delta += max(-cc.ack.delta_dot_max*dt,min(cc.ack.delta_dot_max*dt,target_delta-delta))
        x,y,yaw=pts[-1]
        offset=cc.ack.reference_rear_offset
        rx=x+offset*math.cos(yaw)+speed_cur*math.cos(yaw)*dt
        ry=y+offset*math.sin(yaw)+speed_cur*math.sin(yaw)*dt
        yaw += speed_cur*math.tan(delta)/cc.ack.wheelbase*dt
        append((rx-offset*math.cos(yaw),ry-offset*math.sin(yaw),yaw),phase)
    # Preserve present curvature while stopping before any wheel-mode request.
    while abs(speed_cur)>1e-9:
        ack_tick(0.,delta,'STOP')
    switching = mode!='ACK'
    if switching:
        wait = max(3.,wait_min_s,ready_at_s if ready_at_s is not None else ready_timeout_s)
        for _ in range(int(math.ceil(wait/dt))): append(pts[-1],'WAIT_READY')
        if ready_at_s is None or ready_at_s>ready_timeout_s:
            trajectory=np.asarray(pts);ts=np.asarray(times)
            safe=cc.time_collision_free(trajectory,ts,obstacles,safety_margin,uncertainty_rate)
            gap=cc.min_temporal_gap(trajectory,ts,obstacles,uncertainty_rate)
            return ManeuverResult('ready_timeout',False,trajectory,ts,phases,gap if math.isfinite(gap) else None,t,protocol.verified)
    elif robot.v*speed < 0.:
        for _ in range(int(math.ceil(.2/dt))): append(pts[-1],'DIRECTION_STOP_CONFIRM')
    action_started=t
    omega=0.; target_yaw=pts[-1][2]+yaw_change
    status='complete'
    steps=max(1,int(math.ceil(motion_time/dt)))
    if mode=='SPIN': steps=max(steps,int(math.ceil((abs(yaw_change)/.12+2.)/dt)))
    for _ in range(steps):
        if ready_lost_at_s is not None and t-action_started>=ready_lost_at_s:
            status='ready_lost';break
        if mode=='ACK':
            ack_tick(speed,steering,'ACTION_ACK_REVERSE' if speed<0 else 'ACTION_ACK_FORWARD')
        elif mode=='SPIN':
            err=target_yaw-pts[-1][2]
            target_omega=math.copysign(min(.8,math.sqrt(2*.8*abs(err))),err) if abs(err)>.005 else 0.
            omega += max(-.8*dt,min(.8*dt,target_omega-omega))
            x,y,yaw=pts[-1];append((x,y,yaw+omega*dt),'ACTION_SPIN')
            if abs(err)<.02 and abs(omega)<.05:break
        else:
            from .ackermann_model import limit_signed_speed
            speed_cur=limit_signed_speed(speed,speed_cur,cc.ack.a_max,cc.ack.dec_max,dt)
            x,y,yaw=pts[-1]
            angle=yaw+translation_angle
            append((x+speed_cur*math.cos(angle)*dt,y+speed_cur*math.sin(angle)*dt,yaw),'ACTION_'+mode)
    if mode=='ACK':
        while abs(speed_cur)>1e-9:ack_tick(0.,delta,'BRAKE')
    elif mode=='SPIN':
        while abs(omega)>1e-9:
            omega=math.copysign(max(0.,abs(omega)-.8*dt),omega)
            x,y,yaw=pts[-1];append((x,y,yaw+omega*dt),'BRAKE')
    else:
        while abs(speed_cur)>1e-9:
            speed_cur=math.copysign(max(0.,abs(speed_cur)-cc.ack.dec_max*dt),speed_cur)
            x,y,yaw=pts[-1];angle=yaw+translation_angle
            append((x+speed_cur*math.cos(angle)*dt,y+speed_cur*math.sin(angle)*dt,yaw),'BRAKE')
    if switching:
        # No assumed instant return to ACK. This is recovery dwell; actual ready
        # must still be confirmed by the production mechanical guard.
        for _ in range(int(math.ceil(max(3.,wait_min_s)/dt))):append(pts[-1],'RECOVERY_WAIT')
    trajectory=np.asarray(pts);ts=np.asarray(times)
    safe=cc.time_collision_free(trajectory,ts,obstacles,safety_margin,uncertainty_rate)
    gap=cc.min_temporal_gap(trajectory,ts,obstacles,uncertainty_rate)
    if not safe:status='unsafe_'+status
    return ManeuverResult(status,safe,trajectory,ts,phases,gap if math.isfinite(gap) else None,t,protocol.verified)
