"""CV Kalman filters and gated Hungarian assignment; no greedy fallback."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

from .types import Detection, planner_label
from .filters import CVKalman,IMMKalman


@dataclass
class TrackerConfig:
    gate: float = 1.2
    mahalanobis_gate: float = 13.82
    acceleration_variance: float = 2.0
    position_sigma: float = 0.15
    max_speed: float = 15.0
    lost_time: float = 0.4
    reset_gap: float = 2.0
    min_motion_hits: int = 4
    moving_threshold: float = 0.3
    stationary_threshold: float = 0.15
    stationary_hits: int = 6
    size_weight: float = 0.3
    semantic_memory_s: float = 1.0
    tracking_filter: str = 'cv'
    imm_stay_probability: float = .95
    imm_cv_probability: float = .7
    imm_jerk_variance: float = 6.
    imm_acceleration_variance: float = 4.

    def __post_init__(self):
        if self.tracking_filter not in ('cv','imm'):raise ValueError('tracking_filter must be cv or imm')
        positive=('gate','mahalanobis_gate','acceleration_variance','position_sigma','max_speed',
                  'lost_time','reset_gap','semantic_memory_s','imm_jerk_variance','imm_acceleration_variance')
        if any(not math.isfinite(getattr(self,k)) or getattr(self,k)<=0 for k in positive):
            raise ValueError('tracker limits/variances must be finite and positive')
        if not 0<self.imm_stay_probability<1 or not 0<self.imm_cv_probability<1:
            raise ValueError('IMM probabilities must be between zero and one')
        if self.min_motion_hits<1 or self.stationary_hits<1 or self.size_weight<0:
            raise ValueError('invalid tracker confirmation/size parameters')


class Track:
    def __init__(self, tid, detection, stamp, config):
        self.id, self.detection, self.config = tid, detection, config
        self.filter = CVKalman([detection.x,detection.y,0.,0.],
                               np.diag([config.position_sigma**2]*2+[4.]*2),config.acceleration_variance)
        self.last_observed = self.state[:2].copy()
        self.last_observed_stamp = stamp
        self.hits, self.misses = 1, 0
        self.moving, self.motion_hits, self.stop_hits = False, 0, 0
        self.semantic_label=detection.label
        self.semantic_score=detection.score if detection.label!='UNKNOWN' else None
        self.semantic_stamp=stamp if detection.label!='UNKNOWN' else None

    def predict(self, dt):
        previous=self.state[:2].copy()
        self.filter.predict(dt)
        self.filter.limit_speed(self.config.max_speed)
        self.filter.limit_displacement(previous,self.config.max_speed*dt)

    @property
    def state(self):return self.filter.state

    @property
    def covariance(self):return self.filter.covariance

    def _select_filter(self):
        desired=self.config.tracking_filter if self.moving else 'cv'
        if desired==self.filter.name:return
        state,p=self.state.copy(),self.covariance.copy()
        if desired=='cv':self.filter=CVKalman(state,p,self.config.acceleration_variance)
        else:self.filter=IMMKalman(state,p,self.config.acceleration_variance,self.config.imm_jerk_variance,
                                  self.config.imm_acceleration_variance,self.config.imm_stay_probability,
                                  self.config.imm_cv_probability)

    def update(self, detection, stamp):
        self.filter.update([detection.x,detection.y],self.config.position_sigma)
        self.filter.limit_speed(self.config.max_speed)
        self.last_observed = np.array([detection.x, detection.y])
        self.last_observed_stamp, self.misses = stamp, 0
        # A geometry-only observation must not erase an established semantic label.
        if detection.label == 'UNKNOWN':
            if self.semantic_stamp is not None and stamp-self.semantic_stamp<=self.config.semantic_memory_s:
                detection = Detection(**{**detection.__dict__, 'label': self.semantic_label})
        else:
            self.semantic_label,self.semantic_score,self.semantic_stamp=detection.label,detection.score,stamp
        self.detection, self.hits = detection, self.hits + 1
        speed = np.linalg.norm(self.state[2:])
        self.motion_hits = self.motion_hits+1 if speed >= self.config.moving_threshold else 0
        self.stop_hits = self.stop_hits+1 if speed <= self.config.stationary_threshold else 0
        if self.hits >= self.config.min_motion_hits and self.motion_hits >= self.config.min_motion_hits:
            self.moving = True
        if self.stop_hits >= self.config.stationary_hits:
            self.moving = False
        self._select_filter()

    def export(self, stamp):
        d = self.detection
        # Unknown/unconfirmed tracks stay at the latest measurement, as required by
        # the downstream static-obstacle contract. Only confirmed movers extrapolate.
        xy = self.state[:2] if self.moving else self.last_observed
        velocity = self.state[2:] if self.moving else np.zeros(2)
        speed = float(np.linalg.norm(velocity))
        semantic_age=stamp-self.semantic_stamp if self.semantic_stamp is not None else None
        semantic_label=self.semantic_label if semantic_age is not None and semantic_age<=self.config.semantic_memory_s else 'UNKNOWN'
        typ = planner_label(semantic_label) if self.moving else 'unknown'
        motion_proxy = self.moving and typ == 'unknown'
        if motion_proxy:
            typ = 'pedestrian'  # Existing planner has no moving-UNKNOWN enum.
        if not self.moving and self.stop_hits >= self.config.stationary_hits:
            typ = 'block' if max(d.length, d.width) <= .6 else 'static_large'
        length, width = d.length, d.width
        if self.moving:
            # Enclose the latest observed shape at the filtered centre as well.
            delta = self.last_observed-xy
            c, s = math.cos(d.yaw), math.sin(d.yaw)
            length += 2*abs(c*delta[0]+s*delta[1])
            width += 2*abs(-s*delta[0]+c*delta[1])
        return dict(id=self.id, x=float(xy[0]), y=float(xy[1]), z=d.z,
                    length=float(length), width=float(width), height=d.height,
                    heading=math.atan2(velocity[1], velocity[0]) if speed else 0.,
                    speed=speed, has_shape_heading=True, shape_heading=d.yaw,
                    type=typ, semantic_label=semantic_label, score=d.score,
                    semantic_score=self.semantic_score,semantic_observation_stamp=self.semantic_stamp,
                    last_semantic_label=self.semantic_label,semantic_valid=semantic_label!='UNKNOWN',
                    semantic_age=semantic_age,semantic_remembered=semantic_label!='UNKNOWN' and self.semantic_stamp!=stamp,
                    dynamic_unknown_proxy=motion_proxy,
                    vx=float(velocity[0]), vy=float(velocity[1]), moving=self.moving,
                    observation_stamp=self.last_observed_stamp,
                    observation_age=max(0., stamp-self.last_observed_stamp),
                    predicted=self.misses > 0, hits=self.hits, misses=self.misses,
                    covariance=self.covariance.tolist(), source=d.source,
                    configured_filter=self.config.tracking_filter,active_filter=self.filter.name,
                    filter_model_names=['CV','CA'] if self.filter.name=='imm' else ['CV'],
                    filter_model_probabilities=self.filter.probabilities,
                    acceleration=self.filter.acceleration)


class MultiObjectTracker:
    def __init__(self, config=None):
        self.config = config or TrackerConfig()
        self.tracks, self.last_stamp, self.next_id = {}, None, 1
        if self.config.gate <= 0 or self.config.position_sigma <= 0 or self.config.lost_time <= 0:
            raise ValueError('invalid tracker configuration')

    def reset(self):
        self.tracks.clear()
        self.last_stamp = None
        # IDs do not repeat after localization/time discontinuities.

    def step(self, detections, stamp):
        if not math.isfinite(stamp) or (self.last_stamp is not None and stamp <= self.last_stamp):
            raise ValueError('tracking requires increasing source timestamps')
        for detection in detections:
            detection.validate()
        dt = stamp-self.last_stamp if self.last_stamp is not None else 0.
        if dt > self.config.reset_gap:
            self.reset()
            dt = 0.
        self.last_stamp = stamp
        # Expire before matching, so a long unobserved gap cannot resurrect an ID.
        for tid in list(self.tracks):
            if stamp-self.tracks[tid].last_observed_stamp>self.config.lost_time:
                del self.tracks[tid]
        for t in self.tracks.values():
            t.predict(dt)
        ids = list(self.tracks)
        n, m = len(ids), len(detections)
        # Dummy rows/columns explicitly allow both unmatched tracks and detections.
        cost = np.full((n+m, n+m), 1e6)
        cost[n:, m:] = 0.
        for i in range(n):
            cost[i, m+i] = 2.
        for j in range(m):
            cost[n+j, j] = 2.
        allowed = np.zeros((n, m), bool)
        for i, tid in enumerate(ids):
            t = self.tracks[tid]
            elapsed = stamp-t.last_observed_stamp
            s = t.covariance[:2, :2] + np.eye(2)*self.config.position_sigma**2
            for j, d in enumerate(detections):
                previous = planner_label(t.detection.label)
                current = planner_label(d.label)
                if previous != current and previous != 'unknown' and current != 'unknown':
                    continue
                delta = np.array([d.x, d.y])-t.state[:2]
                residual = float(np.linalg.norm(delta))
                reachable = np.linalg.norm(np.array([d.x, d.y])-t.last_observed)
                mahal = float(delta @ np.linalg.solve(s, delta))
                if (residual > self.config.gate+self.config.max_speed*min(elapsed, .3)
                        or reachable > self.config.gate+self.config.max_speed*elapsed
                        or mahal > self.config.mahalanobis_gate):
                    continue
                size = abs(math.log(d.length/t.detection.length))+abs(math.log(d.width/t.detection.width))
                cost[i, j] = min(3.9, math.sqrt(mahal/self.config.mahalanobis_gate)
                                 + self.config.size_weight*size)
                allowed[i, j] = True
        matched_tracks, matched_detections = set(), set()
        if n+m:
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if i < n and j < m and allowed[i, j]:
                    self.tracks[ids[i]].update(detections[j], stamp)
                    matched_tracks.add(ids[i])
                    matched_detections.add(j)
        for tid in ids:
            if tid not in matched_tracks:
                self.tracks[tid].misses += 1
            if stamp-self.tracks[tid].last_observed_stamp > self.config.lost_time:
                del self.tracks[tid]
        for j, d in enumerate(detections):
            if j not in matched_detections:
                if self.next_id >= 2**31:
                    raise OverflowError('planner int32 track IDs exhausted')
                self.tracks[self.next_id] = Track(self.next_id, d, stamp, self.config)
                self.next_id += 1
        return [self.tracks[tid].export(stamp) for tid in sorted(self.tracks)]
