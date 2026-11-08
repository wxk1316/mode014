"""Pure orchestration: ego-compensated sweeps, geometry, tracking, semantic cache."""
from dataclasses import dataclass
import threading
import time
import numpy as np
from scipy.spatial import cKDTree

from .coordinates import matrix, transform_points, transform_detection, wrap
from .tracker import MultiObjectTracker


@dataclass
class CloudFrame:
    sequence: int
    stamp: float
    received: float
    points: np.ndarray
    pose: object
    sensor_to_base: np.ndarray
    source_frame: str = 'rslidar'
    source_age_at_receive: float = 0.
    stamp_ns: int | None = None

    @property
    def sensor_to_world(self):
        return matrix(self.pose)@self.sensor_to_base


class SemanticCache:
    def __init__(self, max_age=.35, confidence=.6, distance=.3):
        self.max_age,self.confidence,self.distance = max_age,confidence,distance
        self.lock = threading.Lock()
        self.tree = self.labels = self.scores = self.stamp = None
        self.reset_stamp = -float('inf')

    def clear(self, stamp):
        with self.lock:
            self.reset_stamp = max(self.reset_stamp, stamp)
            self.tree = self.labels = self.scores = self.stamp = None

    def update(self, frame, labels, scores):
        valid = (labels >= 0) & (scores >= self.confidence)
        points = transform_points(frame.points, frame.sensor_to_world)
        tree = cKDTree(points[valid,:2]) if valid.any() else None
        with self.lock:
            if frame.stamp <= self.reset_stamp or (self.stamp is not None and frame.stamp <= self.stamp):
                return False
            self.tree,self.labels,self.scores,self.stamp = tree,labels[valid],scores[valid],frame.stamp
        return True

    def road_mask(self, frame):
        with self.lock:
            tree,labels,stamp = self.tree,self.labels,self.stamp
        mask = np.zeros(len(frame.points), bool)
        if stamp is None or frame.stamp-stamp > self.max_age or frame.stamp < stamp or tree is None:
            return mask, dict(semantic_available=False,semantic_cache_available=False,
                              semantic_matched_fraction=0.,semantic_source_stamp=stamp)
        points = transform_points(frame.points, frame.sensor_to_world)
        distances,indices = tree.query(points[:,:2],distance_upper_bound=self.distance)
        matched = np.isfinite(distances)
        from .harp_backend import project
        matched &= project(frame.points)[1]
        mask[matched] = labels[indices[matched]] == 10
        return mask, dict(semantic_available=bool(matched.any()),semantic_cache_available=True,
                          semantic_matched_fraction=float(matched.mean()),semantic_source_stamp=stamp,
                          semantic_age=frame.stamp-stamp)


class PerceptionPipeline:
    def __init__(self, detector, geometry, semantic_cache=None, tracker=None,
                 past_frames=1, max_sweep_age=.25, ego_max_speed=5., ego_max_yaw_rate=2.):
        self.detector,self.geometry,self.semantic_cache = detector,geometry,semantic_cache
        self.tracker = tracker or MultiObjectTracker()
        self.history = []
        self.past_frames,self.max_sweep_age = past_frames,max_sweep_age
        self.ego_max_speed,self.ego_max_yaw_rate = ego_max_speed,ego_max_yaw_rate

    def process(self, frame):
        begin = time.monotonic()
        if self.tracker.last_stamp is not None and frame.stamp <= self.tracker.last_stamp:
            raise ValueError('repeated or out-of-order cloud')
        reset = False
        if self.history:
            old = self.history[-1]
            dt = frame.stamp-old.stamp
            jump = np.hypot(frame.pose.x-old.pose.x,frame.pose.y-old.pose.y)
            turn = abs(wrap(frame.pose.yaw-old.pose.yaw))
            reset = dt > self.tracker.config.reset_gap or jump > self.ego_max_speed*dt+.5 or turn > self.ego_max_yaw_rate*dt+.3
            if reset:
                self.history.clear();self.tracker.reset()
                if self.semantic_cache:self.semantic_cache.clear(frame.stamp)
        current = frame.points.copy()
        current[:,3] = 0.  # XYZT; intensity belongs only to HARP's independent input.
        sweeps = [current]
        inverse = np.linalg.inv(frame.sensor_to_world)
        for previous in self.history[-self.past_frames:] if self.past_frames else []:
            age = frame.stamp-previous.stamp
            if 0 < age <= self.max_sweep_age:
                p = transform_points(previous.points, inverse@previous.sensor_to_world)
                p[:,3] = age
                sweeps.append(p)
        phase = time.monotonic()
        boxes,detector_diag = self.detector.predict(np.concatenate(sweeps))
        inference_time = time.monotonic()-phase
        road,semantic_diag = (self.semantic_cache.road_mask(frame) if self.semantic_cache
                              else (None,dict(semantic_available=False)))
        phase = time.monotonic()
        unknown,grid,geometry_diag = self.geometry.run(frame.points,boxes,frame.sensor_to_base,road)
        geometry_time = time.monotonic()-phase
        detections = [transform_detection(d,frame.sensor_to_world) for d in boxes+unknown]
        phase = time.monotonic()
        tracks = self.tracker.step(detections,frame.stamp)
        tracking_time = time.monotonic()-phase
        self.history.append(frame)
        self.history = self.history[-max(1,self.past_frames):]
        return dict(sequence=frame.sequence,stamp=frame.stamp,
                    stamp_ns=frame.stamp_ns if frame.stamp_ns is not None else round(frame.stamp*1e9),
                    source_stamp=frame.stamp,source_frame=frame.source_frame,
                    frame_id='map',tracks=tracks,grid=grid,
                    duration_s=time.monotonic()-begin,inference_s=inference_time,
                    geometry_s=geometry_time,tracking_s=tracking_time,tracking_reset=bool(reset),
                    sweep_count=len(sweeps),neural_objects=len(boxes),
                    **detector_diag,**geometry_diag,**semantic_diag)

    def __call__(self,frame):
        return self.process(frame)

    def close(self):
        if hasattr(self.detector,'close'):
            self.detector.close()


class LatestWorker:
    """One active task and at most one pending frame; exceptions are observable."""
    def __init__(self, factory, name):
        self.factory,self.name = factory,name
        self.condition = threading.Condition()
        self.pending = self.result = None
        self.closed = False
        self.replaced = 0
        self.state = 'initializing'
        self.thread = threading.Thread(target=self._run,name=name,daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.condition:
            if self.closed:return
            if self.pending is not None:self.replaced += 1
            self.pending = frame
            self.condition.notify()

    def poll(self):
        with self.condition:
            result,self.result = self.result,None
            return result

    def _run(self):
        processor = None
        try:
            processor = self.factory()
            self.state = 'ready'
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.closed or self.pending is not None)
                    if self.closed:return
                    frame,self.pending = self.pending,None
                if frame.source_age_at_receive+time.monotonic()-frame.received > .4:
                    with self.condition:self.result = (frame,None,'input_expired_before_inference')
                    continue
                try:
                    output = processor(frame)
                    error = None
                except Exception as exc:
                    output,error = None,f'{type(exc).__name__}: {exc}'
                with self.condition:self.result = (frame,output,error)
        except Exception as exc:
            self.state = 'failed'
            with self.condition:self.result = (None,None,f'{type(exc).__name__}: {exc}')
        finally:
            if processor is not None and hasattr(processor,'close'):
                processor.close()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify()
        self.thread.join(timeout=3.)
