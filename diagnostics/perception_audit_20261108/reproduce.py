"""Reproduce four current lidar detection defects without ROS or vehicle I/O.

Run with Python + NumPy. The assertions describe the observed defects, so a
future fix is expected to make these assertions fail until they are updated.
Production source and recordings are read only; this script writes results.json.
"""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src/rda_dwa_real"), str(ROOT / "src/rda_dwa")]
from rda_dwa_real.obstacle_fusion import Clusterer
from rda_dwa_real.perception_core import LidarPerceptionCore, PerceptionParams, point_cloud_xyz


def params():
    # Values used by real_vehicle.yaml; other fields use node/core defaults.
    return PerceptionParams(max_range=8., y_min=-8., y_max=8.)


def floor_points():
    x, y = np.meshgrid(np.linspace(-4., 4., 41), np.linspace(-4., 4., 41))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.81)))


def obstacle_points(xy):
    return np.column_stack((xy, np.full(len(xy), -.51)))


def detect(xy):
    return LidarPerceptionCore(params()).process(
        np.vstack((floor_points(), obstacle_points(xy))), 100.)


def enclosed(point, detections):
    for cx, cy, _, length, width, angle in detections:
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = point[0] - cx, point[1] - cy
        if abs(c * dx + s * dy) <= length / 2. + 1e-8 and abs(-s * dx + c * dy) <= width / 2. + 1e-8:
            return True
    return False


def synthetic_cases():
    # Non-extremal points carry the only cross-voxel edge (< 0.30 m).
    a = np.array([[1.001, 1.05], [1.099, 1.01], [1.05, 1.001], [1.01, 1.099], [1.098, 1.098]])
    b = np.array([[1.301, 1.39], [1.399, 1.35], [1.39, 1.301], [1.35, 1.399], [1.302, 1.302]])
    xy = np.vstack((a, b))
    exact = Clusterer(gate=.30, min_points=10).cluster([tuple(p) for p in xy])
    detections, metrics = detect(xy)
    assert len(exact) == 1 and metrics['cluster_count'] == 2 and not detections and metrics['valid']
    extrema = dict(raw_xy=xy.tolist(), exact_raw_components=len(exact), detections=detections,
                   closest_cross_voxel_m=float(np.sqrt(np.min(np.sum((a[:, None] - b[None, :]) ** 2, axis=2)))), metrics=metrics)

    # The parent has enough supports, every 1.2 m tile has fewer than ten.
    xy = np.column_stack((np.linspace(1., 5., 16), np.full(16, 2.)))
    exact = Clusterer(gate=.30, min_points=10).cluster([tuple(p) for p in xy])
    detections, metrics = detect(xy)
    assert len(exact) == 1 and metrics['cluster_count'] == 1 and metrics['structural_tiles'] == 5 and not detections
    sparse = dict(raw_xy=xy.tolist(), exact_raw_components=len(exact), detections=detections, metrics=metrics)

    # Empty 2 x 2 m U cavity has clearance from the actual 1.2 x 0.8 m car.
    edge = np.linspace(-1., 1., 41)
    xy = np.vstack((np.column_stack((np.ones(len(edge)), edge)),
                    np.column_stack((edge, np.ones(len(edge)))),
                    np.column_stack((edge, -np.ones(len(edge))))))
    detections, metrics = detect(xy)
    assert len(detections) == 1 and metrics['structural_tiles'] == 0 and enclosed((0., 0.), detections)
    assert not np.any((np.abs(xy[:, 0]) <= .6) & (np.abs(xy[:, 1]) <= .4))
    cavity = dict(raw_xy=xy.tolist(), detections=detections, cavity_centre_enclosed=True,
                  raw_supports_inside_car=0, metrics=metrics)

    # A known correct ground plane is replaced by a dense 30 cm platform top.
    core = LidarPerceptionCore(params())
    core.process(floor_points(), 100.)
    previous = core.plane.copy()
    x, y = np.meshgrid(np.linspace(1.5, 2.5, 51), np.linspace(-.5, .5, 51))
    xy = np.column_stack((x.ravel(), y.ravel()))
    detections, metrics = core.process(np.vstack((floor_points(), obstacle_points(xy))), 100.1)
    assert not detections and metrics['valid'] and abs(core.plane[2] - previous[2] - .30) < 1e-8
    platform = dict(platform_height_m=.30, platform_supports=len(xy),
                    previous_ground_plane=previous.tolist(), detections=detections, metrics=metrics)
    return dict(extrema_loses_raw_connectivity=extrema, sparse_tiles_drop_supported_parent=sparse,
                small_u_fills_empty_cavity=cavity, platform_top_replaces_ground=platform)


def recorded_cloud_sample(limit=30):
    sys.path.insert(0, str(ROOT / "diagnostics/engineering_20261108"))
    from cloud_stats import decode_cloud
    recording = ROOT / "record_data/run_19700109_151544_181276302/raw_bag/raw_bag_1.db3"
    with sqlite3.connect('file:' + str(recording) + '?mode=ro', uri=True) as db:
        topic = db.execute('select id from topics where name=?', ('/rslidar_points',)).fetchone()[0]
        rows = db.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit ?', (topic, limit)).fetchall()[::-1]
    core = LidarPerceptionCore(params())
    durations, counts, selected, reasons = [], [], [], {}
    for _, data in rows:
        cloud = decode_cloud(data)
        xyz = point_cloud_xyz(cloud)
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        started = time.monotonic()
        detections, metrics = core.process(xyz, stamp)
        durations.append(time.monotonic() - started)
        counts.append(len(detections))
        selected.append(metrics.get('cluster_input_points', 0))
        reasons[metrics['reason']] = reasons.get(metrics['reason'], 0) + 1
    return dict(scope='Last 30 recorded raw clouds, current core, fixed pose. Excludes ROS/DDS/KF and vehicle CPU; no obstacle ground truth.',
                frames=len(rows), reasons=reasons, detection_count_range=[min(counts), max(counts)],
                selected_points_range=[min(selected), max(selected)],
                duration_p50_ms=float(np.median(durations) * 1000.),
                duration_p95_ms=float(np.percentile(durations, 95) * 1000.))


if __name__ == '__main__':
    source_files = ['src/rda_dwa_real/rda_dwa_real/perception_core.py',
                    'src/rda_dwa_real/rda_dwa_real/lidar_track_node.py',
                    'src/rda_dwa_real/config/real_vehicle.yaml']
    report = dict(date='2026-11-08', python=sys.version, numpy=np.__version__,
                  params=asdict(params()),
                  source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in source_files},
                  scope='Review only; no production changes, ROS launch, or vehicle I/O.',
                  issues_reproduced=synthetic_cases(), recorded_sample=recorded_cloud_sample())
    Path(__file__).with_name('results.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(dict(issues_reproduced=list(report['issues_reproduced']), recorded_sample=report['recorded_sample']), indent=2, ensure_ascii=False))
