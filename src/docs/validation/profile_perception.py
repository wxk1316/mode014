"""Repeatable 80k-point full-surround benchmark; no ROS or vehicle output."""
from pathlib import Path
import cProfile
import io
import json
import math
import random
import pstats
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'rda_dwa'),str(ROOT/'rda_dwa_real')]
from rda_dwa_real.frame_utils import filter_lidar_points
from rda_dwa_real.obstacle_fusion import Clusterer,WorldObstacleRepresenter


def run(scenario='dense_80k'):
    if scenario == 'dense_80k':
        points = [(-12+(i%283)*24/282,-12+(i//283)*24/282,-.2) for i in range(80000)]
    elif scenario == 'dense_100k':
        rng = random.Random(2026)
        points = [(rng.uniform(-12,12),rng.uniform(-12,12),-.2) for _ in range(100000)]
    elif scenario == 'long_wall':
        points = [(-10+(i%2001)*.01,3,-.2) for i in range(80000)]
    elif scenario == 'raw_u':
        points = [(-1.8,-1.3+i*.05,-.2) for i in range(53)]
        points += [(-1.8+i*.05,y,-.2) for y in (-1.3,1.3) for i in range(97)]
    else:
        raise ValueError(scenario)
    start = time.perf_counter()
    kept = filter_lidar_points(points,max_range=12,x_min=-12,x_max=12,
        y_min=-12,y_max=12,half_length=.6,half_width=.4,
        vehicle_center_x=0)
    filtered = time.perf_counter()
    rx,ry,yaw = 50,-40,.35
    c,s = math.cos(yaw),math.sin(yaw)
    world = [(rx+c*x-s*y,ry+s*x+c*y) for x,y in kept]
    transformed = time.perf_counter()
    clusters = Clusterer(gate=.6,max_radius=30).cluster(world,origin=(rx,ry))
    clustered = time.perf_counter()
    representer = WorldObstacleRepresenter()
    output = representer.step(clusters,100,reference_origin=(rx,ry),vehicle_pose=(rx,ry,yaw))
    finished = time.perf_counter()
    return {
        'observation_radius_m':12,'input_points':len(points),'kept_points':len(kept),
        'scenario':scenario,'parents':len(clusters),'output_obstacles':len(output),
        'stats':representer.stats,'filter_seconds':filtered-start,
        'world_transform_seconds':transformed-filtered,
        'cluster_seconds':clustered-transformed,'representation_and_kf_seconds':finished-clustered,
        'total_seconds':finished-start,
        'scope':'macOS pure Python; includes filtering/world transform/clustering/parent KF/world tiles; excludes ROS PointCloud2 decoding, markers, DDS and vehicle CPU contention',
    }


if __name__ == '__main__':
    result = {scenario:run(scenario) for scenario in ('dense_80k','dense_100k','long_wall','raw_u')}
    if '--profile' in sys.argv:
        profiler = cProfile.Profile()
        profiler.enable()
        run()
        profiler.disable()
        stream = io.StringIO()
        pstats.Stats(profiler,stream=stream).sort_stats('cumtime').print_stats(12)
        result['profile'] = stream.getvalue()
    output = Path(__file__).with_name('perception_performance.json')
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
