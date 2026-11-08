"""Ground/curb/dropoff geometry independent of closed-set neural detections."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from .coordinates import transform_points
from .types import Detection


@dataclass
class GeometryConfig:
    max_range: float = 30.
    voxel: float = .10
    cluster_distance: float = .30
    min_cluster_points: int = 3
    min_extent: float = .08
    ground_reference_z: float = -.36
    ground_tolerance: float = .12
    min_ground_points: int = 50
    obstacle_height: float = .06
    dropoff_height: float = .06
    max_height: float = 2.5
    max_slope: float = .25
    resolution: float = .20
    drivable_height: float = .035
    ground_points_per_cell: int = 2
    structure_tile: float = 1.2
    vehicle_length: float = 1.2
    vehicle_width: float = .8
    max_points: int = 25000
    max_edges: int = 2000000


class PatchworkGround:
    """Real Patchwork++ C++ binding. Missing dependencies are a startup error."""
    def __init__(self, sensor_height=.36):
        import pypatchworkpp
        params = pypatchworkpp.Parameters()
        params.sensor_height = sensor_height
        params.verbose = False
        self.algorithm = pypatchworkpp.patchworkpp(params)

    def indices(self, points):
        self.algorithm.estimateGround(np.ascontiguousarray(points, dtype=np.float64))
        return np.asarray(self.algorithm.getGroundIndices(), dtype=np.int64).reshape(-1)


def points_in_box(points, d, padding=.05):
    delta = points[:, :3] - [d.x, d.y, d.z]
    c, s = math.cos(d.yaw), math.sin(d.yaw)
    return ((np.abs(c*delta[:, 0]+s*delta[:, 1]) <= d.length/2+padding)
            & (np.abs(-s*delta[:, 0]+c*delta[:, 1]) <= d.width/2+padding)
            & (np.abs(delta[:, 2]) <= d.height/2+padding))


def fit_ground(points, indices, cfg):
    ground = points[indices, :3]
    near = (np.linalg.norm(ground[:, :2], axis=1) < 15.)
    ground = ground[near & (np.abs(ground[:, 2]-cfg.ground_reference_z) < cfg.ground_tolerance)]
    if len(ground) < cfg.min_ground_points:
        raise ValueError('ground_unreliable: insufficient calibrated ground support')
    # Fit the lower ground surface, not a raised pavement that Patchwork may accept.
    cells = np.floor(ground[:, :2]/.5).astype(np.int64)
    order = np.lexsort((ground[:, 2], cells[:, 1], cells[:, 0]))
    g, cells = ground[order], cells[order]
    first = np.r_[True, np.any(cells[1:] != cells[:-1], axis=1)]
    support = g[first]
    if len(support) < 8 or min(np.ptp(support[:, :2], axis=0)) < 1.:
        raise ValueError('ground_unreliable: insufficient spatial support')
    a = np.column_stack((support[:, :2], np.ones(len(support))))
    plane = np.linalg.lstsq(a, support[:, 2], rcond=None)[0]
    for _ in range(3):
        residual = support[:, 2]-a@plane
        keep = np.abs(residual) < .04
        if np.count_nonzero(keep) < 8:
            raise ValueError('ground_unreliable: non-planar local road')
        plane = np.linalg.lstsq(a[keep], support[keep, 2], rcond=None)[0]
    if (np.linalg.norm(plane[:2]) > cfg.max_slope
            or abs(plane[2]-cfg.ground_reference_z) > cfg.ground_tolerance):
        raise ValueError('ground_unreliable: slope/height outside calibrated limits')
    return plane


def _voxel_points(points, size, maximum):
    for _ in range(5):
        voxels = np.floor(points[:, :3]/size).astype(np.int64)
        _, indices = np.unique(voxels, axis=0, return_index=True)
        if len(indices) <= maximum:
            return points[np.sort(indices)]
        size *= 1.3
    raise ValueError('geometry_point_budget_exceeded')


def cluster_geometry(points, cfg):
    if not len(points):
        return []
    points = _voxel_points(points, cfg.voxel, cfg.max_points)
    tree = cKDTree(points[:, :2])
    # Count before materialising dense neighbour pairs.
    if tree.count_neighbors(tree, cfg.cluster_distance) > cfg.max_edges*2+len(points):
        raise ValueError('geometry_edge_budget_exceeded')
    pairs = tree.query_pairs(cfg.cluster_distance, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(points), len(points)))
    _, labels = connected_components(graph, directed=False)
    detections = []
    for label in np.unique(labels):
        group = points[labels == label]
        if len(group) < cfg.min_cluster_points:
            continue
        # Tile extended structures so a U-shaped fence is not represented by one
        # filled box covering its empty interior. DNN objects keep their own boxes.
        if max(np.ptp(group[:, :2], axis=0)) > 3.5:
            tile_ids = np.floor(group[:, :2]/cfg.structure_tile).astype(np.int64)
            _, inverse = np.unique(tile_ids, axis=0, return_inverse=True)
            groups = [group[inverse == i] for i in np.unique(inverse)]
        else:
            groups = [group]
        for g in groups:
            if len(g) < cfg.min_cluster_points:
                continue
            centre = g[:, :2].mean(axis=0)
            covariance = (g[:, :2]-centre).T@(g[:, :2]-centre)
            _, directions = np.linalg.eigh(covariance)
            u = directions[:, -1]
            v = np.array([-u[1], u[0]])
            axes = np.column_stack((u, v))
            aligned = (g[:, :2]-centre)@axes
            lo, hi = aligned.min(axis=0), aligned.max(axis=0)
            size = hi-lo
            if max(size) < cfg.min_extent:
                continue
            xy = centre+axes@((lo+hi)/2)
            detections.append(Detection(x=float(xy[0]), y=float(xy[1]),
                                        z=float((g[:, 2].max()+g[:, 2].min())/2),
                                        length=max(.10, float(size[0])+.10),
                                        width=max(.10, float(size[1])+.10),
                                        height=max(.10, float(np.ptp(g[:, 2]))),
                                        yaw=math.atan2(u[1], u[0]), source='geometry'))
    return detections


def build_grid(points_base, relief, cfg, road=None):
    size = int(math.ceil(2*cfg.max_range/cfg.resolution))
    cells = np.floor((points_base[:, :2]+cfg.max_range)/cfg.resolution).astype(np.int64)
    valid = np.all((cells >= 0) & (cells < size), axis=1)
    cells, relief = cells[valid], relief[valid]
    flat = cells[:, 1]*size+cells[:, 0]
    n = size*size
    obstacle = (relief >= cfg.obstacle_height) | (relief <= -cfg.dropoff_height)
    counts = np.bincount(flat, minlength=n)
    ground = np.abs(relief) <= cfg.drivable_height
    if road is not None:
        ground &= road[valid]
    ground_count = np.bincount(flat[ground], minlength=n)
    blocked = np.bincount(flat[obstacle], minlength=n) > 0
    grid = np.full(n, -1, np.int8)
    # All observed returns in a free cell must meet geometric height constraints.
    free = ((ground_count >= cfg.ground_points_per_cell) & (ground_count == counts) & ~blocked)
    grid[free] = 0
    blocked = binary_dilation(blocked.reshape(size, size), iterations=1).reshape(-1)
    grid[blocked] = 100
    return dict(resolution=cfg.resolution, width=size, height=size,
                origin_x=-cfg.max_range, origin_y=-cfg.max_range,
                data=grid.tolist(), semantic=road is not None)


class GeometryDetector:
    def __init__(self, ground, config=None):
        self.ground, self.config = ground, config or GeometryConfig()

    def run(self, points, neural_boxes, sensor_to_base, road=None):
        cfg = self.config
        base = transform_points(points, sensor_to_base)
        # Self-filter in the physical body frame, before either geometry output.
        body = ((np.abs(base[:, 0]) <= cfg.vehicle_length/2+.02)
                & (np.abs(base[:, 1]) <= cfg.vehicle_width/2+.02))
        use = ~body & (np.linalg.norm(base[:, :2], axis=1) <= cfg.max_range)
        p, b = points[use], base[use]
        if not len(p):
            raise ValueError('no points outside vehicle')
        indices = self.ground.indices(p)
        if np.any((indices < 0) | (indices >= len(p))):
            raise ValueError('invalid Patchwork ground indices')
        plane = fit_ground(p, indices, cfg)
        relief = p[:, 2]-(p[:, :2]@plane[:2]+plane[2])
        obstacle = (((relief >= cfg.obstacle_height) & (relief <= cfg.max_height))
                    | (relief <= -cfg.dropoff_height))
        # Remove only individual points covered by a current DNN box. Neighbouring
        # unknown objects remain, even if one cluster spans both objects.
        covered = np.zeros(len(p), bool)
        for d in neural_boxes:
            covered |= points_in_box(p, d)
        unknown = cluster_geometry(p[obstacle & ~covered], cfg)
        grid = build_grid(b, relief, cfg, None if road is None else road[use])
        return unknown, grid, dict(ground_plane=plane.tolist(), ground_points=len(indices),
                                    geometry_obstacle_points=int(obstacle.sum()), unknown_objects=len(unknown))
