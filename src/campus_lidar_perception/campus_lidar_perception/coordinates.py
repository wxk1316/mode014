"""Planar localisation contract and arbitrary static LiDAR mounting transform."""
from dataclasses import dataclass
import math
import numpy as np

from .types import Detection


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    z: float = 0.
    roll: float = 0.
    pitch: float = 0.


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def quat_yaw(x, y, z, w):
    q = np.array([x, y, z, w], float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-9:
        raise ValueError('invalid localization quaternion')
    x, y, z, w = q/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def quat_attitude(x,y,z,w):
    from scipy.spatial.transform import Rotation
    q=np.array([x,y,z,w],float)
    if not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:
        raise ValueError('invalid localization quaternion')
    return tuple(float(v) for v in Rotation.from_quat(q).as_euler('xyz'))


def rotation(roll=0., pitch=0., yaw=0.):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                            math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix(pose):
    t = np.eye(4)
    t[:3, :3] = rotation(pose.roll,pose.pitch,pose.yaw)
    t[:3, 3] = [pose.x, pose.y, pose.z]
    return t


def pose_jump(previous,current,dt,max_speed=5.,max_rotation_rate=2.):
    before,after=matrix(previous),matrix(current)
    if not np.isfinite(before).all() or not np.isfinite(after).all():return True
    displacement=np.linalg.norm(after[:3,3]-before[:3,3])
    relative=before[:3,:3].T@after[:3,:3]
    angle=math.acos(float(np.clip((np.trace(relative)-1)/2,-1.,1.)))
    elapsed=max(0.,dt)
    return bool(displacement>max_speed*elapsed+.5 or angle>max_rotation_rate*elapsed+.3)


def mounting(x=0., y=0., z=.45, roll=0., pitch=0., yaw=0.):
    t = np.eye(4)
    t[:3, :3] = rotation(roll, pitch, yaw)
    t[:3, 3] = [x, y, z]
    return t


def transform_points(points, transform):
    # World/map coordinates may be large; defer float32 conversion to the GPU input.
    result = np.array(points,dtype=np.result_type(points.dtype,transform.dtype),copy=True)
    result[:, :3] = points[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    return result


def transform_detection(d, transform):
    p = transform[:3, :3] @ [d.x, d.y, d.z] + transform[:3, 3]
    direction = transform[:3, :3] @ [math.cos(d.yaw), math.sin(d.yaw), 0.]
    yaw = math.atan2(direction[1], direction[0])
    # Project the full 3D box; a tilted sensor's vertical extent also contributes
    # to the planner footprint. Preserve an enclosing box along projected yaw.
    corners = np.array([(x,y,z) for x in (-d.length/2,d.length/2)
                        for y in (-d.width/2,d.width/2) for z in (-d.height/2,d.height/2)])
    corners = corners@rotation(yaw=d.yaw).T@transform[:3,:3].T
    local = corners@rotation(yaw=yaw)
    size = 2*np.max(np.abs(local),axis=0)
    return Detection(**{**d.__dict__, 'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                        'yaw': yaw, 'length': float(size[0]), 'width': float(size[1]),
                        'height': float(size[2])})


class PoseCache:
    """Duplicate ROS headers may be valid in the existing latest_topic contract.

    Freshness is advanced exclusively by real callback receipt, never by reads.
    """
    def __init__(self):
        self.pose = self.received = self.source_stamp = None

    def receive(self, pose, source_stamp, received):
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,pose.z,pose.roll,pose.pitch,received,source_stamp)):
            return False
        if self.received is not None and received <= self.received:
            return False
        self.pose, self.source_stamp, self.received = pose, source_stamp, received
        return True

    def sample(self, now, max_age):
        if self.received is None or now-self.received > max_age:
            return None
        return self.pose
