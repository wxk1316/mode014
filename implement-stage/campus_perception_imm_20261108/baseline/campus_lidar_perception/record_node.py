"""Separate real-vehicle comparison recorder; original recording code is untouched."""
from datetime import datetime
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


class PerceptionRecorder(Node):
    def __init__(self):
        super().__init__('campus_perception_recorder')
        self.declare_parameter('out_dir',str(Path.home()/'record_data/campus_perception'))
        self.declare_parameter('profile','centerpoint_harpnext')
        self.declare_parameter('grid_hz',5.)
        profile=str(self.get_parameter('profile').value)
        self.root=Path(str(self.get_parameter('out_dir').value))/f"{profile}_{datetime.now():%Y%m%d_%H%M%S}_{time.monotonic_ns()}"
        (self.root/'grids').mkdir(parents=True)
        self.stream=(self.root/'events.jsonl').open('w',buffering=1)
        self.last_grid=-1e9
        self.grid_period=1./float(self.get_parameter('grid_hz').value)
        self.grid_count=0
        self.root.joinpath('metadata.json').write_text(json.dumps(dict(profile=profile,
            grid_hz=1./self.grid_period,accuracy_evaluated=False,raw_pointcloud_bag='Use the original diag_recorder or ros2 bag record'),indent=2))
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String,'/campus_perception/tracks',lambda m:self._json('tracks',m),qos)
        self.create_subscription(String,'/rda/perception_diag',lambda m:self._json('diagnostic',m),qos)
        self.create_subscription(OccupancyGrid,'/rda/drivable_area',self._grid,qos)
        self.get_logger().info('Comparison records: '+str(self.root))

    def _write(self,kind,message):
        self.stream.write(json.dumps(dict(kind=kind,received_monotonic=time.monotonic(),
                                         received_ros_ns=self.get_clock().now().nanoseconds,
                                         message=message),allow_nan=False,separators=(',',':'))+'\n')

    def _json(self,kind,msg):
        try:self._write(kind,json.loads(msg.data))
        except (ValueError,TypeError) as exc:self.get_logger().warning(str(exc))

    def _grid(self,msg):
        if time.monotonic()-self.last_grid<self.grid_period:return
        self.last_grid=time.monotonic();self.grid_count+=1
        data=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
        name=f'grids/{self.grid_count:08d}.npz'
        np.savez_compressed(self.root/name,data=data)
        self._write('grid',dict(stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                    frame_id=msg.header.frame_id,file=name,resolution=msg.info.resolution,
                    origin=[msg.info.origin.position.x,msg.info.origin.position.y],
                    free=int((data==0).sum()),occupied=int((data==100).sum()),unknown=int((data<0).sum())))

    def destroy_node(self):
        self.stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=PerceptionRecorder()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
