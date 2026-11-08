"""ROS2 adapter: independent inference workers -> map tracks -> existing planner."""
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rda_dwa_real_msgs.msg import Obstacle, ObstacleArray
from rcl_interfaces.msg import SetParametersResult

from .coordinates import Pose,PoseCache,quat_attitude,mounting,wrap,rotation,pose_jump
from .pointcloud import read_xyzi
from .pipeline import CloudFrame,LatestWorker,PerceptionPipeline,SemanticCache
from .interfaces import obstacle_array


class CampusPerceptionNode(Node):
    def __init__(self):
        super().__init__('campus_lidar_perception')
        share = Path(get_package_share_directory('campus_lidar_perception'))
        defaults = dict(profile='centerpoint_harpnext',points_topic='/rslidar_points',
                        odom_topic='/localization',obstacles_topic='/rda/obstacles',
                        drivable_topic='/rda/drivable_area',model_dir=str(share/'models'),
                        native_library='',engine_dir=os.path.expanduser('~/.cache/campus_lidar_perception/engines'),
                        localization_source='latest_available',localization_parent_frame='map',
                        localization_child_frame='body',base_frame='base_link',sensor_frame='rslidar',
                        body_to_base_link_x=0.,sensor_x=0.,sensor_y=0.,sensor_z=.45,
                        sensor_roll=0.,sensor_pitch=0.,sensor_yaw=0.,loc_timeout=.35,
                        max_source_age=.4,semantic_hz=5.,semantic_max_age=.35,semantic_confidence=.6,
                        max_range=30.,ground_reference_z=-.36,ground_tolerance=.12,
                        ground_sensor_height=.36,obstacle_height=.06,dropoff_height=.06,
                        vehicle_length=1.2,vehicle_width=.8,past_frames=1,
                        intensity_scale=1.,score_thresholds=[.35]*5,
                        tracker_gate=1.2,tracker_max_speed=15.,tracker_lost_time=.4,
                        tracking_filter='cv',imm_stay_probability=.95,imm_cv_probability=.7,
                        imm_jerk_variance=6.,imm_acceleration_variance=4.,
                        self_filter_min_z=-1.,self_filter_max_z=.6,
                        marker_history_length=20,marker_velocity_horizon=1.,
                        publish_planner=True,strict_single_publisher=True)
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.parameters = {k:self.get_parameter(k).value for k in defaults}
        cfg = self.parameters
        from .tracker import TrackerConfig
        TrackerConfig(tracking_filter=cfg['tracking_filter'],imm_stay_probability=cfg['imm_stay_probability'],
                      imm_cv_probability=cfg['imm_cv_probability'],imm_jerk_variance=cfg['imm_jerk_variance'],
                      imm_acceleration_variance=cfg['imm_acceleration_variance'])
        for key in ('max_source_age','loc_timeout','max_range','semantic_hz','semantic_max_age','ground_sensor_height'):
            if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid parameter '+key)
        if not math.isfinite(cfg['intensity_scale']) or cfg['intensity_scale']<=0:raise ValueError('invalid intensity_scale')
        if not cfg['sensor_frame'] or cfg['sensor_frame']==cfg['base_frame']:
            raise ValueError('sensor_frame must identify the physical LiDAR origin')
        if not math.isfinite(cfg['self_filter_min_z']) or not math.isfinite(cfg['self_filter_max_z']) or cfg['self_filter_min_z']>=cfg['self_filter_max_z']:
            raise ValueError('invalid self-filter vertical bounds')
        if cfg['profile'] not in ('centerpoint_geometry','centerpoint_harpnext','centerpoint_harp'):
            raise ValueError('Unknown profile; use centerpoint_geometry or centerpoint_harpnext')
        if cfg['localization_source'] not in ('latest_topic','latest_available','tf'):
            raise ValueError('localization_source must match the existing latest/TF contract')
        if cfg['localization_parent_frame']!='map':
            raise ValueError('The existing planner requires map-frame obstacles')
        self.semantic_enabled = cfg['profile']!='centerpoint_geometry'
        self.pose_cache = PoseCache()
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.fallback_mount = mounting(cfg['sensor_x'],cfg['sensor_y'],cfg['sensor_z'],
                                       cfg['sensor_roll'],cfg['sensor_pitch'],cfg['sensor_yaw'])
        if not np.isfinite(self.fallback_mount).all():raise ValueError('non-finite mounting parameters')
        self.semantic_cache = SemanticCache(cfg['semantic_max_age'],cfg['semantic_confidence']) if self.semantic_enabled else None
        self.sequence = 0
        self.last_received_stamp = self.last_semantic_submit = None
        self.last_diag = dict(reason='initializing',profile=cfg['profile'])
        self._last_warning = 0.
        root = Path(cfg['model_dir'])
        required = [root/'centerpoint_base/pts_voxel_encoder.onnx',root/'centerpoint_base/pts_backbone_neck_head.onnx']
        if self.semantic_enabled:required += [root/'harpnext_nuscenes/weights.pth',root/'harpnext_nuscenes/net.yaml']
        for path in required:
            if not path.is_file():raise RuntimeError('Model artifact missing: '+str(path))
        native = Path(cfg['native_library']) if cfg['native_library'] else Path(__file__).parent/'lib/libcampus_centerpoint.so'
        if not native.is_file():raise RuntimeError('Build the native Orin backend first: '+str(native))
        # Factories import/construct CUDA consumers inside their worker threads.
        def detection_factory():
            from .centerpoint_backend import CenterPointBackend
            from .geometry import GeometryDetector,GeometryConfig,PatchworkGround
            from .tracker import MultiObjectTracker,TrackerConfig
            detector = CenterPointBackend(root/'centerpoint_base',cfg['engine_dir'],str(native),cfg['score_thresholds'],cfg['max_range'])
            geometry = GeometryDetector(PatchworkGround(cfg['ground_sensor_height']),GeometryConfig(
                max_range=cfg['max_range'],ground_reference_z=cfg['ground_reference_z'],
                ground_tolerance=cfg['ground_tolerance'],obstacle_height=cfg['obstacle_height'],
                dropoff_height=cfg['dropoff_height'],vehicle_length=cfg['vehicle_length'],vehicle_width=cfg['vehicle_width'],
                self_filter_min_z=cfg['self_filter_min_z'],self_filter_max_z=cfg['self_filter_max_z']))
            tracker = MultiObjectTracker(TrackerConfig(gate=cfg['tracker_gate'],max_speed=cfg['tracker_max_speed'],lost_time=cfg['tracker_lost_time'],
                tracking_filter=cfg['tracking_filter'],imm_stay_probability=cfg['imm_stay_probability'],
                imm_cv_probability=cfg['imm_cv_probability'],imm_jerk_variance=cfg['imm_jerk_variance'],
                imm_acceleration_variance=cfg['imm_acceleration_variance']))
            return PerceptionPipeline(detector,geometry,self.semantic_cache,tracker,past_frames=cfg['past_frames'])
        self.worker = LatestWorker(detection_factory,'centerpoint_geometry_worker',cfg['max_source_age'])
        self.semantic_worker = None
        if self.semantic_enabled:
            def semantic_factory():
                from .harp_backend import HarpNeXtBackend
                backend = HarpNeXtBackend(root/'harpnext_nuscenes')
                def process(frame):
                    begin = time.monotonic()
                    labels,scores = backend.predict(frame.points)
                    prediction_s = time.monotonic()-begin
                    self.semantic_cache.update(frame,labels,scores)
                    return dict(stamp=frame.stamp,duration_s=time.monotonic()-begin,
                                prediction_s=prediction_s,
                                scatter_backend=backend.scatter_backend,
                                classified_points=int(np.count_nonzero(labels>=0)))
                return process
            self.semantic_worker = LatestWorker(semantic_factory,'harpnext_worker',cfg['max_source_age'])
        input_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        output_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.obstacle_pub = self.create_publisher(ObstacleArray,cfg['obstacles_topic'],output_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid,cfg['drivable_topic'],output_qos)
        self.tracks_pub = self.create_publisher(String,'/campus_perception/tracks',output_qos)
        self.diag_pub = self.create_publisher(String,'/rda/perception_diag',output_qos)
        self.marker_pub = self.create_publisher(MarkerArray,'/rda/obstacles_markers',output_qos)
        from .markers import TrackMarkers
        self.marker_renderer=TrackMarkers(cfg['marker_history_length'],cfg['marker_velocity_horizon'],cfg['max_source_age'])
        self.create_subscription(Odometry,cfg['odom_topic'],self._odom,input_qos)
        self.create_subscription(PointCloud2,cfg['points_topic'],self._cloud,input_qos)
        self.create_timer(.02,self._poll)
        self.create_timer(1.,self._heartbeat)
        self.add_on_set_parameters_callback(self._parameters_locked)
        self.get_logger().info(f"{cfg['profile']} -> {cfg['obstacles_topic']}; no legacy detector is started")

    def _odom(self,msg):
        if msg.header.frame_id not in ('',self.parameters['localization_parent_frame']):return
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        try:
            roll,pitch,yaw=quat_attitude(q.x,q.y,q.z,q.w)
            pose=Pose(p.x,p.y,yaw,p.z,roll,pitch)
        except ValueError:return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.pose_cache.receive(pose,stamp,time.monotonic())

    def _pose(self):
        cfg = self.parameters
        topic_pose = self.pose_cache.sample(time.monotonic(),cfg['loc_timeout'])
        pose = topic_pose
        if cfg['localization_source'] in ('tf','latest_available'):
            try:
                tf = self.tf_buffer.lookup_transform(cfg['localization_parent_frame'],cfg['localization_child_frame'],Time())
                stamp = tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                t,q = tf.transform.translation,tf.transform.rotation
                if not all(math.isfinite(v) for v in (t.x,t.y,t.z)):raise ValueError('non-finite TF localization')
                roll,pitch,yaw=quat_attitude(q.x,q.y,q.z,q.w)
                tf_pose=Pose(t.x,t.y,yaw,t.z,roll,pitch)
                if (-.05<=age<=cfg['loc_timeout'] and stamp>0 and
                    (cfg['localization_source']=='tf' or topic_pose is None or stamp>=(self.pose_cache.source_stamp or 0))):
                    pose = tf_pose
                elif cfg['localization_source']=='tf':pose = None
            except (TransformException,ValueError):
                if cfg['localization_source']=='tf':pose = None
        if pose is None:return None
        if not all(math.isfinite(getattr(pose,k)) for k in ('x','y','z','roll','pitch','yaw')):return None
        shift = cfg['body_to_base_link_x']
        offset=rotation(pose.roll,pose.pitch,pose.yaw)@np.array([shift,0.,0.])
        return Pose(pose.x+offset[0],pose.y+offset[1],pose.yaw,pose.z+offset[2],pose.roll,pose.pitch)

    def _mount(self,frame):
        cfg = self.parameters
        if frame==cfg['base_frame']:return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(cfg['base_frame'],frame,Time())
            t,q = tf.transform.translation,tf.transform.rotation
            transform = np.eye(4)
            transform[:3,:3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            transform[:3,3] = [t.x,t.y,t.z]
            if not np.isfinite(transform).all():raise ValueError('non-finite sensor TF')
            return transform
        except TransformException:
            if frame != cfg['sensor_frame']:raise ValueError('unknown pointcloud frame '+frame)
            return self.fallback_mount.copy()

    def _diag(self,reason,**values):
        payload = {**values,'schema_version':1,'reason':reason,'profile':self.parameters['profile'],
                   'producer_ros_ns':self.get_clock().now().nanoseconds,
                   'producer_monotonic_ns':time.monotonic_ns()}
        if values.get('component') != 'semantic':
            self.last_diag = payload
        self.diag_pub.publish(String(data=json.dumps(payload,allow_nan=False)))

    def _parameters_locked(self,parameters):
        if any(p.name in self.parameters and p.value!=self.parameters[p.name] for p in parameters):
            return SetParametersResult(successful=False,reason='Perception parameters are startup-only; restart with the new configuration')
        return SetParametersResult(successful=True)

    def _cloud(self,msg):
        cfg = self.parameters
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if stamp<=0 or age>cfg['max_source_age'] or age<-.05:
            self._diag('cloud_time_invalid',source_stamp=stamp,source_age_s=age);return
        if self.last_received_stamp is not None and stamp<=self.last_received_stamp:
            self._diag('cloud_repeated_or_out_of_order',source_stamp=stamp);return
        pose = self._pose()
        if pose is None:
            self._diag('localization_unavailable',source_stamp=stamp);return
        try:
            if msg.header.frame_id!=cfg['sensor_frame']:
                raise ValueError('model input must use the configured physical sensor frame; transformed map/base_link clouds are not raw LiDAR')
            transform = self._mount(msg.header.frame_id)
            points,_ = read_xyzi(msg,cfg['intensity_scale'])
            from .geometry import self_mask,GeometryConfig
            from .coordinates import transform_points
            body=self_mask(transform_points(points,transform),GeometryConfig(vehicle_length=cfg['vehicle_length'],
                           vehicle_width=cfg['vehicle_width'],self_filter_min_z=cfg['self_filter_min_z'],self_filter_max_z=cfg['self_filter_max_z']))
            points=np.ascontiguousarray(points[~body])
            if not len(points):raise ValueError('cloud contains only self-vehicle points')
        except ValueError as exc:
            self._diag('pointcloud_invalid',detail=str(exc),source_stamp=stamp);return
        self.sequence += 1
        self.last_received_stamp = stamp
        frame = CloudFrame(self.sequence,stamp,time.monotonic(),points,pose,transform,msg.header.frame_id,age,
                           msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
        self.worker.submit(frame)
        if self.semantic_worker and (self.last_semantic_submit is None or stamp-self.last_semantic_submit>=1./cfg['semantic_hz']-1e-6):
            self.last_semantic_submit = stamp
            self.semantic_worker.submit(frame)

    def _poll(self):
        if self.semantic_worker:
            semantic = self.semantic_worker.poll()
            if semantic:
                frame,result,error = semantic
                if error:self._diag('semantic_error',component='semantic',detail=error)
                elif result is not None:
                    self._diag('semantic_ok',component='semantic',source_stamp=frame.stamp,
                               semantic_prediction_s=result['prediction_s'],
                               semantic_processing_s=result['duration_s'],
                               scatter_backend=result['scatter_backend'],classified_points=result['classified_points'])
        result = self.worker.poll()
        if result is None:return
        frame,output,error = result
        if error:
            self._diag('inference_error',detail=error,obstacle_published=False);return
        cfg = self.parameters
        age = frame.source_age_at_receive+time.monotonic()-frame.received
        current_pose = self._pose()
        if current_pose is None or age>cfg['max_source_age']:
            self._diag('result_stale_or_localization_lost',source_stamp=frame.stamp,source_age_s=age,obstacle_published=False);return
        if pose_jump(frame.pose,current_pose,age):
            self._diag('localization_jump_during_inference',source_stamp=frame.stamp,obstacle_published=False);return
        publish = cfg['publish_planner']
        if publish and cfg['strict_single_publisher'] and self.count_publishers(cfg['obstacles_topic'])>1:
            publish = False
            if time.monotonic()-self._last_warning>2.:
                self.get_logger().error('Multiple obstacle publishers; stop the legacy detector or remap obstacles_topic for shadow comparison')
                self._last_warning = time.monotonic()
        if publish:self.obstacle_pub.publish(obstacle_array(output,ObstacleArray,Obstacle))
        rich = {k:v for k,v in output.items() if k!='grid'}
        rich.update(source_age_s=age,obstacle_published=bool(publish),profile=cfg['profile'],tracking_filter=cfg['tracking_filter'])
        self.tracks_pub.publish(String(data=json.dumps(rich,allow_nan=False)))
        self._grid(output,frame)
        self._markers(output)
        self._diag('ok',**rich)

    def _grid(self,result,frame):
        g = result['grid']
        msg = OccupancyGrid()
        total_ns = frame.stamp_ns if frame.stamp_ns is not None else round(frame.stamp*1e9)
        msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(total_ns,1000000000)
        msg.header.frame_id = self.parameters['base_frame']
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution,msg.info.width,msg.info.height = float(g['resolution']),int(g['width']),int(g['height'])
        msg.info.origin.position.x,msg.info.origin.position.y = g['origin_x'],g['origin_y']
        msg.info.origin.orientation.w = 1.
        msg.data = g['data']
        self.grid_pub.publish(msg)

    def _markers(self,result):
        self.marker_pub.publish(self.marker_renderer.render(result,Marker,MarkerArray,Point))

    def _heartbeat(self):
        self._diag(self.last_diag.get('reason','initializing'),
                   centerpoint_worker=self.worker.state,
                   semantic_worker=self.semantic_worker.state if self.semantic_worker else 'disabled',
                   dropped_pending=self.worker.replaced)

    def destroy_node(self):
        self.worker.close()
        if self.semantic_worker:self.semantic_worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CampusPerceptionNode()
        rclpy.spin(node)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
