#!/usr/bin/env python3
import os
import pickle
import numpy as np
import networkx as nx

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class RoadPlanner(Node):
    def __init__(self):
        super().__init__('road_planner')
        self.declare_parameter('tf_frame', 'map')
        self.tf_frame = self.get_parameter('tf_frame').value
        
        # Load the graph dynamically from the install share directory
        try:
            pkg_share = get_package_share_directory('road_planner')
            pkl_path = os.path.join(pkg_share, 'data', 'road_graph.pkl')
            with open(pkl_path, 'rb') as f:
                self.G = pickle.load(f)
            self.get_logger().info(f"Loaded road graph: {self.G.number_of_nodes()} nodes, {self.G.number_of_edges()} edges from {pkl_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load road graph: {e}")
            return

        # Cache the latest FAST_LIO global localization in map frame
        self.current_pose = None
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.network_pub = self.create_publisher(MarkerArray, '/road_network', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/localization', self.localization_callback, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Network Markers
        self.network_markers = self.create_network_markers()
        
        # Publishers Timer
        self.create_timer(1.0, self.publish_network_timer)
        
        self.get_logger().info("Road Planner (ROS 2) successfully started. Waiting for 2D Nav Goal...")

    def get_current_pose(self):
        """Fetch current robot position from the latest FAST_LIO localization."""
        if self.current_pose is None:
            self.get_logger().warn("Could not get current robot pose: /localization not received yet")
            return None
        return self.current_pose

    def localization_callback(self, msg):
        pose = msg.pose.pose.position
        self.current_pose = (pose.x, pose.y, pose.z)

    def create_network_markers(self):
        """Create road network markers for RViz"""
        marker_array = MarkerArray()
        
        # Edges
        edge_marker = Marker()
        edge_marker.header.frame_id = self.tf_frame
        edge_marker.ns = "roads"
        edge_marker.id = 0
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.05
        edge_marker.color.a = 1.0
        edge_marker.color.r = 0.5
        edge_marker.color.g = 0.5
        edge_marker.color.b = 0.5
        
        for u, v in self.G.edges():
            p1 = self._to_point(u[0], u[1], u[2])
            p2 = self._to_point(v[0], v[1], v[2])
            edge_marker.points.append(p1)
            edge_marker.points.append(p2)
        marker_array.markers.append(edge_marker)
        
        # Nodes
        node_marker = Marker()
        node_marker.header.frame_id = self.tf_frame
        node_marker.ns = "nodes"
        node_marker.id = 1
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.1
        node_marker.scale.y = 0.1
        node_marker.color.a = 1.0
        node_marker.color.b = 1.0
        
        for node in self.G.nodes():
            node_marker.points.append(self._to_point(node[0], node[1], node[2]))
        marker_array.markers.append(node_marker)
        
        return marker_array
    
    def publish_network_timer(self):
        """Timer callback to publish the network steadily at 1Hz"""
        now = self.get_clock().now().to_msg()
        for marker in self.network_markers.markers:
            marker.header.stamp = now
        self.network_pub.publish(self.network_markers)
    
    def find_nearest_node(self, target):
        nodes = np.array(list(self.G.nodes()))
        dists = np.linalg.norm(nodes - np.array(target), axis=1)
        return tuple(nodes[np.argmin(dists)])
    
    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def plan_path(self, start_point, goal_point):
        start_node = self.find_nearest_node(start_point)
        goal_node = self.find_nearest_node(goal_point)
        self.get_logger().info(f"Start: {start_point} -> Nearest: {start_node}")
        self.get_logger().info(f"Goal: {goal_point} -> Nearest: {goal_node}")
        
        try:
            path_nodes = nx.astar_path(self.G, start_node, goal_node, self.heuristic)
            self.get_logger().info(f"Successfully planned path! Total nodes: {len(path_nodes)}")
            return path_nodes
        except nx.NetworkXNoPath:
            self.get_logger().error("No viable path found between start and goal!")
            return None
    
    def publish_path(self, path_nodes):
        path_msg = Path()
        path_msg.header.frame_id = self.tf_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for node in path_nodes:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(node[0])
            pose.pose.position.y = float(node[1])
            pose.pose.position.z = float(node[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info("Path successfully published to /planned_path")
    
    def goal_callback(self, msg):
        goal = msg.pose.position
        self.get_logger().info(f"Nav Goal received: ({goal.x}, {goal.y}, {goal.z})")
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("Cannot plan path without knowing the robot's current position!")
            return
            
        path_nodes = self.plan_path(current_pose, (goal.x, goal.y, goal.z))
        if path_nodes:
            self.publish_path(path_nodes)
    
    def _to_point(self, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

def main(args=None):
    rclpy.init(args=args)
    node = RoadPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
