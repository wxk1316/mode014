#!/usr/bin/env python3

import math
import os
import pickle

import networkx as nx
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from path_planner.srv import PlanPath
from rclpy.node import Node


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # 参数（保持原参数名和默认值）
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('accel_dist', 1.0)
        self.declare_parameter('decel_dist', 1.0)
        self.declare_parameter('normal_interval', 0.1)
        self.declare_parameter('angle_threshold', 60.0)
        self.declare_parameter('max_adhesion_dist', 3.0)
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('roadmap_path', '')

        self.max_speed = self.get_parameter('max_speed').value
        self.accel_dist = self.get_parameter('accel_dist').value
        self.decel_dist = self.get_parameter('decel_dist').value
        self.norm_int = self.get_parameter('normal_interval').value
        self.angle_thresh = math.radians(self.get_parameter('angle_threshold').value)
        self.max_adhesion = self.get_parameter('max_adhesion_dist').value
        self.resolution = self.get_parameter('resolution').value
        self.virtual_seq = 0

        # 加载路网。若参数路径不存在，则回退到安装目录中的 data/road_graph.pkl。
        roadmap_path = self.get_parameter('roadmap_path').value
        if not roadmap_path or not os.path.exists(roadmap_path):
            fallback = os.path.join(get_package_share_directory('path_planner'), 'data', 'road_graph.pkl')
            self.get_logger().warn(f'路网参数路径不存在，使用安装目录路网: {fallback}')
            roadmap_path = fallback

        with open(roadmap_path, 'rb') as f:
            self.load_roadmap(pickle.load(f))

        # 加密路网
        self.densify_roadmap(self.resolution)

        self.create_service(PlanPath, '/plan_path', self.handle_plan_request)
        self.get_logger().info(f'Path Planner ready (分辨率: {self.resolution}m)')

    def load_roadmap(self, data):
        """加载路网"""
        self.graph = data
        self.nodes = {node: (node[0], node[1]) for node in self.graph.nodes()}
        self.edges = list(self.graph.edges())
        self.get_logger().info(f'原始路网: {len(self.nodes)} 节点, {len(self.edges)} 边')

    def densify_roadmap(self, step):
        """加密路网"""
        self.get_logger().info(f'加密路网，步长: {step}m')

        new_graph = nx.DiGraph() if self.graph.is_directed() else nx.Graph()

        for node in self.graph.nodes():
            pos = self.graph.nodes[node].get('pos', node)
            new_graph.add_node(node, pos=pos)

        for u, v in self.edges:
            attributes = dict(self.graph.get_edge_data(u, v) or {})
            u_pos = self.nodes[u]
            v_pos = self.nodes[v]

            dist = math.hypot(u_pos[0]-v_pos[0], u_pos[1]-v_pos[1])
            num = max(1, int(dist / step)) + 1

            prev = u
            for i in range(1, num):
                t = i / num
                x = u_pos[0] + t * (v_pos[0] - u_pos[0])
                y = u_pos[1] + t * (v_pos[1] - u_pos[1])

                new_node = (round(x, 6), round(y, 6))
                new_graph.add_node(new_node, pos=new_node)
                edge_attributes = dict(attributes)
                edge_attributes['weight'] = step
                new_graph.add_edge(prev, new_node, **edge_attributes)
                prev = new_node

            edge_attributes = dict(attributes)
            edge_attributes['weight'] = dist / num
            new_graph.add_edge(prev, v, **edge_attributes)

        self.graph = new_graph
        self.nodes = {node: (node[0], node[1]) for node in self.graph.nodes()}
        self.edges = list(self.graph.edges())

        self.get_logger().info(f'加密后: {len(self.nodes)} 节点, {len(self.edges)} 边')

    def point_on_edge(self, x, y):
        """找边上最近点"""
        best = None
        best_dist = float('inf')
        seen = set()
        for a, b in self.edges:
            physical_edge = frozenset((a, b))
            if physical_edge in seen:
                continue
            seen.add(physical_edge)
            x1, y1 = self.nodes[a]
            x2, y2 = self.nodes[b]
            dx, dy = x2-x1, y2-y1
            if dx == dy == 0:
                dist = math.hypot(x-x1, y-y1)
                px, py = x1, y1
                t = 0.0
            else:
                t = max(0, min(1, ((x-x1)*dx + (y-y1)*dy) / (dx*dx+dy*dy)))
                px, py = x1+t*dx, y1+t*dy
                dist = math.hypot(x-px, y-py)
            if dist < best_dist:
                best_dist, best = dist, (px, py, a, b, t)
        return best if best_dist <= self.max_adhesion else None

    def _new_virtual_id(self, graph):
        """生成虚拟节点ID"""
        self.virtual_seq += 1
        virtual = ('virtual', self.virtual_seq)
        while graph.has_node(virtual):
            self.virtual_seq += 1
            virtual = ('virtual', self.virtual_seq)
        return virtual

    @staticmethod
    def _add_chain(graph, nodes, attributes):
        """添加链式边"""
        for first, second in zip(nodes, nodes[1:]):
            first_pos = graph.nodes[first].get('pos', first)
            second_pos = graph.nodes[second].get('pos', second)
            edge_attributes = dict(attributes)
            edge_attributes['weight'] = math.hypot(
                second_pos[0] - first_pos[0], second_pos[1] - first_pos[1]
            )
            graph.add_edge(first, second, **edge_attributes)

    def _insert_virtuals(self, graph, insertions):
        """处理多个虚拟节点插入"""
        groups = {}
        for virtual, projection in insertions:
            x, y, first, second, ratio = projection
            graph.add_node(virtual, pos=(x, y), virtual=True)
            key = frozenset((first, second))
            group = groups.setdefault(key, {'first': first, 'second': second, 'points': []})
            normalized_ratio = ratio if first == group['first'] else 1.0 - ratio
            group['points'].append((normalized_ratio, virtual))

        for group in groups.values():
            first = group['first']
            second = group['second']
            middle = [virtual for _, virtual in sorted(group['points'], key=lambda item: item[0])]
            if graph.is_directed():
                forward_attributes = dict(graph.get_edge_data(first, second) or {})
                reverse_attributes = dict(graph.get_edge_data(second, first) or {})
                if graph.has_edge(first, second):
                    graph.remove_edge(first, second)
                    self._add_chain(graph, [first, *middle, second], forward_attributes)
                if graph.has_edge(second, first):
                    graph.remove_edge(second, first)
                    self._add_chain(graph, [second, *reversed(middle), first], reverse_attributes)
            else:
                attributes = dict(graph.get_edge_data(first, second) or {})
                if graph.has_edge(first, second):
                    graph.remove_edge(first, second)
                self._add_chain(graph, [first, *middle, second], attributes)

    def angle(self, p0, p1, p2):
        """三点夹角"""
        v1, v2 = np.array(p1)-np.array(p0), np.array(p2)-np.array(p1)
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 0.01 or n2 < 0.01:
            return 0.0
        return math.acos(max(-1, min(1, np.dot(v1, v2)/(n1*n2))))

    def speed_at(self, dist, total):
        """梯形速度曲线：加速-匀速-减速，短路径自动变为三角形"""
        a = min(self.accel_dist, total / 2)
        d = min(self.decel_dist, total / 2)

        if dist < a:
            return self.max_speed * (dist / a) if a > 0 else self.max_speed
        elif dist > total - d:
            return self.max_speed * ((total - dist) / d) if d > 0 else self.max_speed
        else:
            return self.max_speed

    def build_path_msg(self, pts):
        """从路径点构建Path消息"""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        total = 0.0
        for i in range(len(pts) - 1):
            total += math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])

        accum_dist = 0
        for i, (x, y) in enumerate(pts):
            speed = self.speed_at(accum_dist, total)

            if i < len(pts) - 1:
                theta = math.atan2(pts[i+1][1] - y, pts[i+1][0] - x)
            else:
                theta = 0

            p = PoseStamped()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = speed
            p.pose.orientation.w = math.cos(theta/2)
            p.pose.orientation.z = math.sin(theta/2)
            msg.poses.append(p)

            if i < len(pts) - 1:
                accum_dist += math.hypot(pts[i+1][0]-x, pts[i+1][1]-y)
        return msg

    def plan(self, start, goal):
        """主规划逻辑 - 使用图副本"""
        s = self.point_on_edge(start[0], start[1])
        g = self.point_on_edge(goal[0], goal[1])
        if not s or not g:
            return None, 'Point too far from roadmap'

        graph_copy = self.graph.copy()

        sv = self._new_virtual_id(graph_copy)
        gv = self._new_virtual_id(graph_copy)
        self._insert_virtuals(graph_copy, [(sv, s), (gv, g)])

        def h(n1, n2):
            p1 = graph_copy.nodes[n1].get('pos', n1)
            p2 = graph_copy.nodes[n2].get('pos', n2)
            return math.hypot(p2[0]-p1[0], p2[1]-p1[1])

        try:
            nodes = nx.astar_path(graph_copy, sv, gv, h, weight='weight')
        except Exception:
            return None, 'No path'

        pts = []
        for n in nodes:
            if n == sv:
                pts.append((s[0], s[1]))
            elif n == gv:
                pts.append((g[0], g[1]))
            else:
                pos = graph_copy.nodes[n].get('pos', n)
                pts.append((pos[0], pos[1]))

        segs = [[pts[0], pts[1]]] if len(pts) >= 2 else [pts]
        for i in range(2, len(pts)):
            if self.angle(segs[-1][-2], segs[-1][-1], pts[i]) > self.angle_thresh:
                segs.append([segs[-1][-1], pts[i]])
            else:
                segs[-1].append(pts[i])

        result = []
        for seg in segs:
            result.append(self.build_path_msg(seg))

        return result, 'success'

    def handle_plan_request(self, request, response):
        start = (request.start_pose.position.x, request.start_pose.position.y)
        goal = (request.goal_pose.position.x, request.goal_pose.position.y)
        segs, msg = self.plan(start, goal)
        response.success = segs is not None
        response.message = msg
        response.segments = segs or []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
