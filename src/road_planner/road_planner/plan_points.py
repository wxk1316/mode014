#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('multi_waypoint_navigator')
        
        # 发布者和订阅者
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.odom_sub = self.create_subscription(Odometry, '/localization', self.odom_callback, 10)

        # 你的四个目标点 [x, y, yaw]
        self.waypoints = [
            [20.5574, 5.3164, 1.63331],    # 目标点 1 (索引 0)
            [17.5394, 8.56194, -2.77188],  # 目标点 2 (索引 1) -> 此处将停顿 10s
            [17.2091, 3.00797, -1.50418],  # 目标点 3 (索引 2)
            [2.19262, -0.0373928, -3.1371]    # 目标点 4 (索引 3)
        ]
        
        # 核心参数
        self.current_wp_index = 0
        self.state = 'INIT'
        self.distance_tolerance = 0.8  # 容差
        self.speed_threshold = 0.05    # 停稳阈值
        self.default_wait = 1.0        # 其他点的默认等待时间
        self.special_wait = 10.0       # 第二个点的特殊等待时间
        
        # 运行时数据
        self.current_odom = None
        self.wait_start_time = None
        self.init_start_time = None    # 记录初始获取定位的时间
        self.last_log_time = self.get_clock().now()
        self.last_pub_time = self.get_clock().now() # 用于定时重发目标点

        # 定时器逻辑状态机 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def euler_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def publish_current_goal(self):
        wp = self.waypoints[self.current_wp_index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = wp[0]
        msg.pose.position.y = wp[1]
        q = self.euler_to_quaternion(wp[2])
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.goal_pub.publish(msg)
        self.last_pub_time = self.get_clock().now()
        self.get_logger().info(f"==> 发布目标 {self.current_wp_index + 1}: X={wp[0]:.2f}, Y={wp[1]:.2f}")

    def odom_callback(self, msg):
        self.current_odom = msg

    def control_loop(self):
        if self.current_odom is None or self.state == 'DONE':
            return

        # 获取当前位置和速度
        curr_x = self.current_odom.pose.pose.position.x
        curr_y = self.current_odom.pose.pose.position.y
        curr_v = math.sqrt(self.current_odom.twist.twist.linear.x**2 + 
                           self.current_odom.twist.twist.linear.y**2)

        # 当前目标点坐标
        goal_x = self.waypoints[self.current_wp_index][0]
        goal_y = self.waypoints[self.current_wp_index][1]
        dist = math.hypot(goal_x - curr_x, goal_y - curr_y)

        now = self.get_clock().now()

        # 定时打印状态
        if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
            if self.state == 'NAVIGATING':
                self.get_logger().info(f"[导航] 距离目标 {self.current_wp_index+1}: {dist:.2f}m, 速度: {curr_v:.2f}m/s")
            self.last_log_time = now

        # --- 状态机逻辑 ---
        
        if self.state == 'INIT':
            if self.init_start_time is None:
                self.init_start_time = now
                self.get_logger().info("已获取初始定位，等待 1.5 秒以确保底层通信握手完成...")

            elapsed = (now - self.init_start_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self.publish_current_goal()
                self.state = 'NAVIGATING'

        elif self.state == 'NAVIGATING':
            # 保险机制：每隔 5 秒重新发布一次当前目标点，防止主程序因异常丢失目标
            if (now - self.last_pub_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().info("重新发送目标点以防丢失...")
                self.publish_current_goal()

            # 到达判定
            reached = (dist < self.distance_tolerance) or \
                      (dist < self.distance_tolerance * 1.5 and curr_v < self.speed_threshold)
            
            if reached:
                wait_time = self.special_wait if self.current_wp_index == 1 else self.default_wait
                # self.get_logger().info(f"[*] 已到达目标点 {self.current_wp_index + 1}。将停顿 {wait_time} 秒...")
                
                self.wait_start_time = now
                self.state = 'WAITING'

        elif self.state == 'WAITING':
            current_target_wait = self.special_wait if self.current_wp_index == 1 else self.default_wait
            elapsed = (now - self.wait_start_time).nanoseconds / 1e9
            
            if elapsed >= current_target_wait:
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info(">>> 全部任务完成！")
                    self.state = 'DONE'
                else:
                    self.publish_current_goal()
                    self.state = 'NAVIGATING'

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()