#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_data_monitor.py — wxk_car 实车「四轮频繁换向 / 扭着走」问题数据监听记录器
=====================================================================
监听并记录以下数据（按到达时间写入同一个 CSV，供后续离线分析）：
  * 目标点   /move_base_simple/goal            (PoseStamped)
  * 定位     /localization                     (Odometry, 算法系 50Hz)
  * 规划输出 /cmd_vel                          (Twist, 20Hz)   ← DWA 决策
  * 控制指令 /car_cmd                          (control_msgs/CarCtr) ← 模式/角度/速度
  * 底盘反馈 /car_state                        (control_msgs/CarState) ← 底盘自认模式
  * 控制桥执行状态 /car_bridge_state           (Int32: 0=ACK 1=挂起 2=SPIN_RUN)
  * 障碍物   /rda/obstacles                    (rda_dwa_real_msgs/ObstacleArray)
  * 轮转向角 /rda/wheel_steer                  (Float32MultiArray, 4 轮)
  * 轮速     /rda/wheel_speed                  (Float32MultiArray, 4 轮)
  * 状态文本 /rda/status, /rda/ctrl_status     (String)
  * [DIAG]日志 /rosout                         (过滤埋点探针行)
另自动生成事件行 EVENT_MODE / EVENT_STEER_FLIP：turnmode 切换、angle 变号。

运行前提（WSL2 / Linux 车机）：
  source /opt/ros/humble/setup.bash
  source <你的工作区>/install/setup.bash     # wxk_car 编译后
用法：
  python3 car_data_monitor.py                          # 默认记录 120 秒
  python3 car_data_monitor.py --duration 60            # 记录 60 秒
  python3 car_data_monitor.py --out /tmp/car_data.csv  # 指定输出文件
建议流程：先启动 run_rda_dwa.launch.py 并确认能发布目标点，
         然后在本终端运行本脚本 → 在 RViz 发布目标点 → 让车走 30~60 秒（复现扭行）
         → 等待自动结束（或 Ctrl+C），把 CSV 文件发给分析方。
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String, Int32
from rcl_interfaces.msg import Log

# ---- 可选消息（未 source 工作区时优雅降级，仅警告）----
try:
    from control_msgs.msg import CarCtr, CarState
    HAVE_CTRL = True
except Exception as _e:  # pragma: no cover
    CarCtr = CarState = None
    HAVE_CTRL = False
    print(f"[WARN] control_msgs 导入失败: {_e}", file=sys.stderr)

try:
    from rda_dwa_real_msgs.msg import ObstacleArray
    HAVE_OBS = True
except Exception as _e:  # pragma: no cover
    ObstacleArray = None
    HAVE_OBS = False
    print(f"[WARN] rda_dwa_real_msgs 导入失败: {_e}", file=sys.stderr)


def qos_sub(depth=50):
    """与各发布者匹配的 QoS：RELIABLE + VOLATILE（与 bridge/control_bridge 一致）。"""
    return QoSProfile(depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


def yaw_from_q(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class CarDataMonitor(Node):
    def __init__(self, out_path: str, duration: float, capture_rosout: bool):
        super().__init__('car_data_monitor')
        self._out_path = out_path
        self._f = open(out_path, 'w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._f)
        self._csv.writerow(['t_wall', 't_ros', 'topic', 'data'])

        # 事件/统计状态
        self._last_cmd_turnmode = None
        self._last_state_turnmode = None
        self._last_angle = None
        self._last_bridge_state = None

        # ---------------- 订阅 ----------------
        self._subs = []

        def sub(topic, mtype, cb, qos=None):
            try:
                self._subs.append(self.create_subscription(mtype, topic, cb, qos or qos_sub()))
                self.get_logger().info(f'监听 {topic} ({mtype.__name__})')
            except Exception as e:
                self.get_logger().warn(f'订阅 {topic} 失败: {e}')

        sub('/move_base_simple/goal', PoseStamped, self._cb_goal)
        sub('/localization', Odometry, self._cb_loc)
        sub('/cmd_vel', Twist, self._cb_cmd_vel)
        if HAVE_CTRL:
            sub('/car_cmd', CarCtr, self._cb_car_cmd)
            sub('/car_state', CarState, self._cb_car_state)
        else:
            self._warn('未订阅 /car_cmd /car_state：control_msgs 未导入')
        sub('/car_bridge_state', Int32, self._cb_bridge_state)
        if HAVE_OBS:
            sub('/rda/obstacles', ObstacleArray, self._cb_obstacles)
        else:
            self._warn('未订阅 /rda/obstacles：rda_dwa_real_msgs 未导入')
        sub('/rda/wheel_steer', Float32MultiArray, self._cb_wheel_steer)
        sub('/rda/wheel_speed', Float32MultiArray, self._cb_wheel_speed)
        sub('/rda/status', String, self._cb_status)
        sub('/rda/ctrl_status', String, self._cb_ctrl_status)
        if capture_rosout:
            sub('/rosout', Log, self._cb_rosout, qos=qos_sub(200))

        # 定时结束
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.5, self._tick)
        self._duration = duration
        self._wall_t0 = time.time()
        self._stats = {
            'n_goal': 0, 'n_loc': 0, 'n_cmd_vel': 0, 'n_car_cmd': 0, 'n_car_state': 0,
            'n_bridge_state': 0, 'n_obs': 0, 'n_steer': 0, 'n_wspd': 0,
            'n_turnmode_change': 0, 'n_angle_flip': 0, 'n_rosout': 0,
        }

    # ---------------- 工具 ----------------
    def _warn(self, s):
        self.get_logger().warn(s)

    def _row(self, topic, data, t_ros=None):
        if t_ros is None:
            t_ros = self.get_clock().now().nanoseconds * 1e-9
        self._csv.writerow([f'{time.time():.6f}', f'{t_ros:.6f}', topic, data])
        self._f.flush()

    def _event(self, name, data):
        self._row('EVENT_' + name, data)

    # ---------------- 回调 ----------------
    def _cb_goal(self, m: PoseStamped):
        self._stats['n_goal'] += 1
        y = yaw_from_q(m.pose.orientation)
        self._row('/move_base_simple/goal',
                  f'x={m.pose.position.x:.3f} y={m.pose.position.y:.3f} yaw={y:.3f} frame={m.header.frame_id}')

    def _cb_loc(self, m: Odometry):
        self._stats['n_loc'] += 1
        y = yaw_from_q(m.pose.pose.orientation)
        t = m.twist.twist
        self._row('/localization',
                  f'x={m.pose.pose.position.x:.3f} y={m.pose.pose.position.y:.3f} yaw={y:.3f} '
                  f'vx={t.linear.x:.3f} vy={t.linear.y:.3f} wz={t.angular.z:.3f} frame={m.header.frame_id}')

    def _cb_cmd_vel(self, m: Twist):
        self._stats['n_cmd_vel'] += 1
        v, w = m.linear.x, m.angular.z
        self._row('/cmd_vel', f'v={v:.3f} w={w:.3f}')
        # angle 变号事件（规划层角度意向翻转）
        if self._last_angle is not None and v != 0.0 and self._last_angle * w < 0 and abs(w) > 0.02:
            self._stats['n_angle_flip'] += 1
            self._event('STEER_FLIP', f'w sign flip v={v:.3f} w={w:.3f}')
        self._last_angle = w

    def _cb_car_cmd(self, m):
        self._stats['n_car_cmd'] += 1
        self._row('/car_cmd',
                  f'source={m.source} ctr_mode={m.ctr_mode} up_down={m.up_down_flag} '
                  f'enable={m.enable} turnmode={m.turnmode} speed={m.speed:.3f} '
                  f'angle={m.angle:.3f} hb={m.heartbeat}')
        if self._last_cmd_turnmode is not None and m.turnmode != self._last_cmd_turnmode:
            self._stats['n_turnmode_change'] += 1
            self._event('MODE', f'car_cmd turnmode {self._last_cmd_turnmode} -> {m.turnmode}')
        self._last_cmd_turnmode = m.turnmode

    def _cb_car_state(self, m):
        self._stats['n_car_state'] += 1
        sp = ','.join(f'{s:.3f}' for s in m.speed)
        err = ','.join(str(e) for e in m.errcode)
        self._row('/car_state',
                  f'enable={m.enable} ctrmode={m.ctrmode} turnmode={m.turnmode} '
                  f'speed=[{sp}] errcode=[{err}]')
        if self._last_state_turnmode is not None and m.turnmode != self._last_state_turnmode:
            self._event('MODE', f'car_state turnmode {self._last_state_turnmode} -> {m.turnmode}')
        self._last_state_turnmode = m.turnmode

    def _cb_bridge_state(self, m: Int32):
        self._stats['n_bridge_state'] += 1
        name = {0: 'ACK', 1: 'HANGUP', 2: 'SPIN_RUN'}.get(m.data, str(m.data))
        self._row('/car_bridge_state', f'state={m.data} ({name})')
        if self._last_bridge_state is not None and m.data != self._last_bridge_state:
            self._event('MODE', f'bridge_state {self._last_bridge_state} -> {m.data} ({name})')
        self._last_bridge_state = m.data

    def _cb_obstacles(self, m):
        self._stats['n_obs'] += 1
        dmin = float('inf'); near = ''
        for ob in m.obstacles:
            d = math.hypot(ob.x, ob.y)
            if d < dmin:
                dmin = d
                near = f'{ob.type}@{ob.x:.2f},{ob.y:.2f}'
        dmin = 99.9 if dmin == float('inf') else round(dmin, 3)
        self._row('/rda/obstacles',
                  f'n={len(m.obstacles)} min_dist={dmin} nearest={near} frame={m.header.frame_id}')

    def _cb_wheel_steer(self, m: Float32MultiArray):
        self._stats['n_steer'] += 1
        self._row('/rda/wheel_steer', '[' + ','.join(f'{v:.3f}' for v in m.data) + ']')

    def _cb_wheel_speed(self, m: Float32MultiArray):
        self._stats['n_wspd'] += 1
        self._row('/rda/wheel_speed', '[' + ','.join(f'{v:.3f}' for v in m.data) + ']')

    def _cb_status(self, m: String):
        self._row('/rda/status', m.data)

    def _cb_ctrl_status(self, m: String):
        self._row('/rda/ctrl_status', m.data)

    def _cb_rosout(self, m: Log):
        s = m.msg
        keys = ('[DIAG]', 'MODE', 'mismatch', 'DROPPED', 'SteerModeCtrl', 'Car Run',
                'UDP', 'watchdog', 'EMERGENCY', 'gate')
        if any(k in s for k in keys):
            self._stats['n_rosout'] += 1
            self._row('/rosout', f'[{m.name}] {s}')

    # ---------------- 结束与统计 ----------------
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._t0 >= self._duration:
            self._finish_and_exit()

    def _finish_and_exit(self):
        try:
            self.get_logger().info('记录完成，统计：' + str(self._stats))
            st = os.path.splitext(self._out_path)[0] + '.stats.txt'
            with open(st, 'w', encoding='utf-8') as f:
                f.write('=== car_data_monitor 统计 ===\n')
                f.write(f'记录时长: {time.time() - self._wall_t0:.1f} s\n')
                for k, v in self._stats.items():
                    f.write(f'{k}: {v}\n')
            self.get_logger().info(f'统计已写入 {st}')
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f'写统计失败: {e}')
        rclpy.get_default_context().try_shutdown()  # 触发退出
        raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description='wxk_car 数据监听记录器')
    ap.add_argument('--out', default=None, help='CSV 输出路径（默认 ./car_data_log_<时间戳>.csv）')
    ap.add_argument('--duration', type=float, default=120.0, help='记录时长秒（默认 120）')
    ap.add_argument('--no-rosout', action='store_true', help='不记录 /rosout [DIAG] 日志')
    args = ap.parse_args()

    if not args.out:
        args.out = os.path.join(os.getcwd(), f'car_data_log_{time.strftime("%Y%m%d_%H%M%S")}.csv')

    rclpy.init()
    node = CarDataMonitor(args.out, args.duration, not args.no_rosout)
    node.get_logger().info(f'输出文件: {args.out} | 记录时长: {args.duration}s '
                           f'| Ctrl+C 可提前结束')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f'\n[完成] 数据已写入 {args.out}')


if __name__ == '__main__':
    main()
