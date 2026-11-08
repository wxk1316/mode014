#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_data_recorder —— 实车震荡排查数据记录器（ROS2 / rclpy 单文件，免编译）

记录话题（与 rda_dwa_real 实车链路一一对应，消息类型已核实）：
    /car_cmd             control_msgs/msg/CarCtr          # 下发到底盘的最终指令（关键：angle 帧间跳变）
    /car_bridge_state    std_msgs/Int32                   # 0=ACK 1=挂起 2=SPIN_RUN（关键：模式横跳）
    /cmd_vel             geometry_msgs/Twist              # 规划输出 v,ω（关键：5~7Hz 抖动）
    /localization        nav_msgs/Odometry                # FAST_LIO 定位（map→body，50Hz）
    /move_base_simple/goal  geometry_msgs/PoseStamped     # RViz 目标点
    /rda/obstacles       rda_dwa_real_msgs/ObstacleArray  # 障碍物（id/x/y/heading/speed…）
    /car_state           control_msgs/msg/CarState        # 底盘四轮轮速反馈（speed[4]）
    /rda/wheel_steer     std_msgs/Float32MultiArray       # 4WIS 四轮转向角 rad×4
    /rda/wheel_speed     std_msgs/Float32MultiArray       # 四轮轮速 m/s×4
    /rda/status          std_msgs/String                  # 规划帧诊断（v/w/障碍数/可达性）
    /rda/ctrl_status     std_msgs/String                  # 控制桥诊断
    /odom                nav_msgs/Odometry                # 定位融合输出（可选，无发布则自动跳过）

用法（在小车上，先 source 工作空间，再与 run_rda_dwa.launch.py 并行开一个终端）：
    source ~/wxk_car/install/setup.bash     # 换成你实际的 overlay 路径
    python3 record_car_data.py
    # 可选参数：
    # python3 record_car_data.py --ros-args -p log_dir:=~/car_data_logs -p topics:=/car_cmd,/car_bridge_state
    # 复现震荡（发目标点让车跑起来），Ctrl+C 停止。

输出（默认 ~/car_data_logs/run_YYYYmmdd_HHMMSS/）：
    每个话题一个 CSV（首列 wall_ns 系统墙钟纳秒，第二列 ros_ns 节点时钟纳秒，其余为消息字段，
    数组/嵌套字段展开为 xxx[0]、pose.position.x 等，与 .msg 定义一一对应）；
    rates.csv    每 2s 一次各话题实测频率（可验证 /cmd_vel 是否只有 5~7Hz、/localization 是否 50Hz）；
    meta.json    解析到的消息类型 / 各话题收包统计 / 运行时间。

说明：订阅用 BEST_EFFORT+VOLATILE（兼容所有发布端）；消息类型用 try-import，
缺少自定义消息时该话题自动跳过并告警，不影响其他话题记录；全程只读，不影响控制闭环。
"""

import csv
import importlib
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy


# ---------------------------------------------------------------------------
# 话题清单：(话题名, 消息模块, 类型名, 备用模块列表[(模块,类型)...])
# 备用用于兼容旧版消息包（如 car_ctr_msgs 历史版 CarCtr）
# ---------------------------------------------------------------------------
TOPIC_SPECS = [
    ('/car_cmd',            'control_msgs.msg',       'CarCtr',          [('car_ctr_msgs.msg', 'CarCtr')]),
    ('/car_bridge_state',   'std_msgs.msg',           'Int32',           []),
    ('/cmd_vel',            'geometry_msgs.msg',      'Twist',           []),
    ('/localization',       'nav_msgs.msg',           'Odometry',        []),
    ('/move_base_simple/goal', 'geometry_msgs.msg',   'PoseStamped',     []),
    ('/rda/obstacles',      'rda_dwa_real_msgs.msg',  'ObstacleArray',   []),
    ('/car_state',          'control_msgs.msg',       'CarState',        []),
    ('/rda/wheel_steer',    'std_msgs.msg',           'Float32MultiArray', []),
    ('/rda/wheel_speed',    'std_msgs.msg',           'Float32MultiArray', []),
    ('/rda/status',         'std_msgs.msg',           'String',          []),
    ('/rda/ctrl_status',    'std_msgs.msg',           'String',          []),
    ('/odom',               'nav_msgs.msg',           'Odometry',        []),
]

# 为控制 CSV 体积：Odometry 跳过 36 维协方差（pose.covariance / twist.covariance）
SKIP_FIELD_PREFIXES = {
    '/localization': ('pose.covariance', 'twist.covariance'),
    '/odom':         ('pose.covariance', 'twist.covariance'),
}


# ---------------------------------------------------------------------------
# 消息字段反射：把任意 ROS2 消息压平成 [(列名, 值), ...]
# ---------------------------------------------------------------------------
def _field_types(msg):
    """返回 {字段名: 类型字符串}。优先 get_fields_and_field_types，回退 SLOT_TYPES。"""
    cls = type(msg)
    if hasattr(cls, 'get_fields_and_field_types'):
        try:
            return cls.get_fields_and_field_types()
        except Exception:
            pass
    if hasattr(msg, 'SLOT_TYPES'):
        return dict(msg.SLOT_TYPES)
    raise RuntimeError('无法反射消息字段: %s' % cls)


_PRIM = (int, float, bool, str)


def flatten_fields(msg, prefix='', skip_prefixes=()):
    """递归展开字段，返回 [(列名, 值), ...]。值保持原类型，CSV 写入时统一 str()。"""
    out = []
    for fname, ftype in _field_types(msg).items():
        key = prefix + fname
        if skip_prefixes and (key in skip_prefixes or any(key.startswith(p) for p in skip_prefixes)):
            continue
        val = getattr(msg, fname)
        if ftype.endswith('[]'):                      # 数组：原始数组 或 消息数组
            arr = val if val is not None else []
            for i, item in enumerate(arr):
                if item is None or isinstance(item, _PRIM):
                    out.append(('%s[%d]' % (key, i), item))
                else:
                    out.extend(flatten_fields(item, '%s[%d].' % (key, i), skip_prefixes))
        elif '/' in ftype:                            # 嵌套消息（Header/Time/Pose/Twist…）
            if val is None:
                out.append((key, None))
            else:
                out.extend(flatten_fields(val, key + '.', skip_prefixes))
        else:                                         # 原始类型
            out.append((key, val))
    return out


# ---------------------------------------------------------------------------
# 记录节点
# ---------------------------------------------------------------------------
class CarDataRecorder(Node):

    def __init__(self):
        super().__init__('car_data_recorder')
        self.declare_parameter('log_dir', os.path.join(os.path.expanduser('~'), 'car_data_logs'))
        self.declare_parameter('topics', '')          # 空 = 全部内置话题；逗号分隔子集

        log_root = os.path.expanduser(str(self.get_parameter('log_dir').value))
        run_id = time.strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(log_root, run_id)
        os.makedirs(self.run_dir, exist_ok=True)

        want = [t.strip() for t in str(self.get_parameter('topics').value).split(',') if t.strip()]
        spec_map = {t: s for s in TOPIC_SPECS for t in [s[0]]}

        self._writers = {}      # topic -> csv.writer
        self._fh = {}           # topic -> 文件句柄
        self._counts = {}       # topic -> 累计消息数
        self._first_wall = {}   # topic -> 首条 wall_ns
        self._last_wall = {}    # topic -> 末条 wall_ns
        self._prev_counts = {}
        self._meta_topics = {}

        qos = QoSProfile(depth=1000,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self._resolved = {}
        for topic, mod_path, type_name, fallbacks in TOPIC_SPECS:
            if want and topic not in want:
                continue
            msg_cls = self._import_msg(mod_path, type_name, fallbacks)
            if msg_cls is None:
                self.get_logger().warn('[%s] 消息类型 %s.%s 导入失败，跳过（确认已 source install/setup.bash）'
                                       % (topic, mod_path, type_name))
                continue
            self._resolved[topic] = '%s/%s' % (mod_path.replace('.msg', ''), type_name)
            self._meta_topics[topic] = {'msg_type': self._resolved[topic],
                                        'received': 0, 'first_wall_ns': None, 'last_wall_ns': None}
            self.create_subscription(msg_cls, topic, self._make_cb(topic, msg_cls), qos)
            self.get_logger().info('订阅 %s  <- %s' % (topic, self._resolved[topic]))

        if not self._resolved:
            self.get_logger().error('没有任何话题订阅成功，退出。请先 source 工作空间再运行。')
            sys.exit(1)

        # 频率统计定时器（每 2s 写一行 rates.csv + 控制台摘要）
        self._rates_fh = open(os.path.join(self.run_dir, 'rates.csv'), 'w', newline='', encoding='utf-8')
        self._rates_w = csv.writer(self._rates_fh)
        self._rates_w.writerow(['wall_ns', 'topic', 'hz_last_2s', 'total_msgs'])
        self._rates_fh.flush()
        self.create_timer(2.0, self._rates_tick)

        # 启动信息
        self._start_wall = time.time_ns()
        self._start_ros = self.get_clock().now().nanoseconds
        self._write_meta()
        self.get_logger().info('记录开始 -> %s （Ctrl+C 停止后目录整体发回分析）' % self.run_dir)

    # ---------------- 工具 ----------------
    @staticmethod
    def _import_msg(mod_path, type_name, fallbacks):
        for mp, tn in [(mod_path, type_name)] + list(fallbacks):
            try:
                mod = importlib.import_module(mp)
                return getattr(mod, tn)
            except Exception:
                continue
        return None

    def _make_cb(self, topic, msg_cls):
        skip = SKIP_FIELD_PREFIXES.get(topic, ())

        def cb(msg):
            self._counts[topic] = self._counts.get(topic, 0) + 1
            wall = time.time_ns()
            ros = self.get_clock().now().nanoseconds
            if topic not in self._first_wall:
                self._first_wall[topic] = wall
                self._meta_topics[topic]['first_wall_ns'] = wall
            self._last_wall[topic] = wall

            w = self._writers.get(topic)
            if w is None:
                fields = flatten_fields(msg, '', skip)
                headers = ['wall_ns', 'ros_ns'] + [n for n, _ in fields]
                fh = open(os.path.join(self.run_dir, topic.replace('/', '_').strip('_') + '.csv'),
                          'w', newline='', encoding='utf-8')
                w = csv.writer(fh)
                w.writerow(headers)
                self._writers[topic] = w
                self._fh[topic] = fh
                self.get_logger().info('[%s] 开始写文件，%d 列' % (topic, len(headers)))
            w.writerow([wall, ros] + [v for _, v in flatten_fields(msg, '', skip)])
            self._fh[topic].flush()
        return cb

    def _rates_tick(self):
        wall = time.time_ns()
        for topic, total in self._counts.items():
            prev = self._prev_counts.get(topic, total)
            hz = (total - prev) / 2.0
            self._rates_w.writerow([wall, topic, round(hz, 3), total])
            self._prev_counts[topic] = total
        self._rates_fh.flush()
        alive = sorted(t for t in self._counts if self._counts[t] > 0)
        self.get_logger().info('[live] %d 个话题有数据: %s' % (len(alive), ', '.join(alive)))

    def _write_meta(self):
        meta = {
            'start_wall_ns': self._start_wall,
            'start_ros_ns': self._start_ros,
            'note': 'wall_ns=系统墙钟纳秒; ros_ns=节点时钟纳秒; 各话题CSV字段与.msg定义一一对应; rates.csv每2s频率',
            'topics': self._meta_topics,
        }
        with open(os.path.join(self.run_dir, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

    def shutdown_and_close(self):
        for topic, total in self._counts.items():
            self._meta_topics[topic]['received'] = total
            self._meta_topics[topic]['last_wall_ns'] = self._last_wall.get(topic)
            if self._first_wall.get(topic) and self._last_wall.get(topic):
                dur = (self._last_wall[topic] - self._first_wall[topic]) / 1e9
                if dur > 0:
                    self._meta_topics[topic]['avg_hz'] = round(total / dur, 3)
        self._write_meta()
        for fh in self._fh.values():
            try:
                fh.close()
            except Exception:
                pass
        try:
            self._rates_fh.close()
        except Exception:
            pass
        self.get_logger().info('记录结束：%d 个话题，目录 %s' % (len(self._counts), self.run_dir))


def main():
    rclpy.init()   # 自动解析 --ros-args
    node = CarDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
