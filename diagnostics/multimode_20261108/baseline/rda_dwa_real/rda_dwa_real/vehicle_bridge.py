"""
vehicle_bridge.py — 实车（非模型版本）核心闭环逻辑（纯 Python，可单测）。

与仿真主循环（trial_runner._run 的每帧流程）保持同构：
  1. 定位 → robot 位姿（算法坐标系，x/y/theta）
  2. 障碍物：外部感知 → 坐标系转换 → TrackManager 构造/更新 Obstacle
     （type/L/W/radius 创建时锁定；每帧只更新位置）
  3. predictor.update(obs, x, y)：真值就地更新 → IMM/外推产出 fx/fy/speed/heading
     （与仿真 pred.update 完全一致，DWA/DQN 消费同一 obs 列表）
  4. trap 逃逸状态机（ClassicTrapController，与仿真同款，不依赖 Scene）
  5. planner.plan(robot, goal, obs, trap) → (v_cmd, ω_cmd)
  6. 4WIS 运动学约束（adapt_velocity，与仿真同款）
  7. 闭环碰撞检测（急停/日志用）

说明：C8 A* 全局引导依赖仿真栅格地图，实车默认关闭；goal 为外部目标点。
"""
from __future__ import annotations

import logging
import math
import time
from typing import Optional

# P0-1 (U6/U11)：VehicleBridgeCore 为纯 Python（无 rclpy），用模块级 logger 输出假0轮速告警。
# 在 ROS2 节点（bridge_node）中该告警走 stderr → ros2 launch 日志可见；如需节点式 logger，
# 可由 bridge_node 后续 attach（本次不改 bridge_node，保持最小侵入）。
_logger = logging.getLogger('vehicle_bridge')


def compute_spin_target(trap_active: bool, sub_goal, robot_x: float, robot_y: float,
                        theta_apf: float) -> float:
    """自转目标朝向纯函数（可单测，无 ROS 依赖）。

    口径（★2026-11-06 用户口述逻辑）：
      - 顶层阿克曼绕不过 → 自转目标 = **当前车位置的 APF 合力方向**（theta_apf，
        由 bridge_node 用 planner.apf_force([x,y], goal, obs) 算出）；
      - trap 激活期间 → 自转目标 = **trap 子目标点方向** atan2(sy-y, sx-x)
        （子目标在激活时冻结，方向稳定，不会随 APF 抖动改主意）；
      - 子目标缺失/异常 → 回落 theta_apf。
    坐标系：robot/sub_goal 均在 algorithm_frame=map 系（与 /rda/traj 同系）。
    """
    if trap_active and sub_goal is not None:
        try:
            sx, sy = float(sub_goal[0]), float(sub_goal[1])
            return math.atan2(sy - robot_y, sx - robot_x)
        except Exception:
            return theta_apf
    return theta_apf

from rda_dwa.robot_model import RobotState
from rda_dwa.four_wheel_steer_model import (
    FourWheelSteerParams, FourWheelSteerModel, FourWheelSteerState,
)
from rda_dwa.ackermann_model import AckermannModel, AckermannState
from rda_dwa.trial_runner import ClassicTrapController
from rda_dwa.predictor import ObstaclePredictor, IMMUKFPredictor
from rda_dwa.obstacle import Obstacle
from .obstacle_fusion import TrackManager
from .frame_utils import (  # noqa: F401  (re-export 供 ROS 层使用)
    body_to_world_2d, world_to_body_2d, quat_to_yaw, is_body_frame,
    SensorExtrinsics, wrap_angle,
)


class BridgeConfig:
    """实车适配参数（默认值；ROS2 节点从 yaml 覆盖）。"""

    def __init__(self):
        # 控制周期（s）；算法 dt 与仿真一致 0.1 → 默认 10Hz
        self.control_dt = 0.1
        # 算法坐标系：'odom' 或 'map'（必须与定位位姿所在 frame 一致）
        self.algorithm_frame = 'map'
        # 车体系/传感器系 frame 列表（需要按定位位姿转换）
        # self.body_frames = ['base_link', 'base_footprint', 'laser', 'lidar', 'sensor']
        # 传感器 frame 列表（在 body_frames 基础上还要叠加 sensor→body 外参）。
        # 若雷达 frame 即 base_link（外参=0），可留空。
        # self.sensor_frames = ['laser', 'lidar', 'sensor', 'laser_frame', 'lidar_frame']
        # 传感器外参（sensor → 车体几何中心）
        self.sensor_x = 0.0
        self.sensor_y = 0.0
        self.sensor_yaw = 0.0
        # 碰撞急停：plan 判定不可行/闭环碰撞时是否输出 0 速
        self.emergency_stop = True
        # 定位缺失超时（s）：超过则急停
        self.localization_timeout = 1.0
        # 障碍物缺失超时（s）：超过则清空障碍（开阔行驶）
        self.obstacle_timeout = 2.0
        # 目标点不设超时：收到一次即持久有效，直到到达（reached）或被新目标替换
        # 逃逸状态机开关（与仿真 C3 一致；默认开）
        self.enable_trap = True
        # 动态障碍最大速度（predictor 上限，m/s）
        self.obstacle_max_speed = 1.5
        # 底盘轮速反馈新鲜度（s）：/car_state 20Hz，超过此间隔视为反馈失效 → fallback 计划回写
        self.wheel_feedback_timeout = 0.3
        # ---- P0-1 (U6/U11)：轮速反馈合理性门（第二批：保守默认值，实车标定） ----
        # 假 0 轮速（Wheel 段缺失）被当成真速度 → robot.v 钉死 0 → 动态窗坍缩 [0,0.1]。
        # 判据：0.3s 内有帧 且 四轮均值|v|<wheel_fb_valid_min 且 最近指令|v|>wheel_fb_cmd_ref
        #       且 持续 > wheel_fb_suspect_dur → 视为反馈异常，回退 None（走原 fallback：
        #       计划速度/定位差分）。阈值先用保守默认值，实车标定见落地报告取证指引。
        self.wheel_fb_valid_min = 0.02      # 四轮均值|v| 低于此(m/s) 视为可疑假0
        self.wheel_fb_suspect_dur = 0.5     # 可疑状态持续超过此(s) 才判异常（防抖）
        self.wheel_fb_cmd_ref = 0.05         # 最近指令|v| 高于此(m/s) 说明本应在动
        # ---- 模式仲裁参数（2026-11-04：SPIN/trap 仲裁全部上收 planner 侧；
        #   参数名与 real_vehicle.yaml rda_dwa_real_bridge 段完全一致，节点层可覆盖）----
        self.v_streak_threshold = 0.05      # tracker /car_cmd.speed |v| 低于此(m/s) 计低速帧
        self.v_streak_count = 25             # 连续低速帧数（50Hz×25≈0.5s）达标才边触发
        self.ack_guard_time = 2.0            # bridge_state==0 持续 ≥此(s) 才允许 V-streak 计数
        self.spin_reset_progress = 0.5       # 距上次自转锚点 <此(m) 视为同一问题 → V-streak 再达标进 trap
        self.trap_exit_dist = 0.3            # trap 退出条件1：到达逃逸子目标此(m) 内
        self.trap_exit_goal_ang_deg = 20.0   # trap 退出条件2a：车头与"车→全局目标"map系连线夹角 <此(°)
        self.trap_exit_sector_deg = 60.0     # trap 退出条件2b：车头 ±此(°) 扇区统计窗
        self.trap_exit_sector_range = 2.5     # trap 退出条件2b：扇区内障碍物统计半径(m)
        self.trap_exit_sector_max_obs = 3     # trap 退出条件2b：扇区内障碍物 <此 才判清场退出
        self.trap_max_duration = 16.0         # trap 兜底最大持续时长 s（2026-11-05：
        # 原 trap_max_steps=120 按 bridge 10Hz=12s 折算偏短；改时间制 16s，不受规划频率漂移影响）
        self.spin_request_timeout = 15.0   # P2-修复④：SPIN_REQUEST 未见 tracker 执行的最长等待 s（正常周期≈13s，留余量）
        # ---- ★2026-11-06 卡顿判据（与 /car_cmd 50Hz 帧计数解耦）----
        # 原唯一触发源是 /car_cmd.speed 连续 25 帧 <0.05（V-streak），实车 197s 日志里
        # traj_mode 恒 0、trap_active 恒 False（自转/陷阱从未触发），该链路一旦断流即永久失效。
        # 新增：规划帧级（10Hz）按时间累计的"贴近障碍且不动"判据，达标即注入 V-streak 边触发。
        self.stall_obs_dist = 2.5          # 最近障碍 <此(m) 且本帧下发速度≈0 → 计卡顿
        self.stall_escape_time = 0.5       # 卡顿持续 >此(s) → 计为一帧"阿克曼不可行"

        # ================= ★2026-11-06 按用户口述逻辑重构的仲裁参数 =================
        # ---- (1) 阿克曼输出的 1.5 倍分辨率死区保持 ----
        # 选中 (v, ω) 与上一帧输出比较：|Δv|<1.5·v_res 且 |Δω|<1.5·ω_res 且上一帧
        # v/ω 均不为 0 → 保持上一帧不变（抑制栅格边界抖动直接传导到下层）。
        self.ack_hold_v_ratio = 1.5        # v 死区倍数（×params.v_resolution）
        self.ack_hold_w_ratio = 1.5        # ω 死区倍数（×params.omega_resolution）
        self.ack_hold_enable = True        # 死区保持总开关
        # ---- (2) ACK → SPIN 判据 ----
        # 连续 N 帧"选中 (v, ω) 不满足阿克曼运动学"，且阿克曼模式已连续运行 ≥2s。
        # 不满足运动学包含两类（任一即计该帧）：
        #   a) 曲率超限：|ω| > |v|·κ_max（含 v≈0 却要转向 → 无法执行）；
        #   b) 物理卡住：贴近障碍（<stall_obs_dist）且下发速度≈0 持续 stall_escape_time。
        self.ack_bad_count = 5             # 连续不可行帧数（10Hz 规划帧 = 0.5s）
        self.ack_bad_ratio = 0.02          # κ 超限判定的相对余量（防浮点边界反复）
        # ---- (3) 两次自转位移 <0.5m → 不切自转、改激活 trap ----
        self.spin_count_for_trap = 2       # 自转激活次数 ≥此 才考虑 trap
        self.spin_gap_eps = 0.5            # 最近两次自转激活点间距 <此(m) → 同一问题
        # ---- (4) trap 退出（用户两套条件）----
        self.trap_exit_far = 1.5           # 激活时距子目标 >此(m) → 用绝对 0.3m 退出
        self.trap_exit_ratio = 0.25        # 否则用"激活距离的 1/4"退出
        self.trap_exit_ang_deg = 60.0      # 条件2a：车头与"车→子目标"连线夹角 <此(°)
        self.trap_exit_sector_deg = 30.0   # 条件2b：子目标连线 ±此(°) 内的障碍统计窗
        self.trap_exit_sector_range = 2.5  # 条件2b：统计半径 m
        self.trap_exit_block_max = 0       # 条件2b：落在"车头↔子目标"夹角内的障碍 ≤此 才退出
        # ---- (5) 路基（下发跟踪层的参考）----
        self.road_base_len = 2.5           # 前方路基长度 m（用户口径）
        self.road_base_ds = 0.1            # 路基弧长采样步长 m


class VehicleBridgeCore:
    """实车闭环核心：每帧 step() 产出控制指令，不依赖任何 ROS 类型。"""

    def __init__(self, planner, pp, fws_params: FourWheelSteerParams | None = None,
                 predictor=None, config: BridgeConfig | None = None,
                 ack_model: AckermannModel | None = None):
        """ack_model: A 方案（阿克曼对齐）——非 None 时按 Ackermann 非完整约束适配：
        规划输出 (v_cmd, ω_cmd) → AckermannModel.adapt_velocity 限幅为 (v, δ)，
        发布角速度 ω=κ·v（与底盘实际执行转角严格一致）；v≈0 禁止转向。
        ack_model=None 保持原 4WIS 路径（v≈0 原地自转仍有效）。"""
        self.planner = planner
        self.pp = pp
        self.cfg = config or BridgeConfig()
        self.fws = FourWheelSteerModel(fws_params or FourWheelSteerParams())
        self.ack_model = ack_model
        self._ack_delta = 0.0          # Ackermann 前轮转角状态（rad，δ_dot 限幅基准）
        self.ext = SensorExtrinsics(self.cfg.sensor_x, self.cfg.sensor_y, self.cfg.sensor_yaw)

        if predictor is None:
            predictor = ObstaclePredictor(dt=self.cfg.control_dt,
                                          max_speed=self.cfg.obstacle_max_speed)
        self.predictor = predictor

        # 逃逸状态机（与仿真同款 ClassicTrapController）
        self.trap_ctrl = ClassicTrapController(self.planner, self.pp)
        self.trap = self.trap_ctrl.trap

        # 跨帧障碍物跟踪（type/L/W/radius 创建时锁定，每帧只更新位置）
        self.tracker = TrackManager(
            predictor=self.predictor,
            gate=1.0, lost_ttl=5, max_speed=self.cfg.obstacle_max_speed)

        # 状态
        self.robot = RobotState()
        self.state = FourWheelSteerState()
        self._frame = self.cfg.algorithm_frame
        self._detections: list = []
        self._last_plan_ok = True
        self._last_obs = []

        # 时间戳缓存（用于超时判断）
        self._t_loc = 0.0
        self._t_obs = 0.0
        self._t_now = 0.0
        # 目标点持久化：收到一次即一直有效，直到到达；未收到过目标时停车等待
        self._goal_received = False

        # ---- 底盘速度反馈（2026-11-01 用户确认：vw 采样/加速度基准用真实速度）----
        # 融合：v 一律用轮速均值；ω 自转执行(SPIN_RUN)用轮速差合成，其余用定位 θ 差分
        self._bridge_state = 0            # control_bridge 执行状态：0=ACK 1=HANGUP 2=SPIN_RUN
        # 【诊断只读】freeze_stall 最近一帧取值（bridge_state in (1,2)），step 内刷新；
        # 仅供 /rda/diag 发布读取，不参与任何控制分支。
        self._freeze_stall = False
        self._wheel_speeds: list = []     # 最近一轮四轮轮速（m/s）
        self._wheel_ts = -1e9             # 轮速反馈到达时刻（核心时钟 _t_now）
        self._prev_loc_t = None           # 上次定位注入时刻（θ 差分用）
        self._loc_dtheta = None           # 定位 θ 差分角速度（rad/s，阿克曼/挂起用）
        self._wheel_suspect_since = None  # P0-1：假0轮速怀疑状态起始时刻（核心时钟 _t_now）

        # ---- 模式仲裁（2026-11-04：SPIN/trap 判定全部上收 planner 侧，tracker 只执行） ----
        # 计数源 = tracker 实际下发 /car_cmd.speed（50Hz，set_tracker_cmd_speed 回调内累加）；
        # 仅 bridge_state==0 且 ACK 持续 ≥ack_guard_time 才累计；连续 v_streak_count 帧
        # |speed|<v_streak_threshold 才在 _arbitration_step 做边迁移（每规划帧 10Hz 执行）。
        self._arbit = 'ACK'               # 'ACK'=纯跟踪 / 'SPIN_REQUEST'=已请求 tracker 自转
        self._spin_saw_exec = False       # SPIN_REQUEST 后是否观察到 tracker 进入 1(HANGUP)/2(SPIN_RUN)
        self._v_streak = 0                # 低速连续帧计数（/car_cmd 回调内按门控累加）
        self._tracker_cmd_v = None        # tracker 最近下发 /car_cmd.speed（m/s，诊断用）
        self._ack_since = 0.0             # bridge_state==0 连续起始时刻（核心时钟 _t_now）
        self._spin_used = False            # 当前问题是否已用过自转（同一问题只许一次）
        self._spin_anchor = None          # (x,y) 上次自转请求锚点（"同一问题"判定基准）
        # ---- trap 内自转计数（2026-11-05 第二版规格：区分第 1 次 vs 后续自转朝向） ----
        # 第 1 次=trap 激活当帧子目标快照方向；第 2 次起=当前 APF 合力方向。
        # trap 任意退出路径时清零（与 spin_used 重置并存、互不干扰）。
        self._spin_in_trap_count = 0
        self._trap_initial_sub_goal = None   # trap 激活当帧 sub_goal 快照 (x,y) map 系
        self._trap_active_since = None       # trap 激活时刻（核心时钟 _t_now；16s 时间制兜底）
        # ---- P2-修复④：自转结果区分（tracker 经 /car_bridge_state=3/4 上报）----
        self._last_spin_result = None      # 最近一次自转会话结果：'reached'/'timeout'/None
        self._spin_req_since = 0.0         # 进入 SPIN_REQUEST 的时刻（未见执行兜底用）
        # ★2026-11-06 卡顿计时（规划帧级 dt 累计；见 BridgeConfig.stall_*）
        self._stall_time = 0.0             # 距障碍<stall_obs_dist 且下发达标速度≈0 的累计时长 s
        self._omega_meas = 0.0             # 定位 θ 差分偏航率低通（无轮速反馈时的 ω 基准）
        # ---- ★2026-11-06 用户口述逻辑新增状态 ----
        self._out_v = 0.0                  # 上一帧"实际下发"的 v（死区保持基准）
        self._out_w = 0.0                  # 上一帧"实际下发"的 ω（死区保持基准）
        self._hold_acc_v = 0.0             # 死区保持期间同向偏差累积（超过一个死区即释放，防爬行死锁）
        self._hold_acc_w = 0.0
        self._ack_bad_cnt = 0              # 连续"阿克曼不可行"规划帧计数
        self._spin_count = 0               # 自转模式累计激活次数
        self._spin_anchors: list = []      # 最近两次自转激活位置 [(x,y), ...]
        self._trap_sub_goal = None         # trap 激活时冻结的世界系子目标 (x,y)
        self._trap_sub_dist0 = 0.0         # 激活时车→子目标距离（1/4 退出比例基准）
        self._clear_traj_seq = 0           # 清空跟踪路径命令序号（模式切换/trap 边沿递增）
        self._clear_traj_pending = False   # 本帧是否需要发清空命令

    # ------------------------------------------------------------------
    # 输入注入（ROS 节点回调调用；纯数据，无 ROS 类型）
    # ------------------------------------------------------------------
    # 超时语义：_t_loc/_t_obs 记录"最近一次到达时刻"（核心时钟 _t_now），
    # 与消息自身时间戳解耦——实车话题时间戳（ROS time）或为 0 或与本地时钟
    # 不一致，若直接用消息 stamp 做老化判断会把新数据误判为超时。
    def set_localization(self, x: float, y: float, theta: float, stamp: float = None):
        """定位注入（算法坐标系位姿）。同时计算 θ 差分角速度（阿克曼模式 ω 反馈）。"""
        prev_theta = self.robot.theta
        self.robot.x = float(x)
        self.robot.y = float(y)
        self.robot.theta = float(theta)
        self._t_loc = self._t_now
        if self._prev_loc_t is not None:
            dlt = self._t_now - self._prev_loc_t
            if dlt > 0:
                self._loc_dtheta = wrap_angle(theta - prev_theta) / dlt
                # ★2026-11-06：α=0.35 低通，压定位差分噪声（50Hz 位姿抖动 → 原始差分可达数 rad/s）
                self._omega_meas = 0.65 * self._omega_meas + 0.35 * self._loc_dtheta
        self._prev_loc_t = self._t_now

    def set_wheel_feedback(self, speeds, stamp: float = None):
        """底盘四轮轮速反馈（m/s，来自 /car_state speed[4]）。"""
        vals = [float(s) for s in speeds if math.isfinite(float(s))]
        if vals:
            self._wheel_speeds = vals
            self._wheel_ts = self._t_now if stamp is None else stamp

    def set_tracker_cmd_speed(self, v: float):
        """tracker 实际下发速度注入（/car_cmd.speed，car_ctr_msgs/CarCtr，50Hz）。

        V-streak 仲裁计数核心源（2026-11-04）：
          仅当 tracker 处于 ACK 态（bridge_state==0）且 ACK 已持续 ≥ ack_guard_time(s)
          时，|v|<v_streak_threshold 累计 +1，否则该帧清零；非 ACK 窗口一律清零——
          自转/挂起期间 tracker 下发速度不是"跟踪卡顿"证据（SPIN 期 speed=ω 不能计入）。
        """
        self._tracker_cmd_v = float(v)
        if self._bridge_state == 0 \
                and (self._t_now - self._ack_since) >= self.cfg.ack_guard_time:
            if abs(self._tracker_cmd_v) < self.cfg.v_streak_threshold:
                self._v_streak += 1
            else:
                self._v_streak = 0
        else:
            self._v_streak = 0

    def set_bridge_state(self, state: int):
        """control_bridge 执行状态注入：0=正常阿克曼 1=挂起(车静止) 2=自转执行中。
        扩展（P2-修复④）：3=SPIN_REACHED（自转真转到位）4=SPIN_TIMEOUT（超时未到位），
        由 traj_tracker 在自转退出 STOP 期上报；仲裁据此区分"真转完"与"没转完"。

        新增（2026-11-04）：0 态重新进入时记录 _ack_since——V-streak 计数要求
        ACK 已持续 ≥ ack_guard_time（刚从 HANGUP/SPIN_RUN 返回的窗口不计入卡顿）。"""
        state = int(state)
        if state == 3:
            self._last_spin_result = 'reached'
        elif state == 4:
            self._last_spin_result = 'timeout'
        if state == 0 and self._bridge_state != 0:
            self._ack_since = self._t_now
        self._bridge_state = state

    def _feedback_speed(self):
        """合成底盘反馈 (v, ω)；反馈失效(超时/无数据)返回 None（fallback 计划回写）。

        融合规则（用户确认）：v 一律四轮均值；ω —— 自转执行(SPIN_RUN)用轮速合成
        （4WIS 自转四轮切向同速，ω=v/R_half，R_half=半对角线）；其余状态用定位 θ 差分
        （挂起期车静止 θ 不变 → ω≈0，恢复自然 ramp）。"""
        if not self._wheel_speeds \
                or (self._t_now - self._wheel_ts) > self.cfg.wheel_feedback_timeout:
            return None
        v_w = sum(self._wheel_speeds) / len(self._wheel_speeds)
        # ---- P0-1 (U6/U11)：假0轮速合理性门 ----
        # 最近指令 v 取 planner._last_cmd_v（上一周期 adapt_velocity 后注入，与 /cmd_vel 一致；
        # 比 robot.v（本帧反馈/回写值）更可靠，因为 robot.v 此刻可能正被假0反馈污染）。
        # 判据：有帧(0.3s 内)且四轮均值|v|<valid_min 且 最近指令|v|>cmd_ref 且 持续 suspect_dur
        # → 反馈异常，回退 None（走原 fallback：反馈失效 → 下一帧 robot.v 回写计划速度/定位差分）。
        cmd_v = getattr(self.planner, '_last_cmd_v', None)
        _suspect = (abs(v_w) < self.cfg.wheel_fb_valid_min
                    and cmd_v is not None and abs(cmd_v) > self.cfg.wheel_fb_cmd_ref)
        if _suspect:
            if self._wheel_suspect_since is None:
                self._wheel_suspect_since = self._t_now
            elif (self._t_now - self._wheel_suspect_since) \
                    >= self.cfg.wheel_fb_suspect_dur:
                _dur = self._t_now - self._wheel_suspect_since
                _logger.warning(
                    '[P0-1] wheel feedback SUSPECT (假0轮速): mean|v_w|=%.4f < %.3f, '
                    'cmd|v|=%.4f > %.3f, suspect_dur=%.2fs → 回退计划速度/定位差分',
                    abs(v_w), self.cfg.wheel_fb_valid_min, cmd_v,
                    self.cfg.wheel_fb_cmd_ref, _dur)
                return None
        else:
            self._wheel_suspect_since = None
        if self._bridge_state == 2:
            L = getattr(self.fws.p, 'length', 1.2)
            W = getattr(self.fws.p, 'width', 0.8)
            r_half = math.hypot(L / 2.0, W / 2.0)
            w_fb = v_w / r_half if r_half > 0 else 0.0
        else:
            w_fb = self._loc_dtheta if self._loc_dtheta is not None else 0.0
        return v_w, w_fb

    def set_goal(self, gx: float, gy: float, stamp: float = None, frame: str = 'odom'):
        """目标点注入（frame: 算法坐标系名或车体系名，由 ROS 层先转换好）。

        目标点持久化：每次收到新目标即整体替换并永久有效（无超时过期），
        直到到达（reached）或收到下一个目标点。
        """
        self._goal = (float(gx), float(gy))
        self._goal_frame = frame
        self._goal_received = True
        # 新目标到达：自旋问题状态重置（新任务允许再用一次自转；2026-11-04 仲裁）
        self._reset_spin_problem()
        # P3-修复：新目标到达清零 V-streak——启动期/无目标期 tracker 下发 speed=0 会以
        # 50Hz 累积低速帧，不清零会导致首帧目标即误触发一次性 SPIN_REQUEST
        # （表现为"刚启动自动 阿克曼→自转→阿克曼"）。
        self._v_streak = 0

    def set_obstacles(self, detections, stamp: float = None, frame: str = 'odom'):
        """障碍物注入。

        detections: 列表，每项 (x, y, otype, L, W, heading, speed[, id])，
        坐标已在调用方转换为算法坐标系（ROS 层负责 frame 转换）。
        frame: 'odom'/'map'（算法系）或车体系（自动转换）。
        """
        self._t_obs = self._t_now
        if frame is not None and is_body_frame(frame, self.cfg.body_frames):
            detections = self._to_world_detections(detections)
        self._detections = detections
        self._frame = frame if frame else self.cfg.algorithm_frame

    def _to_world_detections(self, dets):
        out = []
        rx, ry, rth = self.robot.x, self.robot.y, self.robot.theta
        apply_ext = (self._frame is not None
                     and is_body_frame(self._frame, self.cfg.sensor_frames))
        for d in dets:
            x, y, otype, L, W, heading, speed = d[0], d[1], d[2], d[3], d[4], d[5], d[6]
            if apply_ext:
                xw, yw, hw = self.ext.to_map(x, y, heading, rx, ry, rth)
            else:
                # 已是 base_link 系（或等效）：只做 body→world，不加传感器外参
                xw, yw, hw = body_to_world_2d(x, y, heading, rx, ry, rth)
            sid = d[7] if len(d) > 7 else None
            out.append((xw, yw, otype, L, W, hw, speed, sid))
        return out

    # ------------------------------------------------------------------
    # 主循环单步
    # ------------------------------------------------------------------
    def step(self, dt: Optional[float] = None) -> dict:
        """执行一个控制周期。

        返回 info 字典：
          ok          plan 是否返回可行 (v, ω)
          v_cmd/ω_cmd  受限后的控制指令（未受限值在 raw_v_cmd/raw_w_cmd）
          reached     是否到达目标（goal_tolerance）
          collision   闭环碰撞判定是否命中
          n_obs       当前障碍物数
          trap_active 逃逸状态机是否激活
          dist_goal   到目标距离
        控制周期内若有超时/急停条件，v_cmd=ω_cmd=0。
        """
        cfg = self.cfg
        dt = dt or cfg.control_dt
        self._t_now += dt

        info = {'ok': True, 'v_cmd': 0.0, 'w_cmd': 0.0,
                'raw_v_cmd': 0.0, 'raw_w_cmd': 0.0,
                'reached': False, 'collision': False,
                'n_obs': 0, 'trap_active': bool(self.trap.get('active', False)),
                'dist_goal': math.inf, 'plan_ok': None}

        # ---- 超时守卫 ----
        if not hasattr(self, '_goal'):
            self._goal = (0.0, 0.0)
        if not self._goal_received:
            return self._stop_info(info, reason='no_goal')
        if self._t_now - self._t_loc > cfg.localization_timeout:
            return self._stop_info(info, reason='localization_timeout')

        gx, gy = self._goal
        dg = math.hypot(self.robot.x - gx, self.robot.y - gy)
        info['dist_goal'] = dg
        if dg < self.pp.goal_tolerance:
            info['reached'] = True
            self._reset_spin_problem()   # 到达目标：自旋问题状态重置（2026-11-04 仲裁）
            return self._stop_info(info, reason='reached')

        # ---- 底盘速度反馈（用户确认 2026-11-01）：vw 采样/加速度约束基准用真实速度 ----
        # 反馈覆盖 robot/state 速度；反馈失效(超时/无数据)保持原值 fallback 计划回写。
        fb = self._feedback_speed()
        if fb is not None:
            fb_v, fb_w = fb
            self.robot.v, self.robot.omega = fb_v, fb_w
            self.state.v, self.state.omega = fb_v, fb_w
            # A 方案：由反馈 (v, ω) 恢复 Ackermann 前轮转角状态（δ=atan(ω·L/v)），
            # 供 δ_dot 限幅作基准；v≈0 时保持上一帧 δ（不转向）。
            if self.ack_model is not None and abs(fb_v) > self.ack_model.p.v_eps:
                self._ack_delta = max(-self.ack_model.p.delta_max,
                                      min(self.ack_model.p.delta_max,
                                          math.atan(fb_w * self.ack_model.p.wheelbase / fb_v)))

        # ---- 障碍物：转换 + 跟踪 + 预测（与仿真 pred.update 同构） ----
        detections = getattr(self, '_detections', [])
        if self._t_now - self._t_obs > cfg.obstacle_timeout:
            detections = []
        self.tracker.update_measurements(detections)
        obs = sorted(self.tracker.tracks.values(), key=lambda o: o.id)
        info['n_obs'] = len(obs)
        self._last_obs = obs

        # ---- 逃逸/模式仲裁（2026-11-04：SPIN/trap 激活权全部上收 planner 侧） ----
        if cfg.enable_trap:
            if self.trap_ctrl.prev_pos is None:   # 首次 step 时以当前位姿为基线
                self.trap_ctrl.reset(self.robot)
            # 【诊断只读】旧 bridge-state 冻结语义保留记录（供 /rda/diag 观察执行模式）。
            self._freeze_stall = self._bridge_state in (1, 2, 3, 4)   # P2-修复④：3/4（自转退出 STOP 期）同样冻结 trap 内部通道
            # trap 激活态"帧前"快照：覆盖 trap_ctrl.step 内部旧退出 / trap_max_steps 兜底
            # 与本帧 _force_trap_off 全部退出路径，确保任意 active→非 active 跳变都触发重置。
            trap_was_active = bool(self.trap.get('active', False))
            # ★旧 stall 位移通道 + 几何 detect_trap 激活通道整体冻结：
            #   freeze_stall=True 传入后，trial_runner.py:946-997 检测/触发侧整体跳过
            #   （stall 清零、`not freeze_stall` 门使 _stall_trig 与 detect_trap 均不激活 trap），
            #   彻底消除双触发；trap 激活唯一入口为本帧 _arbitration_step 的 V-streak 边触发。
            #   trap 已激活时逃逸推进/sub_goal 重生成/trap_max_steps=120 兜底不受 freeze 影响
            #   （trial_runner.py:915-936 的 trap['active'] 分支内无 freeze 判断）。
            self.trap_ctrl.step(self.robot, self.robot, obs, (gx, gy),
                                t=self._t_now, freeze_stall=True,
                                v_cmd=None, dt=dt)
            # ★2026-11-06 用户口径：trap 子目标的距离/方向在激活时冻结。
            # trap_ctrl.step 内部每帧会重算 sub_goal/escape_heading（子目标随车漂移），
            # 这里每帧回写激活时的世界系快照 → 子目标表现为"固定航点"，
            # 使规划目标切换、自转目标方向、退出距离判据三者严格同一基准。
            if self.trap.get('active', False) and self._trap_sub_goal is not None:
                self.trap['sub_goal'] = self._trap_sub_goal
                if self._trap_escape_heading is not None:
                    self.trap['escape_heading'] = self._trap_escape_heading
            # 新仲裁状态机：V-streak 边触发 → SPIN 请求 / trap 激活（纯逻辑，见下）
            self._arbitration_step(self.robot, obs, gx, gy, dt)
            # 新 trap 退出条件（子目标 0.3m / 朝向对齐+扇区清场）命中则强制关 trap。
            # 门：trap_steps >= pp.trap_min_steps(=10) 才评估——激活当帧 generate_local_goal
            # 生成的子目标仅 ~0.2m 远（escape.py:538），若不加门会当帧命中"0.3m 内"立即退出，
            # 与旧退出条件 trial_runner.py:928 的 trap_min_steps 门语义一致。
            if self.trap.get('active', False) \
                    and self.trap_ctrl.trap_steps >= self.pp.trap_min_steps \
                    and self._trap_exit(self.robot, obs, gx, gy):
                self._force_trap_off('arbit_exit')
            # 2026-11-05：trap 兜底改时间制——激活超过 trap_max_duration(16s) 强制退出。
            # 折算依据：原 trap_max_steps=120 按 bridge 10Hz=12s 偏短；时间制不受频率漂移影响，
            # trap_ctrl 内部 120 步兜底保留为二层保险。
            if self.trap.get('active', False) and self._trap_active_since is not None \
                    and (self._t_now - self._trap_active_since) > cfg.trap_max_duration:
                self._force_trap_off('timeout_16s')
            # 任意 trap→非 trap 跳变（新退出 / 16s 时间兜底 / 旧 trap_max_steps / 旧 disp 门）
            # → 重置自旋问题 + trap 内自转计数与子目标快照（2026-11-05）。
            if trap_was_active and not self.trap.get('active', False):
                self._reset_spin_problem()
                self._spin_in_trap_count = 0
                self._trap_initial_sub_goal = None
                self._trap_active_since = None
            info['trap_active'] = bool(self.trap.get('active', False))
        trap_arg = self.trap if cfg.enable_trap else None

        # ---- 规划 ----
        try:
            ok, v_cmd, w_cmd = self.planner.plan(self.robot, (gx, gy), obs, trap_arg)
        except Exception:
            ok, v_cmd, w_cmd = False, 0.0, 0.0
        self._last_plan_ok = ok
        info['plan_ok'] = ok
        info['raw_v_cmd'] = float(v_cmd)
        info['raw_w_cmd'] = float(w_cmd)

        # ---- 闭环碰撞判定（急停/日志） ----
        if self.ack_model is not None:
            ack_s = AckermannState(self.robot.x, self.robot.y, self.robot.theta,
                                   self.state.v, self._ack_delta)
            corners = self.ack_model.rectangle_corners(ack_s)
        else:
            corners = self.fws.rectangle_corners(self.state)
        hit = self._collision_hit(corners, obs)
        info['collision'] = hit

        # ---- 运动学约束（与仿真 adapt_integrate 同款；实车不积分，只约束） ----
        # 注意：仿真主循环忽略 plan 的 ok 标志，直接采用算法兜底链输出的
        # (v_out, ω_out)（M6/C4 等保证任何情形都有指令）。此处同样不因
        # ok=False 强制置零——只有闭环碰撞/超时才急停。
        if self.ack_model is not None:
            # A 方案：Ackermann 适配——v 限幅+加速度约束、δ=atan(ω·L/v)、
            # δ 限幅 ±δmax / δ 变化率限幅；发布 ω=κ·v 保证 |ω|≤v·κ_max、v≈0 时 ω=0
            # （与控制桥 δ=atan(ω·L/v) 换算一致，规划/执行严格对齐）。
            v, delta, kappa = self.ack_model.adapt_velocity(
                v_cmd, w_cmd, self.state.v, self._ack_delta, dt)
            self._ack_delta = delta
            w = kappa * v
        else:
            v, w = self.fws.adapt_velocity(v_cmd, w_cmd,
                                           self.state.v, self.state.omega, dt)
        # ★第四轮 改动3：实车不支持负线速度，末道闸钳 v≥0（ack v_min=-0.3/倒车段、
        #   FWS 负向样本一律钳到 0；ω 保留转向量，原地自转由 tracker SPIN 态负责）。
        v = max(0.0, v)
        # ---- ★2026-11-06 用户口径(2)：1.5 倍分辨率死区保持 ----
        # 选中 (v, ω) 与上一帧下发比较：|Δv|<1.5·v_res 且 |Δω|<1.5·ω_res 且上一帧
        # v/ω 均不为 0 → 保持上一帧，不把栅格边界抖动直接传导给下层跟踪器。
        v, w = self._apply_deadband_hold(v, w)
        self.state.v, self.state.omega = v, w
        # P2-修复③：SPIN_REQUEST 期间不发布前向指令（/cmd_vel 置 0、路基置空）——
        # 自转由 tracker 直接执行，前向指令与自转命令矛盾时会"切了自转却不转"。
        if self._arbit == 'SPIN_REQUEST':
            v, w = 0.0, 0.0
            self.state.v, self.state.omega = 0.0, 0.0
            self._out_v, self._out_w = 0.0, 0.0
        # 回写执行速度到 robot：反馈有效时 robot 保持真实速度（step 开头已覆盖，
        # 供下一帧动态窗口以真实速度为基准）；反馈失效时回写计划约束值（fallback）
        if fb is None:
            self.robot.v = v
            # ★2026-11-06 蛇形主因修复：无轮速反馈（实车 /car_state 0Hz）时，ω 基准
            # 不能回写上帧"指令 ω"——动态窗口 wmin/wmax = ω_prev ± acc_ang_max*dt 会变成
            # 棘轮：每帧只能朝同一方向走 0.2rad/s，反向需 5~7 帧 ≈0.5~0.7s，期间车按
            # 0.65rad/s 继续偏航 → 过冲后大幅反打（实车 ±30° 打满、约 5s 周期蛇形）。
            # 改用定位 θ 差分（真实偏航率，_omega_meas，已低通）作为窗口基准，指令 ω 不再自激。
            w_ref = self._omega_meas if math.isfinite(self._omega_meas) else 0.0
            w_lim = float(getattr(self.pp, 'max_angular_velocity', 0.7))
            self.robot.omega = max(-w_lim, min(w_lim, w_ref))

        if hit and cfg.emergency_stop:
            info['ok'] = False
            info['v_cmd'], info['w_cmd'] = 0.0, 0.0
            info['reason'] = 'collision'
            # 碰撞急停：上周期基准置零（停车/急停期平滑与粘性均不生效）
            if hasattr(self.planner, 'set_last_cmd'):
                self.planner.set_last_cmd(0.0, 0.0)
            return info
        info['ok'] = ok
        info['v_cmd'], info['w_cmd'] = float(v), float(w)
        # ---- ★2026-11-06 用户口径(3)：发布"当前位置 + (ω, v)"积分的前方 2.5m 路基 ----
        # 路基点 (x, y, θ, v, t_rel)：几何=恒曲率圆弧精确步进，t_rel=弧长/|v|（参考时刻）。
        # 自转模式 / 停车（|v|<v_eps）不发（返回 []）——跟踪层此时无路径，只执行转向。
        info['road_base'] = [] if self._arbit == 'SPIN_REQUEST' else \
            self._build_road_base(v, w)
        # ---- 模式仲裁产出：/rda/traj_mode 0=阿克曼跟踪 1=自转；清空命令边沿 ----
        info['traj_mode'] = 1 if self._arbit == 'SPIN_REQUEST' else 0
        info['arbit'] = self._arbit
        info['v_streak'] = int(self._v_streak)
        info['ack_bad_cnt'] = int(self._ack_bad_cnt)
        info['spin_count'] = int(self._spin_count)
        if self._clear_traj_pending:
            self._clear_traj_seq += 1
            self._clear_traj_pending = False
        info['traj_clear_seq'] = int(self._clear_traj_seq)
        # trap 子目标（冻结的世界系航点；未激活/已退出 → None）——诊断与自转目标方向来源
        info['trap_sub_goal'] = self._trap_sub_goal if info['trap_active'] else None
        info['trap_sub_dist'] = float(self._trap_sub_dist0)
        info['stall_time'] = float(self._stall_time)
        # 注入本周期最终下发指令（adapt_velocity 之后，与 /cmd_vel 一致）——
        # 供 planner 代价平滑项参考基准与死区粘性匹配（2026-11-03 第二轮）。
        if hasattr(self.planner, 'set_last_cmd'):
            self.planner.set_last_cmd(v, w)
        return info

    # ------------------------------------------------------------------
    # 模式仲裁（2026-11-04：SPIN/trap 判定全部上收 planner 侧；纯逻辑、无 ROS 类型，
    # 可脱离 rclpy 单测——测试入口见文件头交付说明）
    # ------------------------------------------------------------------
    def arbit_mode_spin_requested(self) -> bool:
        """/rda/traj_mode 命令语义读取：True=请求 tracker 自转(1)，False=纯跟踪(0)。"""
        return self._arbit == 'SPIN_REQUEST'

    # ------------------------------------------------------------------
    # ★2026-11-06 用户口述逻辑：输出死区保持 / 阿克曼可执行性 / 路基 / 自转计数
    # ------------------------------------------------------------------
    def _apply_deadband_hold(self, v: float, w: float):
        """阿克曼输出 1.5 倍分辨率死区保持（用户口径(2)）。

        规则：|Δv| < 1.5·v_resolution 且 |Δω| < 1.5·omega_resolution，且**上一帧 v、ω 均不为 0**
              → 本帧输出 = 上一帧（保持），否则直接采用本帧。
        保持期间 _out_v/_out_w 不更新（锁存语义），否则连续小步变化会被"逐帧刷新基准"
        吃掉、退化为无滤波（见 deadband-filter-frozen-output 经验）。

        ★纠偏（2026-11-06 实测）：纯锁存会"卡死爬行"——规划每帧只加 0.05m/s（< 死区
       0.075），锁存后输出恒为 0.2m/s，而 M1 动态窗口以该输出为基准（每帧只能 +acc·dt），
        形成"输出不涨→窗口不涨→规划不涨→输出不涨"的闭环（repro:15s 只走 1.95m vs 8.91m）。
        因此改为**同向偏差累积器**：抖动区保持输出，但把 (plan - out) 累加到 _hold_acc_*；
        累积超过一个死区宽度即释放（输出跟上一次实测值）。交替抖动（±）自动抵消 → 仍然保持，
        单调小步（爬升起步）每 2 帧释放一次 → 不死锁。
        """
        cfg = self.cfg
        vf, wf = float(v), float(w)
        if not getattr(cfg, 'ack_hold_enable', True):
            self._out_v, self._out_w = vf, wf
            self._hold_acc_v = self._hold_acc_w = 0.0
            return vf, wf
        v_res = float(getattr(self.pp, 'v_resolution', 0.05))
        w_res = float(getattr(self.pp, 'omega_resolution', 0.1))
        dv_tol = cfg.ack_hold_v_ratio * v_res
        dw_tol = cfg.ack_hold_w_ratio * w_res
        pv, pw = self._out_v, self._out_w
        # 上一帧 v、ω 均为 0（或本帧全 0=停车）→ 不保持，直接采用（用户口径"均不为0才保持"）
        if (pv == 0.0 and pw == 0.0) or (vf == 0.0 and wf == 0.0):
            self._out_v, self._out_w = vf, wf
            self._hold_acc_v = self._hold_acc_w = 0.0
            return vf, wf
        dv, dw = vf - pv, wf - pw
        if abs(dv) < dv_tol and abs(dw) < dw_tol:
            # 偏差反号 = 在抖动（±）→ 累积器清零重来，实现"交替抖动自动抵消"
            if dv * self._hold_acc_v < 0.0:
                self._hold_acc_v = 0.0
            if dw * self._hold_acc_w < 0.0:
                self._hold_acc_w = 0.0
            self._hold_acc_v += dv
            self._hold_acc_w += dw
            if abs(self._hold_acc_v) < dv_tol and abs(self._hold_acc_w) < dw_tol:
                return pv, pw                      # 真·抖动区：保持上一帧输出
            # 单调小步累积超限 → 释放（不死锁）
            self._hold_acc_v = self._hold_acc_w = 0.0
            self._out_v, self._out_w = vf, wf
            return vf, wf
        self._hold_acc_v = self._hold_acc_w = 0.0
        self._out_v, self._out_w = vf, wf
        return vf, wf

    def _kappa_max(self) -> float:
        """阿克曼可执行曲率上限 κ_max = tan(δ_max)/L。"""
        if self.ack_model is not None:
            try:
                return abs(math.tan(self.ack_model.p.delta_max)) / max(self.ack_model.p.wheelbase, 1e-3)
            except Exception:
                pass
        delta = math.radians(float(getattr(self.pp, 'ack_delta_max_deg', 30.0)))
        return abs(math.tan(delta)) / max(float(getattr(self.pp, 'ack_wheelbase', 0.69)), 1e-3)

    def _curvature_infeasible(self) -> bool:
        """本帧选中 (v, ω) 是否**不满足阿克曼运动学**（用户口径(4)a）。

        判据：|ω| > |v|·κ_max(1+余量)。v≈0 却要转向（|ω|>0）必然命中——这正是
        "阿克曼转不过去、需要原地自转"的物理本质；曲率饱和的样本也无法由底盘执行。
        """
        cfg = self.cfg
        v, w = float(self.state.v), float(self.state.omega)
        return abs(w) > abs(v) * self._kappa_max() * (1.0 + cfg.ack_bad_ratio) + 1e-6

    def _spin_gap(self) -> float:
        """最近两次自转激活位置的间距 m（<1 次自转时返回 inf）。"""
        if len(self._spin_anchors) < 2:
            return math.inf
        a, b = self._spin_anchors[-2], self._spin_anchors[-1]
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _build_road_base(self, v: float, w: float) -> list:
        """按当前位置与最终 (ω, v) 积分前方 2.5m 路基（点含 x/y/θ/v/t_rel）。"""
        from .traj_predict import build_road_base
        if self.ack_model is not None:
            wheelbase = float(self.ack_model.p.wheelbase)
            dmax = math.degrees(abs(self.ack_model.p.delta_max))
        else:
            wheelbase = float(getattr(self.pp, 'ack_wheelbase', 0.69))
            dmax = float(getattr(self.pp, 'ack_delta_max_deg', 30.0))
        return build_road_base(v, w, self.robot.x, self.robot.y, self.robot.theta,
                               wheelbase=wheelbase, delta_max_deg=dmax,
                               length=self.cfg.road_base_len, ds=self.cfg.road_base_ds,
                               v_eps=float(getattr(self.pp, 'traj_v_eps', 0.05)))

    def _reset_spin_problem(self):
        """重置"当前问题"的自转/陷阱状态（新问题开始）。

        触发时机：到达目标 / 新目标点 / trap 退出（任意路径）——都表示"这一段问题结束了"，
        下一次卡顿重新走"自转 → 自转 → trap"的完整链条（用户口径：自转 ≥2 次且原地不动才 trap）。
        """
        self._spin_used = False
        self._spin_anchor = None
        self._spin_count = 0
        self._spin_anchors = []
        self._ack_bad_cnt = 0
        self._stall_time = 0.0
        self._spin_in_trap_count = 0

    def _activate_trap(self, robot, obs, goal):
        """新仲裁通道激活 trap（替代 trial_runner.py:996-1004 旧 stall/几何触发路径）。

        内部状态重置与字段填充与 ClassicTrapController.step 激活分支逐一对齐：
        trap_steps=0 / stall=0 / _stall_time=0 / _trap_active_disp=0 /
        n_trap_activations+=1 / sub_goal+escape_heading=generate_local_goal(...)
        （enhanced_dwa.py:174-176 消费 sub_goal 替换全局目标）。"""
        trap = self.trap
        trap['active'] = True
        self.trap_ctrl.trap_steps = 0
        self.trap_ctrl.stall = 0
        self.trap_ctrl._stall_time = 0.0
        self.trap_ctrl._trap_active_disp = 0.0
        self.trap_ctrl.n_trap_activations += 1
        trap['sub_goal'], trap['escape_heading'] = \
            self.planner.generate_local_goal(robot, obs, goal)
        # 2026-11-05：记录激活当帧子目标快照（trap 内第 1 次自转朝向基准）、
        # 自转计数清零、激活时刻（16s 时间制兜底用）。
        sub = trap.get('sub_goal')
        self._trap_initial_sub_goal = (float(sub[0]), float(sub[1])) if sub is not None else None
        self._spin_in_trap_count = 0
        self._trap_active_since = self._t_now
        # ---- ★2026-11-06 用户口径：trap 子目标的距离与方向在激活时**冻结** ----
        # 激活后 trap_ctrl.step 每帧会重新 generate_local_goal（子目标随车漂移），
        # 与"内部产生子目标点的距离和方向保持不变"不符；此处存世界系快照，
        # step() 内每帧回写覆盖，使子目标成为固定世界航点（退出判据也以它为准）。
        self._trap_sub_goal = self._trap_initial_sub_goal
        try:
            self._trap_escape_heading = float(trap.get('escape_heading')) \
                if trap.get('escape_heading') is not None else None
        except Exception:
            self._trap_escape_heading = None
        # 激活时刻车→子目标距离（用户退出条件(1) 的"1/4 初始距离"基准）
        if self._trap_sub_goal is not None:
            self._trap_sub_dist0 = math.hypot(robot.x - self._trap_sub_goal[0],
                                              robot.y - self._trap_sub_goal[1])
        else:
            self._trap_sub_dist0 = 0.0
        # trap 激活 → 跟踪层路径必须清空（子目标已变，旧路基不再对应新目标）
        self._clear_traj_pending = True
        # trap 内自转目标由子目标方向决定（bridge_node 侧按 sub_goal 计算 spin_target）
        self._ack_bad_cnt = 0

    def _force_trap_off(self, reason: str = ''):
        """强制关闭 trap 并清零内部计数（新退出条件命中时覆盖 trap_ctrl 内部状态）。"""
        was = bool(self.trap.get('active', False))
        self.trap['active'] = False
        self.trap_ctrl.trap_steps = 0
        self.trap_ctrl.stall = 0
        self.trap_ctrl._stall_time = 0.0
        self.trap_ctrl._trap_active_disp = 0.0
        # ★2026-11-06：trap 退出瞬间 → 跟踪层路径清空（子目标已失效，路基不再对应）
        if was:
            self._clear_traj_pending = True
            self._trap_sub_goal = None
            self._trap_sub_dist0 = 0.0
            self._ack_bad_cnt = 0

    def _trap_exit(self, robot, obs, gx: float, gy: float) -> bool:
        """trap 退出（用户口径两套条件，任一满足即退出）。

        条件1（距离）：
          - 激活时车→子目标距离 > trap_exit_far(1.5m) → 车进入 0.3m( trap_exit_dist )内退出；
          - 否则（激活时本来就近）→ 车进入"激活距离的 1/4"内退出
            （子目标常在车旁 0.2~0.5m，用绝对 0.3m 会在激活当帧就判成"已到达"）。
        条件2（朝向+遮挡）：
          - |车头 − "车→子目标"连线朝向| < trap_exit_ang_deg(60°)；
          - 且"车→子目标"连线 ±trap_exit_sector_deg(30°) 扇区内、range(2.5m) 内的障碍，
            落在【车头 ↔ 子目标】这个夹角楔形里的数量 ≤ trap_exit_block_max(0)
            ——即"要从当前朝向转到子目标方向所扫过的扇面里没有障碍堵着"。
        子目标为激活时冻结的世界系航点（self._trap_sub_goal）。
        """
        cfg = self.cfg
        sub = self._trap_sub_goal if self._trap_sub_goal is not None \
            else self.trap.get('sub_goal')
        if sub is None:
            return False
        d_sub = math.hypot(robot.x - sub[0], robot.y - sub[1])
        d0 = self._trap_sub_dist0 if self._trap_sub_dist0 > 1e-3 else d_sub
        if d0 > cfg.trap_exit_far:
            if d_sub < cfg.trap_exit_dist:
                return True
        else:
            if d_sub < cfg.trap_exit_ratio * d0:
                return True
        # ---- 条件2：朝向对齐 + 转向楔形无障碍 ----
        beta = math.atan2(sub[1] - robot.y, sub[0] - robot.x)
        if abs(wrap_angle(robot.theta - beta)) >= math.radians(cfg.trap_exit_ang_deg):
            return False
        half = math.radians(cfg.trap_exit_sector_deg)
        lo, hi = min(robot.theta, beta), max(robot.theta, beta)
        # 车头已与子目标连线对齐时楔形退化（lo≈hi）→ 把整个 ±sector 当前向楔形：
        # 否则"正前方 20° 处就有障碍"会因 lo<ang<hi 恒假被判成可退出（实测 bug）。
        if hi - lo < half:
            lo, hi = beta - half, beta + half
        blocked = 0
        for o in obs:
            d = math.hypot(o.fx - robot.x, o.fy - robot.y)
            if d > cfg.trap_exit_sector_range or d < 1e-6:
                continue
            ang = math.atan2(o.fy - robot.y, o.fx - robot.x)
            if abs(wrap_angle(ang - beta)) > half:
                continue          # 不在"车→子目标"连线 ±30° 扇区内，不算遮挡
            if lo < ang < hi:     # 落在"当前车头 ↔ 子目标朝向"的楔形里 → 堵住转向
                blocked += 1
        return blocked <= cfg.trap_exit_block_max

    def _arbitration_step(self, robot, obs, gx: float, gy: float, dt: float):
        """每规划帧执行一次的模式仲裁（纯逻辑，可单测）。

        ★2026-11-06 按用户口述逻辑重写（取代 V-streak 单通道）：

        判据链（阿克曼模式）：
          frame_bad = 曲率超限(|ω|>|v|κ_max，含 v≈0 要转向) 或 物理卡住(贴障且 v≈0 持续)
          连续 ack_bad_count(5) 帧 frame_bad 且 阿克曼模式已运行 ≥ack_guard_time(2s)
            → 触发一次模式决策：
               A) trap 已激活            → 请求自转，目标=子目标方向（trap 内可多次自转）
               B) 自转激活次数 ≥2 且 最近两次自转点间距 <0.5m
                                          → **不切自转**，在阿克曼模式下激活 trap
               C) 其余                    → 请求自转（目标=APF 合力方向），自转计数+1

        自转会话语义：tracker 报 bridge_state∈(1,2) → 见执行；∈(3,4) → 记录 reached/timeout；
        回到 0 → 回 ACK（v=ω 从 0 重新采样，清空跟踪路径由 traj_mode 边沿触发）。

        trap 语义：激活即冻结子目标（距离/方向不变）+ 清空跟踪路径；trap 内**不允许再激活**；
        退出满足两套条件之一（见 _trap_exit），退出瞬间再清空跟踪路径。
        """
        cfg = self.cfg

        # ---- ① 本帧"阿克曼不可行"判定与连续帧计数 ----
        try:
            near = min([math.hypot(o.fx - robot.x, o.fy - robot.y) for o in obs] or [1e9])
        except Exception:
            near = 1e9
        if abs(self.state.v) < cfg.v_streak_threshold * 2.0 and near < cfg.stall_obs_dist:
            self._stall_time += dt        # 贴障且下发达标速度≈0：卡住计时
        else:
            self._stall_time = 0.0
        frame_bad = self._curvature_infeasible() or (self._stall_time >= cfg.stall_escape_time)
        self._ack_bad_cnt = self._ack_bad_cnt + 1 if frame_bad else 0

        # ---- ② 自转会话进行中：只做握手观测 ----
        if self._arbit == 'SPIN_REQUEST':
            if self._bridge_state in (1, 2, 3, 4):
                self._spin_saw_exec = True
            if self._spin_saw_exec and self._bridge_state == 0:
                # tracker 自转结束回到 ACK（reached 或 timeout 都算一次"自转激活"已落地）
                if self._last_spin_result == 'timeout':
                    _logger.warning('[ARBIT] 自转超时未到位，按失败处理（下次卡顿走 trap）')
                self._arbit = 'ACK'
                self._spin_saw_exec = False
                self._ack_bad_cnt = 0
                self._stall_time = 0.0
                self._out_v, self._out_w = 0.0, 0.0   # 回 ACK：v/ω 归零重新采样
                self._ack_since = self._t_now
                self._last_spin_result = None
            elif not self._spin_saw_exec and \
                    (self._t_now - self._spin_req_since) > cfg.spin_request_timeout:
                _logger.warning('[ARBIT] 自转请求超时未见 tracker 执行(>%.1fs)，强制回 ACK',
                                cfg.spin_request_timeout)
                self._arbit = 'ACK'
                self._spin_saw_exec = False
                self._ack_bad_cnt = 0
                self._stall_time = 0.0
                self._out_v, self._out_w = 0.0, 0.0
                self._ack_since = self._t_now
                self._last_spin_result = None
            return

        # ---- ③ ACK：连续 5 帧不可行 + 阿克曼已运行 ≥2s → 模式决策 ----
        if self._ack_bad_cnt < cfg.ack_bad_count:
            return
        if (self._t_now - self._ack_since) < cfg.ack_guard_time:
            return
        self._ack_bad_cnt = 0            # 消费掉本次判定
        self._stall_time = 0.0

        if self.trap.get('active', False):
            # A) trap 内：请求自转（目标=子目标方向，bridge_node 按 sub_goal 算 spin_target）
            self._request_spin(robot, in_trap=True)
        elif (self._spin_count >= cfg.spin_count_for_trap
              and self._spin_gap() < cfg.spin_gap_eps):
            # B) 已自转 ≥2 次且最近两次几乎没挪窝（<0.5m）→ 同一问题，自转无解 → 激活 trap
            _logger.info('[ARBIT] 自转 %d 次、最近两次间距 %.2fm < %.2fm → 激活 trap',
                         self._spin_count, self._spin_gap(), cfg.spin_gap_eps)
            self._activate_trap(robot, obs, (gx, gy))
        else:
            # C) 正常：请求自转
            self._request_spin(robot, in_trap=False)

    def _request_spin(self, robot, in_trap: bool):
        """进入 SPIN_REQUEST：计数+锚点+清空跟踪路径（模式切换边沿）。"""
        self._arbit = 'SPIN_REQUEST'
        self._spin_saw_exec = False
        self._spin_req_since = self._t_now
        self._last_spin_result = None
        self._spin_count += 1
        self._spin_anchors.append((float(robot.x), float(robot.y)))
        if len(self._spin_anchors) > 2:
            self._spin_anchors.pop(0)
        if in_trap:
            self._spin_in_trap_count += 1
        # 模式切换 → 跟踪层路径清空（旧路基对应的是阿克曼目标，不再是当前模式）
        self._clear_traj_pending = True
        # 自转期间不发布前向指令：输出基准清零，回来时从 0 平滑起步
        self._out_v, self._out_w = 0.0, 0.0

    def _collision_hit(self, corners, obs) -> bool:
        from rda_dwa.trial_runner import _rect_hit
        for o in obs:
            if _rect_hit(corners, o):
                return True
        return False

    def _stop_info(self, info, reason: str):
        info['ok'] = False
        info['v_cmd'], info['w_cmd'] = 0.0, 0.0
        info['reason'] = reason
        # 急停/无目标/定位超时/到达目标等停车路径：上周期基准置零
        if hasattr(self.planner, 'set_last_cmd'):
            self.planner.set_last_cmd(0.0, 0.0)
        return info

    def wheel_commands(self):
        """当前 (v, ω) → 四轮转向角/轮速（4WIS 输出，供 /rda/wheel_* 话题）。"""
        return self.fws.wheel_steer_angles(self.state.v, self.state.omega), \
            self.fws.wheel_speeds(self.state.v, self.state.omega)
