# -*- coding: utf-8 -*-
"""把 real_vehicle.yaml 的 traj_tracker_node 段替换为新架构参数（按行区间精确替换）。"""
import io

PATH = r"C:\Users\hhhwxk\Desktop\src_doubao\rda_dwa_real\config\real_vehicle.yaml"

NEW = '''# ---------------- 轨迹跟踪控制（★2026-11-06 重构：里程域接续融合 + B样条 + 阿克曼LQR + 纵向PID） ----------------
# 规划层按"当前位置 + 最终(ω, v)"发前方 2.5m 路基（点含 x/y/θ/v + 时间戳）→ 本节点：
#   里程域接续融合（前方 0.6m 替换、车后历史保留）→ 三次 B 样条 → 按时间 t 查询参考状态
#   [x_ref, y_ref, θ_ref, v_ref] → LQR 出期望前轮转角 + PID 出期望车身速度 → /car_cmd。
# 自转模式不跟踪路基（进入瞬间清空滑窗），按 /rda/spin_target 转到目标角后回阿克曼。
traj_tracker_node:
  ros__parameters:
    # ---- 车辆几何 / 执行限幅 ----
    wheelbase: 0.69                # 轴距 m（BITCar.h L40 Length=0.69）
    max_steer_deg: 30.0            # 前轮最大转角 deg（BITCar.cpp max_angle=±30°）
    v_max: 0.8                     # 期望车身速度上限 m/s（与 bridge v_max 对齐）
    speed_eps: 0.05                # |v|<此 → 转角给 0（防低速打角发散）
    # ---- 里程域接续融合（滑窗）----
    fusion_len: 0.6                # 新规划替换区间 m（自车当前位置起前方 0.6m）
    keep_behind: 1.0               # 车身后方保留历史 m（给 B 样条连续支撑）
    window_len: 3.5                # 车前方窗口上限 m（>2.5m 路基，留余量）
    tail_join_tol: 0.35            # 远端历史与新段端点连续性容差 m（超差则按新规划重建）
    min_pts: 4.0                   # 有效窗口最少点数
    # ---- B 样条 ----
    bspline_ds: 0.10               # 等弧长重采样步长 m（控制点间距）
    bspline_min_pts: 6.0           # 少于该点数退回线性插值
    # ---- 阿克曼 LQR（s 域误差模型 e=[e_y, e_θ]，u=δ-δ_ff；增益与速度无关，初始化算一次）----
    lqr_ds: 0.10                   # 误差模型离散步长 m
    lqr_q_y: 12.0                  # 横向偏差权重（1/σy²）
    lqr_q_th: 3.0                  # 航向偏差权重（1/σθ²）
    lqr_r: 1.0                     # 转角增量控制权重
    preview_time: 0.20             # 参考查询时间前视 s（t_q = now + 此）
    max_steer_rate_deg: 45.0       # 前轮转角速率上限 deg/s（BITCar turn_limspeed）
    # ---- 纵向 PID（v_ref 前馈 + 误差反馈）----
    pid_kp: 1.2
    pid_ki: 0.4
    pid_kd: 0.0
    pid_i_limit: 0.4               # 积分限幅 m/s
    acc_max: 1.5                   # 加速度上限 m/s²（= ack_a_max）
    dec_max: 1.5                   # 减速度上限 m/s²
    v_meas_gain: 0.3               # 位姿差分测速低通系数
    # ---- 自转（SPIN；目标角 = /rda/spin_target）----
    spin_w_max: 1.0                # 自转转速上限 rad/s
    rotate_precision_deg: 4.0      # ±4° 死区（到位判据）
    rotate_decel_deg: 25.0         # ±25° 内转速线性减速
    spin_wait_time: 3.0            # 进入自转后底盘模式切换空窗 s（车静止）
                                   # 待标定：执行器 130°行程/(40°/s)=3.25s，实测超 3.0s 时上调 3.5~4.0
    spin_stop_hold: 0.5            # 到位后停转保持 s
    spin_timeout: 6.0              # 自转执行超时 s（从 RUN 起算）
    # ---- 安全 ----
    cmd_timeout: 2.0               # /cmd_vel 超时急停 s（规划链路看门狗）
    traj_timeout: 2.0              # 无新路基超时停车 s
    ref_window_stale: 3.0          # 参考窗口陈旧阈值 s（诊断用）
    body_to_base_link_x: -0.15     # 与 bridge_node 同一补偿，坐标系两侧一致
    rate: 50.0                     # 控制频率 Hz
    # ---- 话题 ----
    traj_topic: "/rda/traj"                 # 前方 2.5m 路基（点含 x/y/θ/v + 时间戳）
    traj_clear_topic: "/rda/traj_clear"     # 清空跟踪路径（模式切换/trap 激活退出）
    traj_mode_topic: "/rda/traj_mode"       # 0=阿克曼跟踪 1=自转
    spin_target_topic: "/rda/spin_target"   # 自转目标角 rad
    cmd_vel_topic: "/cmd_vel"
    localization_topic: "/localization"
    car_cmd_topic: "/car_cmd"
    status_topic: "/rda/ctrl_status"
    bridge_state_topic: "/car_bridge_state"
'''

raw = io.open(PATH, encoding='utf-8').read()
lines = raw.split('\n')
# 定位段起点（traj_tracker_node 上方的注释块首行）与段末（bridge_state_topic 行，属于该段）
i_start = None
for i, ln in enumerate(lines):
    if ln.startswith('# ---------------- 轨迹跟踪控制'):
        i_start = i
        break
assert i_start is not None, 'traj_tracker 段注释未找到'
i_end = None
for j in range(i_start, len(lines)):
    if lines[j].startswith('    bridge_state_topic: "/car_bridge_state"'):
        i_end = j + 1
        break
assert i_end is not None, '段末 bridge_state_topic 未找到'
old_block = '\n'.join(lines[i_start:i_end])
assert 'traj_tracker_node' in old_block and 'lookahead_short' in old_block, '区间不匹配'
new_lines = lines[:i_start] + NEW.rstrip('\n').split('\n') + lines[i_end:]
io.open(PATH, 'w', encoding='utf-8', newline='\n').write('\n'.join(new_lines))
print('replaced lines', i_start + 1, '..', i_end)
