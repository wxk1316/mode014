"""
params.py — Python 移植版参数结构（一一对应 MATLAB params.m）。
RDA-DWA 改进 DWA 局部路径规划算法（Actuators 2025, 14, 207 框架下的扩展实现）。

移植规则：所有参数名/取值与 params.m 完全一致；算法逻辑与参数禁止改动，
只做语言/平台迁移。
"""
from __future__ import annotations

import math


class Params:
    """params() 的 Python 等价：普通属性容器（无 MATLAB struct 语法，用类字段）。"""

    def __init__(self):
        # ---------------- 运行控制 ----------------
        self.case_id = 'II'                 # 'I-1' / 'I-2' / 'II'（兼容旧 Case 结构）
        self.planner_mode = 'both'          # 'conventional' / 'enhanced' / 'both'
        self.n_trials = 20                  # Case II 重复次数
        self.trial_index = 0                # 试验序号（0-based）
        self.random_seed = 2025             # 随机种子

        # ---------------- 机器人（Table 1） ----------------
        self.robot_radius = 0.1             # m
        self.max_linear_velocity = 1.5      # m/s
        self.min_linear_velocity = 0.0      # m/s
        # ★第五轮 改动1：DWA 正常 ω 采样窗上限 1.0→0.7（与 yaml omega_max 对齐）。
        #   近停原地自转走独立 omega_spin_max=0.8 窗口（enhanced_dwa L548），不受本值影响。
        self.max_angular_velocity = 0.7     # rad/s
        self.min_angular_velocity = -1.0    # rad/s
        # 4WIS 原地转向（v≈0 时允许的最大角速度，用户确认 0.8 rad/s）
        self.omega_spin_max = 0.8           # rad/s
        self.spin_v_eps = 0.05              # v 低于此值视为近似静止（允许原地转向）
        # 4WIS 后退脱困（真实车双向驱动；仅在旋转净空不足的贴墙死锁时启用）
        # ★第四轮 改动3：实车不支持负线速度，禁用后退机动（backup_v 倒车段不再输出）。
        #   突围改由正向/原地自转（SPIN）完成；末道闸在 vehicle_bridge 钳 v≥0 兜底。
        self.backup_enabled = False         # 是否允许后退机动（不进正常窗口，仅兜底）
        self.backup_v = 0.4                 # 后退速度 (m/s)
        # [v16_kinematic] Ackermann 多点转向 (3-point turn) 逃逸原语
        # Reeds-Shepp 式摆尾：forward-max-lock → reverse-opposite-lock → forward
        # 用于 Ackermann 在 DWA 全不可行且 trap 激活时执行 U 型陷阱掉头
        self.enable_mpt_escape = False      # 默认关闭（动态场景倒车撞障碍）
        self.mpt_velocity = 0.5             # 多点转向速度 (m/s)
        self.mpt_fwd_time = 1.8             # 前进满舵阶段时长 (s)
        self.mpt_rev_time = 1.5             # 倒车反舵阶段时长 (s)
        self.mpt_heading_tol = 0.8          # 退出机动的航向容差 (rad，放宽避免过早退出)
        # ① 动态障碍原地转向守卫（实验配置启用，基线为 0=不启用）：
        # 最近动态障碍（speed>0）距离 < spin_dynamic_guard 时禁用原地旋转兜底，
        # 避免"车不动、动态障碍逼近"导致的碰撞（S7/S9/S13 高 spin 失败现象）
        self.spin_dynamic_guard = 0.0       # m；0 = 关闭守卫
        # v14_spinfree 实验开关（默认 False，v13_c12 行为完全不变）：
        # C12 对向威胁存续的主动横向逃逸中（trap.active 且 headon_threat 存续），
        # 豁免 spin_timeout 强制停车，避免逃逸中途被 4s 原地转向超时打断。
        self.spin_timeout_evade_exempt = False

        # ---------------- 障碍物（Table 1 / §2.2） ----------------
        self.obstacle_radius = 0.15         # m
        self.obstacle_max_speed = 1.05      # m/s = 机器人最大速度的 70%
        self.obstacle_speed = 1.05          # m/s

        # ---------------- 采样 / 预测 ----------------
        self.dt = 0.1                       # s
        self.v_resolution = 0.05            # m/s
        self.omega_resolution = 0.1         # rad/s
        self.sim_time = 3.0                 # s 轨迹前瞻时长（★2026-11-04 改造 1.5→3.0；fixed 模式）
        # /rda/traj 发布用积分参数（2026-11-03 第二轮：从 bridge_node 硬编码参数化到
        # real_vehicle.yaml [rda_dwa_real_bridge] traj_dt/traj_v_eps；此处仅作口径注释）：
        #   traj_dt=0.05s 积分步长（sim_time/dt=60 点；★2026-11-04 sim_time 1.5→3.0）；
        #   traj_v_eps=0.05m/s 静止阈值
        #   （|v|<v_eps → 路径点全为当前位姿，轨迹长度≈0，供 traj_tracker 双分支判据消费）。
        self.n_trajectory_samples = 15      # 轨迹采样点数
        self.d_safe = 0.375                 # m = Vmax/4
        self.goal_tolerance = 0.2           # m
        self.max_sim_time = 200.0           # s

        # ---------------- 代价函数权重 ----------------
        self.alpha_h = 0.5
        self.alpha_d = 0.5
        self.alpha_c = 0.5
        self.alpha_f = 0.5
        self.alpha_v = 0.5                  # 传统 DWA 速度项权重
        self.alpha_count = 1.0
        self.beta = [0.05, 0.1, 0.2, 0.4, 0.8]
        # [代价平滑 + 死区粘性]（2026-11-03 第二轮，实车指令抖动抑制）：
        #   alpha_smooth 第 9 项代价权重：惩罚本帧候选速度相对上一周期最终指令
        #   的跳变 t.smooth = -((v-v_last)² + (ω-ω_last)²)；trap 激活 / 自转/停车期
        #   （|v|<spin_v_eps）自动门控为 0，不干扰逃逸与原地转向。
        #   sticky_eps 死区粘性阈值（归一化加权和 score 空间）：最优样本与上周期指令
        #   匹配样本的代价差小于该值时沿用上周期指令，抑制边界抖动；每周期至多粘一次，
        #   trap/自转/停车期间不生效。运行时可由 real_vehicle.yaml smooth_weight/sticky_eps 覆盖。
        self.alpha_smooth = 0.3
        self.sticky_eps = 0.05

        # ---------------- 航向/距离代价预测时域（★2026-11-06 蛇形修复） ----------------
        # t.head / t.dist 的预测时域（s）。原实现固定用单步 dt=0.1s，等价于航向 P 增益
        # =1/dt=10 rad/s per rad：任何 >0.07rad 的航向误差都会把 |ω| 顶到采样窗/δ 上限，
        # 配合 δ 速率限制(45°/s)与 0.1s 周期形成 ±δ_max 极限环（实车 ±30° 打满蛇形）。
        # 取 0.4s → 隐含增益降到 2.5，δ 不再每帧打满；0.0 表示保持旧行为（单步 dt）。
        self.cost_horizon = 0.4

        # ---------------- 局部绕行子目标（★2026-11-06 绕障修复） ----------------
        # 根因：DWA 的可行域只覆盖 v_max*sim_time ≈ 0.8*3 = 2.4m 的前向走廊，而阿克曼
        # 绕开一个 0.4m 路面障碍需要在 5~6m 外就开始横向让出 0.8m（R_min=1.195m，
        # 曲率上限 0.837/m）。于是"看到障碍时已来不及绕"：t.clear 在 6m 处被
        # normalize_trunc_D 截断成常数、t.risk≈0，代价里只剩 head/dist 在起作用，
        # 而两者都奖励"直冲目标"，任何转向都被 head 项判为更差 → 车只会直行到贴脸，
        # 全部样本不可行 → M6 减速停车（实车表现：不转弯、原地停住）。
        # 对策：前方走廊被挡时，把 local_goal 侧向偏移出一个绕行点，让 head/dist
        # 主动把车"引"向障碍侧面（纯目标选择，不动 DWA 的可行域与权重）。
        self.enable_detour = True
        self.detour_look_dist = 6.5      # 该距离内、且挡住走廊的障碍才触发绕行
        self.detour_min_along = 2.0      # 障碍纵向距离小于此值 → 来不及绕，交停车/自转
        self.detour_clear_margin = 0.20  # 侧向净空附加余量（在 clearance 半径之上再加）
        self.detour_pass_ahead = 0.80    # 绕行点取"越过障碍"的纵向距离
        self.detour_max_lateral = 2.5    # 侧向偏移上限（避免把局部目标甩太远）

        # ---------------- 朝向/风险口径 ----------------
        self.use_predicted_heading = True
        self.risk_use_current_distance = True

        # ---------------- RDA-DWA 改进模块 M1~M6 ----------------
        self.enable_m1_dynamic_window = True
        self.enable_m2_multistep = True
        self.enable_m3_gaussian_risk = True
        self.enable_m4_normalize = True
        self.enable_m5_filter = True
        self.enable_m6_fallback = True

        # M1 加速度限制
        # ★第五轮 改动2：acc_lin_max 1.0→1.5（与 yaml a_max/ack_a_max 对齐，DWA 动态窗 v 扩张率）
        self.acc_lin_max = 1.5
        self.dec_lin_max = 1.5
        self.acc_ang_max = 1.0
        self.dec_ang_max = 1.5

        # M2 多步时距
        self.n_future_steps = 5
        self.headway_time = 0.3
        # ② M1/M2 4WIS 重调覆盖值（实验配置启用，基线保持 MATLAB 原值）：
        #   M1 角加速度 1.0→3.0 rad/s²：4WIS 原地转向 ω_spin_max=0.8 可在 ~0.3s 到位，
        #     原 Ackermann 级 1.0 需 1s，动态避障反应过慢（S6 ALL 30 全撞现象之一）
        #   M2 headway 0.3→0.15s：接近速度放大半径减半，
        #     缓解"原地旋转中迎面障碍 v_rel 虚高 → 机动被全判死"（S7/S9/S13）
        self.m12_acc_ang_4wis = 3.0
        self.m12_dec_ang_4wis = 3.0
        self.m12_headway_4wis = 0.15

        # M3 高斯风险场
        self.risk_amp = 1.0
        self.risk_sigma_x0 = 0.5
        self.risk_sigma_y0 = 0.3
        self.risk_kappa = 0.5
        self.risk_tau_s = 0.5
        self.alpha_r = 0.5

        # M4 归一化
        self.normalize_epsilon = 1e-6
        self.normalize_trunc_D = 0.0

        # M5 前置 α-β 滤波
        self.obs_filter_alpha = 0.6
        self.obs_filter_beta = 0.6

        # ---------------- 创新点 C1~C4 ----------------
        # C1 截断归一化
        self.enable_c1_trunc_norm = False

        # C2 IMM-UKF
        self.enable_m7_imm_ukf = False
        self.imm_transition = [
            [0.95, 0.025, 0.025],
            [0.025, 0.95, 0.025],
            [0.025, 0.025, 0.95],
        ]
        self.imm_init_prob = [0.6, 0.2, 0.2]
        self.imm_meas_noise = 0.02
        self.imm_q_p = 1e-4
        self.imm_q_v = 0.30
        self.imm_q_a = 1.50
        self.imm_q_w = 1.20

        # C3 前瞻 + U 形逃逸
        self.enable_c3_lookahead = False
        self.lookahead_steps = 3
        self.lookahead_gamma = 0.9
        self.enable_c3_trap_escape = False
        self.trap_stall_steps = 15          # [统计保留] 帧计数（兼容；触发已改用时间 trap_stall_time）
        # 0.01m/帧：实车 20Hz 低速标定（<0.2m/s 判卡住）；仿真 0.05 在实车 0.4m/s@20Hz 会误触发。
        # ★2026-11-04 整定：规划周期 5~7Hz（墙钟 dt 漂移 0.1~0.9s），原"0.01m/帧"随 dt 漂移
        #   （dt=0.1→0.1m/s；dt=0.9→0.011m/s，判据差 10 倍）。改速度判据 disp < trap_stall_speed×dt。
        self.trap_progress_eps = 0.01      # [标称参考] 仅在 dt≈trap_nominal_dt 时对应 0.01m/帧
        self.trap_nominal_dt = 0.15        # ★标称规划周期 s（≈6.7Hz）；disp 阈值按 dt/此值归一
        self.trap_stall_speed = 0.05        # ★stall 速度判据 m/s：disp/dt < 此值视为未移动（0.4m/s 的 1/8）
        self.trap_stall_time = 1.5          # ★连续真停时长 s 才触发 trap（原 15 帧在 dt=0.15≈2.25s，
                                            #   dt=0.9→13.5s 过长；改固定时间 1.5s 与 dt 解耦）
        # 朝向变化阈值（rad）：stall 窗口内朝向变化超过此值视为原地转向（自转/模式切换），
        # 不计被困。★第五轮 改动3：0.3→0.533。推导：自转下发角速度 omega_spin_max=0.8 rad/s
        #   × 2/3 ≈ 0.533 rad(≈30.6°)，语义=窗口内累计朝向变化（非每帧角速度）。
        #   阈值 0.533 / stall 窗 1.5s ⇒ 仅 ω>≈0.36 rad/s 的持续转向才在窗内被识别为"在转"
        #   清零 stall；更慢摆尾仍计入被困——修正原 0.3 对慢摆尾过敏误清 stall 的问题。
        self.trap_heading_eps = 0.533
        # freeze 语义（2026-11-03 第二轮固化）：freeze_stall=True（bridge_state∈{1:HANGUP,
        # 2:SPIN_RUN}）期间 stall/stall_run/stall_theta_ref 清零，几何 detect_trap 通道同帧
        # 跳过调用与 trap 激活赋值；解除冻结后从 0 重新累加（无残留）。trap 已 active 时
        # 逃逸推进（trap_steps/sub_goal）不冻结，保持 trap_max_steps 超时兜底有效。
        self.trap_exit_progress = 0.15
        self.trap_max_steps = 120
        # ★2026-11-05：trap 兜底改时间制 16s。原 trap_max_steps=120 按 bridge 规划
        # 10Hz 折算=12s 偏短；vehicle_bridge 在 trap 激活时刻起算，超过此时长强制退出，
        # 不受规划频率漂移（5~15Hz）影响；trap_max_steps=120 保留为 trap_ctrl 内部二层保险。
        self.trap_max_duration = 16.0
        self.trap_min_steps = 10
        self.trap_block_angle = math.pi / 3.0
        self.trap_block_range = 2.5
        self.trap_min_obs = 3
        self.trap_scan_dirs = 36
        self.trap_goal_bias = 0.5
        self.trap_scan_range = 6.0
        self.trap_subgoal_dist = 2.0
        self.trap_subgoal_frac = 0.8
        self.trap_v_weight = 0.3
        # [MTDWA P1] 近目标逃逸循环修复：
        #   trap_near_goal_m            —— 距目标小于该距离时关闭几何 trap 检测（原硬编码 1.0m）
        #   trap_near_goal_constraint_m —— 距目标小于该距离时，逃逸子目标方向被硬约束
        #   trap_near_goal_ang          —— 近目标段允许的逃逸方向与目标方向最大夹角（rad）
        self.trap_near_goal_m = 1.0
        self.trap_near_goal_constraint_m = 1.5
        self.trap_near_goal_ang = math.pi / 3.0
        # ★P0-4b (U5)：stall 触发路径近目标闸门。距目标小于此 m 时跳过 stall 触发
        #   （detect_trap 几何通道自带 trap_near_goal_m=1.0 闸门；stall 路径单独取 1.5m 留裕度）。
        self.trap_near_goal_stall_m = 1.5

        # ---------------- planner 侧模式仲裁（2026-11-04 改造） ----------------
        # 模式仲裁上收 planner：tracker 由 /rda/traj_mode 命令触发自转。
        # 与 real_vehicle.yaml [rda_dwa_real_bridge] 数值一一对应；yaml 为参数唯一权威。
        self.v_streak_threshold = 0.05        # V 计数阈值 m/s（|v| 低于此视为静止 streak 累计）
        self.v_streak_count = 25              # 连续帧数 ≈0.5s @50Hz
        self.ack_guard_time = 2.0             # ACK 态持续下限 s（防自转刚切回误判再自转）
        self.spin_reset_progress = 0.5        # 距上次自转锚点位移 m，超过则允许再自转
        self.trap_exit_dist = 0.3             # 到达子目标距离 m
        self.trap_exit_goal_ang_deg = 20.0    # 车头朝向与"车位置→全局目标"map 系连线夹角 deg
        self.trap_exit_sector_deg = 60.0      # 车头±60° 扇区
        self.trap_exit_sector_range = 2.5     # 扇区范围 m
        self.trap_exit_sector_max_obs = 3     # 扇区内障碍物计数上限

        # C4 APF ω 窗口缩窄
        self.enable_c4_apf_omega = False
        self.apf_k_att = 1.0
        self.apf_k_rep = 0.6
        self.apf_d0 = 1.5
        self.apf_lateral_threshold = 0.05
        self.apf_omega_band = 0.05
        self.apf_straight_band = 0.15
        self.apf_narrow_dist = 1.5

        # ---------------- 新增改进模块 C5~C9（默认关闭，不改变 V4 行为） ----------------
        # C5 上下文感知逃逸（P3）：区分"等待让行"与"被静态几何困住"。
        #   当动态障碍的直线路径在 c5_wait_horizon 内接近自车当前位置，且其运动
        #   不是"同车道对向逼近"（对向角 < c5_headon_ang 或横向偏移 >= c5_lateral_off），
        #   判定为合法让行等待 → 抑制 C3 陷阱逃逸触发。
        #   参数标定依据：S11 行人横穿（对向角≈90°，与自车航向垂直）；
        #                 S13 对向车流在相邻车道（横向偏移≈1.0m ≥ 0.8m）；
        #                 S9 反弹车同车道对向（偏移≈0、对向角≈180°）→ 不抑制（需横向避让）。
        self.enable_c5_wait_aware_escape = False
        self.c5_wait_horizon = 8.0          # s：预测障碍直线路径接近自车的检查时距
        # （8s 覆盖 C12 ttc_max=3s × 相对接近速度 2.55m/s ≈ 7.7m 的对向车流前向窗口，
        #  使 S13 相邻车道对向车流在 C12 触发前即被判定为"合法等待"）
        self.c5_wait_threat_dist = 1.1      # m：路径接近自车当前位置的危险距离
        # [MTDWA P3c] C5 抑制范围：True 时 C5 仅抑制 stall 触发的逃逸，
        # detect_trap 静态陷阱 / C12 对向 / 行人横穿仍可触发（默认 False = 原全域抑制）
        self.enable_c5_stall_only = False
        self.c5_headon_ang = 2.2            # rad≈126°：障碍运动方向与自车航向夹角超过此值视为对向
        self.c5_lateral_off = 0.8           # m：障碍路径对自车位置的横向偏移 >= 此值视为相邻车道通过

        # C6 逃逸方向动态重校验（P3）：逃逸期间周期性地用更长时距（2s）检查冻结的
        #   逃逸方向对动态障碍预测位置的净空；不安全则基于当前障碍状态重新生成
        #   逃逸子目标/方向（解决 S7 逃逸方向冻结过期）。
        #   参数：c6_escape_horizon=2.0s 为 M2 视界（0.5s）的 4 倍；速度取 0.6m/s（低速滚动
        #   保守估计）；净空 0.5m = 自车半宽 0.25 + 障碍半宽 0.15 + 0.1 余量
        #   （旧值 0.35 使"向东撞向对向车"的 2s 滚动净空 0.36 被判安全 → S9 逃逸方向过期）。
        self.enable_c6_escape_recheck = False
        self.c6_escape_horizon = 2.0        # s
        self.c6_escape_speed = 0.6          # m/s
        self.c6_escape_clearance = 0.5      # m
        self.c6_recheck_every = 5           # 步：逃逸激活期间每 N 步重校验一次
        # C6 IFGM 效用评分（第8篇 IFGM-DWA）：U = m1*d_gap + m2*(pi-|delta|)，d_gap=间隙
        # 长度（free distance），delta=间隙中心相对目标方向偏角。替代纯"最大间隙"评分，
        # 避免逃逸时选"远离目标的超长直道"（S9 掉头向西根因；第8篇创新点2）。
        self.c6_ifgm_gap_w = 0.35           # m1：间隙长度权重
        self.c6_ifgm_goal_w = 1.0           # m2：目标偏向权重（rad 尺度）
        # C12 横向避让距离：对向威胁路径线的垂直偏移量（自车半宽 0.25 + 障碍半宽
        # 0.15 + 0.7 余量，把车拉出对向车道线）；走廊半宽约 1.5m 内可实现。
        self.c12_evade_offset = 1.1         # m
        self.c12_evade_forward = 0.4        # m：避让子目标保留少量前进分量，避免原地转圈

        # C7 采样饿死软松弛（P4）：正常窗口零可行样本（全判死→停车等死）时，
        #   以放宽后的碰撞/制动阈值重建窗口，选出"最不坏"样本并施加重惩罚，
        #   避免落到 M6 停车被动态障碍撞上（借鉴 ISA-MPC slack ρ / 软约束思想）。
        #   参数：relax_ratio=0.5 将 M2 危险半径减半；vel_relax=1.15 允许 15% 超制动距离；
        #   penalty=2.0 大于归一化总分权重和（≈3.0 的 2/3），保证仅在无其它可行样本时被选中。
        self.enable_c7_soft_relax = False
        self.c7_relax_ratio = 0.5           # M2 危险半径缩放
        self.c7_vel_relax = 1.15            # M1 制动距离判据缩放
        self.c7_relax_penalty = 2.0         # 松弛样本得分惩罚

        # C8 全局引导（P7）：基于静态障碍地图的 A* 路径 + 子目标序列（A*/D* 引导线思想）。
        #   参数：栅格 0.25m；膨胀 0.45m = 车宽半宽 0.3 + 障碍半径 0.15（引导用，非硬约束）；
        #   lookahead 1.5m ≈ 1s@1.5m/s 的前瞻。
        self.enable_c8_global_guide = False
        self.c8_grid_res = 0.25             # m
        self.c8_inflate = 0.45              # m
        self.c8_lookahead_dist = 1.5        # m

        # C9 预测残差驱动的障碍不确定性膨胀（P1）：预测残差超阈值（方向突变/反弹/换向）
        #   时对该障碍临时膨胀有效半径，使 M2/M1 更早判死前进样本 → 提前制动/规避。
        #   参数：残差阈值 0.15m < 反弹一步位移（2×0.105≈0.21m）且 > 正常恒速残差（≈0）；
        #   inflate=0.5m 为对"行为不可预测"障碍的保守安全裕量（GMM σ 膨胀思想）；
        #   hold=8 步（0.8s）持续，若残差持续超阈值则每步重新触发。
        #   航向跳变阈值 0.8 rad/步：反弹车掉头（Δθ≈π）与 S8/S12 换向可检出；
        #   正常缓转向（如 S9 切入车 0.3 rad 渐变）≈0.03 rad/步，不误触。
        self.enable_c9_uncertainty_inflate = False
        self.c9_resid_thresh = 0.15         # m
        self.c9_inflate_radius = 0.5        # m
        self.c9_hold_steps = 25             # 步（2.5s，覆盖对向逼近的关键接近阶段）
        self.c9_heading_jump = 0.8          # rad/步

        # C12 对向威胁主动横向逃逸（P1/P3，S9）：同车道对向逼近（TTC<3s、横向偏移<0.5m、
        #   运动方向与自车航向近似对向）时，不等 stall 15 步，立即触发 C3 横向逃逸
        #   （反弹车掉头后 0.5s 视界看不到，需以相对速度 TTC 提前判威胁）。
        #   横穿（S11 行人）/相邻车道（S13 对向车流）不触发（与 C5 的"合法等待"判定一致）。
        self.enable_c12_headon_evade = False
        self.c12_ttc_max = 3.0              # s：相对速度最近点时间阈值
        self.c12_cpa_max = 0.5              # m：相对速度最近点距离阈值（≈自车半宽0.25+障碍半宽0.15+余量）
        self.c12_closing_min = 0.4          # m/s（兼容保留，CPA 路径不再使用）
        self.c12_headon_ang = 2.2           # rad≈126°
        # v14_s9lock 实验开关（默认 False，v13_c12 行为不变）：
        # 仅对"已由 C12 对向威胁激活的逃逸"在 persist（存续保持）阶段跳过角度门，
        # 防止逃逸中 robot.theta 旋转 ~60° 后 rel_ang 跌破 c12_headon_ang 被误释放
        # （S9 慢车掉头后中途退出逃逸 → 对向车道停车被撞根因）。初始触发仍保留角度门以保护 S10。
        self.c12_persist_skip_angle = False
        # v14_s9band 双频带触发（默认 0.0=禁用追尾带，v13_c12 行为不变）：
        # rel_ang > c12_headon_ang（对向带，现状）∪ rel_ang < c12_rear_ang（同向追近带），
        # 排除 90° 横穿带（0.6~2.2 不触发）。S9 同向慢车追尾接近阶段提前触发给逃逸余量；
        # S10 横穿车 rel_ang≈1.57 落在禁带 → 不误触发。
        self.c12_rear_ang = 0.0
        # v14_s9move 实验开关（默认 False）：C12 激活的逃逸中禁用 4WIS 原地 spin 捷径，
        # 改为带前进速度的转向弧（spin 捷径在对向逃逸里让车原地转 → 横向位移不足被撞）。
        self.c12_evade_nospin = False
        # v15_sigmoid（默认 False，v21_s9band_c11 行为不变）：P4 危险度调制逃逸横移量。
        # 边缘威胁（cpa 接近 c12_cpa_max）用小横移（c12_offset_min≈0.3m）不挤邻道（治 S8 横移挤压），
        # imminent 威胁（cpa→0）回退到 c12_evade_offset=1.1m 兜底护 S7。
        self.c12_offset_sigmoid = False
        self.c12_offset_min = 0.3
        self.c12_offset_kappa = 4.0
        # v15_latesc（默认 False）：对向带逃逸强制垂直横移（robot.heading ± π/2），
        # 不再取 free_distance 的自由方向（治 S9 free_distance 选向西长直道→原地掉头横移不足）。
        # 仅对向带（rel_ang>c12_headon_ang）生效；追尾带/stall/detect_trap 逃逸保持现状。
        self.c12_escape_perp = False
        # v15_latesc_rear（默认 False，v15_latesc 行为不变）：把垂直横移逃逸扩展到
        # 追尾带触发（rel_ang<c12_rear_ang）。S9 胜种子靠追尾带触发但 free_distance 选 U-turn 向西
        # → 原地掉头横移不足被撞；本开关让追尾带逃逸也改垂直横移。对向带/stall/detect_trap 现状不动。
        self.c12_escape_perp_rear = False

        # ---------------- [MTDWA 融合] g3：同向横向接近意图触发（默认关） ----------------
        # S9 同速切入车：与自车同速 → CPA vrel≈0 时 t_cpa 无定义 → C12 headon_threat
        # 永不触发。本开关新增"横向合拢+TLC"几何谓词：同向（|Δθ|<s9_lateral_same_ang）
        # 障碍在横向带（|lat|<s9_lateral_lat_max）内朝自车路径合拢（lat·lat_v<0）且
        # 到车道线时间 TLC=|lat|/|lat_v|<s9_lateral_ttc → 判定切入意图威胁（论文卡
        # 09/20/49：意图=横向接近速率×到车道线距离）。触发后走 c12_escape_perp
        # 垂直横移逃逸。默认 False → mtdwa 行为不变。
        self.enable_s9_lateral = False
        self.s9_lateral_lat_max = 1.3    # m：横向带（自车半宽0.25+障碍半宽0.15+车道余量0.9）
        self.s9_lateral_same_ang = 0.6   # rad≈34.4°：同向带（S10/S11 横穿 |Δθ|≈π/2 排除）
        self.s9_lateral_ttc = 3.5        # s：TLC 上限（S9 切入反弹后 1.0m/0.31m/s≈3.2s 可触发）

        # ---------------- [MTDWA 融合] g4：对向流间隙感知 persist 扩展（默认关） ----------------
        # S13 对向车流：C12 persist 在车流间隙释放→回归车道中心→下一辆对向车到→撞。
        # persist 期间若"相邻车道对向流阻塞"（|Δθ|>c12_headon_ang 且横向带
        # [s13_gap_lat_min, s13_gap_lat_max] 内且沿程<s13_gap_dist）→ 维持逃逸横向偏移；
        # 间隙开放（窗口内无对向阻塞）才允许正常释放回归（论文卡 07/01/11：
        # 间隙接受 Gap Acceptance / 安全距离 / 主从博弈时序）。默认 False → mtdwa 不变。
        self.enable_s13_gap = False
        self.s13_gap_lat_min = 0.6       # m：横向带下界（排除穿越自车车道 lat<0.6 的横穿车）
        self.s13_gap_lat_max = 1.5       # m：横向带上界（排除远车道/远处障碍）
        self.s13_gap_dist = 8.0          # m：沿程视距（对向接近早期判定窗口）

        # ---------------- [MTDWA 第二轮] g5：LC-RSS 纵向安全距离限速（默认关） ----------------
        # S9 终点区反弹车迎头 + cpa 边缘失判（cpa=0.501≥0.5，1mm 漏过）：C12 是"最近点
        # 几何点值"判据，无 worst-case 时序安全距离约束。本开关用卡 33（LC-RSS 博士论文
        # 式5.2）的纵向安全距离不等式：同车道（|lat|<rss_lat_max）对向/追尾动态障碍，
        # 实际距离 d < ρ·v_rel + v_rel²/(2·a_brake) + dlon → 判定不可安全保持 → vmax
        # 受限到 v_safe=√(2·a_brake·(d−dlon))（proper response 减速，时间换空间，不等
        # C12 几何触发）。区别于已证伪的 cpa 阈值放宽（点值微调）：判据层（worst-case
        # 安全距离不等式+速度窗口约束）不同，动作层（限速）与 C12（横移）互补不互斥。
        self.enable_rss_speed_limit = False
        self.rss_lat_max = 0.5           # m：同车道横向带（自车半宽0.25+障碍半宽0.15+余量0.1）
        self.rss_rho = 0.5               # s：反应+决策时间（=5 控制周期）
        self.rss_dlon = 0.2              # m：最小间距（=goal_tolerance）
        self.rss_a_brake = 1.5           # m/s²：最小减速度（=M1 dec_lin_max）

        # ---------------- [MTDWA 第二轮] g6：让行/超越决策状态机（默认关） ----------------
        # S13 无超车决策层：逃逸-回归被动博弈（C12 persist 永不释放）。本开关用卡 67
        # （切入非合作动态博弈 Eq.2.1-2.2，Nash 均衡=(让行,切入)/(加速,让行)）的离散
        # 决策状态机：检测"前方慢车+对向流"→ F(跟随)/O(超车)/M(回归) 三态；O 仅在
        # Gap Acceptance 窗口（对向车到达时间 > 超车完成时间 + ovk_t_safe）时进入；
        # M 由状态机显式触发（超车完成），替代 C12 persist 释放判据。区别于已证伪的
        # P2（子目标内容）/g4（persist 释放判据微调）：本候选是显式决策状态机。
        self.enable_overtake_sm = False
        self.ovk_detect_dist = 3.0       # m：慢车检测距离（前方）
        self.ovk_slow_ratio = 0.7        # 慢车判据：v_slow < 0.7*v_max
        self.ovk_safe_gap = 2.0          # s：对向车到达时间须大于超车完成+该裕量
        self.ovk_t_safe = 1.5            # s：超车完成时间裕量
        self.ovk_lat_off = 0.8           # m：超车横向偏移（=c12_overtake_lateral）
        self.ovk_ahead = 2.0             # m：超车超前距离（=c12_overtake_ahead）

        # ---------------- [MTDWA 第二轮] g7：横向运动带刹车等待（默认关） ----------------
        # S11 行人 90° 横穿落 C12 禁带不触发 / S8 C12 提前横向逃逸破坏通道。本开关用
        # 卡 34（行为状态 DWA：横切夹角>45°→刹车等待让行）实现 §11.2 方向三"横向运动带
        # 专用低速让行"：障碍夹角 rel_ang∈[bw_band_min,bw_band_max]（横向带 45°~135°）
        # 且接近（距离<bw_dist）→ vmax≤bw_vmax 低速让行（不触发/不抑制逃逸状态机，只在
        # 采样层限速）。区别于已证伪 P3（行人分类器+让行子目标）/C5（全域抑制）/g3
        # （横向意图触发逃逸）：本候选是横向带专用 vmax 约束，障碍类别无关。
        self.enable_brake_wait = False
        self.bw_band_min = 0.785         # rad≈45°：横向带下限（排除对向/同向带）
        self.bw_band_max = 2.356         # rad≈135°：横向带上限
        self.bw_dist = 1.2               # m：触发距离
        self.bw_vmax = 0.4               # m/s：低速让行速度（=backup_v）

        # ---------------- [MTDWA 第二轮] g8：换向检测→恒角速度弧线外推（默认关） ----------------
        # S12 随机换向障碍：M2 恒速直线外推失真（0.5s 视界内折线≠弧线）。本开关用卡 8
        # （ISA-MPC 多模态预测：机动原语概率+名义轨迹模型）的简化实现：IMM 更新时检测
        # |Δθ/Δt|>arc_w_thresh（持续 arc_hold 步）→ 该障碍标记弧线模式（heading_rate
        # 写入）→ M2 多步外推切换为恒角速度弧线积分（θ(t)=θ0+ω·t 逐段累积），近距
        # （<arc_min_dist）才生效（ε 门控，借鉴 ε_k 保守度旋钮）。区别于已证伪 IMM 调参
        # （滤波矩阵）/g1 C9（判死膨胀半径）：本候选是预测模型切换（直线→弧线）。
        self.enable_maneuver_arc = False
        self.arc_w_thresh = 0.4          # rad/s：换向角速度阈值
        self.arc_hold = 5                # 步：弧线外推保持步数
        self.arc_min_dist = 1.5          # m：近距门控（障碍距自车<该值才弧线外推）

        # ---------------- [MTDWA 第三轮] g9：目标区对向 TTC 减速让行（默认关） ----------------
        # 第一轮报告 §8 方向一官方首选：S9 终点区"慢车反弹迎头 + cpa≈0.501≥0.5 边缘失判"
        # 时 C12 几何判据失效 → 本开关在接近目标（d_goal<gtz_dist）时，对前方对向
        # （rel_ang>c12_headon_ang）且 TTC<gtz_ttc 的动态障碍施加 vmax≤gtz_vmax 让行
        # （时间换空间，不等 C12 触发；无横向带门控——目标区任何侧偏对向车都让行）。
        # 机制层区别于：cpa 阈值放宽（几何点值，已证伪）、g5 RSS lat 门控（中心横向带
        # 0.5m 被 S9 威胁侧偏击穿）。论文依据：卡 01（安全距离 d≥v·t_react+v²/2a）、
        # 卡 07（让行语义）。仅在目标区生效（副作用局部化）。
        self.enable_goal_ttc = False
        self.gtz_dist = 1.5            # m：目标区半径（距目标小于此值才评估）
        self.gtz_ttc = 2.0             # s：对向接近时间阈值（along/v_rel 小于此值触发）
        self.gtz_vmax = 0.4            # m/s：目标区让行限速

        # ---------------- [MTDWA 第三轮] g10：高残差近距速度窗口保守收缩（默认关） ----------------
        # S12 随机换向障碍：IMM 恒速预测失真（残差超阈）→ M2 选中碰撞轨迹（18 撞）。
        # g1 C9 判死（半径膨胀→样本剔除）在 S10 堵塞交叉口通行（0/15 红线破坏）；本开关
        # 改为"速度窗口保守收缩"：近距（d<unc_d_gate）高残差（res>unc_r_thr）障碍 → vmax
        # 乘收缩因子（不判死样本、不剔除轨迹、vmax≥vmin 保底）→ 慢速通过不阻塞。
        # 机制层区别于 g1（判死→收缩）、g8（预测模型切换→规划层兜底）。论文依据：
        # 卡 8（ISA-MPC ε_k 保守度旋钮）、卡 24（GMM 不确定性→控制保守化）。
        self.enable_unc_vmax = False
        self.unc_r_thr = 0.15          # m：残差阈值（与 C9 c9_resid_thresh 同值）
        self.unc_k = 0.5               # 收缩强度（vmax 最大收缩 1-unc_k）
        self.unc_r_max = 0.5           # m：残差饱和值（r≥此值取最大收缩）
        self.unc_d_gate = 2.5          # m：近距门控（障碍距自车小于此值才收缩）

        # ---------------- [MTDWA 第四轮] g11：前方锥内迎面逼近风险代价项（默认关） ----------------
        # S9 终点区慢车反弹迎头：cpa=0.501 边缘失判 → C12 不触发、M2 选中碰撞轨迹。
        # 动作层=新增代价项（非 vmax、非阈值微调、非全局偏置）：对"轨迹方向前方锥内、
        # 迎面逼近"的障碍加风险惩罚 → 正面撞样本代价↑ → 选绕行/减速样本。
        # 与已证伪项区别：g9 TTC 限速被 vmin 保底中和（本开关走代价排序不受影响）；
        # P4/g3 已有项偏置被 min-max 吸收（本开关条件触发，非触发帧 max==min→pbar=0）。
        # 论文依据：卡 38（θ_r 相对速度方向风险：相向高/同向低）、卡 20（MTTC 相对速度
        # 差碰撞时间）、卡 12（概率碰撞时间分布）。公式见 07_理论筛选_第四轮.md §3。
        self.enable_headon_cost = False
        self.hc_cone_rad = 0.87        # rad（≈50°）：障碍方位相对轨迹方向的前方锥角门
        self.hc_along_thr = 0.15       # m/s：迎面沿向速度阈值（障碍速度沿轨迹方向分量 < -此值）
        self.hc_dist_gate = 3.0        # m：风险作用距离（距离权重线性衰减到 0）
        self.hc_weight = 0.8           # min-max 归一化权重（与 α_r=0.5 同量级）

        # ---------------- [MTDWA 第四轮] g12：近距高模型分歧惩罚项（默认关） ----------------
        # S12 随机换向障碍：M2 恒速预测失真选中碰撞轨迹（18 撞）。g10 用后验残差收缩
        # 幅值不足（S12 残差 0.15~0.2）；g12 改用 IMM 三模型（CV/CA/CT）1 步预测位置
        # 两两分歧（前瞻指标：换向瞬间 CT 与 CV 预测即分歧，不必等落空）。
        # 动作层=新增代价项（样本级保守化，不判死→区别于 g1 C9 S10 停滞；
        # 不走 vmax→不被 vmin 中和）。论文依据：卡 8（ε_k 保守度）、卡 12（概率加权风险）、
        # 卡 57（信念不确定性→保守化）。
        self.enable_imm_cost = False
        self.unc_d_gate4 = 3.0         # m：近距门控（障碍距自车小于此值才评估分歧）
        self.unc_div_thr = 0.5         # m：模型分歧阈值（1 步预测位置两两距离最大）
        self.unc_div_max = 2.0         # m：分歧饱和值（≥此值取最大惩罚）
        self.unc_weight4 = 0.8         # min-max 归一化权重

        # ---------------- [MTDWA 第五轮] g13：RA-DWA 风险自适应权重（默认关） ----------------
        # 保留 8 项代价结构不变，仅对前 6 项固定权重做"风险指标→权重"闭式在线映射
        # （卡 56 RA-DWA：增益调度、免训练、Lipschitz 连续）。区别于 g11 新增代价项
        # （被 min-max 吸收）：本候选只调已有项相对主导比例，不引入样本间无差异的新项。
        # 指标：RF=1-dF/Dmax（前向接近）、A=(dL-dR)/(dL+dR+ε)（左右不对称）、
        # RTTC=max(0,1-t_cpa/3)（逼近时间风险）。通道：Shead=1-RF、Sclear=RF+|A|、
        # Sspeed=1-RTTC；归一化后乘性缩放基线权重（band 限幅 ≤30%），一阶低通
        # （λ=2s⁻¹，dt=0.1s）抑制帧间抖动。band 小→开阔场景≈基线→红线行为近不变。
        self.enable_ra_weight = False
        self.ra_d_max = 8.0             # m：距离归一化尺度 Dmax
        self.ra_lp_lambda = 2.0         # 低通时间常数 λ（卡 56 原值 2 s⁻¹）
        self.ra_band = 0.3              # 权重自适应限幅带（最大 ±30% 相对基线）

        # ---------------- [MTDWA P2] 追尾带/超车专用逃逸（默认关） ----------------
        # S9 追尾带（同向慢车堵前）：垂直横移（perp_rear）被证伪（299/420），改用
        # "邻道超前"子目标：横向切到净空大的一侧 + 沿威胁 heading 超前 c12_overtake_ahead 米，
        # 利用 c12_persist 锁存直到超车完成。默认 False → v15_latesc 行为不变。
        self.enable_c12_overtake = False
        self.c12_overtake_lateral = 0.8     # m：旁侧横移量（进入邻道）
        self.c12_overtake_ahead = 2.0       # m：沿威胁 heading 的超前距离（子目标放在威胁前方）

        # ---------------- [MTDWA P3] 行人横穿分类器（默认关） ----------------
        # S11 行人横穿：速度慢且近横向运动 → 走"减速让行 + 小偏移"，不触发大横移/硬闯。
        # 参数窄带设计（speed≤1.2、接近角∈[ang_min, π-ang_min]、CPA 紧迫）尽量不伤 S10 路口横穿车。
        self.enable_ped_cross_evade = False
        self.ped_cross_speed_max = 1.2      # m/s：最大"行人"速度
        self.ped_cross_ang_min = 0.9        # rad≈51.6°：横向运动带下限（排除对向/同向带）
        self.ped_cross_ttc = 2.0            # s：相对速度最近点时间上限（紧迫才介入）
        self.ped_cross_cpa = 0.6            # m：相对速度最近点距离上限
        self.ped_cross_offset = 0.5         # m：让行小偏移量
        self.ped_cross_fwd = 0.3            # m：让行前进漂移（自然减速）

        # ---------------- [MTDWA P5] 自适应横移量（默认关） ----------------
        # 对向带垂直横移：不再固定 c12_evade_offset=1.1，而是 3 档 offset × 2 档 fwd
        # 生成候选子目标，用未来 c12_adapt_horizon 秒（直线滚动）的最小净空选优；
        # 净空相同取更小 offset（少挤邻道，治 S8），都危险则仍返回最优候选（兜底护 S7）。
        self.c12_adapt_evade = False
        self.c12_adapt_offsets = (0.8, 1.1, 1.5)   # m：offset 候选档
        self.c12_adapt_fwds = (0.4, 0.8)           # m：fwd 候选档
        self.c12_adapt_horizon = 1.0               # s：未来滚动评估时距
        self.c12_adapt_speed = 0.6                 # m/s：评估滚动速度

        # C11 AABB 预筛（P6，纯提速不改语义）：可行性检查前用轨迹包围盒跳过远处障碍。
        self.enable_c11_aabb_filter = False

        # ---------------- [P0-1] RC4 近目标逃逸修复（默认关，按车型查表开启） ----------------
        # 修复版（v2）：HP 闸门方向确认 + M-line 保护 + 开阔场景 stall 抑制 + 强制兜底
        self.enable_p0_1_hp_gate = False
        self.p0_1_mline_band_m = 0.1        # Bug2 M-line 退出带宽 ±0.1m
        self.p0_1_force_clear_steps = 300  # trap_steps 超此值强制清 escape（硬兜底）
        self.p0_1_mline_min_len = 0.5       # |goal-hp| < 此值时跳过 M-line 判定
        self.p0_1_stall_suppress_dgoal = 5.0  # d_goal > 此值且无动态障碍时抑制 stall-trap

        # ---------------- [P0-2] RC5 连续碰撞检查（默认关，按车型查表开启） ----------------
        # 修复版（v2）：半宽 capsule 半径 + 仅动态障碍 + early termination
        self.enable_p0_2_capsule = False
        self.enable_p0_2_early_term = False
        self.p0_2_safety_margin = 0.05      # capsule 额外安全裕量 m
        # 车型分支开关（module_config 根据 collision_model 查表设置）
        self.p0_1_enabled_by_model = {'rect4wis': False, 'rectangle': False, 'rect_diff': False}
        self.p0_2_enabled_by_model = {'rect4wis': False, 'rectangle': False, 'rect_diff': False}

        # ---------------- Case I-1（兼容保留） ----------------
        self.case_I1_start_x = 0.0
        self.case_I1_start_y = 0.0
        self.case_I1_start_theta = 0.7853981634
        self.case_I1_goal_x = 8.0
        self.case_I1_goal_y = 8.0
        self.case_I1_n_obstacles = 4

        # ---------------- Case I-2 ----------------
        self.case_I2_start_x = 0.0
        self.case_I2_start_y = 0.0
        self.case_I2_start_theta = 1.5707963268
        self.case_I2_goal_x = 0.0
        self.case_I2_goal_y = 8.0
        self.case_I2_n_obstacles = 4

        # ---------------- Case II ----------------
        self.case_II_start_x = 0.0
        self.case_II_start_y = 0.0
        self.case_II_start_theta = 0.0
        self.case_II_goal_x = 8.0
        self.case_II_goal_y = 8.0
        self.case_II_n_obstacles = 12
        self.direction_update_period = 0.5
        self.scene_x_min = 0.0
        self.scene_x_max = 10.0
        self.scene_y_min = 0.0
        self.scene_y_max = 10.0
        self.wrap_scene = True


def params() -> Params:
    """params() 等价入口（保持与 MATLAB 调用习惯一致）。"""
    return Params()
