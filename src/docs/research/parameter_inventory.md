# 有效生产参数与接口总表

核对日期2026-11-06。来自共享real_vehicle.yaml；adapter_contract.json另验证声明/读取。下表值是配置，不表示独立实车标定。完整运行时对象、input ages与新失败原因可由/rda/diag记录。定位补偿/TF保持0；后轴距离.345仅轴对称假定。

## rda_dwa_real_bridge

|参数|配置值|单位/语义|
|---|---|---|
|algorithm_frame|`"map"`|topic/frame/enum|
|body_frames|`["base_link", "base_footprint", "laser", "lidar", "sensor", "rslidar"]`|list|
|sensor_frames|`["laser", "lidar", "sensor", "laser_frame", "lidar_frame", "rslidar"]`|list|
|sensor_x|`0.0`|m|
|sensor_y|`0.0`|m|
|sensor_yaw|`0.0`|rad|
|body_to_base_link_x|`0.0`|m|
|control_rate|`10.0`|Hz|
|traj_predict_mode|`"fixed"`|topic/frame/enum|
|sim_time|`3.0`|s|
|traj_dt|`0.05`|s|
|traj_v_eps|`0.05`|m/s|
|traj_cache_enable|`false`|bool|
|smooth_weight|`0.6`|score weight|
|sticky_eps|`0.05`|score|
|apf_k_att|`1.0`|APF模型权重，见实现|
|apf_k_rep|`0.6`|APF模型权重，见实现|
|apf_d0|`1.5`|m|
|model_path|`""`|topic/frame/enum|
|gps_topic|`""`|topic/frame/enum|
|gps_yaw_topic|`""`|topic/frame/enum|
|localization_timeout|`0.5`|s|
|localization_jump_residual|`0.15`|m|
|localization_jump_yaw_deg|`5.0`|deg|
|obstacle_timeout|`0.5`|s|
|planning_timeout|`0.3`|s|
|planning_budget_s|`0.08`|s|
|reaction_time|`0.6`|s|
|direction_zero_hold|`0.2`|s|
|emergency_stop|`true`|bool|
|enable_trap|`true`|bool|
|wheel_feedback_timeout|`0.3`|s|
|car_state_topic|`"/car_state"`|topic/frame/enum|
|bridge_state_topic|`"/car_bridge_state"`|topic/frame/enum|
|mode_ready_topic|`"/car_mode_ready"`|topic/frame/enum|
|mode_ready_timeout|`0.5`|s|
|obstacle_max_speed|`1.5`|m/s|
|cluster_gate|`0.6`|m|
|cluster_min_points|`2`|count|
|cluster_max_radius|`30.0`|m (unless enum limit)|
|track_gate|`1.0`|m|
|track_lost_ttl|`5`|帧数/legacy生命周期|
|static_obstacle_aabb|`true`|bool|
|static_detour_enable|`true`|bool|
|static_detour_timeout|`24.0`|s|
|static_search_enable|`true`|bool|
|static_search_budget|`0.015`|s|
|static_search_max_expansions|`200`|count|
|vehicle_length|`1.2`|m (unless enum limit)|
|vehicle_width|`0.8`|m (unless enum limit)|
|v_max|`0.4`|m/s|
|v_min|`-0.3`|m/s|
|omega_max|`0.7`|rad/s|
|omega_spin_max|`0.8`|rad/s|
|a_max|`0.5`|m/s²|
|dec_max|`1.5`|m/s²|
|omega_acc|`0.8`|rad/s²|
|delta_max_deg|`30.0`|deg|
|goal_tolerance|`0.3`|count/enum/model parameter|
|kinematics_mode|`"ackermann"`|topic/frame/enum|
|ack_wheelbase|`0.69`|m|
|ack_length|`1.2`|m|
|ack_width|`0.8`|m|
|ack_delta_max_deg|`30.0`|deg|
|ack_delta_dot_max_deg|`45.0`|deg/s|
|ack_v_max|`0.4`|m/s|
|ack_v_min|`-0.3`|m/s|
|ack_a_max|`0.5`|m/s²|
|ack_dec_max|`1.5`|m/s²|
|footprint_center_offset|`0.0`|m (unless enum limit)|
|reference_center_to_rear|`0.345`|m|
|spin_pivot_offset|`0.0`|m (unless enum limit)|
|safety_margin|`0.2`|m|
|safety_dt|`0.05`|s|
|obstacle_uncertainty_rate|`0.05`|m/s|
|moving_stall_duration|`6.0`|s（新source运动窗口）|
|moving_recovery_enable|`false`|bool；默认仅观测、实验自动恢复关闭|
|blocked_duration|`1.5`|s|
|ack_guard_time|`2.0`|s|
|spin_reset_progress|`0.5`|m|
|spin_entry_deg|`10.0`|deg|
|spin_wait_time|`3.0`|s|
|spin_recover_time|`3.0`|s|
|spin_timeout|`12.0`|s|
|spin_stop_timeout|`3.0`|s|
|spin_w_min|`0.12`|rad/s|
|spin_acc_max|`0.8`|rad/s²|
|spin_request_timeout|`24.0`|s|
|spin_cooldown|`5.0`|s|
|max_spin_attempts|`2`|count|
|trap_max_duration|`24.0`|s|
|trap_exit_hold|`1.0`|s|
|trap_exit_progress|`0.1`|m|
|trap_cooldown|`5.0`|s|
|trap_static_probe_budget|`0.02`|s|
|trap_exit_dist|`0.3`|m (unless enum limit)|
|trap_exit_goal_ang_deg|`20.0`|deg|
|trap_exit_sector_deg|`60.0`|deg|
|trap_exit_sector_range|`2.5`|m (unless enum limit)|
|trap_exit_sector_max_obs|`3`|count（legacy诊断）|
|v_streak_threshold|`0.05`|m/s|
|v_streak_count|`25`|count（仅诊断）|

## traj_tracker_node

|参数|配置值|单位/语义|
|---|---|---|
|algorithm_frame|`"map"`|topic/frame/enum|
|wheelbase|`0.69`|count/enum/model parameter|
|max_steer_deg|`30.0`|deg|
|steering_gain|`1.0`|无量纲|
|lookahead_short|`0.6`|m|
|max_speed|`0.4`|m/s|
|v_min|`-0.3`|m/s|
|reference_center_to_rear|`0.345`|m|
|direction_zero_hold|`0.2`|s|
|speed_feedback_timeout|`0.3`|s|
|direction_speed_eps|`0.03`|m/s|
|car_state_topic|`"/car_state"`|topic/frame/enum|
|v_max|`0.4`|m/s|
|acc_lin_max|`0.5`|m/s²|
|dec_lin_max|`1.5`|m/s²|
|steer_rate_deg|`45.0`|deg/s|
|steer_hold_deg|`0.25`|deg|
|tracking_correction_max_deg|`0.0`|deg|
|spin_w_max|`0.8`|rad/s|
|spin_acc_max|`0.8`|rad/s²|
|spin_wait_time|`3.0`|s|
|recover_delay|`3.0`|s|
|switch_timeout|`8.0`|s|
|spin_timeout|`12.0`|s|
|spin_progress_timeout|`2.0`|s|
|spin_progress_deg|`0.5`|deg|
|spin_stop_hold|`0.5`|s|
|spin_require_stopped_feedback|`true`|bool|
|spin_require_yaw_stop_feedback|`true`|bool|
|spin_yaw_stop_rate|`0.03`|rad/s|
|spin_stop_timeout|`3.0`|s|
|rotate_entry_deg|`10.0`|deg|
|rotate_precision_deg|`4.0`|deg|
|rotate_decel_deg|`25.0`|deg|
|cmd_timeout|`0.5`|s|
|traj_timeout|`0.5`|s|
|localization_timeout|`0.5`|s|
|mode_timeout|`0.5`|s|
|target_timeout|`0.5`|s|
|pose_speed_limit|`0.5`|m/s|
|pose_yaw_rate_limit|`0.8`|rad/s|
|pose_jump_margin|`0.15`|m|
|pose_yaw_jump_margin_deg|`5.0`|deg|
|ready_timeout|`0.3`|s|
|body_to_base_link_x|`0.0`|m|
|rate|`50.0`|Hz|
|traj_topic|`"/rda/traj"`|topic/frame/enum|
|traj_mode_topic|`"/rda/traj_mode"`|topic/frame/enum|
|cmd_vel_topic|`"/cmd_vel"`|topic/frame/enum|
|command_topic|`"/rda/command"`|topic/frame/enum|
|localization_topic|`"/localization"`|topic/frame/enum|
|spin_target_topic|`"/rda/spin_target"`|topic/frame/enum|
|car_cmd_topic|`"/car_cmd"`|topic/frame/enum|
|status_topic|`"/rda/ctrl_status"`|topic/frame/enum|
|bridge_state_topic|`"/car_bridge_state"`|topic/frame/enum|
|mode_ready_topic|`"/car_mode_ready"`|topic/frame/enum|

## lidar_track_node

|参数|配置值|单位/语义|
|---|---|---|
|points_topic|`"/rslidar_points"`|topic/frame/enum|
|odom_topic|`"/localization"`|topic/frame/enum|
|obstacles_topic|`"/rda/obstacles"`|topic/frame/enum|
|use_z_filter|`true`|bool|
|rslidar_z_min|`-0.3`|m (unless enum limit)|
|rslidar_z_max|`1.5`|m (unless enum limit)|
|vehicle_half_length|`0.6`|m (unless enum limit)|
|vehicle_half_width|`0.4`|m (unless enum limit)|
|vehicle_outside|`0.0`|m|
|tile_size|`0.5`|m (unless enum limit)|
|tile_extent_threshold|`2.0`|m|
|tile_occupancy_threshold|`0.5`|无量纲占空比|
|max_scene_tiles|`512`|count|
|vehicle_center_x|`0.0`|m|
|lidar_height|`0.45`|m (unless enum limit)|
|pass_z_min|`0.15`|m (unless enum limit)|
|pass_z_max|`1.95`|m (unless enum limit)|
|sensor_x|`0.0`|m|
|sensor_y|`0.0`|m|
|sensor_yaw|`0.0`|rad|
|cloud_timeout|`0.5`|s|
|pose_sync_tolerance|`0.15`|s|
|track_lost_ttl_seconds|`0.5`|s|
|max_obstacle_range|`12.0`|m (unless enum limit)|
|pass_x_min|`-12.0`|m (unless enum limit)|
|pass_x_max|`12.0`|m (unless enum limit)|
|pass_y_min|`-12.0`|m (unless enum limit)|
|pass_y_max|`12.0`|m (unless enum limit)|
|body_to_base_link_x|`0.0`|m|
|cluster_gate|`0.6`|m|
|cluster_min_points|`2`|count|
|cluster_max_radius|`30.0`|m (unless enum limit)|
|track_dt|`0.05`|s|
|track_gate|`1.0`|m|
|track_lost_ttl|`10`|帧数/legacy生命周期|
|kf_q|`0.3`|Q模型噪声系数（未统计标定）|
|kf_r|`0.15`|位置测量R模型噪声系数（未统计标定）|
|motion_guard_enable|`true`|bool|
|measurement_position_slack|`0.15`|m|
|velocity_innovation_limit|`0.8`|m/s|
|velocity_reconfirm_time|`0.2`|s|
|obstacle_max_speed|`1.5`|m/s|
|dyn_speed_thr|`0.3`|m/s|
|dyn_hold_frames|`5`|源量测帧数|
|static_speed_thr|`0.15`|m/s|
|static_hold_frames|`10`|源量测帧数|
|static_small_thr|`0.6`|m|
|loc_timeout|`0.5`|s|

## 底盘与协议

|字段/参数|实际值与状态|
|---|---|
|ACK command/feedback|0/0用户确认|
|SPIN command/feedback|4/4用户确认，反馈表示轮型到位|
|SPIN UDP speed conversion|-2已标定，未修改|
|mode_change_time|3s，guard强制最小3s|
|mode_retry_period|3.5s|
|mode_max_retries|2，初次+最多2次补发；fresh intermediate反馈不重发|
|cmd_timeout/feedback_timeout|.3s/.3s|
|CRAB/TRANSLATE|消息注释1/2不是已确认物理协议；离线ModeProtocol command/feedback/unit默认None、enabled/verified=False；生产底盘拒绝|
|speed/angle feedback|四轮速度有报文；无实际转角，轮速平均未独立标定纵向速度|

动态对象和车辆最大速度、位置slack、Q/R、刹停/角加速度阈值需实车辨识。max轮速和新源yaw率只提供停止观察条件，不是新增硬件测量。source time与arrival time见framework_and_dataflow。

当前共享YAML共206项（以adapter实际声明/读取为准）。横移仅线速度；斜行线速度及±45°为用户确认类型/范围；m/s为软件建模单位，固件比例/方向/ready编号未完整验证，生产不启1/2。
