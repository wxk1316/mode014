# 规划与仲裁审计（2026-11-06，修改前快照）

审计范围：实际入口 `ros2 launch car_launch run_rda_dwa.launch.py`，本文件由 planner_audit agent 完成；仅静态阅读与纯 Python 离线复现，未连接车辆。行号指修改前源文件。主结论：目前动态安全硬约束有确定接线缺陷，trap 与仲裁有进展/退出判据冲突，不能将左右抖动归结为单纯调参。

## 实际运行链路与模块身份

`rl_dwa_car.launch.py` 启动 `lidar_track_node → bridge_node → traj_tracker_node → car_ctr`。老 control_bridge 不在 launch。`bridge_node` 消费 `/localization`、`/rda/obstacles`、`/move_base_simple/goal`，还同时消费 `/amcl_pose`；下发 `/cmd_vel`、`/rda/traj`、`/rda/spin_target`、`/rda/traj_mode`。控制回流 `/car_state`、`/car_bridge_state`、`/car_cmd`。

`module_config('v15_latesc')` 开 M1~M6、C1~C4、C11/C12。但 ROS bridge 实际构造的是 `ObstaclePredictor(dt=.1, alpha=beta=1)`，未启用 IMMUKFPredictor，也未设置 M5 smoothing。因此参数声称 C2/M5 开启，不代表运行链路真的使用这些滤波。`model_path=''`，DQN 不工作；真实模式为固定时域 DWA。`global_guide.py` 的 GlobalGuide 依赖仿真 Scene 静态地图，bridge 明确关闭 C8，当前无全局绕行路径。

`VehicleBridgeCore.step` 顺序：时间累加 → 无目标/定位超时/目标到达守卫 → 四轮/定位反馈 → TrackManager 转换 → ClassicTrapController.step(永远 freeze_stall=True) → 自定义 `_arbitration_step` → 新旧 trap 退出 → DWA plan → 当前位姿碰撞 → Ackermann adapt → v>=0 → 若 SPIN_REQUEST 则 v=w=0 → 输出。随后 node 重新按最终 (v,w) 恒速积分并发布参考路径；历史轨迹缓存可再次改写路径。

`trial_runner.py` 对实车仅使用 ClassicTrapController 及 `_rect_hit`；其它 Scene、trial、机动仿真 runner 未运行，但被模块导入。`escape.py` 的 C12 主动对向威胁通道依赖其它 trap controller 的调用；当前 ClassicTrapController 的激活路径冻结，实车并未完整走仿真 C12 行为。

## 确定缺陷与后果

|编号/优先级|修改前位置|证据与后果|建议|
|---|---|---|---|
|P0-DYN-POS|enhanced_dwa.py:1082~1115 / collision_checker.py:418~436|M2 计算未来位置 p，但调用 helper 只传 obs/oth；helper 始终用 `o.x/o.y`。动态行人未来位置完全没有进入矩形碰撞，风险代价有预测也弥补不了硬约束缺失。`_is_trajectory_safe`:366~379 同样计算 ox/oy 后未用。|helper 接受 ox/oy，所有预测调用必须显式传入；统一全时域同步时间。|
|P0-DYN-FALLBACK|enhanced_dwa.py:237~252|预测判断全不可行后按当前位置重判，并可直接选前进样本。动态危险被回退静态模型放行。|实车禁止降级安全模型；仅安全停车或经过同一预测检查的受限制动。|
|P0-NOFEAS|enhanced_dwa.py:306~311 / vehicle_bridge.py:475~480|M6 返回 False 与受限减速，bridge 明确不因 plan_ok False 停车，无未来碰撞检查。无法依赖“可行性失败等于急停”。|执行安全守卫、可靠制动时域，区分不可行停车与正常缓降。|
|P0-SPIN-GAP|vehicle_bridge.py:625~676|低速计数直接请求自转，与 `planner.try_spin` 完全不同路；没有旋转矩形扫掠净空，也没有 3 秒等待时行人到达风险。Ackermann planner 本身禁spin，所以已有 try_spin 检查无法保护真实自转。|模式切换前独立扫掠/等待/旋转风险检查；执行中再次检查。|
|P0-OBS-STALE|vehicle_bridge.py:395~398 / obstacle_fusion.py:238~246|障碍超时直接 detections=[]；TrackManager 直接 tracks=new_tracks，所谓 TTL 与保留静态不生效，一次漏检全清。|新消息版本驱动更新，保持遮挡 track，超时停车而非当开阔地。|
|P1-EXEC-MISMATCH|collision_checker.py:189~204 / bridge_node.py:636~652 / vehicle_bridge.py:481~490|候选 Ackermann 预测每次 v=δ=0 起步；真实 adapt 从当前速度/δ起步；发布轨迹恒速，tracker另做纯跟踪与速度整形。三者实际轨迹不同。|碰撞预测从当前运动状态起，检查最终执行轨迹，移除未经重检路径融合。|
|P1-MPT|bridge_node.py:257 / vehicle_bridge.py:497|enable_mpt_escape=True、ack_v_min=-.3，MPT reverse 输出随后钳 v>=0但w仍留着。算法认为倒车脱困，实车停住，时间机动虚推进。|用户支持原地旋转，实车路径关闭MPT倒车；若未来支持倒车则完整协议再开。|
|P1-OBS-VEL|obstacle_fusion.py:191~214 / predictor.py:64~116|带id同type输入忽略感知 speed/heading，每规划帧差分；predictor.dt固定.1而规划周期变，重复感知差分为0，新检测间隔长则放大速度；同type尺寸锁定。静态 heading置0但speed未冻结导致幻影动障。|直接消费已有KF速度，测量时间/预测时间分离，保留检测包围角与更新尺寸。|
|P1-YAW-TIME|vehicle_bridge.py:206~220 / bridge_node.py:562~574|定位差分用只在规划时推进的_t_now。规划阻塞后多个定位回调同_t_now，dlt=0跳过，首帧又把小yaw增量除大规划周期；角速度噪声/失真影响动态窗与δ反馈。真实dt还被.5截断，内部超时慢于墙钟。|定位按源时间差分，输入年龄用单调墙钟，控制dt单独限幅。|
|P1-SAMPLE-BIAS|params.py:31~34 / enhanced_dwa.py:676~680|omega上下限[-1,.7]不对称；yaml omega_max仅到fws不接pp。栅格从变动wmin起，dt变化时0缺失，直行被迫有小w；多个原始w经_eff_omega压同一曲率，smooth却用原始w。|对称硬限制、显式0/历史指令候选、在可执行曲率/转角空间去重并评分。|
|P1-STICKY|enhanced_dwa.py:1554~1588|仅找近邻可行样本，却返回未经检查的精确last_cmd；阈值可能使近似样本安全、旧真实指令碰撞。|复查返回的真实指令，或返回已检查的样本；禁止以滤波覆盖安全判断。|
|P1-TRAP-MOVINGGOAL|trial_runner.py:925~932 / escape.py:543~546 / vehicle_bridge.py:588~614|每帧重生成相对当前机器人子目标；最短.2m，exit_dist=.3m，第10步可无位移退出并重置spin_used。远子目标一直随车移动，很难真正到达。|激活时锁定子目标/逃逸侧，危险重校验才替换；退出须净进展+重新可通行持续成立。|
|P1-TRAP-CLEAR|vehicle_bridge.py:608~618|注释“±60°”但half=60/2实际±30°，计数<3允许单个大型墙也判清场。不考虑物体尺寸或可通行路径。|用碰撞可通行轨迹判断，正确角语义，时间滞回。|
|P1-ARBIT-OSC|vehicle_bridge.py:228~244,625~682|仅/car_cmd.speed<.05计25帧+ACK2秒，自转后再次低速进入trap，trap内允许无限重复spin。低速可为行人让行、传感器丢失、目标附近、tracker超时，没有区分。计数是请求速度而非真实运动，输出.1但物理卡住反而不会检测。|区分合法停车、让行、无路径、物理卡住；时间化进展窗口+必要性/角误差/扫掠门；会话ID、冷却、失败上限。|

## 实际参数覆盖重点

`wheelbase=.69m，δmax=30°，δdot=45°/s，Rmin≈1.195m，ack车身1.2×.8m，fws车身1.5×.8m，vmax=.8m/s，sim_time=3s，n_trajectory_samples=15`。`pp.min_linear_velocity`初步yaml=-.3但窗口rectangle覆盖0；MPT却仍返回负值。`pp.max_angular_velocity=.7/min=-1`没有yaml对称覆盖。`omega_spin_max`yaml只到FWS；独立tracker spin_w_max=1，因此实际自转峰值不受规划.8限制。M1 acc_lin默认1.5不是yaml统一口径；module override acc_ang=dec_ang=2.0。M2未来只有5个dt，.1→.5秒，dt=.5→2.5秒；轨迹3秒而动态障碍时间不足且采样稀疏(.2秒)容易越过交叉。

`traj_cache_enable=true`，旧轨迹未来时间覆盖新轨迹时只替换更远端，规划换侧后仍追踪历史轨迹；停车旁路不清缓存。`smooth_weight=.6`，但平滑在 trap 与上周期近停车禁用，最容易抖动阶段没有该保护。`sticky_eps=.1`是score空间，不是yaml注释m/s。

## 已完成离线复现

运行环境：bundled Python+numpy；没有 ROS2。以 wheelbase=.69,length=1.2,width=.8 的 Ackermann 构造器验证：

1. `dt=.147`，无障碍动态窗omega列表 `[-.294,-.194,-.094,.006,.106,.206]`；0候选不存在。
2. `min_clearance_to_obstacle_at([[0,0,0]], o_at_[2,2])` 返回gap `1.6404m`，而传入未来位置 `[.3,0]`真实净距为 `-.4m`。预测位置缺线属确定代码事实。
3. TrackManager 带id检测后一帧空检测即 `len(tracks)=0`，TTL未保留。
4. `planner.escape_score(VelocitySample(.1,0), RobotState(), (1,0), [])` 正常返回 `1.2725`。总控交叉核查纠正了本 agent 初步阅读时“inflation_radius缺失”的判断：该属性确实定义于 collision_checker.py:541，不能作为生产缺陷。

## 动静态可靠性判断与工程限制

检测准确并不足以保证可靠：当前动态碰撞没有使用未来位置，输入时间/遮挡保留有错；静态矩形SAT本身有价值，但检测角/尺寸锁定、front-only感知、坐标参考不齐、执行曲线再整形未复查会破坏保证。局部DWA+APF没有全局地图/路线，U形死胡同或必须远离目标绕行不能承诺最终到达。原地转向可以提高可达性，但必须把停车、3秒切换、旋转包络、恢复3秒看成完整机动，不能当瞬时ω控制。

## 提交给总控的实现建议顺序

先修P0硬接线、统一输入时间/运动几何/执行安全，并让 planner 异常有显式日志；再建立单一planner仲裁与tracker闭环旋转的有状态会话，角已对齐则不切换，必须角到位+停车稳定+模式ready才恢复。最后把平稳目标写进可执行转角采样和变角速率约束，禁止历史路径平滑越过安全判断。增加离线回归：交叉/迎头/遮挡/过期、曲率可执行、trap正常评分、无进展不退trap、3秒等待和模式确认、自转过程中急停、已对齐不进入自转、长运行切换次数与转角总变差。实车需从低速受控环境复测，软件回归不能替代底盘ready和转速符号标定。
