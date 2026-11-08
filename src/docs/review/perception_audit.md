# 定位、感知、跟踪与诊断审查

审查日期：2026-11-06。阶段一仅阅读运行代码，未连接车辆。行号对应审查时的原始源码；后续修改会改变行号。入口为 `ros2 launch car_launch run_rda_dwa.launch.py`。结论是：即使聚类检测很准确，当前坐标/时序/可见区域/预测接口仍不足以证明动静避障可靠。

后续用户最新纠正：长1.2米、宽0.8米、轴距0.69米；定位和雷达已经在几何中心，平面偏移为0，旧-0.15米假定不成立；命令和真实反馈自转均为4且反馈表示已到位，UDP乘-2方向已实测正确。本文下面的原始-0.15及反馈2保留为原代码证据，不代表当前实车协议。最新实现和验证见 `perception_implementation.md` 与 `cross_review.md`。

## 1. 实际运行数据流

1. `car_launch/launch/run_sensor.launch.py` 立即启动外部 `lidar`，2 秒后启动外部 `imu`，3 秒后启动外部 `rs_to_velodyne`。这些驱动包不在当前源码中，不能确认原始雷达话题 QoS、帧率、frame、点时间语义及外部转换是否正确。
2. 总入口 6 秒后启动 `run_fastlio.launch.py`。FAST LIO 吃 `/velodyne_points`、`/imu/data`，`laserMapping.cpp:628-634` 输出 `/Odometry`，`frame=odom, child=body, stamp=lidar_end_time`；发布 odom→body TF。`velodyne.yaml` 配置 `scan_rate=10`，不能由注释认定定位源为 50 Hz。
3. `global_localization.py:58-70,269-358` 以 `/cloud_registered`（odom 点云）和 `/Odometry` 做 ICP，约 0.5 Hz 加上计算耗时更新 map→odom；仅 fitness>0.70 时接受；`/initialpose` 用于初始值。
4. `transform_fusion.py:51-118` 每 20 ms 重发 map→body 位姿 `/localization`，位姿是最新 map→odom 乘最新 odom→body，保留原始里程计 stamp。50 Hz 是输出重发频率，不是 50 Hz 新观测。
5. 静态 TF 为 `body→base_link (-0.15,0,0)`，`base_link→rslidar (0,0,+0.45)`。`frame_utils.body_pose_to_base_link:145-158`、lidar `_cb_odom:182-193`、bridge `_cb_odom:440-455`、tracker 使用同一个 -0.15 米 x 补偿，把 `/localization` 的 body 位姿转换到 base_link。x/y 主链补偿一致；是否 base_link 真处于车身矩形几何中心须实测。
6. 总入口 12 秒后启动 `lidar_track_node`。实际链为 `/rslidar_points → _cloud_arround → _cloud_front → Clusterer → bbox → map 位姿转换 → GlobalKFTracker → /rda/obstacles`。输出 `frame=map`；结构化 ObstacleArray 含统一 Header，以及每个对象的 id/type/x/y/L/W/heading/speed。
7. `bridge_node._cb_obstacles:403-412` 保存结构化对象。`vehicle_bridge.step:404-411` 每规划周期调用 `TrackManager.update_measurements`，再进入预测、trap 和 DWA。这不是每新检测调用一次，而是每规划帧重复处理缓存。
8. 当前 `model_path=""`，`traj_predict_mode="fixed"`，DQN 不执行；只有填写模型路径时，`bridge_node:284-295` 才加载 `DQNPolicy`，以网络 argmax 从 `[0.8,1.0,1.2,1.5]` 秒选预测时域。

## 2. 确定问题与优先级

### P0：全周雷达被裁成仅车头 0.6～3 米，旋转与倒车缺少障碍输入

`lidar_track_node.py:204-215` 虽提取周围点云，却将 `_cloud_front` 的结果送入聚类；`329-337` 默认只保留 `0.6≤x≤3.0`。侧方、后方和车头横向近角点未进入对象列表。`max_obstacle_range=5.0` 也无法恢复 x>3 的点。

影响：阿克曼贴近侧墙、trap 倒车和原地旋转的车身角点碰撞不能依赖当前对象列表发现；行人从侧方横穿到规划路径时，预测开始太迟。原地旋转还需要覆盖 3 秒轮型切换期间和整个旋转时域的移动障碍。建议实际检测改为周围区域，front 仅用于诊断或专项策略，不能作为总碰撞输入。

### P0：雷达高度变换符号错误，且配置文件声称的 z 过滤参数没有接线

静态 TF 的 +0.45 米表示 `z_base=z_rslidar+0.45`；`lidar_track_node.py:277-284,312-320` 却做 `z_rslidar-0.45`。实际默认 base z 过滤 `[-0.5,0.5]` 因错误符号选择雷达 z `[-0.05,0.95]`，倾向保留较高点并漏掉低矮障碍；正确坐标下的地面也未被真正做平面剔除。

`real_vehicle.yaml` 的 `use_z_filter/rslidar_z_min=-0.3/rslidar_z_max=1.5/vehicle_half_length/vehicle_half_width` 没有在节点 declare/read，运行代码使用另一套 `pass_*`、`vehicle_length/width/outside` 参数。不能靠调现有 YAML 注释改善漏检。应统一参数接口并按实际 TF 修正 z，加清晰的地面高度或平面模型过滤。

### P0：消息接收新鲜不等于源观测新鲜

`transform_fusion.py:57-118` 无限重发旧 `/Odometry`，保留旧 stamp。lidar `_cb_odom:193` 却用到达时间刷新 `_loc_t`；planner `set_localization:208-219` 完全忽略传入 stamp，用离散核心 `_t_now` 作为刷新时间。FAST LIO 停止时融合节点仍活着，定位超时可能永远不触发；yaw 冻结下控制层也没有独立定位超时（控制 agent 已核对）。

lidar 点云同样无 stamp 老化判断，在计算结束时 `_publish_obstacles:346` 写 now，抹去采集时间；用“最新位姿”变换旧点云而非匹配点云时刻的定位。队列/计算迟滞直接成为障碍位置误差和虚假速度。建议源时间戳做 freshness 和重复帧门控；收到单调新 stamp 才刷新定位有效性；点云使用采集 stamp 对应的定位插值；ROS 接收时钟与源 stamp 的时钟不一致应由显式配置处理，不能无条件忽略 stamp。

### P0/P1：障碍过期被当作无障碍继续规划

`vehicle_bridge.py:405-408` 障碍超时后传空检测；`obstacle_fusion.TrackManager:238-250` 立刻 `self.tracks=new_tracks`，丢失对象删除发生在 TTL 检查之前。注释“静态保持/动态TTL”没有实现。感知空周围点云 `_cb_cloud:206-208` 直接发空列表且不执行 KF miss 处理；定位超时或传感器处理故障不能与确实无障碍混同。建议感知故障安全停车；短时遮挡保留对象并增加不确定性，只在真实观测确认空闲或TTL到期时删除。

### P1：固定 dt、预测前关联位置、两次滤波导致速度估计及轨迹外推不一致

`GlobalKFTracker` 的 dt 创建时固定，YAML为0.05秒，而 FAST LIO lidar preprocess 为10Hz，真实原始点云帧率尚未证明为20Hz。运动估计和 `lost_ttl=10` 的持续秒数会随输入频率漂移。`kf_tracker.py:242-250` 先取 `pred=position()`，再 KF predict，却用 predict 前的位置做匈牙利匹配，未按函数文档关联预测位置。

`TrackManager:193-207` 同 id/type 每规划帧 predictor.update，仅送 x/y，忽略上游 speed/heading。下游 predictor 初始化 `dt=control_dt=0.1`，而 bridge 的墙钟规划 dt 可变化；重复缓存同一位置被解释为一帧静止观测。类型变更虽然重置 predictor（并非永久锁静态），同 type 仍重复差分/滤波，引入迟滞和速度缩放。应每新感知帧更新一次、使用感知实测 dt，并优先消费已经估计好的速度，区分估计器更新与规划时域外推。

### P1：初始静态判定延迟动态预测

`KFTrack:83-86,117-136` 初始 moving=False，速度持续>0.3m/s至少5帧才动态；`GlobalKFTracker._output:307-313` 未确认动态前强制 speed=0 和 static/block类型，即使 `infer_type` 给 pedestrian 也被覆盖。这不能称“静态优先安全”：行人新出现或刚起步时，运动预测被抑制。10Hz时仅5帧就是0.5秒，另加KF收敛、点云裁剪和下游滤波。应新对象以速度未知/可动对象对待并保留上游运动估计和不确定性；静态确认是停止预测的依据，不能用未确认运动替代确认静态。

### P1：尺寸首帧锁定及包围盒中心/角度丢失

`KFTrack:79-82` 锁定第一次 L/W；`GlobalKFTracker._refresh_attr:286-288` 是 pass。检测从小局部看到整物体后，尺寸无法增大。TrackManager同type也不刷新L/W/radius。`cluster_bounding_box:129-149` 用点均值作为中心，PCA投影最大最小范围作为尺寸，但正确包围盒中心应该是投影极值中点；不均匀采样时返回矩形甚至不包含全部点，外接圆也不能保证覆盖。

lidar `_cb_cloud:227-229` 完全丢弃 bbox theta；发布 heading 是动态速度方向、静态为0（`356`，`kf_tracker:311`）。Obstacle消息也把 heading 定义成运动方向，无法同时表示静态斜墙的矩形朝向和运动对象车身朝向。若碰撞使用外接圆，只要中心/尺寸包含实体，丢朝向主要影响效率；若细查为OBB，朝向错误会漏碰。建议稳定保守更新尺寸，投影极值中点作为bbox中心，先采用包围圆确保覆盖；进一步升级消息时区分 shape_yaw 和 velocity_heading。

### P1/P2：聚类方法与文档不符，对点顺序敏感

`Clusterer.cluster:103-118` 按与现有簇均值距离逐点加入；不是邻接连通分量，也不会合并已有簇。长墙会被切片，邻近物体可合并，对点顺序敏感；每个点循环簇且重算整个簇均值，Python高密度点云耗时大。全周检测放开后应采用体素下采样+空间网格邻接/欧式连通聚类，确保运算预算；不要直接全周保留所有点后沿用当前二重成本。跨帧簇分裂合并与尺寸首帧锁定叠加会造成id变化和避让方向抖动。

### P1/P2：全局定位跳变没有创新门控，低速控制直接吃 map 修正

`global_localization.py:323-336` 仅fitness阈值，无位移/yaw创新限制、速度物理可行性或covariance门控。`transform_fusion.py:103-111` 直接叠加新 map→odom。ICP大幅修正会把静态物体的map位置瞬移，使KF估出虚假动态速度，同时车头目标误差、纯跟踪和planner航向差分突变。

`cb_initial_pose:385-390` 直接把 RViz 指定 map→body 位姿当成 map→odom；若 odom→body 已非identity，应初始化 `T_map_odom=T_map_body_initial * inverse(T_odom_body)`。第一次ICP可能修正，也可能因此裁错FOV导致初始化失败。定位融合在未收到map_to_odom时使用identity（`transform_fusion:68-71`），以map名发布尚未完成全局对齐的odom，且没有ready信号。

建议控制用连续odom局部参考、map仅全局导航；若保留map链，加入跳变门控并在大修正时安全停止/重建障碍估计与轨迹。跨线程ICP同时读取最新cur_scan和cur_odom，二者未同步（`269-282,333`），应成对保存时间戳并取同步快照。

### P2：诊断最近障碍距离计算错，无法据此判断安全余量

`diag_recorder_node._cb_obstacles:587-604` 对map障碍算 `hypot(ob.x,ob.y)`，得到地图原点距离，不是障碍相对车辆距离。应使用同步定位减去车辆位置，再减车体及障碍的包围范围求净空。当前记录只有最近id/type和数量，缺对象完整L/W/速度、感知源stamp、定位跳变量、KF动态置信计数，难以复盘横穿/遮挡。记录的 `loc_age` 是源stamp老化（`519-529`），是识别融合重复旧数据的有用现成指标，但控制并未使用。

### P2：DQN备用路径观测协议存在两个确定缺陷

当前主入口不启用 DQN，以下不构成当前抖动根因：`dqn_state.make_state:106-110` 用wrapped检查FOV，却用未wrap的bearing划扇区，车辆跨±π时扇区错误；`input_dim=26`（`132-163`）从21维加5个nearest/realized维，实际是26？逐项为nearest3+realized2=5，因此26正确（初看注释并不表示风险25再加）。`_risk_features:75-77` 固定车体半长0.5/半宽0.3，实车为半长0.6/半宽0.4，风险特征不能作为可靠碰撞安全门。DQN只有选时域，安全仍须由实际足迹、制动与动态碰撞约束保证，不能依赖训练性能替代。

## 3. 参数口径表

| 参数/数据 | YAML或注释 | 运行含义 | 风险 |
|---|---|---|---|
| 定位50Hz | /localization 50Hz | 原始Odometry反复重发 | 必须检查源stamp推进 |
| track_dt | 0.05 | 每点云一步固定KF dt | 与真实雷达帧率未核对 |
| track_lost_ttl | 10 | 雷达KF miss帧 | 空周围点云绕过KF，另一层TTL无效 |
| rslidar_z_min/max | -0.3 /1.5 | 无declare/read | 参数无效 |
| vehicle_half_length/width | 0.6/0.4 | 无declare/read | 使用节点默认1.2×0.8及outside=0.05 |
| front_x_min/max | YAML无配置 | 默认0.6/3.0米 | 全局碰撞输入前向裁剪 |
| pass_z_min/max | YAML无配置 | 默认-0.5/0.5；且z转换反号 | 低障碍漏检 |
| sensor x/y/yaw | 0/0/0 | 主雷达track节点直接认为投影对齐，不查frame或TF | 实际安装偏角会直接系统误差 |
| obstacle_timeout | 2秒 | 超时清空，非安全停车 | 感知断流后放松避障 |
| localization_timeout | 1秒 | 核心接收时刻差 | 重发旧观测时失效 |
| max_obstacle_range | 默认5米 | 聚类质心后筛 | 不能恢复front裁剪 |
| dyn_hold_frames | 5 | 5帧>0.3才开始动态预测 | 新出现行人启动延迟 |
| heading | 动态运动方向/静态0 | 同时被对象几何使用 | bbox方向丢失 |

## 4. 与其他 agent 的接口核对

- 已发 planner/control：前向裁剪、z符号、重发旧定位stamp与类型dt风险。
- planner agent 确认：当前 rectangle M2 调 `min_clearance_to_obstacle_at` 未传预测位置，helper仍取o.x/y；动态预测未真正用于这一运行分支。该问题归规划修复，应和感知接口共同验收。
- control agent 确认：控制无定位freshness门，自转w由yaw闭环独立生成；planner仅发布v=w=0不能取消自转。需明确可靠停止/取消协议。
- control agent 确认：规划制动1.5m/s²，tracker对称加减速0.2m/s²；实际制动假设不一致会放大所有感知迟滞风险。桥另积分、轨迹缓存和纯跟踪轨迹未重新碰撞验证。

## 5. 建议修复顺序与验收

1. 先封住数据失效继续行驶：源stamp推进、定位/障碍超时、停止/取消自转传播。
2. 改全周输入和正确z；采用有界计算成本的聚类；处理合法空帧与解析失败区别。
3. 动态接口更新频率独立于planner；按新感知帧更新时间；保留速度/不确定性；修复TTL、预测后关联和尺寸刷新。
4. 与规划 agent 联测动态未来位置参与碰撞以及3秒换轮静止期的安全性；与控制 agent 对齐足迹、转向速率与实际减速度。
5. 离线造点云验证前/侧/后、低矮路障、空帧、迟到帧、姿态插值；纯跟踪输入模拟横穿、起步、停止、遮挡、物体从部分可见到完整尺寸；冻结Odometry但保持fusion发布，必须停车。
6. 复盘指标：感知源年龄、真实定位刷新Hz、最小车身净空、预测残差、动态确认时间、转向反号频次、模式每分钟切换次数、停止原因、轮型切换实测耗时与目标角误差。地图原点距离不能作净空。

当前未运行ROS2或车辆，以上是源码和接口证据，不能冒称实车验收通过。外部驱动、实车几何、转角反馈、轮速标定与传感器时间同步仍需要实际日志确认。
