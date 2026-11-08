# 感知、定位输入与诊断修复实现

2026-11-06。依据 `系统框架与问题定位.md` 的 P10/P11/P12 及跨模块接口修复。本模块修改没有启动 ROS2 或向车辆发送指令；原始源码由主 agent 保存于 baseline。

用户最新纠正覆盖旧假定：实车长1.2米、宽0.8米、轴距0.69米；`/localization` 已经是车身几何中心，雷达平面原点也在几何中心，`body_to_base_link_x=0.0`，禁止再应用旧-0.15米偏移；命令与真实`/car_state`自转反馈都为4，反馈表示已到位，UDP速度乘-2方向已实测正确。以下实现采用这些最新确认值。

用户随后明确定位算法保持原样，并最终更正补偿归零。`global_localization.py`、`transform_fusion.py`和FAST_LIO的`CMakeLists.txt`已逐字恢复原始备份；定位链仅保留`run_fastlio.launch.py`的平面静态TF x=0。下述源时间戳与连续性检查属于感知、规划和控制对定位消息的消费逻辑。

## 已实现行为

### 点云与坐标

- `lidar_track_node.py` 的规划对象提取改为全周过滤点集，不再把只有 `x∈[0.6,3]` 的front点集送入总碰撞输入。前/侧/后都经过同一聚类接口。
- 运行节点全周观察半径提高为12米，x/y直通范围同步为[-12,12]，cluster_max_radius仍为30米。依据障碍最大速度1.5m/s、两次机械切换各3秒、reaction_time=.6秒和约1米车身/障碍余量，原5米无法支持模式等待和撤销恢复的观察时域，12米提供配置上的余量。纯filter函数默认5米仅供兼容，运行node显式传12米，YAML已同步。
- `frame_utils.filter_lidar_points` 按静态 TF 使用 `z_base=z_rs+lidar_height`。`use_z_filter/rslidar_z_min/max` 和 base z上下限实际接线；默认雷达z[-0.3,1.5]对应base z[0.15,1.95]。同时剔除非有限点、车体点和超范围点。
- 车体自剔除中心使用 `vehicle_center_x=0.0`，与规划 `footprint_center_offset=0.0`、`spin_pivot_offset=0.0` 对齐；半长0.6米、半宽0.4米，`vehicle_outside=0.0`严格仅剔真实车身内部，修复原0.05米外扩盒把侧墙y=.42米屏蔽的盲带。默认sensor_x/y/yaw=0、body_to_base_link_x=0.0；`run_fastlio.launch.py`的body→base_link静态TF同步改x=0，位置直接使用中心定位。雷达height=+0.45的z变换保留。车身反射/轮型细mask仍需实测，不能用盲带替代。
- 原始点云订阅改为 `BEST_EFFORT, depth=1`，兼容可靠或best effort传感器发布并减少旧帧排队；定位接收也允许两类发布。
- 支持 sensor_msgs_py 的tuple和numpy structured record；解码失败不发布“空场”。fallback按row_step、point_step、endianness解析。有效空点云继续推进KF失联时间。
- 聚类改为空间网格连通分量，先体素聚合并保留每体素原始点数。12米扩大后根据80k点profile优化为消耗已访问候选桶，voxel只返回中心及投影margin，不再生成四倍角点记录；margin=voxel×(|cosθ|+|sinθ|)与原四角包络等价，保持raw点覆盖。PCA矩形中心改为投影极值中点，避免不均匀采样质心导致包围框漏包点。
- 最终运行链先把过滤点转map，再使用world固定voxel/grid聚类。大且稀疏/细长簇按world固定0.5米tiles输出局部AABB，保留小行人和高占空实心簇的整框，修复单bbox填满U空腔的问题。对于大bbox包含车心、但原点/voxel不占实际1.2×.8车身矩形的簇，按实际车辆姿态与point-to-rectangle净距强制拆，occupancy仅辅助；没有用外接圆+.2米把侧墙±.8米的窄U错误当成占据，也不替代DWA安全margin。
- tiles使用稳定world锚点/ID，父完整cluster仍做KF运动估计，动态子tile继承父速度和unknown状态，不能因固定网格中心而误判静态。父测量使用保存的raw_bbox边界，避免voxel均值变化导致速度prior偏差。确认静态tiles用block标签；所有active源测量都是map AABB，fusion明确shapeYaw0后保留>=.05米的实测物理尺寸，不施加类型先验的.75米静态宽度或未知对象正方形膨胀。
- 默认tiling参数`tile_size=.5/tile_extent_threshold=2./tile_occupancy_threshold=.5/max_scene_tiles=512`。超限明确记录representation overload并停止刷新，让既有感知超时安全停车；不丢掉12米内远处点，也不粗合并填回车心。满占空80k/100k密集场景仍为1对象，不产生几百个瓦片。不能据此承诺任意复杂迷宫或死胡同最终脱困。

### 源时刻与定位

- `SourcePoseBuffer` 只接收单调推进且新鲜的定位源stamp；重复fusion心跳无法刷新有效性。按点云stamp插值定位与最短yaw差，而不是旧点云直接乘最新位姿。
- lidar检查定位源年龄、cloud源年龄、重复/倒序stamp和frame；成功输出ObstacleArray保留输入cloud采集stamp。不是计算完成后用now掩盖迟滞。
- `transform_fusion.py`保持原发布逻辑，包括重复发送带原Odometry源stamp的定位消息；下游拒绝重复源stamp刷新有效性，正常的新stamp仍可接收。原全局初始化行为也保留。
- `global_localization.py`保持原ICP、初始化、保存/加载和点云回调逻辑。之前新增的创新门控、初始化逆变换及保存格式调整已撤销。

### 跟踪与动态接口

- KF每新cloud source stamp执行一次；F/Q使用真实delta时间，重复/倒序stamp不再构成新量测。第二个真实观测建立速度初值，随后继续KF更新；运动速度限制同步约束预测状态。
- 匈牙利关联使用KF predict后的当前位置；尺寸持续保守增大并包含测量到滤波中心的残差。初始“未确认静止”对象按可动对象输出，估计速度不会等5帧动静分类完成后才传给规划。静态确认时间和失联TTL按秒计。
- `TrackManager` 消费上游KF速度，移除重复缓存位置送入第二次速度差分/IMM的行为。尺寸/type刷新、失联按时间保留，snapshot外推只生成副本，不修改原测量锚点。
- 当前ROS Obstacle消息只有一个heading（运动方向），没有独立几何heading。最终active lidar对所有type都输出map AABB，planner源契约统一追加shapeYaw0，fusion保留实测尺寸；动态/unknown安全检查用包络circle与对应速度/可达上界。只有没有明确几何契约的外部legacy输入才使用保守正方形包络。`velocity_heading`与`shape_heading`分离，不用运动方向覆盖几何长轴。
- 丢检后会按年龄扩大足迹；达到时间TTL再删除。TrackManager默认TTL为旧lost_ttl×predictor.dt（默认0.5秒），KF默认明确TTL0.5秒。两层遮挡保持可能累计约1秒，感知链整体超时则由规划安全守卫停车。
- 首个可动/未知对象速度为0时，新增`velocity_unknown/observed_count/max_unknown_speed`属性：snapshot按最大障碍速度×源年龄扩大当前位置包络，规划器还按同一最大速度扩大未来可达域。KF只有一个真实位置观测时通过现有type='unknown'传播状态，预测miss的鲜活array不能充当第二次真实量测；第二真实观测建立速度prior或确认静态后释放。尚未建立速度时的miss可达域保存在KF输出尺寸中，防止下游新header刷新抹掉过去不确定性；`physical_length/width`区分真实尺寸和临时未知运动增长，避免首次等待永久放大对象。

## 与规划 agent 的最终API

```python
tracker.update_measurements(detections, now=None, measurement_dt=None, stamp=None)
obstacles = tracker.snapshot(now)   # predict_tracks is an alias
```

`detections` 保持 `(x,y,type,L,W,velocity_heading,speed,id)`；可选第九项为真实shape_heading。`now` 是测量发生时的核心时钟，而不是处理完成时刻；snapshot使用当前核心时钟。主规划core已按新输入revision更新一次，按source_age把测量时间回推，再按snapshot外推到当前时刻。调用方不能直接读 `tracker.tracks` 把旧量测锚点当作当前估计。

新增纯world表示API：

```python
parts = split_world_cluster(cluster, tile_size=.5, extent_threshold=2.,
    occupancy_threshold=.5, protected_pose=(rx, ry, yaw), half_length=.6, half_width=.4)
# parts: [(tile_key_or_None, cx, cy, L, W)]，都是map AABB
output = WorldObstacleRepresenter(tracker).step(
    world_clusters, stamp, reference_origin=(rx, ry), vehicle_pose=(rx, ry, yaw))
# output: [(id, x, y, speed, velocity_heading, type, L, W)]，ROS布局未改
```

`reference_origin`用于world聚类/观察范围的车位置，`vehicle_pose`仅用于确认bbox空腔是否不占实际车身；该触发净距不是DWA安全margin。首帧unknown/移动tile继承父运动，静态确认后tiles用block。超过max_scene_tiles时抛出`RepresentationLimitError`，ROS适配器停止刷新并明确记录原因。

## 诊断

- `/car_mode_ready` 记录-1/0/4，按最新实车确认比较指令4自转与反馈4自转、ACK0与反馈0。旧代码注释中的反馈2不会被当作正常到位，否则会掩盖底盘枚举错误。
- `nearest_obs_dist` 改为map对象中心减去最新robot定位的中心距，避免使用地图原点距离。它仍不是车身净空；真正净空需用规划足迹算。
- 原始 `/rslidar_points` 只记录frame/点数/源stamp/源年龄，不解析或重新发布点云。`loc_source_age/obstacle_source_age/cloud_source_age` 随快照持续增加；接收年龄独立命名为`*_arrival_age`，避免旧rates覆盖源年龄。
- 保留完整障碍L/W/速度列表；planner新增字段自动落为 `diag_*`，tracker新增字段自动落为 `tk_*`，包括fault和physical_ready，不再丢掉仲裁失败、年龄与尝试计数。

## 验证

`docs/validation/test_perception_safety.py` 的 **38条纯Python/numpy回归全部通过**，覆盖源时刻/外参/全周过滤、bbox包络、真实dt和关联、unknown首帧可达域、TTL、倒车静态速度、center0无二次偏移和4→4到位协议。最终tiling补充raw U全部点覆盖且车腔不占据、厚高占空U、±.8米窄U矩形空腔、point顺序独立、15帧ego旋转world ID/中心稳定且v0、动态长簇继承父motion/unknown、small/dense不拆、tile上限fail-closed、所有AABB类型实测尺寸，以及侧墙y=.42保留/self y=.39与x=.59剔除。

运行：

```bash
/tmp/rda-dwa-validation-20261106/bin/python docs/validation/test_perception_safety.py
/tmp/rda-dwa-validation-20261106/bin/python docs/validation/check_ros_adapters.py
```

第二条采用fake ROS，仅验证当前active节点构造和YAML参数读取；bridge/tracker/lidar均没有undeclared或unread参数。它不证明DDS行为或ROS构建成功。修改文件py_compile通过。一个20,000点聚合到单个障碍的示例在本机约0.003秒，只展示高密度重复点经过体素聚合后成本受限，不作为车机所有场景耗时保证。

最终全周性能见 `docs/validation/profile_perception.py --profile` 和 `perception_performance.json`。当前包含filter、world变换、world cluster、父KF和tile表示；80k dense约.234秒/1对象，100k random dense约.250秒/1对象，80k长墙约.083秒/49瓦片，247点raw U约.00068秒/31瓦片（world旋转.35场景；world0的U回归为26瓦片）。此前旧实现仅filter+cluster约.330秒，profile定位到重复扫描visited候选和四倍voxel角点；优化与新表示都没有缩12米范围。最终测量仍排除ROS点云解码、markers、DDS和车机CPU争用，不能保证10Hz；实际端到端年龄必须验收，超预算继续profile/有包络的降采样，不应静默缩观察范围。

## 文献与剩余实车边界

`literature.md` 已完成原论文/官方文档检索并阅读可达正文，包括[CMU DWA作者页](https://www.ri.cmu.edu/publications/controlling-synchro-drive-robots-with-the-dynamic-window-approach-to-collision-avoidance/)、[RPP原论文](https://arxiv.org/html/2305.20026v1)、[Nav2 RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)、[Rotation Shim](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/)、[Velocity Smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)与[VO原摘要](https://www.osti.gov/biblio/665350)。原始DWA的安全停车、RPP执行指令检查及shim角度滞回是工程依据，机械3秒ready协议由本车单独设计。

仍需日志/实测：传感器时钟与ROS时钟同源、实际rawframe、雷达安装倾角与高度、map修正小幅跳变是否仍影响方向，以及倒车和全周转向的实际跟踪误差。地面固定z裁剪适合当前平地假定，坡道应引入姿态补偿/地面平面估计；真实物体部分遮挡和突然加速/折返仍可能超出匀速预测及固定增长包络。首个观测无法确定运动方向，当前可动分类和保守包络降低迟滞，但不是未知运动可达集的完整安全证明。未做ROS2构建、bag回放或实车安全验收。

12米是软件观察范围，必须确认雷达在该距离对目标的有效点数和探测可靠性、全周遮挡以及计算耗时。3秒换轮和约6秒撤销恢复期间车辆有不能立即改变运动方式的物理限制；扩大观察范围不能声称保护车辆免受任意人主动撞击。
