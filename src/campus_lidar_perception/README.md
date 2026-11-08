# 独立校园激光雷达感知包

针对 AGX Orin 64GB、JetPack6.2 基线、TensorRT10.3、ROS2 Humble 和 RS-Helios-5515 32线/10Hz。此包独立于现有检测算法；已有检测源码与已有 launch 均未改写。新全链路只启动本包的感知节点，使用既有定位、规划和控制代码。

## 已确认的两套对照

| profile | 组成 | 用途 |
|---|---|---|
| `centerpoint_geometry` | CenterPoint base v4.1 + Patchwork++ + 几何占用/聚类 | 几何基线，地面通行条件不等于道路语义 |
| `centerpoint_harpnext` | 相同 CenterPoint base + HARP-NeXt nuScenes32×480 + 相同几何支路 | 语义道路约束；语义失效时栅格保留UNKNOWN |

用户已确认按“几何 vs HARP-NeXt语义”对比。`centerpoint_harp` 只是后一配置的别名，不代表另一个HARP模型。两套检测权重、跟踪逻辑和几何参数一致；语义模型额外运行会影响GPU负载/实际输出时序，应测并行表现。

两套配置的3D对象检测均来自CenterPoint和几何支路；HARP在当前版本用于可行驶语义约束，不将逐点语义直接冒充3D实例框。

## 输入输出与下游兼容

- 输入 `/rslidar_points`：PointCloud2，按真实字段类型、offset、point_step、row_step和大小端读取XYZI，不假设RoboSense消息内存布局等于Autoware。
- 输入 `/localization`：Odometry，沿用 `latest_available` / `latest_topic` / `tf` 定位约定。当前实车配置将 body 看作车体几何中心，`body_to_base_link_x=0.0`。不要把旧launch的−0.15m偏移混进当前规划配置。
- 输出 `/rda/obstacles`：原有 `rda_dwa_real_msgs/ObstacleArray`，map坐标、整数源时间戳、稳定id、type、xy、长宽、运动heading/speed、独立shape_heading。无需改消息或下游订阅。
- 输出 `/campus_perception/tracks`：JSON，另含semantic_label、z/height、vx/vy、KF协方差、观察年龄、预测标志、时延、模型profile。
- 输出 `/rda/drivable_area`：base_link系OccupancyGrid，0=自由候选、100=占用、−1=未知；只有实际观测满足地面条件的格子可变自由。语义方案还要求新鲜道路标签。
- 输出 `/rda/perception_diag` 与 `/rda/obstacles_markers`：诊断和RViz矩形。

现有规划源码目前消费ObstacleArray，未发现它直接订阅drivable_area；新包保留栅格接口与记录，但不宣称既有规划已自动按语义道路约束选路径。

跟踪使用CV状态[x,y,vx,vy]、Joseph协方差更新和SciPy匈牙利算法，没有贪心替代。真实多帧观测确认运动后才输出速度；未确认目标保持最新实测位置/框和零速度。UNKNOWN几何目标确认运动时，旧规划固定枚举使用pedestrian作为保守动态代理，rich输出仍标记UNKNOWN和dynamic_unknown_proxy，不冒称语义识别。漏检时不刷新观察时间，超时删除。定位跳变重置跟踪，id不复用。

### CV / IMM可选跟踪

`tracking_filter`启动参数可选`cv`（默认）或`imm`。未确认/静止目标保持CV，确认运动后使用所选滤波器；切换保留当前状态/协方差，真实观测确认静止后恢复CV。IMM是共同6维[x,y,vx,vy,ax,ay]的CV＋CA模型银行，包含条件状态/协方差交互、创新似然模型概率更新及模型间不确定性融合，模型概率随观测变化。CA是笛卡尔加速度近似，未添加显式CTRV转弯模型。结构参考[FilterPy IMM文档](https://filterpy.readthedocs.io/en/latest/kalman/IMMEstimator.html)，本包独立实现。

转移概率按每个实际处理的预测步应用，不是每秒概率，比较不同处理Hz时需注意。max_speed是硬预测速度/位移界：预测与校正都限速，每步融合预测位置位移不超过max_speed×dt；IMM通过公共位置修正保留模型间偏移和总协方差。定位重定位检查同时覆盖XYZ和完整姿态，处理过程中发生纯旋转跳变也拒绝发布旧坐标结果。

```bash
ros2 launch car_launch run_campus_all.launch.py \
  profile:=centerpoint_harpnext tracking_filter:=imm fastlio_map_dir:=/你的既有地图目录

ros2 launch campus_lidar_perception perception.launch.py tracking_filter:=cv
```

ROS直接运行时可通过`--ros-args --params-file`使用YAML。launch中同名`tracking_filter`参数优先于YAML，默认为cv。参数在启动时固定，运行中设置会明确拒绝，避免设置成功却没有改变后台滤波器。IMM转移自保持概率默认0.95、初始CV概率0.7、CA白噪声jerk方差6.0、加速度初始方差4.0，可在perception YAML中调整。rich追踪消息增加configured_filter、active_filter、filter_model_probabilities、acceleration；原ObstacleArray完全保持兼容。

MarkerArray在`/rda/obstacles_markers`中显示3D框、ID/语义/速度/活动滤波器及IMM概率文字、1秒速度箭头和最近20个估计状态的轨迹。动态目标绿色、静态橙色、预测帧灰色；删除只作用于自己的命名空间，标记随数据超时过期。RViz选择`Tracked obstacles`即可查看，2D Goal工具使用原`/move_base_simple/goal`。

定位保留XYZ与roll/pitch/yaw用于3D框变换和历史帧补偿，再投影为旧规划的map平面包围框。rich的score表示当前观测来源的分数；semantic_score/semantic_observation_stamp/semantic_age单独表示最近语义观测，UNKNOWN更新最多记忆1秒旧语义。静止时type会成为block/static_large，因此type是规划运动/形状枚举，不能当作神经网络分类。预测目标的数组header表示当前估计状态时间；逐目标观察年龄只在rich输出，旧规划接口无法逐目标辨认该年龄，预测保留最多0.4秒。

## 模型已经下载

`models/centerpoint_base/` 包含官方encoder/head ONNX及参数文件，固定Autoware v4.1 revision `494c8171def40bd36cc2feb323e0a5acbfab132b`。

`models/harpnext_nuscenes/` 包含原始 `ckpt_best.pth`、net/classes配置，以及等值提取的纯tensor `weights.pth`。原始checkpoint中训练统计含NumPy对象；提取仅保存net的293个tensor，已逐tensor验证完全相同，未训练、微调或量化。

模型来源、大小和下载文件记录在 `models/manifest.json`，真实ONNX输入结构和提取检查在 `models/input_contracts.json`。HARP网络必要源码已随包提供，保留原始 `.upstream` 文件；仅改包导入路径和scatter操作接入。缺少torch-scatter时使用原生Torch等价推理归约，实际时延要另测。

CenterPoint base 输入按上游当前实现准备 **XYZ＋time_lag**，特征张量 `[N,32,9]`；HARP输入为 **XYZI**。base固定480×480网格、0.32m柱格，dim头顺序是[w,l,h]，ROS形状yaw为`-raw_yaw-pi/2`。默认使用当前＋1历史雷达帧，用定位补偿历史自车运动；动态目标仍可能拖影，`past_frames:=0`需要通过perception参数文件修改后对照。

本包输出使用30m径向ROI、同类0.5m circle/10m内IoU>0.1抑制，最多500框；未应用下载的area class remapper。这是两套方案共同的本地后处理，不套用官方后处理的精度数字。地面使用一个局部拟合平面，默认±6cm凸起/下台阶视为占用，自由格至少2点且全部高度残差≤3.5cm，占用外扩1格；可靠平面缺失时整帧不向规划发布。

预处理复核修复：有足够垂直支持的细杆也输出UNKNOWN框；TF/框/点/完整HARP logits及置信值统一检查有限性；语义缓存使用3D XYZ近邻而非仅XY；大map坐标变换保持float64；CPU柱格边界与CUDA一致夹紧。栅格扣除当前DNN和追踪状态投影框，因此车底可见地面不会覆盖障碍占用。目标TTL在关联前执行，长时间漏观测后不复用已过期ID。两个worker与发布检查共享配置的max_source_age。

模型输入必须使用物理sensor_frame原点的原始XYZI，不能把已转换到map/base_link的点云直接当raw输入。自车点从两模型及几何输入一起去除，使用base_link下有限车体体积，默认z范围[-1.0,0.6]m，可通过self_filter_min_z/max_z调整；高度范围是初始配置，需实车标定，不能无限竖直去除树枝等上方障碍。

HARP保持32×480和训练视场−30°～+10°，先做W1920的邻点插值。Helios5515超出训练视场的点不挤入范围图边缘，标签为UNKNOWN；几何支路仍使用这些点。垂直视场/安装位置的域差异没有因为免训练而消失。

## 在 Orin 上构建

当前开发主机是macOS，没有ROS2/CUDA；没有生成假TensorRT engine。engine和本地`.so`必须在目标Orin构建。以下命令从你车机的工作空间根目录执行。

1. 保留模型与整个新包。确认系统已有Humble、CUDA12.6、TensorRT开发头文件和CMake。HARP需要能实际执行CUDA的NVIDIA Jetson PyTorch；按[NVIDIA官方安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)选与JetPack/CUDA相符的wheel及cuSPARSELt依赖，不能用PyPI CPU wheel替代。若已有兼容wheel路径/URL，可在构建时设置`JETSON_TORCH_WHEEL`。

```bash
bash src/campus_lidar_perception/scripts/build_orin.sh
source src/campus_lidar_perception/.venv-orin/bin/activate
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select campus_lidar_perception rda_dwa_real_msgs car_launch
source install/setup.bash
```

保持该venv激活。构建脚本在隔离环境中安装所需Python依赖与colcon，并编译仅依赖CUDA/TensorRT的本地推理库；不安装整个Autoware。源码的既有规划、控制、驱动包需按原流程已构建。`car_launch/package.xml`新增了本包依赖，已有旧launch内容不变。

2. 先生成并验收本机TensorRT engine，避免把首次构建时间混进实车对照。第一次可能需要几分钟。

```bash
PYTHONPATH=src/campus_lidar_perception python src/campus_lidar_perception/scripts/verify_orin.py \
  --model-dir src/campus_lidar_perception/models \
  --library src/campus_lidar_perception/campus_lidar_perception/lib/libcampus_centerpoint.so \
  --output diagnostics/campus_perception_orin_smoke.json
```

该检查运行真实TensorRT和HARP GPU推理，并比较CUDA柱特征与CPU参考。合成输入只验证接口/算子，不评估校园检测精度。若检查失败，保留错误输出修复后再联调；不要把“模型文件下载成功”当成此步骤通过。

## 新启动入口

### 仅检测，复用已运行的定位

```bash
ros2 launch car_launch run_campus_perception.launch.py profile:=centerpoint_geometry
ros2 launch car_launch run_campus_perception.launch.py profile:=centerpoint_harpnext
```

以上两条分开运行。默认不启动雷达/定位；可用`with_sensors:=true`单独启传感器，但仍需要定位输出。若旧检测还在发布/rda/obstacles，新包会暂停自己的规划输出并报多发布者；停止旧检测节点或把新输出重映射到影子话题，旧检测文件无需删除或修改。

独立入口`ros2 launch campus_lidar_perception perception.launch.py`不再依赖car_launch存在；可选sensors.launch只在请求启动驱动时查找已有lidar/imu/rs_to_velodyne包。新包不会修改原传感器启动文件。

### 仅定位（含传感器）

```bash
ros2 launch car_launch run_campus_localization.launch.py \
  fastlio_map_dir:=/你的既有地图目录
```

复用fastlio_mapping、global_localization.py、transform_fusion.py。没有重写定位算法。默认从`FAST_LIO_MAP_DIR`或fast_lio安装目录/PCD查找global_ss.pcd与global1226.pcd；可分别覆盖`map_cloud_file`、`localization_map_file`、`initial_pose_file`和`fastlio_config`。本工作空间未包含这些车机驱动/定位源包，也未包含实际PCD地图，所以不伪造地图或替换你的定位数据。

### 定位＋检测（不启动规划/底盘）

```bash
ros2 launch car_launch run_campus_localization_perception.launch.py \
  profile:=centerpoint_harpnext fastlio_map_dir:=/你的既有地图目录
```

### 全链路

```bash
ros2 launch car_launch run_campus_all.launch.py \
  profile:=centerpoint_geometry fastlio_map_dir:=/你的既有地图目录

ros2 launch car_launch run_campus_all.launch.py \
  profile:=centerpoint_harpnext fastlio_map_dir:=/你的既有地图目录
```

分开运行以做对照。链路为雷达/IMU→原FAST_LIO定位→新感知→原bridge_node规划→原traj_tracker_node控制→car_ctr。不会包含原lidar_track_node或obs_check节点。底盘默认与旧全链路一样启用；只做联调/回放时加`with_car_ctr:=false`，已有传感器运行时加`with_sensors:=false`。

加`with_rviz:=true`使用新包的对照视图，显示原始点云、跟踪框和可行驶栅格。原有RViz配置未改写，可用`rviz_config:=...`选择其他配置。

机器人初始位置、目标发布、现有规划参数和模式控制仍按原系统使用。所有launch支持`use_sim_time:=true`供rosbag `--clock`回放；若定位已经在跑，只使用检测入口，不重复启动定位或TF。

## 记录与对比

默认新记录器在`~/record_data/campus_perception`下按profile创建独立目录，保存`events.jsonl`与压缩栅格NPZ。可用`perception_record_dir:=...`指定车机NVMe目录，或`with_perception_record:=false`关闭。全链路还保留旧diag_recorder及其原始点云rosbag记录，互不修改。

使用同一批原始rosbag、相同定位、相同规划配置分别运行两profile，首次engine构建和预热完成后再评估。汇总命令：

```bash
ros2 run campus_lidar_perception campus_perception_compare \
  /记录/centerpoint_geometry_某次运行 /记录/centerpoint_harpnext_某次运行 \
  --output /记录/comparison.json
```

汇总含实际接收Hz、全处理/推理/几何/跟踪/输出年龄P50/P95/P99、对象数、语义可用比例和自由/占用/未知格数。自由格更多不代表精度更高；行人/自行车召回、误检和误放行需要标注校园数据。

语义支路另记录成功帧数、HARP输入准备/预测/返回时间与包含缓存建立的总时间，不把这些数值冒充网络前向单独耗时。即使主检测当时没有新结果，成功语义结果也会单独进入诊断记录与汇总。

时延统计以收到的成功tracks消息为样本，包含规划发布被抑制的结果，失败帧不进入该分布；另列发布尝试帧数及诊断原因计数。接收Hz不是含初始化空窗的全运行规划到达率。语义可用表示当前云有空间匹配标签，并另列当前有限点云的匹配比例；有缓存但完全不匹配不会报可用。道路标签仅接受driveable_surface类10，不接受sidewalk/other_flat作为道路。

CPU无ROS跟踪回放入口：

```bash
PYTHONPATH=src/campus_lidar_perception python -m campus_lidar_perception.replay \
  --input /检测数据.jsonl --output /跟踪结果.jsonl
```

输入每行是`{"stamp":秒,"detections":[{"x":...,"y":...,"length":...,"width":...,"yaw":...,"label":"PEDESTRIAN"}]}`，坐标已经在map系。`--smoke`是明确标记的合成测试，不能作为真实检测结果。

回放可加`--tracking-filter imm`；实车记录目录和metadata同时记录profile与tracking_filter，方便分别比较几何/语义和CV/IMM。滤波器概率不是检测类别置信度。没有标注轨迹真值时，汇总不宣称哪个滤波器更准确。

## 实车前的实际配置检查

`config/perception.yaml`中的sensor_z=0.45只沿用旧TF配置，ground_reference_z=−0.36沿用现有记录参考。根据真实安装、header.frame_id、DIFOP、强度范围、逐点时间、去畸变和定位参考验证后调整；不要从配置数值反推硬件已经标定。网络输入不统一除以255。

通过launch启动时，`body_to_base_link_x`和`sensor_x/y/z/roll/pitch/yaw`以同名launch参数为准，并同时用于新定位TF和新感知；只改YAML不会覆盖这些启动参数。位置单位米，旋转单位弧度。其余地面/跟踪参数可通过`perception_config`传入自定义YAML。仅检测时已有TF优先于外参回退，因此已有TF也需与实测安装一致。

两个工作线程各仅保留最新待处理帧，避免GPU慢时累积延迟。语义默认5Hz，主障碍支路随10Hz雷达处理；实际帧率尚未在本车测出。超时、无可靠地面、无定位、模型异常、过量柱格或无法解码时停止发布新的规划障碍，原下游超时保护继续工作；不会用假空列表报健康。有效路面确实没有物体时允许发布真实空障碍帧。

初始化错误和运行丢帧可从/rda/perception_diag观察。当前路径未做逐点运动去畸变；历史帧自车补偿与逐点去畸变是不同步骤。车机系统时间与雷达/定位时钟域需正确配置，在运行中发生大幅校时后重启感知并检查时间诊断。

## 当前验证边界

本机已完成真实HARP权重加载/CPU前向、ONNX结构检查、真实Patchwork++路面推理、KF/匈牙利/坐标/源时间/旧规划回调与launch结构测试。ROS launch运行、DDS、TensorRT10.3编译、CUDA特征对照、GPU并行10Hz和真实校园精度仍需Orin验收。没有把这些未执行检查报告成通过。
