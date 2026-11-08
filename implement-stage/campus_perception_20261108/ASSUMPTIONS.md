# 接口与语义假设
<!-- ASK mode: never; 用户优先指令下异步澄清 HARP 命名 -->

| ID | 未明确项 | 选择 | Class | Source |
|---|---|---|---|---|
| A001 | HARP与HARP-NeXt是否是两个模型 | 用户确认对比几何方案 vs HARP-NeXt语义方案：centerpoint_geometry 与 centerpoint_harpnext；harp仅保留为同模型别名 | semantic | user |
| A002 | 兼容哪套规划 | 当前 /rda/obstacles 的 ObstacleArray，map 系；新包同时发布丰富 JSON 跟踪，旧接口不增加字段 | interface | default |
| A003 | 定位引用点和时钟 | 跟随当前 real_vehicle.yaml：latest_available map->body，body_to_base_link_x=0；新定位launch与新检测默认都保持0，可显式改并须规划一致 | semantic | default |
| A004 | 推理环境 | 目标 Orin/JetPack6.2/TRT10；本机只验证纯逻辑与模型；CUDA单独原生库，HARP PyTorch，不引入整个Autoware包族 | interface | default |
| A005 | 模型输入 | 根据实际ONNX和上游源码实现；CenterPoint base 使用XYZ+time_lag，HARP使用XYZI；预训练shape固定 | semantic | default |
| A006 | 类别外障碍 | 所有配置都保留几何地面/非地面占用；不得因为DNN无框而发布自由区 | semantic | default |
| A007 | 点云积帧 | CenterPoint沿用官方默认当前+1历史帧；用定位变换补偿历史自车运动，单帧可配置；HARP单帧 | semantic | default |
| A008 | 失效和消息时间 | 新帧获取失败/定位掉线/推理异常不发布伪空障碍；输出保留雷达源stamp，旧下游超时保护仍有效 | interface | default |
| A009 | 时序与速度 | 最新帧后台工作线程；CV Kalman+必需SciPy匈牙利；速度需多次观测确认，unknown保持实测框、零速度；暂时漏检预测不刷新观测时间 | semantic | default |
| A010 | 全链路范围 | 新launch直接启动现有bridge_node、traj_tracker_node及可选car_ctr，不引入含旧检测器的rl_dwa_car.launch；驱动默认启动与原全链路一致 | interface | default |
| A011 | 地面参数 | 当前 ground_reference_z=-0.36 位于旧感知数据参考；新几何支路在雷达原始参考运行，安装高度参数0.45仅沿用旧TF配置，可覆盖 | semantic | default |
| A012 | 可行驶区被现有规划消费 | /rda/drivable_area保持已有输出接口；现有规划实际只消费ObstacleArray，新增grid不宣称规划已自动使用语义栅格 | interface | default |
| A013 | 测试证据 | 算法合成回放只验证关联/坐标/接口；checkpoint实加载验证权重匹配；无ROS/CUDA处明确未执行 | semantic | default |

| A014 | HARP的torch-scatter依赖 | 优先torch-scatter；没有时以原生Torch scatter_reduce/index_add实现这两个推理操作，记录后端；保持checkpoint参数，测速单独验收 | interface | default |
| A015 | HARP预处理和视场 | nuScenes H32/W480，先用W1920的邻点插值；Helios训练视场外点保持未知，几何仍使用全点云；CPU/原生Torch路径均需与官方对照 | semantic | default |
| A016 | CenterPoint优化实现 | 独立CUDA预处理/Scatter，TensorRT10 FP16两段推理，CPU参考预处理与解码测试；实际GPU未测不冒称等价官方精度 | semantic | default |

| A017 | 官方checkpoint含NumPy训练统计而旧Jetson依赖可能不兼容 | 本机以限定NumPy标量加载提取net为纯tensor weights.pth；不改变任何权重、不训练；原始checkpoint仍保留 | interface | default |

| A018 | 点云规划输出多发布者 | 新全链路不启动任何旧检测节点；若同一planner话题检测到多个发布者则暂停新包规划输出，可通过obstacles_topic重映射作影子对照 | interface | default |
| A019 | 感知两支路调度 | 主检测/几何线程与HARP线程分离，各只保留最新待处理帧；语义默认5Hz，缓存转换到map匹配当前观测，超时只输出unknown语义区 | semantic | default |
| A020 | 原车工作空间未包含定位和雷达驱动源码 | 新定位launch复用车机已装fast_lio可执行程序，地图文件改成显式参数并支持旧部署路径；本机只能检查launch契约 | interface | default |

| A021 | 初始定位地图路径 | 新launch参数化fastlio_map_dir，默认FAST_LIO_MAP_DIR或fast_lio安装share/PCD；保留既有定位可执行程序，缺文件报错，不伪造地图 | interface | default |
| A022 | TensorRT与PyTorch环境 | 独立venv使用目标系统ROS/TRT，JetsonTorch由官方兼容wheel安装/验证；不在macOS装CUDA，也不使用PyPI CPU torch冒充GPU | interface | default |

| A023 | CenterPoint yaw约定 | 按官方ros_utils转换ROS形状角yaw=-raw_yaw-pi/2；length/width遵循dim头[w,l,h]；运动角仍由KF速度独立计算 | semantic | default |

| A024 | 移动UNKNOWN如何通过旧固定枚举传速度 | 只有KF真实多次观测确认运动后，规划枚举使用pedestrian作为保守动态代理，rich语义仍UNKNOWN且显式标识；未确认unknown仍固定实测框零速 | semantic | default |
| A025 | 下游源时间精度 | 保留原始header整数纳秒到ObstacleArray/Grid，不从浮点秒重建；KF dt仍用秒 | interface | default |

| A026 | 实车比较记录 | 新包单独记录tracks、drivable grid与语义/处理时延；旧记录器仍可记录原始点云bag；统计不使用无真值数据宣称精度 | interface | default |

| A027 | 外参覆盖优先级 | car_launch同名外参启动参数同时送新TF和感知，优先于YAML；仅检测已有TF优先于参数回退；body引用点须与规划同步 | interface | default |

| A028 | HARP融合范围与类别 | 仅约束drivable grid，只有nuScenes类10 driveable_surface可成为道路；不覆盖实例检测/追踪语义类别，现有规划不消费该grid | semantic | sweep |
| A029 | 车辆姿态参与补偿 | 审查发现yaw-only遗漏；改为保留定位roll/pitch/yaw及XYZ做刚体变换，规划消息仍为map平面包围框；未做逐点deskew | semantic | sweep |
| A030 | CenterPoint后处理 | 本地径向30m输出ROI、同类0.5m circle或10m内IoU>0.1 NMS，最多500框；未应用下载的area class remapper，不套用官方基准精度 | semantic | sweep |
| A031 | 两方案地面与占用口径 | 单局部拟合平面；支持不足拒绝整帧含DNN结果；默认±6cm障碍/下台阶，自由格>=2点且全部高度残差<=3.5cm，占用外扩1格 | semantic | sweep |
| A032 | 语义记忆和score | UNKNOWN观测最多保留1s语义，分开记录当前measurement score与semantic_score/semantic_stamp/age；静止规划枚举由运动/尺寸决定，不能当语义类别 | semantic | sweep |
| A033 | 对比统计分母 | 时延/Hz基于收到的成功tracks消息，包含planner抑制帧、排除失败；另列发布尝试帧数、错误诊断计数和当前语义空间匹配比例；不是全运行规划到达率或精度 | semantic | sweep |
| A034 | 预测目标旧接口时间 | ObstacleArray header表示当前云下状态估计时间，单目标最后观测时间只在rich JSON；旧规划无法区分逐目标观测年龄，预测有效期最多tracker_lost_time | interface | sweep |
