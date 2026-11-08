# 决策记录
<!-- ASK=never；缺少GitHub写权限按外部前提处理 -->

| ID | 请求未明确项 | 选择 | Class | Source |
|---|---|---|---|---|
| I01 | IMM子模型 | CV+CA线性Kalman银行，公共6维状态；转移概率混合、创新高斯似然、均值/协方差融合，模型概率公开 | semantic | default |
| I02 | 动态激活 | 静止/未确认使用CV，真实观测确认运动后使用所选cv/imm；模型切换保留状态协方差，静止恢复CV | semantic | default |
| I03 | 参数接口 | tracking_filter=cv或imm，默认cv保留旧行为；ROS参数/YAML/所有感知launch与离线回放均可选择 | interface | default |
| I04 | RViz内容 | 同一MarkerArray话题发布框、ID/语义/速度/滤波器文字、速度箭头、最近轨迹；寿命有限、删除仅限自己命名空间 | interface | default |
| I05 | Git上传范围 | 完整新包源代码、native CUDA、配置、测试、模型下载/提取脚本、模型配置与来源、car_launch集成和消息接口；不提交venv/缓存/记录数据，二进制权重保留本地并通过固定下载入口重建 | interface | default |
| I06 | 已有接口 | /rda/obstacles和map系字段不变；滤波模式/IMM概率/加速度仅添加rich字段；旧检测文件保留 | interface | user |
| I07 | 验证边界 | 合成轨迹检验数学/关联/机动响应与数值稳定，不当作实车精度或10Hz证明；GPU验证仍由Orin执行 | semantic | default |

| I08 | 窄立柱过滤 | 满足采样支持时按3D跨度而非仅XY跨度保留，最小0.1m平面框；数据不足时仍UNKNOWN，不能用occupied grid代替旧规划障碍 | semantic | review |
| I09 | 语义缓存对应关系 | 改用map XYZ的0.3m近邻，保留双时限与FOV约束；在大map坐标中保留float64到GPU输入边界 | semantic | review |
| I10 | 融合可行驶栅格 | 观测地面/道路之外扣除DNN和当前跟踪状态的投影框，包含有界漏检预测；窗口栅格化不做全图逐框扫描 | semantic | review |
| I11 | 自车去除和参考原点 | 所有模型使用同一物理sensor_frame原始输入；拒绝直接当raw的map/base_link云；按base_link有限车体体积去除自身，z范围默认[-1,.6]可调，未独立测量 | semantic | review |
| I12 | 配置和异常检查 | 启动参数冻结并拒绝运行中伪生效的修改；worker用同一max_source_age；非有限TF/点/模型输出/形状直接失败，空目标不跳过校验 | interface | review |
| I13 | 跟踪失效身份 | TTL过期在关联前删除，避免缺帧后重用旧ID；滤波模式切换不丢当前滤波状态及协方差 | semantic | default |

| I14 | 独立驱动入口 | 新包自带可选sensors.launch并仅在请求时解析已有驱动包，消除单独检测对car_launch的依赖；原run_sensor文件保持不变 | interface | default |

| I15 | IMM预测的物理界 | 预测和观测更新均限制速度；每个预测步的融合位置位移不超过max_speed*dt，IMM以公共位移修正保持模型间偏差/协方差 | semantic | review |
| I16 | 重定位跳变 | 同一SE3平移/完整姿态不连续检查用于pipeline重置和发布前检查，防止旋转跳变中的在途结果进入规划 | interface | review |

| I17 | IMM转移概率时间解释 | 按每个实际处理的预测步应用，非每秒概率；过程噪声/运动方程使用真实dt，掉帧会影响有效模式停留时间 | semantic | review |
