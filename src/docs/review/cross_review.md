# 阶段三：感知 agent 跨模块复核

2026-11-06，针对最新bridge、planner、tracker和定位/感知接口做增量交叉复核。未把正在修复的startup平滑偏0、ACK停车角度归0再次当作最终缺陷。

| 交叉项 | 核对结果 |
|---|---|
| time0与源年龄 | core输入初值为-inf，不把到达时刻0当作未收到；ROS源stamp0按硬件链路无效处理。lidar→bridge保留cloud源stamp，bridge按source_age回推测量时刻，TrackManager snapshot再外推当前时刻；重复源帧不刷新安全年龄。 |
| 运动与形状 | 最终所有active测量均为map AABB，planner统一shapeYaw0，fusion保留>=.05米实测尺寸，不用static_large/.75米或unknown方框永久填空腔。动态/unknown使用circle和motion/reachable界，速度方向不决定几何长轴。 |
| U墙与trap表示 | 大/稀疏/细长world簇按.5米固定tile拆局部AABB，父KF估motion再给tiles；静止tile中心/ID稳定，动态/unknown不被固定网格中心伪装静态。厚U即使占空高，只要原点不占实际车身矩形也强制拆。窄U±.8墙的实际矩形净空.4米用于表示触发，没有圆+.2误挡；DWA独立硬撞检。raw U全部点覆盖且不占车腔，不能承诺任意死胡同liveness。 |
| 自剔除近场 | outside=0，strict实际1.2×.8内部；y=.42外侧墙保留，y=.39/x=.59真实self点剔除，修复原.05外扩盲带。轮型/反射细mask仍需实测。 |
| 全局定位reset | runtime /initialpose可触发大map修正；已协调planner/control独立物理连续性门与锁存fault。lidar SourcePoseBuffer也采用.4m/s、.8rad/s、.15m、5°裕度、源dt上界=定位timeout；首次位姿豁免正常初始化，跳变后新stamp不解除fault。正常.8rad/s10Hz旋转回归通过。 |
| 定位算法保留 | 按用户最终要求，global_localization/transform_fusion及FAST_LIO CMake逐字恢复原始备份；保存/加载、ICP、初始化和定位发布逻辑保持原样。仅静态TF及下游补偿统一归零。原fusion重复源stamp由下游守卫处理，不修改定位生产者。 |
| 空帧与失效 | 有效空cloud执行KF miss与时间TTL；缺字段、无效数据格式、解码错误、旧stamp、定位无效不发布空场。增加structured numpy record和float32/float64 fallback兼容，避免解析异常变成假清场。 |
| 动态初始时序 | 首次速度0但0.5秒晚到的新可动对象不能按静止外推：TrackManager标记velocity_unknown并按max_unknown_speed×年龄扩当前位置，规划同属性扩未来。KF预测miss保持unknown语义与历史可达域，直到第二真实量测建立速度prior；已确认静态可直接释放，未知增长不作为永久物理尺寸。 |
| blind region与模式时域 | 全周12米、pass_x/y±12米，支持1.5m/s×(两次3秒+.6秒reaction)+约1米足迹余量的观察时域。最终80k/100k dense完整pure感知.234/.250秒且各1对象；长墙80k .083秒/49tiles、U247 .00068秒/31tiles。不縮范围，实际有效12米量程、物理遮挡、源年龄和车机端到端耗时仍需验收。 |
| 机械模式与参考 | command/Path同epoch配对、旧定位独立超时、SPIN航向目标锁存已核对。用户最新确认命令4与真实feedback4一致，反馈表示到位，自转UDP速度乘-2方向正确；诊断已改为4→4，2不能伪装正常到位。 |
| 用户最新确认后的几何与倒车 | 实车1.2×.8米、轴距.69米；定位已经是几何中心，雷达也位于中心，body_to_base_link_x=0且静态TF body→base_link x=0，不能重复偏移-.15。lidar自剔除center0/halfLength.6与planner footprint/spin offset0统一。-.3m/s倒车在连续性门内；静态map障碍不会因倒车出现虚假速度，负障碍世界速度用正speed与对应heading编码。 |

验证命令：

```bash
/tmp/rda-dwa-validation-20261106/bin/python -m pytest -q tests rda_dwa_real/test docs/validation/test_perception_safety.py
/tmp/rda-dwa-validation-20261106/bin/python docs/validation/check_ros_adapters.py
```

最终感知 **38条通过**；本agent最后完整离线验证为 **127 passed，8 subtests passed**，最终总数由主agent冻结后统一重跑确认。CLI YAML构造检查active bridge/tracker/lidar均无未声明或未读取配置，实际lidar半径12米、x/y±12米、outside0、中心0，tiling参数已接线。fake ROS构造不是ROS2构建或DDS验收。

严重map跳变的恢复顺序：可靠停车，确认全局初值/TF，重新定位，重启局部感知/规划/控制链以清除旧map下的轨迹、障碍和锁存状态，再开始新目标。单纯不断收到新timestamp不会恢复运动。局部重启与机械模式就绪的实际顺序须实车验收。

仍需实车确认：真实rawframe/时钟、传感器z高度及倾角、坡道地面处理、遮挡和行人突然折返，以及前进/倒车跟踪误差。首次未知速度已经用配置的最大速度包络，但最大速度上界本身与人群运动仍要实测。尺寸、定位和雷达平面中心、模式4反馈到位及自转方向已获用户确认，移出待标定项。此次门控与保守包络修正指定源码反例，不能等价于任意动态场景安全证明。
