# 部署、实车验收与回滚

截止2026-11-06。本机没有ROS2/rclpy/DDS、车机、真实点云bag或UDP车辆连接；已完成的是源代码补丁与离线验证。下面为操作者的部署/验收步骤，本任务没有执行实车启动。

## 交付与依赖

基线baseline_sources.zip及baseline_manifest.json在本轮任何修改之前保存；历史docs/review/baseline_sources.zip/changes.patch及历史JSON保持。最终changes.patch/changes_manifest.json包含实际代码/参数/测试变更，final_validation_summary.json绑定最终hash。原定位、定位TF及三个消费者0补偿按SHA检查，后轴.345m仍为轴对称假设。

优先补丁使用已有Python/NumPy、可选SciPy匈牙利（无SciPy仍有原greedy回退）、C++17；新增LocalStaticGuide仅heapq/NumPy，模式基元只用于离线。没有部署神经权重、JAX、GPDF、Ruckig、Nav2或新的求解器。感知四参数、预算/搜索/trap参数、生产停稳门与yaw率门见parameter_inventory和共享YAML。

ROS2车机上先检查自有common/data_comm/mqtt_comm/control_msgs依赖，当前car_ctr/package.xml声明这些依赖；此离线主机未做其完整构建。将更新源码部署到车机对应src，保留原install以便回退，在工作区父目录执行：

```bash
colcon build --packages-up-to rda_dwa_real car_ctr car_launch
source install/setup.bash
```

核对安装后config/launch与源码hash/有效参数，不只看src中的YAML。停止旧节点并确保/car_cmd只有当前tracker一个发布者。先在不会驱动车轮的隔离/模拟环境核查节点启动与消息契约；确认新source定位、全周障碍、原子epoch和模式ready均正确。最终运行入口仍为`ros2 launch car_launch run_rda_dwa.launch.py`，启动实车运动由操作者在受控测试中决定。

## 验收顺序与测量

1. 不运动检查：几何中心TF/补偿0、FAST_LIO/global/fusion原代码；/localization到达和源推进频率分别统计；frame/map初值有效性由现系统确认，12秒启动延时不是ready证明。/rda/obstacles source与raw cloud一致，unknown/shape heading正确；过期/重复/跳变不能刷新安全输入。
2. 架空/隔离底盘检查：0/4真实反馈、至少3秒、mode_max_retries2、ready丢失与ACK恢复失败锁存；单轮仍动不能用平均低速假装停稳；新源yaw率停止保持、目标仍4°内才返回ACK。模式4speed为rad/s、UDP乘-2保持既有标定。未知1/2保持禁用，另测命令/反馈/angle/speed/左右符号/极限及wheel型足迹后再启。
3. 低速封闭区：当前[-.3,.4]m/s上下限不提高，独立测后轴位置、转角/转角速率、加减速度/制动距离、轮速与中心纵向速度、yaw响应/滞后；记录测量装置与误差。没有实际转角topic不能把指令角当已测角。
4. 同一真实bag回放：raw cloud+全部定位/检测/command/Path/mode/ready/CarState/诊断与goal，保留整数源ns、monotonic与ROS epoch。先基线，再补丁，再逐模块开关消融；不使用只录点云元数据的JSONL重做聚类。计算p50/p95/p99/max/deadline与记录丢失，有无rosbag负载分别比较。
5. 受控场景验收：左右曲线、多块/长墙/U/厚U/窄U/窄通道与后方阻塞，动态横穿/迎面/追尾、加速折返/遮挡/晚帧/假速度，3秒等待接近、ready丢失/晚到/重试，自转w0与停止漂移，trap误进入/移动局部极小/内部冻结/次数耗尽/重接再入、源断流与超时。每例分到达/安全停/近碰/碰撞/模式失败，记录足迹净空/TTC、路径/时间、真实yaw误差、命令与测量转角/jerk、反号、模式等待驻留、trap原因与再入。

真实车的净空、TTC和jerk需有效测量和障碍预测模型条件；合成模型数字与真实值分列。默认.motion_guard阈值在4cm/20Hz噪声仍有保守性，在8cm/20Hz大部分unknown；必须以实际聚类质心统计决定门阈值。全量规划最坏例仍会80ms预算停，不保证所有车机周期无miss。

当前全对照显示持续非合作迎面可能撞上停车车，CV折返/追越还会侵入配置包络，有限局部引导在部分U/多块未到达。这些是算法能力边界与明确失败，不能靠停车fallback、调权重或测试计数证明已解决；最终报告记录进一步独立实验的结果与选择。

## 回滚

`restore_baseline.py`默认只预览；`--apply`按changes_manifest逐项检查当前文件必须仍匹配交付hash，然后只恢复本轮修改文件并删除本轮新增运行/测试模块。之后重新build/source相应包，核对安装hash；保留docs/research和历史证据。若文件已被之后修改脚本拒绝覆盖，先分离后续编辑。

```bash
python3 docs/research/restore_baseline.py
python3 docs/research/restore_baseline.py --apply
```

ZIP不解压为第二套ROS包放在src中。旧历史baseline与本轮baseline不同：恢复本轮前状态使用research/baseline，不能混用review中的更早接口版本。单项感知消融motion_guard_enable:false、static_search_enable:false只回退对应策略；模式/安全guard需要整组代码和配置一起恢复，不能只把新停稳门关掉冒充完整回滚。
