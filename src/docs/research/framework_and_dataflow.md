# 当前框架、数据流与数据契约

检索/审计日期：2026-11-06（Asia/Shanghai）。基线：baseline_manifest.json；无Git，baseline_sources.zip保存本轮前202文件，不解压到ROS src。历史docs/review与validation只作待核对证据。定位生产/TF/补偿禁止修改。

```mermaid
flowchart TD
 LA[run_rda_dwa launch] --> S[雷达 IMU]
 LA --> L[6秒后 原FAST_LIO/global/fusion]
 LA --> STACK[12秒后 算法栈]
 S --> PC[/rslidar_points]
 L --> P[/localization map几何中心 源stamp]
 PC --> PER[规则过滤 world voxel 父KF 固定瓦片]
 P --> PER
 PER --> O[/rda/obstacles map AABB heading为速度方向]
 P --> B[VehicleBridgeCore]
 O --> B
 G[/move_base_simple/goal] --> B
 B --> D[ACK DWA 可执行采样 时序足迹 制动筛选]
 D --> B
 B --> M[静态引导 trap 模式仲裁]
 M --> B
 B --> R[同epoch command Path mode spin_target]
 R --> C[50Hz tracker 前馈 自转yaw闭环]
 P --> C
 C --> CC[/car_cmd]
 CC --> U[car_ctr UDP ModeSwitchGuard]
 U --> V[底盘]
 V --> F[/car_state /car_mode_ready]
 F --> B
 F --> C
 C --> BS[/car_bridge_state]
 BS --> B
```

|接口|坐标/单位/时间|契约/边界|
|---|---|---|
|/localization|map几何中心，m/rad，保留Odometry源stamp|消费端检查源推进/年龄/连续性；50Hz重发不是50Hz独立观测；不证明global ready|
|/rslidar_points|雷达平面中心，m，原cloud stamp|平面外参和补偿0；高度.45m/平地过滤是配置待验|
|/rda/obstacles|map AABB，x/y/length/width m，heading rad表示速度方向，speed m/s|未知速度负数语义；父KF再下传瓦片，动态圆包络；不做二次速度估计|
|/rda/command & /rda/traj|同整数ns epoch/map；v m/s,w rad/s；Path z速度标签|参考由执行guard验证；轨迹不是测量；默认fixed3s dt.05|
|/rda/traj_mode|无量纲 -1取消/0ACK/1SPIN|规划意图，不是硬件enum|
|/rda/spin_target|map yaw rad|会话锁存，不能用ACK w=0终止SPIN|
|/car_cmd|ACK0 speed m/s angle deg；SPIN4 speed rad/s|UDP SPIN乘-2用户已标定；斜行1横移2仅消息注释不足以证明物理语义|
|/car_state|硬件0ACK/4SPIN已到位，4轮m/s和errcode|无实测转角；轮速平均不是独立标定纵向速度|
|/car_mode_ready|-1未ready/0ACK ready/4SPIN ready|至少3s机械等待+新鲜真实反馈|
|/car_bridge_state|0ACK/1等待/2SPIN_RUN/3完成/4失败|软件enum2不是底盘自转反馈|
|/rda/wheel_*|计算角/轮速|不是测量反馈|
|诊断JSONL|monotonic/ROS receive/source ns分别记录|现有样例合成；点云仅元数据不能重做感知|

## 固定几何、运行参数与标定状态

车长1.2m、宽.8m、轴距.69m、定位/雷达中心与补偿0已确认。reference_center_to_rear=.345m为轴对称假定。当前v范围[-.3,.4]m/s、加速.5m/s²、制动1.5m/s²、转角30°、转角速率45°/s、spin最大.8rad/s是配置上限，须实车标定验证。10Hz规划、50Hz跟踪、3s时域、.05s安全积分、.3s硬超时、.6s反应、.2m基础安全裕度、.5s输入超时。最终参数表见运行adapter与新增parameter_inventory.md。

## 文件所有权

A perception：lidar_track_node.py/kf_tracker.py/obstacle_fusion.py/frame_utils.py与独立测试笔记。
B planner：enhanced_dwa.py/collision_checker.py/ackermann_model.py/params.py/module_config.py/global_guide.py与独立新增模块测试。
C control：vehicle_bridge.py/traj_tracker_core.py/traj_tracker_node.py/trial_runner.py/escape.py/mode_switch_guard.hpp/car_ctr.cpp与独立模块测试。
主agent：bridge_node.py/YAML/launch注释、共享主文档、参数接口表、总基准/集成/补丁与回滚。新增模块需向主agent声明。模块交叉由精确接口请求集成，不能覆盖他人文件。


## 本轮最终运行差异（冻结版）

默认启用：KF motion_guard源时一致性/unknown；DWA单次scene批量、保守下界/nominal复用/80ms预算；LocalStaticGuide有界路由和持久锁；静态反事实/真实重接/有限episode；max轮速入前停稳、新source yaw停止/仍对齐、至少3秒+真实ready、ACK恢复失败锁存、UDP最多2次补发。

`moving_recovery_enable:false`：6s新源pose/轮滚动窗口和goal法向两侧实际足迹阻塞只记录moving-local-minimum，不默认自动3m后退/侧出。实验true的RETREAT→ALIGN→ESCAPE→RETURN_ALIGN→REJOIN在三个U模型取得到达，但开放多块回退，故明确关闭。真正confirmed-static无ACK可行候选的原安全恢复仍启用。

用户新确认：横移只使用线速度；斜行线速度+angle，范围±45°。hybrid_primitives默认CRAB max_angle45、translation angle字段unused，软件仿真统一m/s；命令/ready编号、正负方向/物理比例仍未核实，所以完整verified/enabled仍False。生产只0/4，未声称CRAB/TRANSLATE全栈已部署。原ACK/SPIN参考、跟踪、底盘、足迹、日志贯通，未知模式目前仅离线全时序基元与明确的待接入契约。

有效参数/单位总表见parameter_inventory；模式/trap状态图与转移表见control_design。源码/配置最终hash见changes_manifest；定位保护40文件SHA验证见validation/config_and_localization.json。真实位姿/角反馈及车机负载仍待日志。
