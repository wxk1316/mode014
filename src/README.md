# RDA-DWA 小车：修改包集合（2026-11-07）

本目录包含本次工作修改的六个完整ROS源码包，保持原相对路径。涵盖静态/动态避障、几何投影LQR、自转与有限trap恢复、真实车身尺寸、异步规划、完整诊断记录和自适应转向预瞄。

| ROS包名 | 仓库内位置 | 主要修改 |
| --- | --- | --- |
| rda_dwa | rda_dwa/ | 实时Ackermann规划、矩形与动态制动安全检查 |
| rda_dwa_real | rda_dwa_real/ | 实车桥接、跟踪/预瞄、感知、模式/trap、日志与配置 |
| rda_dwa_real_msgs | rda_dwa_real_msgs/ | 障碍物形状朝向字段 |
| car_ctr | control/car_ctr/ | 底盘指令/模式处理、看门狗、UDP与反馈诊断 |
| common | msg/common/ | 有界UDP旁路记录及接收边界处理 |
| car_launch | car_launch/ | 一键启动、诊断与参数传递 |

FAST_LIO、PCD地图和control_msgs源码本次没有改变，车机继续使用原有版本；control_msgs仍需随相关依赖构建。仓库只包含这六个包，车机原有的其他传感器驱动/依赖继续使用原环境。

## 车机更新

在工作空间外clone/pull仓库并备份旧源码，再把六个目录替换到车机源码的相应位置。避免在同一个ROS工作空间中并列保留两份同名包。地图、标定和其他未改包保持车机有效版本。

```bash
# 在车机ROS工作空间根目录
colcon build --symlink-install --packages-up-to common control_msgs rda_dwa_real_msgs rda_dwa car_ctr rda_dwa_real car_launch
source install/setup.bash
```

原入口仍为 ros2 launch car_launch run_rda_dwa.launch.py。本仓库未包含完整ROS环境和其他驱动，构建使用车机已有环境。

物理车身1.2×0.8m、轴距0.69m、定位参考为车体几何中心，最高速度1.0m/s；近障碍/急弯转向预瞄最高车速时不超过0.15m，开阔路段约0.4m；速度预瞄独立为0.25m。具体值在rda_dwa_real/config/real_vehicle.yaml。

日志默认逐消息记录和原始点云/IMU/TF rosbag，总量约8GiB，录制健康信息可见。完整说明见documentation/。验证仅为离线Python核心/ROS替身/C++内存替身，不代表实车或完整ROS构建验证。

SOURCE_MANIFEST.json记录每个文件的SHA256，并列出相对原始版本的全部25个改动文件，可核对源码一致性。
