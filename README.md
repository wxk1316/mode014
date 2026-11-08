# mode014 — 小车三模式规划、感知与控制项目

此仓库保存当前 `car_gpt` 工作区的项目源码、配置、模型、地图、仿真测试及技术报告。
三种底盘模式为：阿克曼 `0`、斜行 `1`（支持连续 ±90°）、原地自转 `4`。

## 项目目录

- `src/rda_dwa/`：局部规划、运动模型与碰撞预测。
- `src/rda_dwa_real/`：ROS 适配、异步规划、轨迹跟踪、模式监督及诊断。
- `src/control/car_ctr/`：底盘 UDP 接口、模式切换与安全门禁。
- `src/campus_lidar_perception/`：校园场景感知及模型资源。
- `src/msg/`、`src/*_msgs/`：消息与公共组件。
- `src/car_launch/`、`src/planning/`、`src/road_planner/`：启动入口、规划组件与项目地图。
- `diagnostics/multimode_20261108/`：三模式仿真脚本、结果、图表和运动回放。
- `diagnostics/road_school_suite_20261108/`：校园场景模拟测试。
- `src/docs/`、`implement-stage/`：设计、实验及实施记录。

历史报告描述的是相应阶段的实现，以当前源码和 YAML 配置为准。

## 获取完整模型和地图

模型权重及点云地图通过 Git LFS 管理：

```bash
git lfs install
git clone https://github.com/wxk1316/mode014.git
cd mode014
git lfs pull
```

## ROS 构建

在已配置相应 ROS 2 和外部驱动依赖的车机工作空间中：

```bash
colcon build --symlink-install --packages-up-to common control_msgs rda_dwa_real_msgs rda_dwa car_ctr rda_dwa_real car_launch
source install/setup.bash
```

工作区中的文件并不包含完整 ROS 安装和所有外部传感器、定位驱动，需使用车机已有依赖。
更新时避免在同一 ROS 工作空间并列放置同名包。

## 测试与报告

- [三模式实验报告](diagnostics/multimode_20261108/REPORT.md)
- [运动回放](diagnostics/multimode_20261108/playback.html)
- [实验复现说明](diagnostics/multimode_20261108/README.md)
- [校园场景测试说明](diagnostics/road_school_suite_20261108/README.md)

这些是工程仿真与组件验证，不能替代实车速度、转向、到位和通信安全标定。

## 上传范围

按照项目拥有者的要求，本仓库**不包含 bag 包或用户提供的实车录制数据**。
`record_data/`、原始 bag、实车逐消息 JSONL、从实车数据提取的逐帧回放文件和相关图像留在本地。
本地虚拟环境、编译缓存、临时克隆、下载 wheel 和重复发布压缩包也不提交；仿真数据及报告保留。
录制、监控工具的源码保留，便于在车机重新采集。
