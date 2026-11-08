# 独立校园激光感知模块

目标：新建 campus_lidar_perception，现有检测文件保持原样。CenterPoint base 与语义/几何支路输出经卡尔曼滤波和匈牙利关联的 map 系追踪信息，复用 rda_dwa_real_msgs/ObstacleArray 接口。增加 car_launch 新文件：检测、定位、定位+检测、完整 RL-DWA 规划控制链路。下载预训练 ONNX、HARP-NeXt checkpoint 与必要网络源码，提供 Orin 构建和比较入口。

输入：/rslidar_points、/localization、既有 TF；输出：/rda/obstacles、/rda/drivable_area、/campus_perception/tracks、诊断与回放报告。

成功命令：新包 tests 的 pytest、replay --smoke、模型文件检查；CUDA/ROS 实车验收单独记录。

Base commit：none (not a git repo)。现有 implement-stage 根目录属于另一项工作，本次仅写此子目录。保护文件清单位于 protected_files.json。

边界：当前 macOS 不含 CUDA/ROS；不伪造 GPU 或实车通过，不修改旧定位、检测、规划算法。