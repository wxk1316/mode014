# 目标

接入ACK/CRAB/SPIN三模式，CRAB连续±90°。保留定位与感知框架，升级原子计划、模式专用跟踪、异步模式有效期和驱动切换。跨模式保留至少3秒；测试4m道路、前方5m目标、指定动态/静态/U形与校园场景。

输入：现有ROS接口及可复现的障碍/定位仿真。输出：修改源码、单元检查、闭环轨迹与分析，路径diagnostics/multimode_20261108；结果JSON记录真实运行状态、几何净距、切换、trap、自转、耗时及失败原因。

成功命令：`diagnostics/multimode_20261108/.venv/bin/python diagnostics/multimode_20261108/campaign.py`，以及该目录和src/rda_dwa_real/test的pytest检查。

Base commit：none (not a git repo)。修改前源码副本位于diagnostics/multimode_20261108/baseline。

范围：本地闭环仿真及感知扰动实验，不启动实车，不声明实车成功或全局最优。
