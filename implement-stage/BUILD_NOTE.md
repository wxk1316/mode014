# 实施记录

技能配置：balanced；ASK=never；assurance=draft。

| Rung | 内容 | 验收命令 | Tier | Status |
|---|---|---|---|---|
| F0 | 原框架运行入口到闭环实验结果，保存基线 | campaign.py --smoke --baseline | MUST | passed |
| F1 | CRAB模型、速度状态、专用跟踪和通用切换 | pytest test_multimode.py | MUST | passed |
| F2 | 多模式规划、监督、协议和驱动接入 | pytest test_multimode.py + adapter/driver checks | MUST | passed |
| F3 | 指定和校园闭环场景、扰动、对照与参数调整 | campaign.py | MUST | passed |
| F4 | 回归、假设审查、分析与交付 | accumulated checks + review | MUST | passed (provisional review) |

## Run record

F0：2026-11-08 baseline smoke退出0；open用10.62s到达，净距1.60m，0切换。新闭环器使用独立积分与多边形距离。实验venv复用本机缓存SciPy，未修改项目依赖。

F1：6项行为测试通过，覆盖90°位移、yaw保持、摆角速率、全平面速度、当前模式停车、反馈失效与READY后重规划。首次发现整数输入导致NumPy数组类型错误，已修复为浮点状态。

F2：113项回归通过、10子测试通过，1项既有xfail；C++独立策略编译运行通过。实验器已直接加载同一C++模式切换策略，检查完整等待与停稳，ROS/UDP运行仍不属于本机测试范围。

F3：最终21场景，17到达、0碰撞；三种子63次扰动，均0碰撞，未达压力案例保留。实际进程/策略与独立几何检查均有原始输出；图和报告正在核对。

F4：120项回归通过、10子测试通过、1既有xfail；实际spawn ACK/CRAB进程通过；六向切换与停稳失效复现通过。图像已查看。两轮审查provisional，最后修正由执行者测试验证。

## Deferred

实车ROS/DDS/UDP联调、真实雷达重放和机械标定：本机未提供可用实车连接与相应新实验数据。

## Blockers

暂无。项目无Git，保留源码基线副本，不初始化仓库。
