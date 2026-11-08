# 感知优先补丁、对照与交叉审查

核对/检索截止2026-11-06（Asia/Shanghai）。此分支处理A感知和动态输入，并受主agent追加委托做只读跨层安全审查。**源码已冻结**，定位生产、TF、0补偿、几何/硬件模式语义未改；未连接车辆、未运行ROS运动或发送UDP。原基线在`baseline_sources.zip`/`baseline_manifest.json`；本分支文件/结果sha在`perception_validation_manifest.json`。

## 当前证据、来源与实施

| 项目 | 问题证据→来源→条件 | 本车适配→代码改法 | 计算/失败路径→验证 |
|---|---|---|---|
| 关联物理可达门 | 原`HungarianMatcher`只看predict中心1m门，`.05s`内`.8m`表面/对象变化仍关联同id并裁速度；A02 III-C、A03 §4.3说明距离关联局限。`||z_k-z_last||≤vmax*Δt+2εp`是专业三角不等式推导，需速度与观测误差界成立 | `GlobalKFTracker.step`在source-time最后真实观测与检测间建立allowed matrix；匈牙利/贪心均遵守。失败关联新建unknown，旧对象依原TTL保留，不删当前框；保留世界固定tiles、AABB与map中心 | 新mask和距离矩阵NumPy向量化；O(NM)+匹配求解，内存O(NM)。匹配失败提高暂时对象数/保守停车，max512瓦片既有fail-closed仍有效。`.8m/.05s`从1条可信1.5m/s错关联变成旧hold+新unknown两条；回归覆盖检测点保留 |
| 速度/反向创新守卫 | 原`KFTrack.update`第二观测直接差分再clip vmax，在`.25m/.05s`示例发布known1.5m/s；突然倒向仍预测旧方向。A01专业KF更新、A03 §4.2/§5.1支持模型/帧质量消融；本门是工程启发式，并非原论文定理 | 保留初始第二真实量测速度prior；速度越界、或位置创新>.15m且瞬时速度创新>.8m/s则当前检测中心直接保留、v=unknown（wire type unknown/speed0），取消静态确认；用源时间`.2s`短窗校验与连续兼容量测重确认。大于窗口的晚帧不能单次完成恢复 | 首帧/异常时unknown未来可达球由既有fusion/cc提供。代价是更保守，对高噪声仍可能长时间unknown；不把配置上限当已标定物理界。突变示例known false speed消除，反向oracle 61时刻覆盖3→61；late/duplicate/source timing保持回归 |
| unknown父子传播 | 原tiled kind按speed>.05选pedestrian、否则unknown，不完整反映父velocity可信状态 | `WorldObstacleRepresenter`按parent.velocity_trusted传播unknown，不从fixed tile差分估速度，静态确认才block；节点声明/read四个新参数；`TrackManager`已有unknown语义保持 | 新长簇反向测试逐点验证瓦片仍覆盖观测，所有tile unknown/0速度；不改消息结构/坐标/尺寸约定 |
| 时间重基准跨层修复 | 只读发现C`_select_spin_target`出口时间从0起算、unknown speed0未带等待/自转/recover增长；当前盘安全不等于出口一米安全。专业可达球Minkowski加法 | 已发给C并由C实施`_obstacles_after_delay`，unknown和dynamic按max(config_rate,unknown_rate)*delay增加半径；已有past source age不重复补；等待时间包含停稳/机械/动作/恢复 | 独立跨层测试对比原time3/3.5与rebased time0/.5，同一unknown gap严格一致；snapshot对象未修改。安全条件仍依实际速度界和执行误差 |
| B批量撞检交叉验证 | 不能以速度提升牺牲安全/夸大净空；SAT分离轴值只是保守净空下界 | 只读B`temporal_gaps_many/min_temporal_gap/time_collision_free`，独立凸多边形顶点/线段距离oracle检验24场景×8位姿×67障碍（跨64 chunk），不用B标量路径作为唯一oracle | 全部批量gap≤独立实际几何距离，min gap与全对象批量最小值一致；B既有random scalar/batch+broadphase等价tests另由主全量集成。没有发现新的过高净空或跨plan旧scene复用 |
| C机械与ready交叉验证 | 新控制生产参数要求停稳、至少3s、真实ready；ACK w=0不得终止yaw会话 | 只读当前状态机，独立生产参数fixture检查停稳feedback、first nonzero>=3s且ready4，ready loss输出mode4/0并fault | 首次测试while跨到3.00s误断言非零失败，修正为按实际调用时间断言；**未改生产时门**。最终4跨层检查通过 |

四个新节点/YAML参数由主agent接线：`motion_guard_enable=true`、`measurement_position_slack=.15m`、`velocity_innovation_limit=.8m/s`、`velocity_reconfirm_time=.2s`。关闭motion_guard是算法消融；完整回滚使用本轮zip按manifest仅恢复变更文件，不覆盖其他历史工作。实车velocity与εp不是全局测量真值，阈值未校准。

## 数据契约与可达界

地图坐标m，内部yaw/velocity_heading为rad；定位/点云source stamp推进一次KF，重复stamp不进入量测/重确认。消息的heading是运动方向，active输入L/W为map AABB，shape heading=0；父KF状态[x,y,vx,vy]。定位生产/TF/归零补偿未改；1.2×.8m车、.69m轴距与中心→后轴.345m假设保持。

首帧或异常运动：`type='unknown',speed=0`传递未知方向，不能解释为实测静止。fusion snapshot按过去年龄增长L/W并重算包络圆（保守，包含未知vmax×age），cc对未来t按max(config_rate,vmax)*t增半径；过去shape增长与未来时间必须分清。速度高于1.5m/s、观测中心误差超.15m、关联结构变化/分裂不遵守中心模型时该界本身未保证，运行阈值不构成安全证明。

## 同输入基线与消融结果

`validation/run_perception_comparison.py`从本轮zip载入**逐字baseline KF**，使用固定source序列和input sha。四版本：baseline、single_step_guard（保留初版负面消融）、vectorized_guard_off、patched短窗。`perception_comparison.json`含输入/基线hash、每场景状态、全部p50/p95/p99/max/deadline misses；这是合成源量测/模型oracle，不能当实车行人轨迹或成功率。

| 合成输入 | 基线 | 本补丁 | 边界 |
|---|---|---|---|
| 匀速1m/s前进/反向各40帧 | 39帧可信运动、1帧unknown | 一致，速度误差约相同 | 不包含真实执行/地面漏检 |
| `.25m/.05s`局部表面跳变 | 第二帧known=1.5m/s | 第二帧unknown=0，中心仍2.25m | 无法证明真实对象没有以更大速度运动 |
| `.8m/.05s`关联候选 | 单旧id known=1.5m/s | 旧unknown hold+新unknown | 暂时更保守/对象数提高 |
| 1m/s突然反向，第一反向观测后的3s oracle | 旧方向速度约.835m/s；覆盖3/61个未来时刻 | unknown；可达域覆盖61/61，连续实观测后恢复反向 | oracle假设实际1m/s，非任意运动/车辆安全证明 |
| 1cm位置σ、10Hz、80帧静态 | unknown1、nonzero known4、末态block | 一致 | 没有雷达实测噪声支持该σ |
| 4cm位置σ、20Hz、80帧静态 | unknown1、nonzero known12、末态block | 单步消融unknown77；短窗unknown5、nonzero known21、末态block | **暂态非零known计数有回退**，不能称整体误速度全面改善 |
| 暂时遮挡 | 旧TTL/预测保持 | 一致 | 两层TTL和未知域增长仍需实车遮挡验收 |

纯KF性能（最终已保存运行，ms，180帧；排除filter/cluster/tiles/ROS/DDS）：

| 父track数 | 版本 | p50 | p95 | p99 | max | 100ms misses |
|---|---|---:|---:|---:|---:|---:|
| 20 | baseline | .371 | .409 | .429 | .483 | 0 |
| 20 | patched | .463 | .475 | .510 | .559 | 0 |
| 120 | baseline | 3.352 | 3.528 | 3.725 | 3.760 | 0 |
| 120 | patched | 3.649 | 3.839 | 4.066 | 4.138 | 0 |

本补丁主要增加一致性安全表达，**KF不是总体速度收益项**。120父track p95额外约.311ms；只与同机/同输入基线比较。旧原始80k dense感知约234ms的端到端缺口未由本补丁解决；本轮没有据此声称10Hz车机保证。

## 多seed与源频率敏感性

`validation/run_perception_noise_sweep.py`用20个独立seed×100帧、`.1/.2/.3s`窗口，不依据某个seed选阈值。默认`.2s`窗口是一项约10Hz两周期/20Hz四周期的工程恢复折中，不是统计最优值。

- 1cm/10Hz：平均unknown1帧、max1，20/20最后确认为静态。
- 4cm/20Hz：平均unknown9.35、max53，17/20最后静态；`.1s`窗口mean43.05、仅2/20最后静态；`.3s`窗口mean8.3、18/20最后静态，恢复更慢。
- 8cm/20Hz：默认mean94.65、max100、0/20最后静态；这是明显过保守失败边界，不能隐去或推广到高噪声真实输入。

## 验证、失败类型与实车验收

原38感知回归+新增11+独立跨层4=**53条通过**；fake ROS参数构造通过（构造检查输出恢复至旧位置防覆盖历史记录）；主全量回归/22场景matrix由主agent统一冻结后执行。程序断流、旧source heartbeat、晚/重复stamp、NaN、表示超限沿已有fail-closed守卫；复现不等同实际DDS watchdog已验收。

实车验收需要原始点云bag与逐消息JSONL同时记录：静止墙/不同可见表面/旋转/倒车、动态横穿/反向/遮挡/晚帧、mode等待接近障碍。估计world cluster中心抖动、真实source频率、source年龄、关联ID/速度创新、unknown连续时长、objects/tiles数量、误停/净空/TTC及感知p50/p95/p99/max/deadline。实际物理速度/角度/制动未标定完整；没有真实转角反馈，轮速均值不是独立纵向真值。固定高度z过滤/倾斜地面、看不见的实体、尺寸永久保守扩张仍是未验证限制。

参考的非模型与学习路线详`perception_literature.md`。Patchwork++是地面替代条件候选，CenterPoint/PointPillars是可选研究目录，本轮没有加载模型、训练或添加推理依赖，资源/权重部署未测不会报告虚构收益。
