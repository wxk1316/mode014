# A 感知与动态安全摘要

检索与核对日期：2026-11-06（Asia/Shanghai）。修改所有权：kf_tracker.py、obstacle_fusion.py、lidar_track_node.py、frame_utils.py；测试与本分支笔记。无真实车数据；禁止运行车辆。

## 已读与契约

已读任务书、日志指南、顶层/算法launch、real_vehicle.yaml、以上四个感知源码、历史perception_audit/implementation、docs/validation/test_perception_safety.py/profile_perception.py、Obstacle.msg、当前collision_checker unknown分支。

- 定位/雷达平面均几何中心，map m/rad/s；定位生产、TF与0补偿保持。
- 点云/source stamp推进一次KF；连续源年龄校验；世界固定voxel与瓦片。
- 父KF状态[x,y,vx,vy]，消息(x,y,L,W,heading,speed,id,type)，heading运动方向，active源shape yaw=0。
- type=unknown目前用于首个观测；fusion生成velocity_unknown/max_unknown_speed。snapshot过去年龄加2(rate+vmax)*age到L/W并重算圆半径；future cc半径加max(config_rate,vmax)*t。
- /car_mode_ready:-1/0/4；真实ready并至少3s等待；unknown也应检查等待可达域。

## 当前/历史问题

历史全周裁切、z符号、重复融合、固定dt、初次运动延迟、U空腔填框均已在当前修复。当前缺口：KFTrack.update第二差分可能超过vmax，随后裁速度并在第二观测立即标known；Hungarian只有1m欧氏门，没有观测间物理可达或模型创新核对；突然反向CV迟滞。短dt质心变动/误关联可成为可信1.5m/s假速度。需要以复现实验区分数学缺陷和实车频率/噪声未标定。

## 方案决策前置

比较：协方差Mahalanobis门（需校准噪声，初始P过宽）、物理可达门+速度/创新异常unknown（轻量、保守）、完整JPDA/PMBM（依赖检测概率/杂波模型与更多CPU）。优先保持CV/KF并使用可配置物理可达门，异常速度重置unknown，当前检测框始终保留、旧轨TTL。稳定连续量测后恢复可信速度，不把异常剪裁伪装已知速度。规划/control收到契约；control修spin整盘unknown路径。

## 文献阅读进度

已核实原作者页面/原文索引：Kalman(1960)、AB3DMOT(2020)、SimpleTrack(ECCV2022 workshop，arXiv2021)、Patchwork++(IROS2022)、Poly-MOT(IROS2023)。原文正文/代码许可核对进行中；每3—5篇汇总，不保存全文到主笔记。

## 下一步

构造同输入质心跳变、反向、遮挡、晚帧、静态噪声、匀速基线，设计门限与可撤销开关；先测现行为，再实施并量化额外时延及恢复条件。地面模型目前固定z仍是未验证平地假设；不凭无地面真值合成实验贸然部署新删点算法。

## 模块完成摘要（第一轮）

KF物理可达关联门、速度创新unknown回退、短源时间窗口重确认、向量化关联矩阵已经实施，修改KF/lidar/WorldRepresenter；frame_utils未改。原38+新增10条48通过。fake ROS构造通过，历史输出已恢复。

同输入hash比较：静态表面质心.25m/.05s旧版known1.5m/s，改版unknown0；.8m错关联保留旧TTL并新增unknown；突然反向时3s模型oracle覆盖3/61到61/61。初版单步速度门造成4cm/20Hz静态噪声77/80帧unknown，因此改为.2s源时间短窗校验/恢复，最终5/80，末态block；该噪声下非零known计数21帧对baseline12帧有回退（必须实车验证）。普通1cm/10Hz两版同为4个非零known/1个unknown。匀速前/倒车估计不变。120父track追加开销约0.3ms p95（最终数字由JSON为准），不能当完整感知性能。

与control核对：past年龄已写入圆包络，future按max(rate,vmax)*t；control将unknown纳入自转整盘检查。新参数请root在YAML接线；单步初始化known和短窗恢复不提供统计置信度，vmax仍是未标定场景假设，超此真实速度/遮挡实体/更大噪声仍未证明。

已读首批原文：AB3DMOT作者2019稿3.2—3.5、SimpleTrack摘要+官方配置说明（正文下载继续）、Patchwork++III/IV、Poly-MOTIII-B/C/IV-B/C。专业集合界推导与论文实践分开；不声称论文给本车3s模式或unknown策略定理。详表由perception_literature.md统一。

## 最终冻结与交叉审查

2026-11-06：感知源码冻结，kf_tracker sha849faf116afc1548a8758b954e4ee7d3c2bac508debca9f5901f51a049f3c988。新增11条含晚帧不单次恢复，加原38=49；跨层4=53全部通过。文献10条与公式条件/模型部署边界在perception_literature.md，实施/负面消融/性能在perception_implementation.md；结果和code sha在perception_validation_manifest.json。

跨层发现C未来ACK出口漏unknown delay增长，已向C和root报告；C新增_obstacles_after_delay并修复，独立时间重基准等价测试通过。B批量SAT/circle conservative bounds经独立顶点/线段几何oracle验证，未发现过高净空；状态机生产停稳+3s+ready失联0/mode4验证通过。原测试tick跨到3.00s的false failure已只修fixture实际时间断言，未改生产。

主全量集成与22场景结果由root持有；不覆盖历史记录。更多改动须重新确认所有权和回归。本分支不宣称实车已验收。

## 追加最终安全审查（恢复新阶段）

独占新增tests/test_research_final_safety.py，最终10条功能测试通过，没有性能基准或修改A冻结源码。审查RETREAT→ALIGN→ESCAPE→RETURN_ALIGN→REJOIN、真实有效planner目标、rear检查/同obs DWA/适配后复核、至少3s+ready、有限2spin/24s ACK预算/route lock、source age+unknown重基准。

确证发现并闭环C：unknown/dynamic等待消耗恢复ACK计数；连续planner deadline在_update_trap后判定会误作trap timeout；新鲜全零/缺轮速下1cm pose noise也满足moving累计travel。C已分别冻结合法等待与取消rejoin、tick phase/budget回滚（**保守消费attempt不恢复**）、加入窗口>=1s新鲜ACK轮滚动证据。独立反例全通过；有效global build_dynamic_window目标、phase/预算rollback但attempt=1、exhausted禁止第三次请求、controller corruptstate保持当前轮型0命令、安全源faultfreeze通过。

cross_final_safety_review.md记录初始失败、当前闭环与滑移/误轮速/定位噪声仍未测边界。主最终全量集成与公平时延由root执行。
