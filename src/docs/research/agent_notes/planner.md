# B 规划与静态避障摘要

更新：2026-11-06。检索截止同日。基线hash见 ../baseline_manifest.json。

## 已读与当前证据
- 任务书、日志指南、current launch/real_vehicle.yaml、planner_audit/implementation/cross_review及planning/closed_loop验证。
- EnhancedDWA.plan/_executable_window/_evaluate_production_window/command_safe，CollisionChecker.predict_trajectory/temporal_gaps/time_collision_free；VehicleBridgeCore.step/static_detour/arbitration/trap；test_planner_safety.py。
- 当前几何1.2×.8m、轴距.69m、map几何中心，后轴距离.345为假定；ACK v m/s, omega rad/s, delta rad；源当前障碍x/y已由TrackManager外推到now；times是未来s。
- 已修复：倒车检查、时间配对/制动驻留、静态AABB与unknown circle、固定尺度分数、模式动态扫掠/3s等待、适配后复查。历史“无倒车/反馈2/错中心/允许软松弛”不可作为当前缺陷。
- 当前可复现热点：每候选nominal rollout与gaps在安全和评分重复；每个瓦片独立NumPy调用，512片将处理成本线性放大。无planner内deadline语义。
- 静态two waypoint仅绕单最近对象，无拓扑路由；仿真GlobalGuide依赖oracle静态点且不在launch运行。

## 契约与修改范围
B独占enhanced_dwa.py/collision_checker.py/params.py/module_config.py/ackermann_model.py/global_guide.py；新增local_guide.py/hybrid_primitives.py/tests/test_research_planner.py。YAML/bridge接线由root，vehicle_bridge/trap归C。
- 计划优先等价批量几何与本次plan内部复用，不跨更新缓存、不降低阈值；适配后单独复查。
- deadline返回last_failure_reason=deadline（不是几何无解）；has_safe_motion静态反事实超时返回None。
- 非完整约束局部搜索只提供guidance，DWA执行段重新安全筛选；不可据有限地图声称完备。
- 多模式离线足迹与机械等待模型默认未知协议禁用；生产反馈语义未核实不得发布非0/4模式。

## 未决与下一步
量化同输入baseline/批量/复用消融；静态搜索方案及时间预算与root协调；控制C共同核trap反事实；感知A创新异常将unknown传播，批量路径必须保留未知最大速度可达集。

## 文献第一批导航（待逐项核查）
Fox/Burgard/Thrun DWA1997；Dolgov et al Hybrid A*2008/2010；Macenski et al Smac2024预印本/2026发表；Zhou et al EGO2021；GF-DWA2025。从原始DWA动态窗与可停控制、Hybrid A*非完整约束、Smac有界启发搜索和EGO必要障碍信息提取比较路线。飞行器/差速条件不能套本车。

## 实施与冻结摘要

- 已完成B01–B09分类文献/理论到代码：planner_literature.md；原文/元数据已核查DWA、Dolgov、Fast2019、Topo2020、EGO正式5作者、Smac、GF-DWA MIT官方库、EXACT2026由root核完整作者/许可未明确。
- 源码冻结后只写报告/实验：增强DWA本次scene+nominal复用、批量SAT/circle、整时域circ及soft SAT下界保守剔除、小场景标量、deadline、has_safe_motion、counts；新增local_guide.py与hybrid_primitives.py。
- planner_benchmark16×12同输入命令等价；near/dense/mixed/reverse-near512压力4×3；12新增测试通过。full far512 p50≈29.4ms，near≈68.6ms；reverse-near未限118.1ms，.08预算80.18ms丢本轮False/0/0，same模型停车独立safe。全量实车10Hz尚无证据。
- local静态SE2guide单块/后退U可出waypoint，不把guidance成功称到达；C接线。多模式32组离线schedule，未知协议disabled，实际ready/轮角/速度需车验。

## 动态迎面追加实验（生产未改）

根22场景同输入发现head_on(5-.7t,0)已停车仍被碰撞，reached不能掩盖失败。独立脚本dynamic_encounter_experiment.py monkeypatch，仅horizon6更保守且t0全不可行；nominal6/brake3也全不可行；早期侧waypoint(1.5,1.4)车移到(.41,.15)后t1.6停车仍碰撞。根因含恒命令初始速度候选局限与reaction=.6后制动并驻留3s的可停备份约束；不能简单放宽碰撞硬约束或让动态yield触发trap/spin。继续独立早遭遇与crossing/overtake/unknown验证，不先改冻结生产。

## 动态追加结论冻结

dynamic_encounter_experiment.json/md已汇总三主方案与full-target全链实验、18初始候选、跨crossing/overtake/unknown、first no-safe/collision、净空/source/trace hash。全部5m迎面仍physical/circle collision，7m提前也失败；0mode/0trap。完整desiredspeed并未足以修复，root明确拒生产改变horizon/速度语义，保留原接口和硬guard。专业边界是有限常值采样及立即停车backup的非递归可行；下一系统须多段控制+安全refuge/backup+误差契约。全部CPU实验结束，不继续扩方案/基准，供主公平冻结复跑。
