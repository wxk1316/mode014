# 追加最终跨模块安全审查

2026-11-06（Asia/Shanghai）。受主agent追加委托，只读C/规划/多模式源码及生产参数，必要独占tests/test_research_final_safety.py。感知motion_guard源码冻结未改。没有CPU性能基准、ROS运动或UDP。

## 新恢复链与已经核对路径

`_moving_local_minimum`：全部确认静态、源stamp真实推进、6s历史、真实观测累计travel和goal/锁route无progress、direct目标通道受阻、rear3m预检和至少一个安全倒车command后才激活RETREAT。每次RETREAT/ESCAPE仍走当前完整obs的DWA和适配后command_safe；不是waypoint直接驱动。ALIGN/RETURN_ALIGN要求全部新鲜轮速<=.03以及robot.v<=.03，真实ready/3s由tracker/底盘再次确认。Spin最大2次，episode中动作位移不清预算；ACK24s、机械等待分开；route锁及24s过期不会声称搜索完整。

实际重接路径：step计算rejoining后传global goal且plan_trap=None，独立测试监视真正build_dynamic_window参数，避免只测plan调用而忽略planner内trap目标override。source age在snapshot保留，unknown整盘等待/出口rebasing已由前轮审查修复；当前新源码仍保留该helper与契约。

## 两处确证发现（已发C与主agent并由C修复）

1. 新活跃RETREAT/ESCAPE等阶段在dynamic/unknown出现时，`_update_trap`直接累加_trap_elapsed；例如unknown让行调用dt=.2使预算0→.2。最终24s可能被错误计作trap失败，尽管DWA当前全obs安全检查仍存在。`_moving_local_minimum`清未激活历史不能解决活跃episode计时。
2. step在planner deadline判断前执行_update_trap；连续fresh输入+deadline经ABORT→RECOVER→ACK仍重复累计动作预算。独立fixture把预算缩短至.15s，仅两次.1s tick就episode_failed；这是24s预算同一线性计时缺口，不是声称实车实际跑完24s。

首次功能测试：2失败、3通过（约.17s）。通过项为sourcefault预算冻结停车、真正global候选目标、单轮未停不发mode4。修复后独立5/5通过；补hardnode phase回滚/有限attempts、第三次spin禁止、控制未知态fail-closed后8/8通过。主agent随后要求修复全零/缺轮速的1cm噪声误moving证据，新增2条后最终10/10通过。

## 区分限制与证据

- 首轮发现6s累计位移sum会把新鲜全零轮速下的1cm位姿噪声计作movement。主agent将其确认为需修当前反例，C已在source history加入新鲜四轮maxabs>.005m/s滚动支持，6s中累计>=1s轮滚动且pose path>=.06m并goal/route无progress才允许moving恢复；全零和缺轮速两条独立反例不再触发，真实滚动的交替stop由C正例核对。轮滚动是ACK下非零证据，不是纵向速度标定；滑移、轮速错误/噪声和更大位姿误差仍是测量边界。
- 所有当前已生成phase均来自内部合法转移；未找到来源会注入未定义phase。未知内部phase/损坏sub_goal的鲁棒处理仍应列失效边界，不能把任意人为改内部dict的异常当真实已发生unsafe。常规phase超时、失败、预算耗尽走停车/锁存而非取消安全筛选。
- 用户新确认：TRANSLATE只有线速度，CRAB线速度及±45°；离线protocol已有上述语义。IDs、反馈、正向仍未核实，enabled/verified=false，production仲裁/底盘不放行。当前离线body方向hypothesis不等同硬件正向。

## 最终状态

C修复已复核：active车辆观测速度<=.03且dynamic/unknown时冻结ACK计数并取消rejoin计时，当前完整obs DWA/phase安全继续检查；实际moving恢复继续计入有限预算。planner deadline、hardnode deadline、exception等回滚本tick的phase/ACK计数后故障停车；请求attempt保守消费，故障不能恢复请求预算。独立hardnode测试按此口径断言phase恢复且attempt=1。

最终tests/test_research_final_safety.py **10条通过**（功能测试，未运行性能基准），此前跨层geometry/source-time/mode4条也通过。A感知源码保持原冻结hash未改。主agent负责全量回归、公平时延和同输入闭环最终结果。
