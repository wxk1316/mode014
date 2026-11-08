# 默认/实验对照与验证结果

日期2026-11-06。以下为synthetic geometry / bounded actuator / fake ROS构造，未运行ROS2、DDS或UDP车辆。原始/当前使用相同场景和物理模型；源码/hash与scope以各JSON为准。

默认配置`moving_recovery_enable:false`。22例两版本均11次到达，但迎面有物理模型碰撞，不计成功；因此各10例到达且无物理碰撞。安全停车/期限制约/局部失败单列。默认本轮0次100ms deadline miss，原测得1次；有限样本不表示车机尾部保证。

|场景|基线/默认结果|基线/默认时间s|默认物理AABB下界m|默认circle/静态分离下界m|基线/默认p95 ms|默认mode/trap次数|
|---|---|---|---|---|---|---|
|straight|reached / reached|12.18/12.18|—|—|17.71/10.58|0/0|
|left_curve|reached / reached|10.68/10.68|—|—|19.00/10.37|0/0|
|right_curve|reached / reached|10.68/10.68|—|—|19.05/9.78|0/0|
|static_block|reached / reached|18.54/18.54|0.592|0.592|32.55/22.25|0/0|
|multi_blocks|safe_stop_or_local_failure / safe_stop_or_local_failure|35.00/35.00|0.347|0.347|34.45/17.10|0/0|
|long_wall|reached / reached|24.38/24.38|0.271|0.271|29.43/22.18|0/0|
|u_shape|safe_stop_or_local_failure / safe_stop_or_local_failure|35.00/35.00|0.428|0.428|40.03/38.81|0/0|
|thick_u|safe_stop_or_local_failure / safe_stop_or_local_failure|35.00/35.00|0.325|0.325|42.03/26.37|0/0|
|narrow_u|safe_stop_or_local_failure / safe_stop_or_local_failure|35.00/35.00|0.325|0.325|33.56/28.85|0/0|
|narrow_passage|reached / reached|12.18/12.18|0.300|0.300|20.70/18.17|0/0|
|dead_end|safe_stop_or_local_failure / safe_stop_or_local_failure|20.00/20.00|0.290|0.290|42.64/35.87|0/0|
|crossing|reached / reached|12.58/12.58|0.339|0.409|28.06/20.48|0/0|
|head_on|physical_aabb_oracle_collision / physical_aabb_oracle_collision|22.78/22.78|-0.650|-0.354|27.71/20.37|0/0|
|overtaking|reached / reached|27.58/17.78|0.250|0.146|27.77/20.66|0/0|
|accelerate_reverse|reached / reached|14.68/14.68|0.270|0.194|31.44/20.45|0/0|
|occlusion|safe_stop_or_local_failure / safe_stop_or_local_failure|20.00/20.00|0.383|0.464|24.92/18.19|0/0|
|late_frame|safe_stop_or_local_failure / safe_stop_or_local_failure|10.00/10.00|0.410|0.502|30.11/20.70|0/0|
|false_velocity|safe_stop_or_local_failure / safe_stop_or_local_failure|10.00/10.00|1.150|1.133|11.84/9.46|0/0|
|reverse_goal|reached / reached|8.02/8.02|—|—|19.71/12.06|0/0|
|rear_blocked|safe_stop_or_local_failure / safe_stop_or_local_failure|5.00/5.00|0.200|0.200|12.06/16.26|0/0|
|localization_dropout|safe_stop_or_local_failure / safe_stop_or_local_failure|5.00/5.00|—|—|17.93/9.70|0/0|
|obstacle_dropout|safe_stop_or_local_failure / safe_stop_or_local_failure|5.00/5.00|—|—|16.45/9.59|0/0|

## 指标口径和局限

CSV式完整指标在[默认整栈JSON](validation/final_comparison/comparison.json)：路径、goal相对yaw RMSE、10Hz模型jerk、50Hz命令角速率/反号、mode等待/次数、trap再入、p50/95/99/max及100/300ms misses。没有真实转角、真实纵速或实车TTC，所以真实值null；另有已知运动的model CV-circle TTC，不能作为真实避障证书。

静态及物理oracle使用矩形SAT分离下界（正表示分离、负表示重叠），动态预测包络用point-to-vehicle-rectangle减circle radius。模型物理AABB不等于动态未知实体真值。追越circle .146m<.2但物理侧距.25m，折返circle .194m/物理.270m；分开报告。head_on物理下界-.650m，后续reached不能抹掉碰撞。晚帧10s场景未到达只是此时限内结果，其源位置已按t-.45构造，不能以oracle current position伪装旧帧。等待总量10Hz采样会约每次switch差.1s，3秒条件由独立状态机时刻断言验证。

## 规划内核 / 感知 / 控制消融

|来源|对照/结论|
|---|---|
|planner_benchmark.json|16×12固定snapshot；batch/full命令与原zip完全一致；far512 p50 2680.77→29.44ms（远场可能超active12m，仅算术热点）|
|planner_worstcase.json|512有效近场68.57ms、混合56.94ms；倒车近场118.14ms在80ms预算约80.18ms整轮丢弃，原模型独立检验停车safe|
|perception_comparison.json / noise_sweep|反向3s可达覆盖3/61→61/61；120父轨KF p95 3.528→3.839ms。4cm/20Hz meanunknown9.35/100，8cm94.65/100，负例保留|
|control_module_comparison.json|两个control版本使用相同当前planner/perception，是模块消融；误模式/侧移预算/原地退出/未停轮反例闭环，不作为整栈benchmark|

## 实验恢复与默认关闭决定

|相同三U/90秒|原源码|当前源码实验true时间s|净空下界m|trap/再入/mode次数|
|---|---|---|---|---|
|u_shape|90秒未达|83.14|0.618|1/0/4|
|thick_u|90秒未达|87.44|0.328|1/0/4|
|narrow_u|90秒未达|82.78|0.325|1/0/4|

当前2b6控制源码实验True且两侧footprint confinement成立的重跑取得上述三U到达，完整记录见[当前实验开关对照](validation/final_experimental_recovery/comparison.json)。这不是默认收益，也不是实车通过。

此前未加confinement的moving自动侧绕/后退在开放multi_blocks 90秒未达（原36.34秒到达）；加入confinement-only仍未恢复其原成功。旧3/4提升和multi回退数据位于[旧实验90s](validation/static_recovery_final_90s/comparison.json)，明确保留旧hash。最终只日志观测，不默认自动恢复。旧35/60秒失败和厚U不同试次近容差失败均未覆盖；bounded搜索墙钟可使不同试次结果变化，没有完备性声明。

## 多模式与动态系统比较

planner_mode_comparison.json和mode_comparison.json包含ACK±/SPIN/CRAB/TRANSLATE动作、后方阻塞/窄路/等待接近/ready超时或丢失，比较的是单基元能力与时间，不伪装相同终点最优导航。CRAB/TRANSLATE接口只部分确认，未启生产。

dynamic_encounter_experiment.md保留H6、旁路及desired-speed迎面失败，均0trap/0mode。mode_opportunity_experiment.json另说明模型完整5秒ACK转向日程可通过5m迎面（.238m下界），但不能给中间安全停车/失联backup；7m横移/倒斜行日程机会依赖未确认物理参数。没有据此执行开环或改变生产速度接口。

## 实车边界

需要真实raw cloud/bag、source进度、四轮逐轮反馈、真实角/刹停/滞后、有效12m观察范围与遮挡、3秒模式到位、位姿噪声、车机CPU/内存/DDS时延。当前FF/correction0未变，命令jerk不等于实际jerk。部署/受控验收/回滚见deployment_and_rollback。
