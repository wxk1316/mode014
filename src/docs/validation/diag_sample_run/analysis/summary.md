# 实车日志取证摘要

日志目录：`/Users/bytedance/learn/src/docs/validation/diag_sample_run`

观察时长：9.680 秒。统计口径：message callback arrivals。

这些结果用于筛选问题时段；需要结合轨迹、障碍、状态机和底层反馈判断原因。

| 记录完整性 | 值 |
|---|---:|
| 无效JSON行 | 0 |
| 消息序号缺口 | 0 |
| 消息序号倒序 | 0 |

写入完整性：clean_close=`True`，丢弃记录=`0`，写入错误=`0`，待写队列=`0`。

缺少正常关闭摘要、存在丢弃/写入错误或无效JSON时，先核对日志完整性，再解释车辆和输入故障。

## 时序与运动指标

| 指标 | 样本数 | 均值 | p95 | 最大 |
|---|---:|---:|---:|---:|
| planning_duration_s | 0 | 缺失 | 缺失 | 缺失 |
| planning_cycle_s | 0 | 缺失 | 缺失 | 缺失 |
| controller_cycle_s | 1 | 0.02 | 0.02 | 0.02 |
| steering_rate_abs_deg_s | 0 | 缺失 | 缺失 | 缺失 |
| command_acceleration_m_s2 | 0 | 缺失 | 缺失 | 缺失 |
| command_feedback_speed_error_m_s | 0 | 缺失 | 缺失 | 缺失 |
| reference_pair_receive_gap_s | 1 | 0.01 | 0.01 | 0.01 |
| cloud_to_obstacle_observed_receive_gap_s | 0 | 缺失 | 缺失 | 缺失 |
| localization_yaw_rate_rad_s | 1 | 0.12 | 0.12 | 0.12 |
| localization_source_dt_s | 1 | 0.1 | 0.1 | 0.1 |

p95超过10000个样本时采用最后10000个样本；均值与最大值覆盖全部有效样本。

命令转角反号次数：0；模式请求变化次数：2。

## 明确上报的停止原因

未记录到原因字段。

## 自转与机械切换时序

| 请求自转 | ready=4 | 首次旋转指令 | 首次观测yaw运动 | 请求ACK | ready=0 |
|---:|---:|---:|---:|---:|---:|
| 0.28 | 3.28 | 3.3 | 3.4 | 6.68 | 9.68 |

录制开始时已在自转的会话为left_censored，不能用首条消息代替实际模式请求时间。

## 待核对观察

- `invalid_wheel_feedback`：1 次；示例时刻 0.3 秒。

参考epoch共1个，完整配对1个；控制器消费而记录端缺少完整配对的epoch数：0。

## 分析边界

- No measured steering angle exists in current CarState; command angle is not actual steering.
- CarState has no source timestamp; wheel averaging and receipt-time errors do not identify physical response latency.
- DDS arrival order and recorder drops can explain unmatched references; controller consumption and writer health must be checked together.
- Command heartbeat gaps show missing observations, not where a message was lost or whether firmware applied it; heartbeat resets are not automatically loss.
- Cloud-to-obstacle matching uses equal source stamps; the receipt gap includes processing and transport, may be negative from DDS ordering, and is not pure perception CPU time.
- Sign changes may be a normal S-bend. Clusters are investigation clues, not proof of instability.
- Threshold flags use the current low-speed 3s/.3s/.5s profile; compare runtime/config provenance before treating them as violations.
- Point-cloud metadata does not reproduce segmentation, missed detections or collision candidates; use rosbag/raw cloud for those questions.
