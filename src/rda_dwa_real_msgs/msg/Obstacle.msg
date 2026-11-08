# 结构化障碍物消息（实车感知 → 算法）
# frame 由 ObstacleArray.header.frame_id 指明（'map'/'odom'/车体系/传感器系）。
int32 id                # 跨帧稳定 id（无则填 -1，由节点内最近邻跟踪）
string type             # pedestrian / bicycle / vehicle / static_large / block / unknown
float64 x               # 位置 x（该消息所在 frame）
float64 y               # 位置 y
float64 length          # 矩形长（沿 heading 方向，m；未知填 -1）
float64 width           # 矩形宽（垂直 heading，m；未知填 -1）
float64 heading         # 运动方向（rad；未知/静态填 0）
float64 speed           # 速度（m/s；未知填 -1）

# Separate shape yaw from velocity direction, including parked cars.
# Legacy publishers may leave has_shape_heading=false (fallback to heading).
bool has_shape_heading
float64 shape_heading
