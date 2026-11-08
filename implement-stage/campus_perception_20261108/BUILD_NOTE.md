# 实施阶梯

技能：balanced / draft。

| Rung | 内容 | 检查 | 状态 |
|---|---|---|---|
| F0 | 独立消息/回放入口到JSON产物 | replay --smoke | passed |
| F1 | Kalman+Hungarian、定位/点云/几何逻辑 | pytest core | passed |
| F2 | 下载模型、真实CenterPoint/HARP推理后端 | model checks+checkpoint smoke；CUDA由Orin验收 | CPU passed; GPU pending |
| F3 | ROS节点、兼容接口、launch/构建入口 | pytest integration+launch contract | passed |
| F4 | 回放比较、文档、假设审查、保留检查 | accumulated pytest+review | passed (provisional review) |

## Run record
F0: PYTHONPATH=src/campus_lidar_perception python3 -m campus_lidar_perception.replay --smoke，退出0，F0_smoke.jsonl存在；占位枚举已于F1代码替换，不进入生产节点。
## Deferred
当前主机没有ROS2和CUDA；GPU编译与实车运行的证据无法在本机生成。将实现并交付对应执行命令。
## Blockers
HARP命名已由用户澄清，实施两套已确认配置。源码交付无待澄清项；GPU/ROS运行验收需目标Orin，不作为本机通过项。

F1: 9项纯逻辑测试通过；覆盖稳定ID、速度、交叉、漏检删除、unknown实测框、定位运动补偿、PointCloud2行填充/大端、路沿与未知栅格。累积replay smoke退出0。使用已存在的本地测试venv执行，未修改旧包。

F2: 模型全部下载；ONNX checker通过；HARP原始net的293个tensor完整提取为weights.pth且逐tensor不变，strict load和真实CPU前向通过。累计19测试通过。Patchwork++实际绑定可加载。CUDA/TRT GPU build不在本机伪造通过。

F3: 25测试通过，包括实际旧规划回调接收新ObstacleArray、整数纳秒源时间、全部launch结构排除旧检测、完整规划控制节点覆盖、真实Patchwork++路面推理。新源码compileall与build脚本bash -n通过。用户已确认geometry vs HARP-NeXt对照。

F4进行中：独立比较记录与统计、Orin GPU验收脚本、中文部署说明已完成；旧感知及旧launch文件保留清单复核变化为[]。

最终检查：32测试通过；14个下载文件完整性匹配；HARP真实CPU前向、Patchwork真实CPU路面推理、原规划回调、模型结构、源时间与launch结构通过；源码语法与shell语法通过。第二轮新代理假设审查0未声明、0陈旧行，same-family/provisional。原始第一轮6个语义缺口和1个接口缺口已记录并修正/声明，保留两轮原始审查。无生产占位模型；只有单测合成fixtures。

交付打包完成：源码+模型zip的CRC与内容检查通过，wheel打包及模型/网络安装内容检查通过，旧检测及旧launch保留清单20项未变。最终32测试与Python/shell语法检查通过。
