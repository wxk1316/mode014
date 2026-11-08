# 感知、估计与可选模型参考目录

检索截止 **2026-11-06（Asia/Shanghai）**。优先读原文、作者仓库和许可文本；综述未作为实施依据。论文速度/检测基准与本车点云、雷达安装、低矮障碍、车机CPU和3秒换轮条件不同。以下“影响证据”采用原文后续比较、官方实现的使用关系或当日公开仓库计数；不虚报学术引用数。标记“专业理论”“官方实现”“工程启发式”区分证据等级。

## 已核实条目

| ID / 类别 | 作者、标题、年份及出处 | 原文/DOI | 官方实现、许可、权重 | 实验条件与影响证据 | 本车适用范围 / 读到位置 |
|---|---|---|---|---|---|
| A01 估计原始理论 | R. E. Kalman, **A New Approach to Linear Filtering and Prediction Problems**, 1960, *Journal of Basic Engineering* 82(1):35–45 | [出版社DOI](https://doi.org/10.1115/1.3662552)，Crossref出版元数据已核；出版社正文403，两个流传PDF链接404；另下载到的MIT PDF实际是Tony Lacey教程，未冒充原论文 | 原年代无官方代码/权重；本项目独立实现CV KF，未复制第三方受限实现 | AB3DMOT原文引用该方法，长期算法采用证据；不以KF名义保证真实分布已校准 | 专业KF状态空间/协方差公式；线性、白噪声和噪声协方差适配是条件；原全文未获取，未宣称章节阅读 |
| A02 非模型估计/跟踪 | Xinshuo Weng, Jianren Wang, David Held, Kris Kitani, **3D Multi-Object Tracking: A Baseline and New Evaluation Metrics**, 2020, IROS；2019先行稿标题为A Baseline for 3D Multi-Object Tracking | [最终作者稿](https://arxiv.org/abs/1907.03961)（PDF已读）、[2020短稿](https://arxiv.org/abs/2008.08063) | [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT)；[许可](https://github.com/xinshuoweng/AB3DMOT/blob/master/LICENSE)是**仅学术/非营利非商业研究**，不是MIT；KF本身无权重，前端检测权重另算 | KITTI/nuScenes、已检测3D box；最终稿报告207.4 FPS是跟踪范围，不含本车原始点云端到端。SimpleTrack/Poly-MOT原文与代码采用/比较；2026-11-06仓库API观察1850 stars仅工程采用信号 | 已读III-B/C/D/E及实验；保留“预测→匹配→更新→birth/death”顺序，不能把检测置信度或3D IoU硬搬到规则瓦片 |
| A03 非模型估计/跟踪 | Ziqi Pang, Zhichao Li, Naiyan Wang, **SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking**, arXiv 2021，ECCV 2022 Workshops | [原文](https://arxiv.org/abs/2111.09621)，[会议DOI](https://doi.org/10.1007/978-3-031-25056-9_43) | [SimpleTrack](https://github.com/tusen-ai/SimpleTrack)，[MIT](https://github.com/tusen-ai/SimpleTrack/blob/main/LICENSE)；跟踪无独立权重，前端CenterPoint检测/置信度另需模型；Python+NumPy/SciPy等 | Waymo/nuScenes；§5.1低质量高频检测可损害跟踪，20Hz不必优于10Hz。Poly-MOT明确比较，2026-11-06仓库观察400 stars | 已读§3、§4.2/4.3/4.4、§5.1、§6；借鉴组件消融/生命周期与“高频不等于高可信”，不复制GIoU或置信度门（本车没有可靠shape yaw/score） |
| A04 非模型地面 | Hyungtae Lim, Minho Oh, Hyun Myung, **Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor**, 2021, *IEEE RA-L* 6(4):6458–6465 | [作者稿](https://arxiv.org/abs/2108.05560)，[DOI](https://doi.org/10.1109/LRA.2021.3093009) | [作者实现](https://github.com/LimHyungTae/patchwork)；当前许可须随选定commit核对，未嵌入生产；非学习模型、无权重 | SemanticKITTI与粗糙地面；Patchwork++为明确后续扩展，构成方法延续证据 | 元数据与后续原文依据已核；当前固定z假设的替代路线，需原始点云/姿态/地面真值；本轮不以缺真值的synthetic地面删点部署 |
| A05 非模型地面 | Seungjae Lee, Hyungtae Lim, Hyun Myung, **Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud**, 2022, IROS:13276–13283 | [原文](https://arxiv.org/abs/2207.11919)，[DOI](https://doi.org/10.1109/IROS47612.2022.9981561) | [官方代码](https://github.com/url-kaist/patchwork-plusplus)，[BSD-2-Clause](https://github.com/url-kaist/patchwork-plusplus/blob/master/LICENSE)；C++/Eigen、pybind11、ROS2 Humble接口；无权重/训练 | SemanticKITTI；自适应地面似然、TGR/R-VPF/RNR。官方2026-11-06 API观察1128 stars、ROS2/C++/Python多接口为工程采用证据；不是本车漏检保证 | 已读§III-A—E、§IV；安装高度/反射/地面姿态条件要重新验收。本轮不引入其“ground”删除结果到总安全输入 |
| A06 非模型估计/路线对比 | Xiaoyu Li, Tao Xie, Dedong Liu, Jinghan Gao, Kun Dai, Zhiqiang Jiang, Lijun Zhao, Ke Wang, **Poly-MOT: A Polyhedral Framework For 3D Multi-Object Tracking**, 2023, IROS | [原文](https://arxiv.org/abs/2307.16675) | [官方代码](https://github.com/lixiaoyu2000/Poly-MOT)，README声称MIT，但仓库API未识别license文件，本轮不复制；检测器权重独立 | nuScenes、Python、Intel 9940X无GPU跟踪；§IV-C报告3 FPS，说明更复杂几何/关联不当然更快；Easy-Poly延续对比及官方README列SimpleTrack/AB3DMOT依赖，观察197 stars | 已读§III-B/C/D、§IV-B/C；类别特定CTRA/bicycle需要可靠类别与朝向，本车规则type不可当semantic class，因此不选其整套替换 |
| A07 近期跟踪探索 | Peng Zhang, Xin Li, Xin Lin, Liang He, **Easy-Poly: An Easy Polyhedral Framework For 3D Multi-Object Tracking**, arXiv 2025，v3于2026-02-13更新；未核实正式会刊 | [原文](https://arxiv.org/abs/2502.17822)，[稳定DOI](https://doi.org/10.48550/arXiv.2502.17822) | 原文模板仍有占位会议DOI，不能当正式出版DOI；公开官方实现/明确许可/权重未查到，不引入 | nuScenes多模态检测、confidence weighted KF/自适应协方差；新方法持续影响证据尚不足，只记最新研究 | 已读§3.3/3.4/3.5、§4；本车没有神经置信度，不能伪造score驱动R/Q；仅作为后续校准的导航 |
| A08 近期非模型地面 | Muhammad Haider Khan Lodhi, Christoph Hertzberg, **GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds**, ICRCV 2025作者接收稿，2026-03-04上传 | [原文](https://arxiv.org/abs/2603.04208)，[出版DOI](https://doi.org/10.1109/ICRCV67407.2025.11349133)；年份按正式2025与上传2026分开 | 原文说代码接收后将公开，未核实官方已公开仓库/许可；无学习权重，不引入 | SemanticKITTI，局部特征值/梯度扩张，原文§V-E列复杂植被/运行时波动；新文无长期影响断言 | 已读§III-B/C/D、§V-E；用作对固定z平地假设的最新对照；本车低障碍false-ground需独立真值验证 |
| A09 可选学习检测（未接入） | Alex H. Lang, Sourabh Vora, Holger Caesar, Lubing Zhou, Jiong Yang, Oscar Beijbom, **PointPillars: Fast Encoders for Object Detection from Point Clouds**, 2019, CVPR | [原文](https://arxiv.org/abs/1812.05784) | [作者代码](https://github.com/nutonomy/second.pytorch)，[MIT](https://github.com/nutonomy/second.pytorch/blob/master/LICENSE)；原仓库README未核到可下载预训练链接，后续实现的权重须另核 | KITTI；§6 Intel i7+1080Ti报告16.2ms全pipeline/约62Hz，不能套Jetson/CPU；CenterPoint等使用pillar backbone为影响证据 | 已读§2、§6；相对稠密真值车/人检测有用，未知低矮静态物体不应靠网络过滤规则点集 |
| A10 可选学习检测/跟踪（未接入） | Tianwei Yin, Xingyi Zhou, Philipp Krähenbühl, **Center-based 3D Object Detection and Tracking**, 2021, CVPR | [原文](https://arxiv.org/abs/2006.11275) | [作者代码](https://github.com/tianweiy/CenterPoint)，[MIT及NOTICE](https://github.com/tianweiy/CenterPoint/blob/master/docs/NOTICE)；[官方模型表](https://github.com/tianweiy/CenterPoint/blob/master/configs/nusc/README.md)列SharePoint下载目录，文件未下载/哈希验证；nuScenes/Waymo数据含非商业约束，模型训练数据条款独立核对 | nuScenes/Waymo，预训练模型，中心+速度回归、最近邻跟踪。SimpleTrack/Poly-MOT显式采用/比较是可核影响证据 | 已读§4、§5、附录A/B；仅条件候选，不声称本车易部署或实时，详下表 |

## 优先实现与理论对应

A02/03支持保持轻量模块、采用源帧顺序和生命周期、对运动模型/关联单独消融；**论文并没有直接提出本补丁所有阈值或本车unknown策略**。本轮具体门与恢复是工程启发式；圆包络可达界是专业集合/三角不等式推导。

- CV KF变量 `s=[x,y,vx,vy]`，位置m、速度m/s，`F(dt)`用真实源间隔，`P⁻=FPFᵀ+Q`、`K=PHᵀ(HPHᵀ+R)⁻¹`。`Q/R`未统计校准，不能称95%置信域。
- 关联约束：两次观测中心距离 `||z_k-z_last|| ≤ v_max Δt + 2 ε_p`。前提是障碍中心速度界和两次位置误差界都成立；`v_max=1.5m/s`、`ε_p=.15m`是现运行假设，不是完整实车/行人标定。失败边仍保留新unknown和旧轨TTL。
- 速度条件：最初第二量测差分，后续约`.2s`真实源窗口差分用于速度界核对；同时瞬时速度差 `||v_fd-v_pred||>.8m/s` 且位置创新 `||z-Hs⁻||>.15m` 为模型不一致。位置立即使用本次框，速度设未知；非重复且间隔不大于窗口的兼容量测累计`.2s`重确认。
- unknown未来集：`p(t)∈B(z_current,v_max t)`，与已观测足迹圆包络作Minkowski和。source age已在snapshot包络；模式动作时间重新从0起算时，必须先把等待/动作/recover `delay`的可达半径带到新的time0。C分支因此新增`_obstacles_after_delay`，不能省略已等待的unknown域。
- 不能保证超过1.5m/s的真实目标、看不见的实体、非高斯/更大质心跳变、错误定位初始化或车辆不能按制动模型停下。阈值/恢复必须记录实际源频率、误差、unknown时长、误停车与漏障；当前代码不会让其成为概率或形式安全认证。

## 可选模型的部署门槛（未实施、无比较收益断言）

| 项目 | CenterPoint 条件候选 |
|---|---|
| 依赖/平台 | [作者INSTALL](https://github.com/tianweiy/CenterPoint/blob/master/docs/INSTALL.md)：Linux、Python3.6+、PyTorch1.1+、CUDA10+、CMake、spconv1/2、可选APEX/旋转NMS扩展；作者测试旧Ubuntu/PyTorch组合，当前车机适配要重新验证 |
| 权重 | 官方模型表有pillar/voxel下载目录；未下载、校验hash或证明链接当前匿名可用，不把README链接等同可部署成品 |
| ROS2接口 | 新适配应消费`PointCloud2`原source stamp，输出同一map AABB/独立velocity contract；推理异步depth1、超时不刷新，模型对象与规则占据取保守并集，避免漏检把低障碍删除。当前没有新推理node |
| CPU/GPU/内存/时延 | 本车均**未测**；官方pillar表报告31 FPS只属于作者环境，不作为车机周期预算。原80k规则pipeline历史约234ms且排除DDS/解码；不以这些异构数值算提升比例 |
| 训练需求 | 提供预训练则不必从零训，但雷达线数/安装高度/场景不同，需要标注验证和可能fine tune；需要真实低矮障碍、遮挡和动静真值 |
| 许可 | 代码MIT及NOTICE；数据/下载模型条款独立，不用MIT覆盖数据的非商业条件 |
| 失败回退/验收 | 推理失败/超时沿现守卫停车并保留规则占据；同一真实bag测对象召回、未知障碍覆盖、track IDS、速度误差、净空/误停车与p50/95/99/max/deadline misses后才能选择。没有GPU/bag时不实施模型替换 |
