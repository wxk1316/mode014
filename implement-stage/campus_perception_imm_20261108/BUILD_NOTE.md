# 执行记录

| Rung | 内容 | 检查 | 状态 |
|---|---|---|---|
| F0 | 基线回放及现有32测试 | pytest/replay | passed |
| F1 | CV/IMM参数化+数学/机动测试 | pytest filters/tracker | passed |
| F2 | 预处理/逻辑修复和MarkerArray | pytest preprocessing/markers | passed |
| F3 | 配置/launch/文档/独立审查 | accumulated pytest+review | passed (provisional) |
| F4 | 指定仓库完整源码提交、push与远端核对 | remote commit equality | passed |

## Deferred
ROS/CUDA/实车测试需Orin；保持明确未执行。
## Blockers
已使用wxk1316完成登录并确认仓库push权限，main分支上传成功。

F0: 原32项测试全部通过，原模块有可运行CPU回放入口；无新占位模型。公开目标仓库为空，已开始独立clone；本机两个旧账号无写权限，用户选择登录新账号，OAuth设备流程已启动。

F1/F2: 49项检查通过，新增IMM混合总协方差、似然/概率稳定、CV↔IMM状态保持、静止恢复、交叉/漏检及过期ID验证；复现并修复6个预处理问题。又解除独立launch对car_launch依赖并增加测试。登录码已过期，最终提交准备好后重新启动用户授权。

最终CPU测试52项通过；复核最终F07/F08已修复并以2个针对性回归复验，scope内无剩余缺陷（same-family/provisional）。消息接口/旧文件保留；ROS/CUDA/Orin仍明确未执行。目标源码目录已独立准备，Github设备登录CD0E-C687等待用户授权。

GitHub: 54788955f7b2ed71033c0c9796e0ca0f0390031f 已成功push至main；远端复核完成。原始vendor文件保留上游空白，git whitespace检查仅对此有提示，不修改上游证据。环境中旧post-commit数据收集hook路径缺失，但commit/push均成功，不修改该全局hook。
