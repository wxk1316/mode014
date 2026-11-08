# CV/IMM跟踪、RViz和预处理检查

用户要求：动态目标滤波可选择CV或IMM，继续匈牙利关联；发布MarkerArray在RViz展示；复核逻辑和预处理；将完整感知代码上传https://github.com/wxk1316/campus_perception.git。

输入/输出保持原ObstacleArray接口。变更限新campus_lidar_perception包和先前新增的car_launch入口。旧检测/定位/规划源码不修改。

基线：none (car_gpt非Git)。当前源包副本baseline保留在本stage目录。目标Git仓库为指定空公开仓库，必须先核实可写账号；不能把本地commit当作上传成功。

成功：本机累计pytest、模型预处理检查、MarkerArray构造检查、源码/部署打包、真实GitHub push与远端提交核对。当前Mac不具ROS2/CUDA，实车/GPU验收不伪造。
