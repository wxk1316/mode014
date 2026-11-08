# 场景测试包

将本包解压到完整原工程根目录，保留diagnostics/road_school_suite_20261108/路径。
生产代码版本32f65fc，具体源文件哈希见test_metadata.json。

```bash
python3 diagnostics/road_school_suite_20261108/run_suite.py --group all --seeds 19,37,71 --duration 65
python3 diagnostics/road_school_suite_20261108/run_suite.py --group dynamic --seeds 19,37,71 --warm --duration 35
python3 diagnostics/road_school_suite_20261108/run_suite.py --group all --seeds 19 --clean --duration 65
python3 diagnostics/road_school_suite_20261108/run_perception_tests.py
python3 diagnostics/road_school_suite_20261108/build_report.py
```

依赖NumPy、PyYAML；出图需Matplotlib和可选中文字体。代码检测SciPy可用性，当前本机测试走贪心匹配分支。
本包不含原工程全部源代码、ROS环境或实车bag，仅含生成的测试脚本、参数快照、模拟记录和结果图。
独立物理间距检查100Hz，首次接触终止；未到达、间距不足和正确停车分别记录。
U形退出判定不等同于到达被封闭端挡住的原目标。
不同硬件的搜索预算及实际依赖可能改变部分结果；请参考完整报告及逐帧plans记录。
