# 三模式实验与改动

先看REPORT.md；playback.html是独立回放，可直接用浏览器打开。requested_scenarios.png、campus_scenarios.png、mode_timeline.png来自实际积分轨迹。

代码已在src中修改。three_mode_changes.patch可审阅；baseline/保存修改前源码。changes_manifest.json列出所有源文件改动。本项目没有Git仓库，没有创建提交。

复现需要Python与C++17编译器。当前本机实验环境在.venv中；移到别的电脑时创建独立虚拟环境并安装requirements.txt。先编译C++策略桥：

```bash
c++ -std=c++17 -shared -fPIC diagnostics/multimode_20261108/driver_bridge.cpp -o diagnostics/multimode_20261108/libdriver_policy.dylib
python diagnostics/multimode_20261108/campaign.py --worker-delay .25 --duration 90
python diagnostics/multimode_20261108/robustness.py
python diagnostics/multimode_20261108/tune.py
python diagnostics/multimode_20261108/real_worker_smoke.py
python -m pytest src/rda_dwa_real/test -q
```

Linux上可将共享库后缀改为.so，同时修改campaign.py的库路径。实验器不启动ROS或实车UDP。

车机部署需要同时构建rda_dwa、rda_dwa_real、control_msgs、car_ctr。新协议依赖/car_driver/mode_state和/rda/body_motion，不能只更新规划器而保留旧驱动。建议车机先构建、再在低速条件下核对方向、速度单位和实际到位时序；当前READY是timed_open_loop，不是实际编码器确认。

开发中失败日志和旧调参结果保留。最终数字只使用candidate_results.json、robustness_final_results.json、tuning_results.json、final_summary.json；它们不是实车测量。
