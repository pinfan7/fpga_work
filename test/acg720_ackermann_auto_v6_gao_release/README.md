# ACG720 阿克曼自动测试 V6 / GAO

先阅读本目录 `GAO操作步骤_先看这里.md`。

- 高云工程：`ackermann_auto_v6_gao.gprj`，顶层 `ackermann_auto_top`。
- 烧录：`prebuilt/ACKERMANN_AUTO_V6_GAO_NOIMU_D16.fs`。
- 匹配的GAO配置：`prebuilt/drive_v6.rao`；`open_gao.cmd`打开分析仪。
- 引脚：舵机PWM为 **F13**；左编码器A相为 **D16**（不再使用B16）。完整定义见 `constraints/drive.cst`。IMU保持闲置。
- S1启动六段各1m测试：前进、后退、左15度弧线前进、保持左舵倒车沿弧线返回、右15度弧线前进、保持右舵倒车沿弧线返回；S3中止，S0复位。
- 两段弧线倒车以编码器累计距离近似原路返回；没有位置/航向闭环保证完全重合。左右弧线也不是90度转弯。
- 左弧线倒退返回后，先停车回正1秒，再打右舵等待1秒，最后启动右弧线行驶。
- 两轮独立计数/测速/PI/PWM；左105%、右100%。修正后的差值保护考虑主动目标比例，并增加500ms起步宽限和150ms持续判定，未删除明显异常保护。
- 新增故障现场与编码器非法跳变诊断；GAO以50Hz记录控制快照，2048点，故障/完成/中止触发。
- 本目录提供源码、仿真、固件和GAO配置；构建及测试脚本不自动连接板卡。

捕获后导出VCD到 `captures/run01.vcd`，可用 `debug/read_gao_vcd.py`离线提取摘要。已有实车GAO分析见 `reports/实车GAO分析_20261024.md`；历史采集不能作为当前故障仍存在的证明。

2026-11-02按本目录源码和F13/D16约束重新构建。固件及构建输入哈希、静态时序和硬件验证状态见 `release_manifest.json`、`reports/验证结果.md`。当前新固件已由参赛团队于2026-11-02确认上车测试通过，记录见 `reports/hardware_test_record.json`；实车结果与静态时序独立记录。

离线仿真：在上一级 `test` 目录执行 `./run_simulations.ps1 -Project v6 -IcarusBin <Icarus安装bin目录>`，或执行本目录 `sim/run_tests.ps1` 并传同一参数。

常用参数：

|文件|参数|
|---|---|
|rtl/ackermann_auto_top.v|LEFT_TRIM_PERCENT=105、RIGHT_TRIM_PERCENT=100；TURN_PULSE_US=167|
|rtl/auto_sequence.v|DIST_MM=1000；MAX_RPM=60；SLOW_RPM=20；WAIT_MS=1000|
|rtl/auto_sequence.v|IMBALANCE_GRACE_MS=500；IMBALANCE_HOLD_MS=150|

左右目标比例统一从顶层传入驱动与序列层；请不要只改其中一个模块的默认值。编译后使用新生成的impl/pnr文件和debug/drive_v6.rao，prebuilt不会自动更新。
