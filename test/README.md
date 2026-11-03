# 独立测试工程

| 工程 | 功能 | 舵机信号 |
|---|---|---|
| acg720_ackermann_auto_v6_gao_release | 前进、后退、左右弧线及倒退返回、GAO采集；IMU闲置 | F13，左编码器A相D16 |
| acg720_ackermann_servo_v2 | 舵机按键及PWM独立测试 | E13，当前源码±10° |
| acg720_tof3_test | 三路TOF接收、配置、LED与串口数据显示 | E13保留输出恒低，不驱动舵机 |

独立工程不是运行时可自动切换的模块。更换固件前按各工程的约束核对接线。

## 一次运行全部仿真

安装Icarus Verilog，执行：

`./run_simulations.ps1 -IcarusBin <Icarus安装bin目录>`

也可通过IVERILOG_BIN环境变量指定，或将iverilog和vvp加入PATH。用-Project v6、servo或tof可只执行一组。测试不会连接板卡；日志保存在各工程reports目录，临时编译产物不加入提交目录。

2026-11-02六项测试均通过，包括更新后的运动序列、10°舵机控制、TOF最后有效值保留及超时测试。

## 构建与下载

各工程使用build.tcl设置顶层与GPIO复用选项并完整构建。只打开.gprj后直接Run All时，应核对同样的选项；本机.gprj.user未提交。

V6已按F13/D16重新构建并同步prebuilt，源码哈希和时序状态见其release_manifest.json。该次新固件已由参赛团队确认上车测试通过，记录与固件哈希关联。舵机和TOF的prebuilt保留历史快照，当前源码修改不自动更新历史固件。

编译缓存impl已清理，可通过完整构建重新生成；时序、引脚报告及历史实车GAO波形已另存。修改前完整备份在main_code/test之外，不属于提交内容。
