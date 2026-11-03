# V24 避障回线系统——已上车实测确认版

参赛团队确认：两米直行、左右选侧绕障、返回原直线及终点停车均已上车测试通过。确认记录日期：2026-10-30。

## 文件入口

- 工程：drive_vision_tof_v24.gprj；顶层 ackermann_auto_top。
- 已实测固件：prebuilt/24_V24_IMU_RETURN_0x2401.fs；GAO 0x2401。
- 操作和参数：开始使用_V24.md。
- 实车确认、仿真与构建记录：reports/交付验证.md。
- 提交文件校验：SUBMISSION_SHA256.json。

## 目录用途

| 目录 | 内容 |
|---|---|
| rtl、src | 控制逻辑、外设模块和工程依赖 IP |
| constraints | 时钟/综合约束；引脚约束在 src/ov5640_ddr3_hdmi.cst |
| debug | 工程启用的 GAO 配置，重新编译需保留 |
| prebuilt | 本次实测固件及 SHA256 |
| tests | 九组仿真源码及运行工具 |
| reports | 验证记录、仿真日志与时序证据 |
| web | 可选网络视频接收工具 |
| impl | 可重新生成的构建产物，不属于源码依赖；可不提交 |

使用 Gowin V1.9.12.03。运行 build.cmd 或 gw_sh build.tcl 可重新构建，脚本固定顶层及 CPU/SSPI GPIO 复用。安装路径与仿真工具路径可按本机调整。

本次整理不改变 HDL、约束、工程文件和实测固件。实车功能确认不等同于静态时序通过：当前构建仍有 176 个 setup、312 个 hold 违规端点，原始报告保留。未提供实测误差、重复次数和视频文件，不补写定量成绩。
