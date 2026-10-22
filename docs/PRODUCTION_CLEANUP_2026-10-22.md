# 2026-10-22：撤除迁移期间的现场测试与诊断

按用户要求，删除旧压力/采集/诊断实现，保留已验证的正式控制行为。
这不是对控制参数、使能门控或安全策略的调整。

## 主机

- 删除 measurements/ 及对应现场运行脚本：UART 转发采集、并发零输出压力、
  进程故障/USB 拔插、遥控/HI91/电机失联、方向检查等。
- 删除 USB startup timing、packet observer、OUT completion observer、fault trace、
  diagnostic Hello page 选择接口、HI91 解析计数和只供测试使用的 SBUS 标志快照。
- 保留 USB 错误文字、必要的采集时间戳/序号、新鲜度计算、HI91 时间基准重建、
  MCU-compatible 遥控丢帧策略、启动 IN barrier、故障事件线程阻塞和停止流程。
- 保留离线生产契约、USB 生命周期/重连等自动回归；默认 `BUILD_TESTING=OFF`，
  需要开发回归时显式开启。独立 regression 构建，不链接进正式程序。
- 已保留另一位 agent 未提交的 Foxglove 代码、主入口/控制器观测接口和遥测测试，
  不将这些更改混入本清理提交。

## MCU / HPM UART

- 删除 FLT1、UART URT 探针、专用 Kconfig 开关、计数、Ready 诊断填充及驱动钩子。
- UART10 线空闲检测是正式接收功能，默认开启并在 prj.conf 显式启用；
  保留 DMA 长度/offset/generation 保护与非缓存接收，不回退为 128B 满才发布。
- 移除旧 probe patch，改存功能性 `src/usb_can_bridge/patches/hpm_uart_line_idle.patch`，
  对应 HPM UART driver/header；未改动本来就脏的 HPM CAN 驱动。
- 三个旧独立测试应用 can_usb_stress_test、can_usb_bridge_test、can_usb_polling_test
  从本机工作树撤掉。它们在当前 MCU Git 中本就是未跟踪目录，已先另存归档。
- 正常 Hello/Ready/Stop、20ms lease 和本地 wheel-zero/DM-Exit 行为继续保留。

## 验证与部署边界

- Orb 干净生产构建成功，保留的自动回归通过；包含 Foxglove 工作树时为 5 项。
- 新 MCU 构建 `/tmp/wbr-usb-can-production-clean` 成功；只有 LINE_IDLE 功能选项，
  没有旧探针/故障诊断选项。已检查映射中不含旧诊断符号。
- 只是清理源码/构建，没有刷写 MCU、启动使能控制或重复热插拔测试。
- 清理时 NUC SSH 不通，不能宣称 NUC 运行产物已更新；原始 results/ 未删除。
- 实机仍需用匹配版本验证。历史交接文档是旧实测基线，不是清理后镜像的验证。

## 本机恢复资料

临时归档在 Mac `/tmp`，并非永久发布文件：

- `/tmp/wbr-cleanup-before-20261022.tar.gz`：清理前主机与 MCU/驱动快照。
- `/tmp/wbr-legacy-can-usb-tests-20261022.tar.gz`：三个旧 MCU 测试应用。

提交过的旧代码也可在 Git 历史中查阅。不要全量还原归档覆盖 Foxglove 开发，
不要覆盖 MCU/HPM 中其他未提交的工作。新的正式 UART 补丁已随 MCU 仓库保留。
