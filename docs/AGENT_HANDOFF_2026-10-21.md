> 历史记录：2026-10-22 已撤掉旧现场采集/压力测试与运行时诊断。本文中的旧脚本、诊断开关和构建路径不再是当前操作指令；清理状态见 `docs/PRODUCTION_CLEANUP_2026-10-22.md`。

# WBR：NUC 底盘控制迁移与后续开发交接

更新：2026-10-21，Asia/Shanghai。本文面向接手 Foxglove、ROS 2 模块通讯、重构及新算法开发的 agent。

## 1. 先读结论与用户意图

底盘算法已从 dust-hpm6750 MCU 移到 wbr-nuc，MCU 负责采集和 USB 转发。
用户已在真实机器人上运行两次各 30 秒的使能控制，两次均正常结束、
`missed control releases=0`，用户反馈“这两次测试都没有发现控制性能问题”。
这是短时实机基线，不是长期稳定性、全动作或全部故障安全认证。

用户确认原 MCU chassis_controller 在该实物上正常工作，要求沿用其算法、
方向、零点和参数，不再展开重复实物标定。用户希望减少冗余测试，下一步
围绕实际功能开发。不要擅自调整控制增益、符号、零点或重新从头标定。
代码必须有可读性、可维护性，不能只追求能运行。

后续方向：Foxglove 可视化调试、ROS 2 与其他模块通讯、代码重构、新算法。
这几项尚未实施。不要把本文的架构建议当成已经存在的实现。

## 2. 仓库、分支、机器与版本

| 对象 | 路径 / 分支 |
|---|---|
| 主机 GitHub | https://github.com/NoneOfEver/wbr_control_companion |
| 主机分支 | feature/usb-can-host-controller |
| 已实跑主机提交 | `3c74661bc3776f407c735d2a1cbf218823c189b5` |
| NUC 仓库 | `/home/wbr-infantry/wbr_realtime_lab/src/wbr_control_companion` |
| NUC 连接 | `ssh wbr-nuc`；用户 wbr-infantry，当前 SSH 配置指向 192.168.1.94 |
| Orb Linux 仓库 | `/home/panpoming/wbr_control_companion` |
| Mac 可访问 Orb 路径 | `/Users/panpoming/OrbStack/ubuntu/home/panpoming/wbr_control_companion` |
| MCU GitHub | https://github.com/NoneOfEver/wbr_control |
| MCU 分支 / HEAD | feature/usb-can-gateway / `12a68ec850e56b15fa7dbdfdc5e2374df25670ae` |
| MCU 工作树 | `/Users/panpoming/Documents/zephyr_projects/wbr_control_ws/wbr_control` |
| Zephyr 工作区 | `/Users/panpoming/Documents/zephyr_projects/wbr_control_ws` |
| HPM 支持仓库 | 工作区下 `hpm_support`，含驱动改动，不能当成干净上游 |

交接时 NUC/Orb 主机源码工作树干净；本文提交会使文档 HEAD 前进，运行源码仍
以上述提交为基线。NUC 内核 `5.15.0-1114-realtime`；没有从 `/opt/ros` 列出
已有 ROS 发行版，不能假定 ROS 2 已安装。版本、网络地址和运行状态需接手时复核。

MCU 和 hpm_support 有大量其他未提交修改，包含其他开发工作。
**不要 reset/clean/全量暂存或覆盖这些改动。** MCU HEAD 单独不足以重建当前刷入镜像。
HPM remote 属于上游 HNUST-Dust，不要向上游推送本地驱动修改。

用户曾因 NUC SSH 离线，要求在本机 Orb 编辑、通过 GitHub 同步。SSH 恢复后已
同步到 NUC。NUC origin 曾有分支抓取问题，不能只凭 Already up to date 判断一致；
实际核对 branch、HEAD、工作树。此前也用 Git bundle 传递同一 GitHub 提交并 ff-only。

## 3. 当前正式运行架构

```text
CAN3 左轮/左B/左D ─┐
CAN2 右轮/右B/右D ─┤
UART10 SBUS ───────┤  HPM6750 + RTOS
UART2 HI91 ────────┤  usb_can_bridge
板载 SPI 原始 IMU ─┘        │ HS USB Bulk
                           │ 128-byte OUT / 64-byte IN
                           ▼
                    NUC libusb 事件线程
                       解码、发布快照
                           │
                    1 kHz 控制线程
                 原 chassis_controller 算法
                           │
                  每周期一个合并 OUT
                           ▼
                  MCU 每条 CAN 按 W/B/D 提交
```

- USB 实际为 480 Mbps High Speed；选择 Bulk，不再默认测试 Interrupt。
- 四个 IN transfer 预提交。启动必须先完成事件线程 RT 设置与所有 IN 提交，
  再 Hello/Ready 握手；不能恢复为先 Hello 后启动接收。
- 命令和反馈解耦；反馈到达后独立上报，不等待本周期六路全部完成。
- 生产路径没有每条命令 ACK；OUT 缓冲在 USB 完成时复用，不等电机反馈。
- 控制线程/USB 事件线程默认同在 CPU3、SCHED_FIFO 80，使用内存锁定。
  CPU 未配置隔离，不能宣称 isolated CPU。参数可调整，保持当前值是已验证基线。
- 控制线程读取一致快照、调用 Tick、收集本周期电机帧后发送。
  默认不允许输出，`--enable-output` 才允许遥控门控使能；启动后还须观察到
  显式失能开关姿态。原算法的 arming/recovery 条件继续生效。
- `SensorStore` 是优先级继承 mutex 保护的快照，不是无锁结构。
- MCU 不跑原底盘算法、AHRS 或 VOFA；板载 IMU 是原始数据，控制 IMU 是 HI91。
- referee UART 当前无实际数据，不能宣称该传感器已验证。
- USB 协议在主机 `chassis_controller/include/wbr_bridge/bridge_protocol.hpp`，
  MCU `src/protocols/usb_bridge/bridge_protocol.hpp`；协议修改必须两端兼容。

### 六路映射

| 顺序/位 | 电机 | CAN | command ID | feedback ID |
|---|---|---:|---:|---:|
| 0 | left_wheel | 3 | 0x200 | 0x201 |
| 1 | right_wheel | 2 | 0x200 | 0x201 |
| 2 | left_joint_b | 3 | 0x000 | 0x010 |
| 3 | left_joint_d | 3 | 0x003 | 0x013 |
| 4 | right_joint_b | 2 | 0x001 | 0x011 |
| 5 | right_joint_d | 2 | 0x002 | 0x012 |

电机过期阈值 3000 us；IMU 3 ms；遥控断流 1000 ms。CAN stale mask
左侧=0x0d（13），右侧=0x32（50）。整体 feedback_valid 还依赖 IMU 与腿运动学。

## 4. 源码入口与构建运行

主机的生产迁移目标是 **chassis_controller 子目录**；仓库顶层还保留旧
MPC/EtherCAT 等目标。不要误编顶层后声称已测试底盘迁移目标。

| 入口 | 职责 |
|---|---|
| `chassis_controller/runtime/main.cpp` | 1 kHz 入口、启动失能门控、Tick、USB 发送 |
| `runtime/usb_link.*` | 单一 USB 事件 owner、握手、transfer 生命周期、故障关闭 |
| `runtime/sensor_decoder.*` | CAN/SBUS/HI91 解码、序号与有效性 |
| `runtime/hi91_time_guard.hpp` | HI91 时间基准重建 |
| `runtime/sensor_store.hpp` | 采集快照及可观测计数 |
| `runtime/motor_commands.hpp` | FrameSink，收集每周期电机指令 |
| `control/chassis_controller.*` | 原控制管线 |
| `control/chassis_input_reader.*` | 新鲜度、运动学、控制输入 |
| `control/chassis_state_machine.*` | 使能/恢复/故障/动作状态转换 |
| `control/balance_controller.*` | LQR、运动估计、VMC 等调用 |
| `config/*.yaml` 与 schema | 参数来源，头文件由构建生成 |
| `source_manifest.json` | 导入原 MCU 文件时的哈希，不代表当前 MCU 工作树完全一致 |
| `measurements/` 与 `tests/` | 独立采集、回归工具，不是生产控制入口 |

依赖：Linux/C++20、CMake、Eigen3、pthread、libusb 开发包、Python3、PyYAML。

NUC 构建：

```bash
cd ~/wbr_realtime_lab/src/wbr_control_companion
./scripts/build_chassis_controller.sh
```

Orb 构建可从 Mac 调用：

```bash
orb -m ubuntu bash -lc 'cd ~/wbr_control_companion && ./scripts/build_chassis_controller.sh'
```

交接时 NUC 子工程 9/9 测试通过；Orb 10/10，额外一项是可选的原 MCU 数值对照。
这项对照覆盖 balance fixture，不等于全控制状态机、全部硬件行为完全等价。

离线 / 默认禁止输出 / 已验证真实使能命令：

```bash
./scripts/run_chassis_controller.sh --simulate --no-rt --seconds 1
sudo ./scripts/run_chassis_controller.sh --seconds 30
sudo ./scripts/run_chassis_controller.sh --enable-output --seconds 30
```

最后一条会允许实际电机输出。用户此前在现场手动执行，不由 agent 在 SSH 中
静默启动。启动保持遥控失能，进程就绪后再操作；失能开关或 Ctrl-C 停止。
root 用于 FIFO/mlock 等，普通用户 USB 访问已可用；不要在代码里存 sudo 密码。

## 5. MCU 镜像与 UART 驱动：重建前必读

MCU 应用：`wbr_control/src/usb_can_bridge`。当前刷入的是 `/tmp/wbr-uart10-plic-probe`
构建产物，包含候选 UART 修复及诊断选项，**并非只用 prj.conf 默认值的无诊断版**。

已确认配置：

```text
CONFIG_WBR_UART10_LINE_IDLE=y
CONFIG_WBR_UART10_PLIC_PROBE=y
CONFIG_WBR_UART10_IIR_HISTOGRAM=y
CONFIG_WBR_GATEWAY_FAULT_DIAGNOSTICS=y
CONFIG_WBR_CONTROL_ONBOARD_IMU_RAW=y
CONFIG_WBR_UART10_FIFO_ERRATUM_PROBE=n
```

`UART10_LINE_IDLE` 默认关闭；直接新建默认构建可能回归 SBUS 128 字节积攒。
当前 UART10 补丁在 hpm_support 中，MCU 仓库另存
`src/usb_can_bridge/diagnostics/hpm_uart_rx_probe.patch`，须核对是否已应用，不能重复盲打。
详细证据见 `src/usb_can_bridge/UART10_INVESTIGATION.md`，尤其 545 行后记录。

原因/修复：RX DMA 会清掉 FIFO timeout 条件，原 UART10 只在 128B DMA 满时上报，
产生约 84 ms 间隔。独立 RX 线空闲检测复用已有 PC04/UART10 与 PB13/TRGM1_P03
连接，TRGM1→GPTMR2，无边沿 1000 us 后部分发布。保留 128B 异步双缓冲、
非缓存内存、DMA offset/generation 检查，未改为缩小缓冲。E00018 要求 RFIFOT=0、
单字节 burst；这一项单独不足以解决问题。HI91 DMA 路径未按此改写。

当前产物 SHA256（产物不保证 /tmp 永久保存）：

```text
zephyr.bin de041c31c136889951429522e052b5bd2a7f916b09bde4c2878a88505bf460d6
.config    e8beb5b055fa1f9d5af7c47490be1ad5a9e8ad16e05097d3e6093c6f5120befc
```

west 路径 `/Users/panpoming/Documents/zephyr_projects/.venv/bin/west`，现有构建可
`CCACHE_DISABLE=1 .../west build -d /tmp/wbr-uart10-plic-probe`。
重建/刷写前先检查 .config、源码脏改与驱动补丁。不要因更换 NUC 代码就重新刷 MCU。
将来移除诊断要保留 LINE_IDLE 功能，并对无诊断固件重新回归，不能直接宣称等价。

## 6. 必须保留的关键修复

### USB 启动与断开

1. IN 全部预提交后才 Hello。旧顺序有约 4.5 ms RT 设置窗口，导致 HI91 启动
   排队长尾；固定顺序后对应观测最大值从数 ms 降到约 313 us。
2. libusb 错误后不能让 FIFO 事件线程继续空转。实测 NO_DEVICE 后 233 万次
   快速返回、持续约 1.591 s，控制线程晚约 1.650 s 才运行。
   当前错误线程阻塞等待 Close，Close 在同一 mutex 下设置 closing 并通知，
   单一 event owner 取消/排空 transfer。不要引入第二个事件处理 owner。
3. IN/OUT 错误均保留实际 status 和 actual/expected length。status=5 是
   LIBUSB_TRANSFER_NO_DEVICE。不能把所有错误都叫“短包”或“物理断线”。
4. healthy() 先加载接收时间再读当前时间，避免并发新时间戳造成无符号减法下溢。
5. MCU 20 ms command lease：主机退出/失联后提交 wheel zero / DM Exit，清除
   session；查询诊断会在 Hello 自己清理前保留旧状态快照。软件提交不等于
   CAN 硬件发送完成或电机实际停止。

### SBUS：刚恢复的原 MCU 策略（3c74661）

- 正常 CH4-low + CH5-low 为显式失能，其余组合沿用原功能映射。
- `frame_lost` 优先处理：丢弃该帧、不更新遥控发布序号，保留上一份状态。
- 从首次 lost 开始持续 1000 ms 后发布一次失能，不因后续 lost 帧重置计时。
  正常帧恢复清除计时。USB 事件 owner 即使没有新 UART 帧也检查该计时。
- 没有 frame_lost 的 failsafe 帧立即失能；两标志同时出现按原 MCU 的
  frame_lost 优先逻辑处理。真正断流继续由输入层 1000 ms 新鲜度判定。
- 迁移初版误把每个 frame_lost 都立刻失能，约 14 ms 后又使能，已修正。
  不要“简化”回去。相关单测覆盖时间边界、只发布一次及恢复。
- 本轮开关复测 4 次使能/4 次失能均对应正常新帧，69 个 lost+enabled tick
  保留状态但未制造额外边沿，最终失能。此前采集工具的 unsafe 判据已同步修正。

### HI91 重启时间基准（f1e7d9a）

旧代码重启后设备 system_time_ms 倒退，持续拒绝有效新帧。实测 37058 帧被拒。
当前 guard 仅在旧样本超时后，用至少 3 个有效、CRC 正确、设备时间递增的
候选帧建立新基准；相邻设备/host sample 间隔不大于 20 ms，观察跨度至少 2 ms。
重复、无效、乱序与流缺口清除候选资格；一次缓存突发不能直接恢复。
正常 uint32 wrap 仍视为前进。正常帧 legacy CRC 策略仍 permissive，未全局改 strict。
恢复只更新输入数据，不表示自动使能算法。实测 time_rebases=1 后持续恢复到结束。

## 7. 已有硬件结果与证据

NUC 原始结果目录 `/home/wbr-infantry/wbr_realtime_lab/results`，不是全部已提交 Git。
需要复盘时读取下面文件，不能假定本 Mac Downloads 中有副本。

| 结果 | 证据 / 结论 |
|---|---|
| 全负载 60 s | `full_load_zero_20261021-034125`：60000 OUT、0 skipped、USB 全完成；六 CAN/SBUS/HI91/板载 IMU 并发正常 |
| 进程 TERM/KILL/STOP-CONT | run_fault_recovery.sh，非 RT 与用户 RT 轮次都通过；约 20–30 ms MCU 停止提交与旧 session 清除；不含物理停机 |
| 物理 USB 拔插 | usb_unplug_fixed_*：自动停止提交、旧会话失效、手动确认后新停止态会话均 PASS；MCU 独立供电 |
| 遥控断流 | `remote_poweroff_20261021-165117.csv`：age 1000 ms fresh，1001 ms stale；恢复新序号 |
| HI91 恢复 | `hi91_recovery_check_20261021-194036.csv`：7.173 s stale，18.839 s 重建，末尾 age 372 us、fresh=1 |
| 左侧失联 | `motor_loss_left_20261021-195715.csv`：实际约 2.75 s；mask 0x0d，右侧不超时；轮恢复附近有短暂额外超时 |
| 右侧失联 | `motor_loss_right_20261021-200220.csv`：约 7.86 s；mask 0x32，左侧不超时，最终 feedback_valid=1 |
| 静态 USB/HI91 | `hi91_baseline_20261021-193635.csv`：60 s，无过期、无拒帧、约 1 kHz |
| 遥控策略复测 | `remote_policy_check_20261021-212015.csv`：无短时丢帧附加使能边沿 |
| 真实闭环 | 用户在 3c74661 连续做两轮 --enable-output --seconds 30：零漏周期、未观察到性能问题；只有用户贴回日志，无内部完整控制遥测 |

已有采集脚本：run_full_load_zero.sh、run_fault_recovery.sh、run_usb_unplug.sh、
run_remote_loss.sh、run_hi91_loss.sh、run_hi91_baseline.sh、
run_motor_feedback_loss.sh、run_conventions.sh、run_uart_forwarding.sh。
各测试操作与边界见 measurements/ 下文档。部分脚本保留通用提示，与具体测试
操作可能不同；不要让使用者一边收到“静态对照”要求一边收到“拔插”提示。

被动方向检查已观察（随后用户要求以原 MCU 参数为准，未继续绝对标定）：
- 用户确认机头向下：pitch 更负；pitch 导数与 X 映射 rate 比例约 0.992。
- 横滚按约定左侧降低：roll 更负；对应 rate 比例约 0.998。
- 左轮顶部向前：原始 RPM 负；右轮同动作：正。左右映射正确。
  当前 feedback sign 左+1/右-1，转换后两侧前向均为负，不要凭直觉反号。
- B/D 同时手动运动无法证明独立映射；用户明确不要求继续重复标定。
- 减速比、绝对零点、命令力矩方向没有在这些被动实验中独立证明。

历史延迟数字必须区分口径：Bulk ACK RTT 曾约 194 us，但它是提交确认往返，
不是六电机执行 RTT；CAN 三帧批次完成约 380 us 含串行发送与仲裁；
跨 1 kHz 采样相位会增加反馈使用年龄。NUC/MCU 未同步时钟，不能把两端
绝对时间戳直接相减称为单向延迟。不要从历史压力测试数字推导新算法稳定裕度。

## 8. 已知问题与未覆盖项

- 热插拔 HI91 或操作电机供电时，多轮出现 USB IN/OUT NO_DEVICE；静态轮次正常，
  也存在成功插拔轮次。用户要求暂搁置，认为与操作/硬件有关。
  **硬件根因未证实，不要写成已修复或已确诊。** 实际发生 NO_DEVICE 仍须停机，
  不能屏蔽错误、重放旧命令或静默重连恢复运动。
- 当前固件带诊断；无诊断生产构建尚未重跑全套验证。
- 两次闭环正常不代表长时间、全部动作模式、带载故障停机均已测。
- 未以启用状态完成全部反馈丢失/遥控失效/断 USB 的实际停机时间测量。
- 当前 ROS 2 / Foxglove 尚无集成，协议到 ROS 帧、单位与时间映射待设计。
- 主机 measurement 文件 remote_loss.cpp 复用多种模式，后续可整理，但不应与
  控制策略变更、ROS 引入、新算法同时混在一次难以审查的改动中。
- PREALGORITHM_ACCEPTANCE.md 及旧 README 中的部分“pending”是早期检查单；
  本文列出的已实测证据优先，并保留未覆盖边界。

## 9. 后续开发建议（尚未实现）

### Foxglove 与 ROS 2

先确定 ROS 2 发行版、消息类型和模块接口；Foxglove 用 Bridge/SDK 的方式待选。
保持 1 kHz 控制与 USB 接收在现有原生线程中，ROS executor、DDS、序列化、网络、
可视化和磁盘日志放在非实时侧。使用有界快照/队列降采样，队列满允许丢遥测，
不要阻塞控制线程。不要把每次 DDS publish/日志输出直接塞进 Tick 或 USB 回调。

建议先发布：姿态/角速度/加速度、六电机反馈与年龄、腿长/腿角、控制状态与
transition reason、遥控使能、力矩命令、周期时长/漏周期、USB 健康。
区分实测、估计、目标、最终限幅输出，显示单位和数据有效性。对陈旧数据显式
标注，不拿缓存 valid 当 fresh。IMU 目前角度 deg、角速度 rad/s、加速度 g；
ROS 常用 SI，转换和坐标系必须写清楚，不可直接把 Euler 填 quaternion。

host monotonic 时钟用于 freshness；ROS time 用于消息生态，两者需显式映射，
不能因 /clock 或系统校时跳变破坏 watchdog。TF/body/world 的正方向须与
原算法约定一致。其他模块命令应有 source/sequence/timestamp/timeout/范围检查。
遥控失能、传感器保护、输出限幅及 MCU lease 不被上层命令绕过。当前没有
“用户已授权 ROS 自动使能”的约定。

### 重构与新算法

保持现有控制器作为可选、可回退基线。推荐拆分：采集/时间与有效性、
控制核心、输出安全门控、非实时观测/ROS 接口。FrameSink 是可用边界，
不要为可视化复制 CAN/UART 解码与方向转换。
先用回放/离线对照验证纯重构的同输入输出一致，再做单项硬件确认。
新算法用独立参数集与显式选择，先对照/影子计算再切换真实输出；不能在
“增加 Foxglove”提交里悄悄更改控制增益、符号、限幅或状态机。

用户不希望重复冗余测试；围绕实际修改挑选必要回归，并说明它解决的具体风险。

## 10. 接手的最短路线

1. 核对两端 HEAD 与工作树，阅读本文、runtime/main.cpp、usb_link、decoder、
   control 的 Tick/状态机；注意 MCU 与 HPM 工作树不可覆盖。
2. 在当前 Linux 环境构建 chassis_controller 子工程，保留现有 9 项回归。
3. 和用户确定 Foxglove/ROS 2 最先需要的状态与接口；不因工具存在就自动安装。
4. 先实现非实时只读观测，验证不会破坏现有 RT 时序；随后扩展模块通讯。
5. 每次改动记录源码版本、运行配置与证据，不自动在远端启动有输出的控制。

可直接交给下一位 agent 的任务：
“以本文记录的 NUC/MCU 已验证基线继续开发，优先增加 Foxglove 可视化与 ROS 2
非实时接口，保留原控制器及全部输出门控/超时策略；随后分阶段重构并开发新算法。
不要重复实物标定，不覆盖 MCU/HPM 未提交修改，不把 ROS 通讯放入实时闭环。”
