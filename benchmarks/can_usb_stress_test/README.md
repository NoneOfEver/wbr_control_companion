# 正式通信链路的六电机 CAN + USB 1 kHz 测试

目标板：dust-hpm6750。测试链接并启动正式工程的 CAN 调度、USB 会话和
VOFA 示波模块，只在应用层生成零输出指令、回显 USB 测试包和发布统计。
不启动底盘平衡、MPC 控制、遥控器或其他电机指令生产者。

所有关节 position/velocity/kp/kd/torque 均为 0，所有 DJI 轮毂电流槽位均为 0。
达妙 MIT 的整数编码有协议分辨率内的零点量化误差。程序不提供位置保持，
不要求架起机器人。

## 复用关系与代码入口

| 测试代码 | 调用的正式工程接口 |
|---|---|
| `src/main.cpp` | 只负责等待主机、阶段切换、1 kHz 释放计数、超时和结束 |
| `src/motor_can.cpp` | `InitializeCanDispatch()`、`SubmitCanStandardFrame()`、`ReadCanTxSlotStats()` |
| `src/usb_echo.cpp` | `InitializeUsbSession()`、`ReceiveUsb()`、`SendUsb()`、`WaitUsbTxComplete()` |
| `src/telemetry.cpp` | 写入 `msg::latest_oscilloscope_sample`，由正式 `OscilloscopeModule` 输出 |
| `src/test_config.hpp` | 测试时长、周期、超时、重试参数 |
| `src/telemetry.hpp` | 33 个 VOFA 通道的具名索引 |

本测试没有自己的 CAN 过滤器、硬件收发回调、USB 描述符、端点 DMA 缓冲区，
也没有自己的 UART 驱动/JustFloat 编码器。CAN 批量发送、槽位覆盖、发送完成同步
均由正式 `can_dispatch` 负责。设备、引脚和波特率来自正式板级 DTS；
电机映射和 MIT 编码范围仍来自正式参数文件。

正式 `usb_session` 在未启用 PC-link/MPC 业务时提供通用原始传输模式，
通过公共消息队列接收 OUT，并在线程中重启接收。USB 测试回显仅是这个接口的调用者。
通用会话的 HS 间隔使用公共 Kconfig `WBR_CONTROL_USB_HS_INTERVAL`；
本测试设为 1（125 µs），与当前正式 MPC 配置一致，FS 仍为 1 ms。
NUC 按 1 kHz 调度实际交换，端点轮询机会不等于通信实测速率。

## 电机配置

当前正式 `motors_params.yaml`：

| 序号 | 电机 | 总线 | 命令 ID | 反馈 ID |
|---|---|---|---|---|
| 0 | 左轮 | CAN3 | 0x200 | 0x201 |
| 1 | 右轮 | CAN2 | 0x200 | 0x201 |
| 2 | 左关节 B | CAN3 | 0x000 | 0x010 |
| 3 | 左关节 D | CAN3 | 0x003 | 0x013 |
| 4 | 右关节 B | CAN2 | 0x001 | 0x011 |
| 5 | 右关节 D | CAN2 | 0x002 | 0x012 |

CAN2/CAN3 当前均为 1 Mbps，每毫秒每路提交三条 8 字节经典 CAN 指令。
如果每个电机也反馈 1 kHz，每路约 6000 帧/秒；按标准帧含帧间隔、未计填充
111 bit 估算约 66.6% 占用，位填充、重发会进一步增加占用。

## 生命周期

1. 启动正式通信组件，给轮毂零电流、关节退出控制，并等待 CAN 槽位空闲。
2. 等待 NUC USB 测试流量。
3. 发送零指令并等待完成，再发送关节 MIT 使能并等待完成，最后发送零指令。
   每个阶段检查实际完成计数，不能只检查调用返回成功，也不能让下一阶段覆盖使能帧。
4. 运行 60 秒，每毫秒向正式 CAN 调度提交六条零指令；错过的释放周期记入统计，
   不突发补发旧指令。USB 超过 1 秒没有流量也会结束测试。
5. 停止周期提交，等待已有发送排空，然后重复轮毂零电流和关节退出指令。
6. USB 回显和 VOFA 继续运行，便于检查结果。重启板子后才能重新开始电机测试。

正式调度采用最新值槽位：提交次数、驱动接受次数、硬件完成次数可能不同。
因此本测试不会将“主循环运行 1 kHz”或“提交成功”当成“六电机各完成 1 kHz”。

## 构建与运行

在工作区根目录，使用现有 Zephyr 环境：

```sh
CCACHE_DISABLE=1 west build -b dust-hpm6750 \
  -s wbr_control/test/can_usb_stress_test -d wbr_control/build/can_usb_stress_test
west flash -d wbr_control/build/can_usb_stress_test
```

USB 接到 NUC 后，从开发机执行：

```sh
ssh wbr-nuc '~/wbr_realtime_lab/src/can_usb_stress_test/tools/run_nuc.sh'
```

NUC 已部署 PyUSB/libusb 和匹配 VID/PID 的权限规则。
脚本保存在 `~/wbr_realtime_lab/src/can_usb_stress_test/tools/`，
JSON 和终端日志保存到 `~/wbr_realtime_lab/results/can_usb_stress_test/<时间戳>/`。
也可在有 PyUSB/libusb 的其他主机执行 `tools/run.py --seconds 65`。

USB 使用正式会话的 VID/PID 0x34b7/0xffff，接口 0，OUT 0x01 / IN 0x81，
Vendor Interrupt，非串口。每包固定 64 字节：小端 magic 0x53545253、序号、
上一包主机 RTT（整数 µs），之后 52 字节为校验负载。板端完整回显。
主机默认使用独立 OUT/IN 线程进行双向测试，最多 8 包在途，并按序号完整校验回显。
包格式和默认运行命令不变；`--mode roundtrip` 可恢复单包串行往返对照。

## VOFA telemetry：33 通道

UART0，**921600 / 8N1，JustFloat，约 100 Hz**。请把 VOFA 通道数改为 **33**，
重新采集 CSV，避免与旧的 24 通道记录混在一起。正式示波模块仍保留启动探针。
文字诊断通过 RTT 输出。

| 通道（从 0 开始） | 含义 | 单位 |
|---|---|---|
| 0–5 | 对应电机所属成功批次完成，到下一反馈的观测间隔 | µs |
| 6–11 | 对应电机最近两次反馈的间隔 | µs |
| 12 | NUC 测得并在下一 OUT 回传的 USB 完整 RTT | µs |
| 13 | 本包 OUT 回调到本包 IN 完成的时间，包含消息队列和回显线程等待 | µs |
| 14 | 正式 USB 会话 OUT 累计包数 | 包 |
| 15 | USB 端点错误、接收队列丢包、回显协议/等待错误累计 | 次 |
| 16 | CAN 指令主循环错过的 1 ms 释放周期累计 | 次 |
| 17 | 六电机正式调度的队列提交、驱动提交、异步发送错误累计 | 次 |
| 18–23 | 六电机正式路由的累计反馈数 | 帧 |
| 24–29 | 六电机硬件发送完成累计数 | 帧 |
| 30 | 六电机槽位指令覆盖累计数（coalesced） | 次 |
| 31 | 正式 USB 接收队列丢包累计数（通道 15 的子集） | 包 |
| 32 | 正式 USB 会话 IN 完成累计包数 | 包 |

六电机顺序见配置表。通道 0–5 只有成功批次完成之后收到反馈才产生样本；
尚无样本或样本超过 100 ms 时输出 -1。反馈超过 100 ms 没有更新，
对应通道 6–11 输出 -1。主机 RTT 报告超过 100 ms 未更新，通道 12–13 输出 -1。

**通道 0–5 的测量起点已经改变，不能与旧测试的“提交到下一反馈”直接比较。**
达妙/DJI 反馈没有命令序号，DJI 反馈还可能自主周期发送，
且正式 CAN 批次的多条指令共享完成事件，所以这个观测间隔仍不是严格匹配 RTT，
也不是电机实际执行耗时。不能用它推算逐条命令丢包。
通道 12 是同一主机时钟测得的真实 RTT，无需时钟同步。
所有遥测统计是独立原子字段的诊断快照，不是同一控制周期的事务记录。

## 判读

- 同时检查 18–23（反馈）和 24–29（实际发送完成）的窗口增量/实际时长，
  确认六路均接近 1000 Hz。电机自身反馈参数不会被本测试修改。
- 15、16、17、31 应为 0。30 应结合完成频率判断：持续覆盖可能意味着调度跟不上。
- RTT 输出每电机 submitted/enqueued/done/err/rx/coalesced/pending，以及各总线
  state/TEC/REC、异步错误和队列深度。检查总线持续 error-active、TEC/REC 为 0。
- NUC JSON 包含独立 OUT/IN 实际 Hz、校验/传输错误、跳过时隙、最大在途包数和
  RTT p50/p99/max。统计结束会等待已提交的包回收，并把尾部排空时间计入交换频率，
  不用“只发出去的包数”判定通过。RTT 仍从本包提交到匹配的回显返回计时。
  `usb_pass` 只判定 USB，不代表 CAN 通过，也不保证每包 RTT 小于 1 ms。
- 主机默认运行 65 秒以覆盖 CAN 全程；最后几秒没有 CAN 指令压力，需按窗口对照。

测试现已使用正式 CAN normal-mode 路径，不再提供单独的 `STRESS_LOOPBACK` 实现。
纯外设内部回环应使用已有 `test/can_test`。旧回环构建目录不代表本次测试。

## 500 Hz 串行往返与 USB 速度

NUC 启动脚本会打印实际枚举速度。12 Mbps 为 Full-Speed，此时 IN/OUT 的
`bInterval=1` 均表示 1 ms；480 Mbps 为 High-Speed，此时本测试的间隔为 125 µs。
仅设置 HS 间隔不能证明设备实际枚举为 HS。

单线程“写 OUT → 读 IN → 下一包”的交换率受整个 RTT 限制，2 ms RTT 对应
约 500 次交换/秒。该结果不能单独证明设备不能每毫秒收、发各一包。
默认 `duplex` 模式让 IN 请求和 OUT 发送独立进行，仍逐包匹配序号测量完整 RTT，
不会将多包在途后的吞吐率当作单包延迟。最多 8 包在途限制队列堆积；
吞吐不足、校验失败或未收到全部响应仍会判定失败。

```sh
# 默认：1 kHz 发送，独立持续读取 IN，分别统计吞吐和 RTT。
./tools/run_nuc.sh
# 对照：发一包等一包，交换率受单次 RTT 限制。
./tools/run_nuc.sh --mode roundtrip
# 不访问真实 USB 的主机逻辑测试。
python3 tools/test_run.py -v
```

是否真正达到 1 kHz 仍以实际 OUT/IN 完整统计为准。
如果目标包含“每包完整 RTT < 1 ms”，必须另外检查 RTT 分布；
即便双向吞吐达到 1 kHz，2 ms RTT 也仍然不满足这个延迟目标。

## 原生实时上位机

当前性能测试使用 C++/libusb 异步收发，直接编译 companion 的实时线程初始化实现。
默认绑定 CPU 3、SCHED_FIFO 优先级 80，并锁定内存；CPU 绑定不等于 CPU 隔离。
使用 CLOCK_MONOTONIC 的绝对周期 timerfd 驱动 1 kHz 发送，poll 同时等待 USB 事件，
避免将 libusb 向上取整到毫秒的事件等待超时用作发送定时器。
预提交 4 个 IN 请求，测量期间不写磁盘，结束后输出逐包 CSV 与汇总 JSON。
RTT 是主机提交 OUT 到匹配 IN 回调的时间，不是总线分析仪测得的线上时间。

在 NUC 执行：

```sh
cd ~/wbr_realtime_lab/src/can_usb_stress_test
./tools/build_native.sh
# 先重启下位机，重新开始 60 秒 CAN 测试，再运行：
sudo ./tools/run_nuc_rt.sh
```

运行 65 秒，结果保存到 `~/wbr_realtime_lab/results/can_usb_stress_test/时间-native/`。
检查实际 OUT/IN 频率、host_skipped_slots、RTT 分布及错误计数，并与 VOFA 的 CAN
有效运行窗口对照。无实时权限时程序会报错退出；`--no-rt` 仅用于非实时对照。
