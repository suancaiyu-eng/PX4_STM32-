# F1：R9DS / S.BUS 遥控接收

工程：`MDK-ARM/F1.uvprojx`，目标 `F1`，STM32F103C8T6，Arm Compiler 5.06u7。

## 已实现

- R9DS CH9 的 S.BUS 信号接飞控 RC IN，经板内 U903 反相后进入 PB11 / USART3_RX。
- USART3 仅接收：100000 bit/s、8 数据位、偶校验、2 停止位；HAL 配置为 `UART_WORDLENGTH_9B` + `UART_PARITY_EVEN`。不启用 PB10 或 S.BUS 输出。
- 内部 HSI / 2 × 16，SYSCLK / HCLK / PCLK2 为 64 MHz，PCLK1 为 32 MHz。不依赖外部晶振。
- 32 字节 `HAL_UARTEx_ReceiveToIdle_IT` 缓冲，四帧队列，中断搬运、主循环解码。直接编译引用 `../../firmware/rc/sbus.c`，没有复制解码器。
- 启动先丢弃一个空闲分隔段以建立边界；UART 错误、缓冲满后丢弃当前数据段，直到下一次 IDLE 恢复边界。只接受 25 字节标准 S.BUS，帧头 0x0F、帧尾 0x00；不支持 S.BUS2。
- 队列满时丢弃新帧并计数。错误段不更新通道或接收时间；接收重挂失败由主循环重试。

原理图的 X801 标为 24 MHz，U801 原始料号为 STM32F100C8T6B；当前工程依据此前确认的 STM32F103C8T6。F103C8 的无源 HSE 晶振规格为 4–16 MHz，因此本版选择内部时钟。实板晶振和 HSI 下的串口精度均未验证。

## 调用及查看数据

`main.c` 已完成以下接入，无需额外调用初始化：

```c
MX_GPIO_Init();
MX_USART3_UART_Init();
(void)R9DS_Init();

/* Main loop, with no blocking delay: */
R9DS_Process();
(void)R9DS_GetSnapshot(&g_r9ds);
```

Keil Watch 可查看全局 `g_r9ds`：

| 字段 | 含义 |
| --- | --- |
| `channel[0..9]` | R9DS CH1..CH10，原始 0..2047；未校准、未转换为微秒 |
| `channel[10..15]` | S.BUS 剩余协议槽，不代表 R9DS 额外支持的通道 |
| `channel_count` | 固定为 10 |
| `status` | `SBUS_NO_DATA`、`SBUS_OK`、`SBUS_FRAME_LOST`、`SBUS_FAILSAFE`、`SBUS_TIMEOUT` |
| `input_valid` | 仅 `status == SBUS_OK` 时为真 |
| `received` | 是否曾收到结构有效的帧，包括丢帧 / failsafe 帧 |
| `last_frame_ms` / `age_ms` | 帧入队时的系统时间 / 距今毫秒数；未收到帧时 age 为 UINT32_MAX |
| `frame_lost` / `failsafe` | 最近一帧的原始标志；超时状态由 status 单独表示 |
| `digital17` / `digital18` | S.BUS 数字通道标志，非 R9DS 额外模拟通道 |
| `counters` | 有效帧、无效帧、同步丢弃、缓冲满、队列丢弃、UART 各类错误、重挂失败计数 |

`R9DS_GetSnapshot(out)` 返回是否曾收到结构有效帧，并且在无数据时仍填充快照。NULL 返回 false。返回 true 不代表输入健康，使用前读取 `input_valid`。`R9DS_Init()` 返回首次接收启动的 HAL 结果，失败后 `R9DS_Process()` 会重试。

默认 100 ms 无结构有效帧时进入超时，最近通道值保留用于查看。Init / Process / GetSnapshot 只能从同一主循环上下文调用；队列和统计快照通过短临界区保护。当前不包含 F4 转发、通道功能映射、自动解锁或 PWM 输出。接收机若在无线失联后持续发送保持值且没有 failsafe 标志，本模块无法只凭连续 S.BUS 数据判断无线失联。

## 工程维护及构建

CubeMX 配置已同步至 `F1.ioc`。重新生成后，运行 `Tools/prepare_keil.py` 恢复自定义源文件、共享协议包含目录、编译器选择和内存限制；不要删除 `main.c` 及 USART3 IRQ 的 USER CODE 区。

本机 Keil：`D:\Users\21144\AppData\Local\Keil_v5\UV4\UV4.exe`。完整重建参数：

```text
-r D:\pixhawk2.4.8\code_keil\F1\MDK-ARM\F1.uvprojx -t F1 -o D:\pixhawk2.4.8\code_keil\F1\MDK-ARM\F1_build.log
```

构建记录见 `MDK-ARM/F1_build.log`，固件为 `MDK-ARM/F1/F1.hex`。本次仅进行完整编译链接；不运行单元测试、不下载固件、不进行接收机或串口实测。

2026-10-19 完整重建结果：Keil 退出码 0，0 errors / 0 warnings。Code 6008、RO-data 312、RW-data 28、ZI-data 2044 字节；Flash 总计 6348 / 65536 字节，RAM 总计 2072 / 20480 字节（含链接进镜像的栈和堆预留）。这些结果仅证明编译、链接和静态内存容量符合限制。

R9DS 需处于 S.BUS 模式（蓝色指示灯，CH9 输出）。接线及模式依据：[乐迪官方说明](https://www.radiolink.com/cn/r9ds_manual)。时钟规格依据：[ST 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)。
