# F1 正式固件：遥控、板间通信和 MAIN2–5 PWM

正式工程只有 `MDK-ARM/F1.uvprojx` / `F1` 目标。四路PWM采用已经由用户在MAIN2示波器验证的定时器输出实现，固定400 Hz、1000–2000 µs；上电关闭，接受F4显式会话及限时执行器命令，包含范围检查、超时停止和配置读回。Flash为62 KB应用及末尾2 KB持久化启动计数。

扫占空比、Watch测试及旧测试固件已移出本工程，独立测试工程位于同级 `F1_test`，双方无源码、包含路径或构建脚本依赖。正式固件不自动扫波形，也不包含测试邮箱。详细接口见 [PWM驱动说明](../../docs/F1_MAIN2-5_PWM驱动与验收.md)。

已追加 F1 USART2 ↔ F4 USART6 板间命令通信（115200 / 8N1）。接口和测试方式见 [板间通信说明](../../firmware/common/README.md)。R9DS 原始十通道和接收状态现在以 20 Hz 的 `EVENT / 0x0200` 自动上报 F4，可在 TELEM1 的 `#LINK_RX` 行查看完整载荷。[遥控测试与标定进展](../../docs/R9DS_通信与范围标定.md)。

工程：`MDK-ARM/F1.uvprojx`，目标 `F1`，STM32F103C8T6，Arm Compiler 5.06u7。

## 已实现

- R9DS CH9 的 S.BUS 信号接飞控 RC IN，经板内 U903 反相后进入 PB11 / USART3_RX。
- USART3 仅接收：100000 bit/s、8 数据位、偶校验、2 停止位；HAL 配置为 `UART_WORDLENGTH_9B` + `UART_PARITY_EVEN`。不启用 PB10 或 S.BUS 输出。
- 内部 HSI / 2 × 16，SYSCLK / HCLK / PCLK2 为 64 MHz，PCLK1 为 32 MHz。不依赖外部晶振。
- 32 字节 `HAL_UARTEx_ReceiveToIdle_IT` 缓冲，四帧队列，中断搬运、主循环解码。直接编译引用 `../../firmware/rc/sbus.c`，没有复制解码器。
- 启动先丢弃一个空闲分隔段以建立边界；UART 错误、缓冲满后丢弃当前数据段，直到下一次 IDLE 恢复边界。只接受 25 字节标准 S.BUS，帧头 0x0F、帧尾 0x00；不支持 S.BUS2。
- 队列满时丢弃新帧并计数。错误段不更新通道或接收时间；接收重挂失败由主循环重试。

原理图的 X801 标为 24 MHz，U801 原始料号为 STM32F100C8T6B；当前工程依据此前确认的 STM32F103C8T6。F103C8 的无源 HSE 晶振规格为 4–16 MHz，因此本版选择内部时钟。实板晶振和 HSI 下的串口精度均未验证。

## 四路 PWM 配置

| 接口 | 引脚 | 定时器 |
| --- | --- | --- |
| MAIN2 | PA1 | TIM2_CH2 |
| MAIN3 | PB8 | TIM4_CH3 |
| MAIN4 | PB9 | TIM4_CH4 |
| MAIN5 | PA6 | TIM3_CH1 |

主循环初始化 `MainPwm_Init()`、周期调用 `MainPwm_Service()`；执行器模块通过 `MainPwm_Submit()` 提交各路最终脉宽，或 `MainPwm_Stop()` 全部关闭。MAIN3/4成组预装载，三组定时器不保证相位同步。单次提交替换整个四路集合，未包含的通道关闭。

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

默认 100 ms 无结构有效帧时进入超时，最近通道值保留用于查看。Init / Process / GetSnapshot 只能从同一主循环上下文调用；队列和统计快照通过短临界区保护。原始通道上报已接入，尚无通道功能映射或自动解锁；新增 PWM 驱动不与遥控通道自动连接。接收机若在无线失联后持续发送保持值且没有 failsafe 标志，本模块无法只凭连续 S.BUS 数据判断无线失联。

使用 `g_r9ds_normalized` 查看 -1000..1000 的分段线性归一化值，必须检查 `valid_mask`。2026-10-19 已实测并烧录九通道标定，配置掩码为 `0x03BF`；CH7 全程固定 1800，保持未标定。无数据、failsafe、丢帧或超时时，所有归一化通道有效位清零。参数和实测记录见 [范围标定记录](../../docs/R9DS_通信与范围标定.md)。

## 工程维护及构建

CubeMX 配置已同步至 `F1.ioc`。重新生成后，运行 `Tools/prepare_keil.py` 恢复自定义源文件、共享协议包含目录、编译器选择和内存限制；不要删除 `main.c` 及 USART3 IRQ 的 USER CODE 区。

本机 Keil：`D:\Users\21144\AppData\Local\Keil_v5\UV4\UV4.exe`。完整重建参数：

```text
-r D:\pixhawk2.4.8\code_keil\F1\MDK-ARM\F1.uvprojx -t F1 -o D:\pixhawk2.4.8\code_keil\F1\MDK-ARM\F1_build.log
```

固件为 `MDK-ARM/F1/F1.hex`。2026-10-19标定版的历史完整重建日志为 `MDK-ARM/r9ds_calibrated_build.log`，0 errors / 0 warnings，Flash 9632 / 65536、RAM 3384 / 20480 字节。已烧录校验，并实读确认接收正常、归一化计算正确。板间双向实测见 [通信验收记录](../../docs/F1_F4_板间通信验收.md)。

历史版本：R9DS 初版 Flash 6348、RAM 2072 字节；加入板间通信后 Flash 9016、RAM 3328 字节。以上均含栈和堆预留，均为 0 errors / 0 warnings。

R9DS 需处于 S.BUS 模式（蓝色指示灯，CH9 输出）。接线及模式依据：[乐迪官方说明](https://www.radiolink.com/cn/r9ds_manual)。时钟规格依据：[ST 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)。
