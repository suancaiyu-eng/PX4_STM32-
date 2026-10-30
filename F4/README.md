# Pixhawk FMU：MPU6000 采集与串口遥测

已接入 MS5611/MS5607 气压计驱动、PROM CRC、非阻塞转换与相对高度接口。**本板依据桂林室内实测选择 MS5611 补偿，默认型号为 5611。** 已烧录并验证采样、共享 SPI 和软件超时失效。Keil 保留 `F4_BARO_TEST` 诊断目标（TELEM1 460800、14 通道），默认目标仍输出 RC、后台采气压。配置与实测记录见 [气压计说明](../../docs/F4_MS56xx_气压计配置与验收.md)。高度方向/幅度的受控测试按用户要求保留到后续脱机进行；手持记录不作为精度验收。

已接入 [执行器客户端](../../docs/F4_F1_执行器命令与验收.md)：显式 OPEN/SET/关闭使能、会话/序号/时效及 F1 输出状态接收。默认目标不自动握手或输出；新增 `F4_ACTUATOR_TEST` 目标提供一次握手后只下发一次的 Watch 测试入口。原 RC-only 串口输出保留；执行器测试目标本次未烧录。

已追加 USART6（PC6 TX / PC7 RX，115200 / 8N1）与 F1 通信，默认运行双向测试消息回包。原 USART2 / TELEM1 遥测保留。协议和扩展命令见 [板间通信说明](../../firmware/common/README.md)，实际验收进度见 [通信验收记录](../../docs/F1_F4_板间通信验收.md)。本目录的 Keil 工程和目标仍名为 MPU6000；本次未修改旁边的 `code_keil/MPU6000` 目录。

当前默认 TELEM1 使用**逐飞助手 Printf 协议，460800、8N1、无流控，约 100 Hz**，仅输出三轴陀螺仪：`gyro:x,y,z\n`，单位 °/s，MPU6000 原生轴，尚未扣除姿态估计器零偏。可在 `Core/Inc/imu_telemetry.h` 注释当前 `IMU_TELEMETRY_VIEW` 定义，取消一个可选定义的注释后重编译；加速度（g）、姿态角（°）、磁力计（µT）、气压计、遥控器格式均已实现，默认关闭。每种格式最多八通道，以真实 LF 结束。默认没有启动/状态/板间日志混入，失效或过期的陀螺仪样本不输出。选遥控器时可在 `board_link_telem.h` 配置八个显示字段。具体配置、各通道单位和验证见 [TELEM1 数据查看](../../docs/F4_TELEM1_板间数据查看.md)。

本工程在STM32F427上实现飞控传感器采集与九轴姿态融合：MPU6000约1 kHz SPI DMA采样，LSM303D提供约100 Hz三轴磁场；主循环逐样本预测四元数，200 Hz修正/发布姿态，USART2以100 Hz输出18通道FireWater。原7个原始值、原64字节CRC二进制格式保留。采样中断不打印，UART DMA发送不等待电脑。

当前FW8已完成本板真实三维磁标定、烧录及静止/降级验证。Yaw受磁北约束，磁异常会清除独立的航向有效标志；不能保证完全消除漂移。实际标定、18通道表和验收见[FW8九轴记录](../../docs/MPU6000_FW8_九轴磁融合实测.md)。[FW7六轴记录](../../docs/MPU6000_FW7_姿态解算移植.md)与[FW6片选修复](../../docs/MPU6000_FW6_实测修复记录.md)保留为历史依据。完整加速度/温漂标定、控制器、飞行解锁和混控尚未实现；新增执行器通信不代表已经具备飞行闭环。

## 已检查并修正的工程配置

| 项目 | 当前实际配置 |
| --- | --- |
| 外部晶体 | 按仓库 FMUv2 原理图使用 24 MHz 无源晶体，HSE ON |
| 系统时钟 | PLL M=24、N=360、P=2、Q=8；180 MHz，Scale1、OverDrive、Flash 5 WS |
| 总线 | APB1=45 MHz、APB2=90 MHz |
| SPI1 | PA5/PA6/PA7，Mode3、8bit、MSB、软件片选、/128，即703.125 kHz |
| SPI DMA | RX：DMA2 Stream0 Channel3；TX：DMA2 Stream3 Channel3；Normal、Byte、MemInc |
| MPU6000 | PC2片选、PD15上升沿DRDY；其余SPI器件片选保持高 |
| LSM303D磁力计 | PC15片选，与MPU共享SPI1仲裁；100 Hz ODR、主循环检查新数据 |
| 传感器供电 | PE3 初值High，保持传感器电源使能；所有片选High后配置SPI引脚，首笔访问前等待供电稳定 |
| 微秒时钟 | TIM2 PSC=89、ARR=0xFFFFFFFF，1 MHz自由运行；不启用TIM2中断 |
| 串口 | USART2，PD5 TX/PD6 RX，对应TELEM1；460800/8N1/无流控 |
| 串口DMA | TX：DMA1 Stream6 Channel4，Normal、Byte、MemInc |
| 中断 | EXTI15_10、SPI1、DMA2 Stream0/3：抢占优先级5；USART2、DMA1 Stream6：8；分组4 |
| 链接布局 | Flash从0x08000000起；所有RW/ZI在0x20000000普通SRAM；不用CCM；栈4 KiB |

原工程使用 HSI 倍频180 MHz，HSE参数为25 MHz，并启用了最高优先级的TIM2中断；这些已同步修正到 `.ioc` 和代码。Keil工程的显式内存区原先为零，现通过 `MDK-ARM/MPU6000.sct` 指定。

PLLQ 输出45 MHz，本版不启用 USB FS；上位机走 TELEM1 + USB转TTL。未来启用USB需重新安排时钟并复核SPI/TIM2。

## 打开、构建与连接

1. Keil打开 `MDK-ARM/MPU6000.uvprojx`，使用已安装的 Arm Compiler 5.06 update7，执行 Rebuild。
2. 构建结果在 `MDK-ARM/MPU6000/MPU6000.axf` 和 `MPU6000.hex`。本工程是从0x08000000启动的独立裸机镜像，供SWD下载；不是PX4 bootloader加载的 `.px4` 包，写入该地址会覆盖原启动代码。当前FW8已通过SWD下载、校验并上板验证。
3. 飞控自行供电，用 **3.3 V TTL** USB转串口连接：TELEM1 pin2（TX）→转换器RXD，pin6（GND）→GND。只输出时两根即可；转换器VCC不接。连接器针号按原理图/板上标识核对，不根据插头朝向猜测。
4. 当前默认输出为FireWater。在上位机选择COM5、460800/8N1、FireWater解析器，即可显示18通道。每行格式及顺序为：

```text
mpu6000:ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,temp_raw,roll_deg,pitch_deg,yaw_deg,att_valid,mx_uT,my_uT,mz_uT,heading_valid,mag_calibrated,mag_used,mag_fresh\n
```

CH0..6仍为原生轴有符号16位原始计数；CH7/8/9为FRD机体系Roll/Pitch/Yaw（度），CH10为姿态/RP有效。CH11..13为标定前原始机体系磁场µT；CH14为航向有效，CH15为磁标定参数有效，CH16为当前磁融合启用，CH17为磁数据健康新鲜。使用Yaw必须检查CH14；正常静止上述5个有效/健康通道均为1。磁失效时RP可继续有效，Yaw失去磁约束。静止校准约2秒，期间角度为0占位、有效标志为0，原始值正常输出；有效标志为1后才能使用姿态。`#ATT`报告校准/姿态故障，`#IMU`报告驱动初始化/故障；IMU失效后停止数值行。文本以真实LF换行结束。将`flight_imu_app.h`的`FLIGHT_IMU_FIREWATER_ATTITUDE`设为0，可恢复原7通道文本，解算继续运行。

在 `Core/Inc/imu_telemetry.h` 修改 `IMU_TELEMETRY_OUTPUT_FORMAT` 并重新编译：

```c
/* 默认：FireWater原始数值 */
#define IMU_TELEMETRY_OUTPUT_FORMAT IMU_TELEMETRY_FORMAT_FIREWATER
/* 恢复原64字节二进制协议时，将上行改为： */
/* #define IMU_TELEMETRY_OUTPUT_FORMAT IMU_TELEMETRY_FORMAT_BINARY */
```

每个固件选择一种格式，共用同一套串口DMA发送与超时处理。采样仍是1 kHz，输出最新样本为100 Hz，不是导出每一份1 kHz样本。

**仅在切换到二进制模式后**运行原上位机脚本（将COM5换成实际端口）：

```powershell
python -m pip install pyserial
python Tools/imu_monitor.py --port COM5 --duration 30 --output imu.csv --raw-log imu.bin
```

`imu_monitor.py` 解码原64字节二进制协议，不能解码FireWater文本。格式切换、字段、CRC、离线解码和错误计数说明见 [Tools/README.md](Tools/README.md)。

## 驱动行为与代码入口

- `Core/Src/mpu6000.c`：可独立测试的传感器状态机、寄存器配置、样本队列、时间与健康检查。
- `Core/Src/mpu6000_port.c`：STM32 HAL/寄存器适配、片选、临界区、DMA IRQ收尾与显式恢复。
- `Core/Src/flight_imu_app.c`：统一回调、主循环消费队列、SI单位转换、遥测记录。
- `Core/Src/attitude_estimator.c`：静止校准、FRD轴映射、逐样本预测、200 Hz重力/航向融合与姿态发布、磁异常降级和故障锁定。
- `Core/Src/lsm303d_mag.c`：LSM303D真实磁采集、完整硬铁/软铁补偿、CRC和配置检查。
- `Core/Inc/mag_calibration_params.h`：本板实际三维转动拟合参数；生成工具为`Tools/calibrate_mag.py`。
- `Core/Src/imu_telemetry.c`：100 Hz发送调度、帧编码、串口缓冲所有权、异步中止。
- `Core/Src/main.c`：初始化后调用 `FlightImuApp_Init()`，循环调用 `FlightImuApp_Service()`。

初始化等待供电稳定、检查WHO_AM_I=68、复位器件；先等待100 ms，再每10 ms核对复位ID/PWR1，最多等待1秒。随后复位信号通路、唤醒并确认PWR1=01，再写USER_CTRL=10。配置项写入至少10 ms后读回，等待由状态机推进；正常约500 ms完成，FireWater构建另有启动探测和100 ms等待。每笔寄存器事务含两段10 µs片选保护及2 ms HAL上限；运行DMA采集和ISR没有等待。

当前量程：陀螺±2000 °/s，加速度±16 g；DLPF_CFG=2，分频0，输出1 kHz。加速度换算 `raw/2048*9.80665` m/s²；角速度 `raw/16.4*pi/180` rad/s；温度 `raw/340+36.53` °C。

每次发送 `0xBA`（0x3A读命令）和15个dummy字节，共16字节。接收第0字节丢弃，第1字节为INT_STATUS，随后14字节为加速度、温度、角速度。INT_PIN_CFG=0x00，仅读INT_STATUS清状态。理论传输约182 µs，接收完成并确认SPI不忙后才释放CS。

TIM2时间戳记录DRDY中断入口时刻，单位µs；不是DMA完成时刻，也尚未补偿传感器滤波延迟。32位计数约71.6分钟回绕，时间差按无符号32位相减。`dt_us`是相邻成功采集的间隔；消费者还应检查 `sequence`、队列丢样统计和样本年龄，不能忽略队列溢出后序号断层。

队列最多16个样本，满时丢弃新样本并计数。主循环按采样率逐个消费；将来的估计器应接在消费位置，不能用100 Hz串口数据驱动板上控制器。FW8的LSM303D已通过仲裁共享SPI1；只在MPU DMA空闲时间窗读磁场，意外DRDY保留原时间戳延后派发。其他SPI器件也必须接入该仲裁。

## 故障与调试观察

- SPI DMA超过500 µs、5 ms没有有效样本、错误ID、配置读回不符或连续3次无效INT_STATUS，会锁定 `MPU6000_FAULT`。
- DMA忙时不重用缓冲，计 `busy_drops`；队列满计 `queue_drops`；无效状态计 `invalid_samples`。
- IRQ只捕获对应中断已使能的DMA错误；屏蔽的FIFO错误标志不再误停直接模式传输。SPI收尾若尚忙则延迟到主循环检查，不在高优先级中断里轮询等待。故障后CS释放、队列清空、旧数据标为无效；主循环继续。二进制模式继续输出状态遥测，FireWater模式暂停输出数值、每秒输出一条 `#IMU` 诊断文本。
- 不自动重启传感器。`MPU6000_RestartDisarmed()` 仅允许在故障状态调用，并确认DMA停止后恢复；调用者负责确认已解除解锁状态。此项目没有实现解锁状态机，因此主程序不会自行调用它。显式恢复会清零本轮驱动统计，不会断电重启整个共享传感器电源。
- 串口20 ms未完成时异步中止；无法确认中止则锁定遥测故障，不重用可能仍由DMA持有的缓冲。

Keil Watch可观察：

| 变量 | 正常预期 |
| --- | --- |
| `g_imu_status.state` | 启动后为6，即RUNNING |
| `g_imu_status.who_am_i` | 0x68 |
| `g_imu_status.last_error` | 0 |
| `g_imu_status.samples` | 每秒增加约1000 |
| `g_imu_sample.dt_us` | 通常约1000 |
| `g_imu_sample_valid` | 1；仅表示采样新鲜且驱动正常，不表示已校准 |
| `g_imu_measurement` | 原生轴m/s²、rad/s、°C |
| `g_telemetry_status` | 无uart_errors/tx_timeouts，frames_completed每秒约100 |
| `g_attitude` | state=2、error=0、heading_state=2；健康flags通常125；角度rad，Yaw为磁北航向 |
| `g_attitude_max_update_us` | FW8实测最大18 µs，包含校准/预测/发布路径 |

错误枚举在 `Core/Inc/mpu6000.h`：1=ID，2=寄存器通信，3=读回，4=DMA启动，5=SPI/DMA错误，6=DMA超时，7=有效样本超时，8=数据状态。`bad_register` 可定位配置失败的寄存器。暂停CPU会错过实时采样；调试器暂停后触发超时并不等同于正常运行时的硬件故障。

## 串口没有输出时

下载最新固件，串口助手选择CH340对应端口、460800/8N1、无流控，打开**原始文本接收窗口**，再复位飞控。仅看FireWater波形不会显示以下诊断文本。

启动时应看到：

```text
#BOOT MPU6000 FW8 USART2=460800 FireWater18 AHRS9
```

这条信息在采集启动前以有20 ms超时的同步UART发送一次，完全不依赖传感器、TIM2或UART DMA；运行期间仍使用DMA。二进制模式不输出这条文本。若连启动信息也没有，先核对是否烧入当前镜像、复位后是否运行、TELEM1 pin2到转换器RXD与共地、波特率；不能直接断定为传感器故障。

没有有效样本时，DMA链路每秒最多发送一条状态，例如（仅格式示例）：

```text
#IMU state=7 error=1 id=0xFF reg=0x00 wr=00 rd=00 drdy=0 samples=0 spi=0 timeout=0
```

- `state`：1..5=初始化阶段，6=运行，7=故障。
- `error`：上方错误枚举；`id`为实际读取的WHO_AM_I，预期0x68。若从未成功读取则初值为0。
- `reg`：最近写入或校验的配置寄存器；ID失败时不靠此字段定位。`wr`和`rd`均为十六进制，分别记录预期值与实际读回值；只有读回成功时rd才有意义，通信失败时保留00。
- `drdy` / `samples`：收到的DRDY次数 / 有效采样次数；`spi` / `timeout`为错误统计。

只有BOOT没有后续任何行时，检查 `g_boot_uart_result`（0=HAL_OK）、`g_app_boot_stage` 和 `g_telemetry_status`。阶段1=进入main、2=HAL完成、3=时钟完成、4=外设完成、5=应用初始化完成、6=主循环；早期Error_Handler可据阶段定位。启动提示只发送一次，必须在接收窗口打开后复位。

## 已有串口输出，但ID一直是0x00

`state=7 error=1 id=0x00` 表示WHO_AM_I检查失败，驱动锁定故障，因此不会启动DRDY/DMA采样。`spi=0 timeout=0`只表示HAL没有报告传输错误；SPI没有从机应答机制，不能据此确认器件存在。`reg=0x00`在这里不是实际读取地址，WHO_AM_I地址仍为0x75，预期值为0x68。

FW3取消了GPIO初始化拉低PE3、SPI初始化又迅速拉高的短暂电源脉冲。仓库原理图中U603的EN1已有R603上拉，启动时保持PE3为High可避免不完整掉电；这是一项时序修正，不能单凭旧日志认定它就是实物故障根因。SPI在所有片选为High时预先使能，使Mode3空闲SCK在首笔事务前稳定。

FireWater构建在采集开始前等待100 ms，执行六笔只读访问并同步发送一次以下探测行（示例表示正常响应，不是本机实测）：

```text
#PROBE mpu=68/68/68 pwr=40 gyro=D4 accmag=49 en=1 csL=0 csH=1 sck=1 hal=00
```

- `mpu`：PC2片选下三次读取0x75。`pwr`：读取MPU的PWR_MGMT_1（0x6B）；冷启动默认通常为40，MCU单独复位时可能保留旧配置，不以40作为通过条件。
- `gyro` / `accmag`：PC13 / PC15片选下读取0x0F。仓库原理图所示L3GD20预期D4、LSM303D预期49；实物可能换装其他型号，不能只凭某个不同ID判坏。
- `en` / `csL` / `csH` / `sck`：PE3、PC2拉低时、PC2拉高时、PA5空闲时的MCU引脚输入电平。预期1/0/1/1；它们不等于测到了传感器端电压或确认线路连通。
- `hal`：六笔访问的失败位掩码，bit0..2对应三次MPU ID，bit3为PWR，bit4为gyro，bit5为accmag。00表示HAL访问均返回成功，FF表示探测未获准执行。

当前板上已运行FW8，串口已释放，可以直接打开接收窗口。重新下载时确认BOOT版本为FW8。若换板后MPU全00而同总线器件ID正确，核查MPU片选、局部供电及实际型号；若三颗全00/FF，核查共享供电和总线。PE3为High不能替代VDD_3V3_SENSORS电压实测。WHO读取的MOSI应为F5 00，MISO第二字节应为68。

该只读探测只用于启动排障；运行中LSM303D通过专用驱动和仲裁读取，不扫描寄存器、不自动改SPI模式或绕过ID检查。原二进制构建不发送BOOT/PROBE文本，原64字节协议保留。

## ID正确，但0x6A读回失败

用户FW3日志为 `state=7 error=3 id=0x68 reg=0x6A`：确认驱动已通过ID检查，失败点是USER_CTRL写入后的严格校验。0x10对应I2C_IF_DIS，用于选择MPU6000的SPI接口；不能把这个位当作无需验证而屏蔽。寄存器定义见[TDK寄存器手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)。

FW4延后校验、FW5调整初始化顺序均未完整解决。FW6通过板上对照确认事务片选时序影响读写；增加10 µs片选间隔和10 µs建立时间后，连续ID与普通寄存器/USER_CTRL写入读回均正常。没有删除0x6A校验，也没有将00当作正常SPI配置。详细原始记录见上方FW6报告。

FW4的错误行增加 `wr=10 rd=XX`，无需暂停CPU即可知道0x6A的实际读值。仍为error=3时必须继续根据rd排查，不会带故障进入采样。error=0、state=1的首条日志属于正常启动等待，不能单凭它判定失败。正常采样后将输出 `mpu6000:...`，当前FW8默认18通道，前7项仍为原始值。

## FW8当前验收

66项测试通过，无跳过；Keil主目标及保留的二进制/7通道兼容构建均0错误、0警告。最终18通道固件实测60.005秒接收5999行，预测1001.24 Hz、发布200.00 Hz、磁融合96.70 Hz；Yaw首尾变化0.114°。IMU/SPI/串口错误与稳态丢样均0，最大解算18 µs。软件磁驱动故障注入会清除航向有效标志而保留RP，复位恢复正常；两轮复位通过。Flash与最终HEX逐字节核对通过。证据为`Logs/fw8_validation.json`和[完整记录](../../docs/MPU6000_FW8_九轴磁融合实测.md)。静态局部统计不代表绝对航向精度或飞行验收。

## 验证和后续生成

自动测试命令（原生C状态机测试需要PATH中有gcc）：

```powershell
python -m unittest discover -s Tests -v
```

测试覆盖真实驱动C的初始化、错误ID/读回、大小端与单位换算、DMA所有权、队列溢出、超时、回绕、迟到回调、显式恢复；真实端口C的只读探测、互斥片选、失败位掩码、忙时拒绝探测、延迟完成/故障停止；真实遥测C的发送/中止状态机及Python协议解码。它们不能验证晶振起振、实际芯片型号、供电与SPI波形。

FW6验证：Keil目标0错误、0警告；30项测试全部通过，无跳过。覆盖片选间隔与建立时间、计时器停止时有界失败、复位暂态重试和1秒期限，以及原有DMA/遥测逻辑。Code=16662 B、RO-data=810 B、RW-data=104 B、ZI-data=6104 B；dma_tx=0x2000045C、dma_rx=0x2000046C、tx_frame=0x2000047C（128字节）均在普通SRAM。实物静态60秒数据、SWD计数快照、DRDY失联和复位恢复记录位于Logs目录，汇总为Logs/fw6_validation.json。

上述为FW6历史验收。FW7新增六轴姿态估计，47项测试通过，Keil 0错误/0警告；最终11通道实物60秒接收5999帧，预测约1001.22 Hz、姿态发布200.003 Hz，最大解算耗时14 µs，SPI/队列/串口错误均为0。DRDY失联会撤销姿态有效标志，两轮软件复位均恢复。证据汇总为Logs/fw7_validation.json；详细范围与限制见FW7移植记录。未进行受控物理姿态精度测量或飞行验收。

CubeMX重新生成时保留USER CODE，使用本次修正的 `.ioc`。随后运行：

```powershell
python Tools/prepare_keil.py
```

该脚本恢复自定义源文件注册、24 MHz编译宏、4 KiB栈和SRAM链接脚本。它不修改下载器设置。确认 `main.c` 中的两个应用入口、`spi.c` 中的PE3使能与SPI提前使能，以及 `stm32f4xx_it.c` 中RX/TX DMA前置检查仍保留，再重新编译和测试。不要用旧 `.ioc` 覆盖当前时钟/电源配置，PE3初始输出必须保留High。
