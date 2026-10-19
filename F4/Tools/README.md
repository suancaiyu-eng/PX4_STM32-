# MPU6000 遥测与上位机解码

该模块以 100 Hz 发送最新的 IMU 样本，采样本身由 IMU 模块独立以 1 kHz 运行。默认输出FireWater文本，原64字节二进制协议保留为可选模式。遥测不会等待电脑接收，也不会补发积压的旧样本。这两种格式用于此工程调试，不是MAVLink。

## 默认FireWater模式

在 `Core/Inc/imu_telemetry.h` 中：

```c
#define IMU_TELEMETRY_OUTPUT_FORMAT IMU_TELEMETRY_FORMAT_FIREWATER
```

选择上位机FireWater解析器，串口使用COM5、460800/8N1/无流控。FW8默认每行`mpu6000:`加18个逗号分隔值，以真实LF结束、不发送NUL：

```text
mpu6000:ax,ay,az,gx,gy,gz,temp,roll,pitch,yaw,att_valid,mx,my,mz,heading_valid,mag_calibrated,mag_used,mag_fresh\n
```

| 通道 | 含义 |
| --- | --- |
| CH0..2 | MPU加速度XYZ原始计数（芯片原生轴） |
| CH3..5 | MPU角速度XYZ原始计数（芯片原生轴） |
| CH6 | MPU温度原始计数 |
| CH7..9 | FRD机体系Roll/Pitch/Yaw，度；Yaw为磁北航向，磁偏角0 |
| CH10 | 姿态/RP有效，不能单凭此项判定Yaw有效 |
| CH11..13 | LSM303D标定前原始机体系磁场XYZ，µT，用于检查/重新拟合 |
| CH14 | 航向有效，使用Yaw必须检查 |
| CH15 | 磁标定参数通过CRC和数值检查 |
| CH16 | 当前使用磁观测修正 |
| CH17 | 磁驱动健康、样本新鲜且未饱和；不等于场无干扰 |

启动约2秒静止陀螺校准期间角度为0占位、CH10=0；正常静止CH10与CH14..17均为1。磁失效时RP可继续有效、CH14/16/17为0、磁XYZ为0占位，Yaw仅作无磁约束的传播值。参数标定有效不等于当前场无干扰。原始磁值未扣偏置/乘矩阵；真实补偿场可看`g_mag_status.field_uT`。当前一次编译只输出一种格式。

`#ATT`用于姿态校准/故障，`#IMU`用于IMU初始化/故障，IMU无有效采样时停止数值行。所有运行格式化在前台，192字节静态普通SRAM缓冲由UART DMA独占到完成/中止；数值行无时间戳/CRC。FireWater不按前缀分组，保持同一行固定18通道。

启动信息为`#BOOT MPU6000 FW8 USART2=460800 FireWater18 AHRS9`，随后`#PROBE`只读探测；它们在采集前有界同步发送，运行输出走DMA。二进制模式不发这些文本。

`Core/Inc/flight_imu_app.h`中的`FLIGHT_IMU_FIREWATER_ATTITUDE=0`可恢复原7通道，融合仍运行。完整磁标定、坐标、降级与实测见[FW8记录](../../../docs/MPU6000_FW8_九轴磁融合实测.md)。

完整三维转动采集后，使用NumPy工具重新拟合本板硬铁/软铁参数：

```powershell
python Tools/calibrate_mag.py --input Logs/new_mag_capture.txt --output Core/Inc/mag_calibration_params.h --report Logs/new_mag_fit.json
```

工具要求当前18通道文本的CH11..13与CH17健康标志，拒绝静止/平面/覆盖不足和残差过大的数据；成功后生成float32参数和CRC。重新构建/烧录后复核磁标定有效及航向有效。50 µT是归一化半径，并非绝对地磁强度实测。

需要原二进制字节发送时，把宏改为 `IMU_TELEMETRY_FORMAT_BINARY` 后重新编译。`encode_frame()` 与原协议保持一致；一次编译只选一种串口格式。下面的Python脚本和二进制协议说明仅适用于该模式。

## 接线与运行

使用支持 460800 波特率的 **3.3 V TTL** USB 转串口模块：TELEM1 的 TX 接模块 RXD，GND 共地；只读取遥测时这两根线即可。飞控自行供电，转换模块 VCC 不接。串口设置为 **460800 / 8N1 / 无流控**。

切换为二进制模式后，在工程目录运行（将 COM5 改为实际端口）：

```powershell
python -m pip install pyserial
python Tools/imu_monitor.py --port COM5 --output imu.csv --raw-log imu.bin
```

按 Ctrl+C 停止，或添加 `--duration 30` 采集约 30 秒。串口助手看到二进制字符属于正常现象，应使用此解码工具查看 CSV。每个 CSV 行包含原始数据、传感器坐标系下的换算值、样本时间戳、状态和错误计数。

离线解码不依赖 pyserial：

```powershell
python Tools/imu_monitor.py --input imu.bin --output decoded.csv
python -m unittest discover -s Tests -p test_imu_monitor.py -v
```

若 PATH 中存在本机 `gcc`，同一测试命令会分别以FireWater和二进制模式编译真实 `imu_telemetry.c`，用最小HAL替身检查DMA缓冲所有权、busy丢帧、发送/abort超时、晚到回调、计数回绕。还验证FireWater通道顺序、正负极值、换行、无NUL及无效样本抑制，并让Python解码C产生的二进制帧。没有gcc时相关项目明确显示skipped，其余协议测试照常运行。

`accel_*_g = raw / 2048`（量程 ±16 g），`gyro_*_dps = raw / 16.4`（量程 ±2000 °/s），温度为 `raw / 340 + 36.53 °C`。这些数值保留芯片原生坐标轴，未校准零偏、比例和安装方向；不得直接当作已标定的飞控机体系数据。必须同时检查 `sample_valid`、`sample_stale`、`driver_state`、`driver_error`，无效样本的换算值没有测量意义。

## 固件集成约定

- 初始化 UART、DMA、1 MHz TIM2 后调用 `ImuTelemetry_Init(&huart2, now_us)`，只初始化一次。
- 主循环持续调用 `ImuTelemetry_Service(&latest_record, now_us)`，记录在本次调用中必须稳定。没有新样本时仍需调用，以处理UART超时；二进制模式可发送stale/invalid健康记录，FireWater模式会限频发送对应状态文本。传入NULL会跳过当次发送并计数。FireWater被限频跳过的无效记录也计入frames_dropped，诊断发送次数另计status_frames。
- 统一 HAL 回调转发：`HAL_UART_TxCpltCallback` → `ImuTelemetry_OnTxComplete`，`HAL_UART_ErrorCallback` → `ImuTelemetry_OnError`，`HAL_UART_AbortTransmitCpltCallback` → `ImuTelemetry_OnAbortComplete`。
- UART TX 仅由本模块使用。不可同时调用 printf 重定向、阻塞 UART 发送或其他 UART TX DMA。
- USART2 和其 TX DMA 中断抢占优先级均为 **8**，分组为 **NVIC_PRIORITYGROUP_4**；IMU 的 EXTI/SPI/DMA 使用优先级 **5**。模块通过 BASEPRI 仅屏蔽 8 及以下优先级，保持 IMU 中断可抢占。
- 静态 `tx_frame` 必须由链接器放在普通 SRAM（0x20000000 区域），不能放进 CCM（0x10000000）。其内存在完成/确认 abort 前不会被改写。
- `ImuTelemetry_GetStats()` 用于主循环/调试器查看启动、完成、丢帧、错误、超时、abort 和故障锁定计数。
- 发送 64 字节在 460800 / 8N1 下约 1.39 ms。20 ms 未完成或报错时，主循环启动 `HAL_UART_AbortTransmit_IT()`；此工程 HAL 的该路径不轮询等待。abort 再过 20 ms 仍不确认时，模块停止遥测并保留缓冲所有权，IMU 继续独立运行。无法确认 DMA 停止时不会重用其内存，也不会重置整个共享 DMA 控制器。

## 二进制协议 v1

总长 **64 字节**。所有多字节字段为小端；不能直接发送 C 结构体，固件显式编码以避免填充、对齐差异。

| 字节偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | 同步头 `A5 5A` |
| 2 | 1 | 版本 `01` |
| 3 | 1 | 整帧长度 `40`（十六进制，即64） |
| 4 | 4 | 遥测时隙序号 `frame_sequence`，uint32 |
| 8 | 4 | IMU 样本序号 `sample_sequence`，uint32 |
| 12 | 4 | 样本数据就绪时间 `timestamp_us`，uint32 |
| 16 | 4 | 本帧打包服务时刻 `sent_us`，uint32 |
| 20 | 6 | accel X/Y/Z，3 × int16 |
| 26 | 6 | gyro X/Y/Z，3 × int16 |
| 32 | 2 | temperature raw，int16 |
| 34 | 2 | flags，uint16 |
| 36 | 1 | IMU 驱动状态，uint8（枚举见驱动头文件） |
| 37 | 1 | IMU 驱动错误，uint8（枚举见驱动头文件） |
| 38 | 4 | IMU sample_drops |
| 42 | 4 | IMU spi_errors |
| 46 | 4 | IMU timeouts |
| 50 | 4 | telemetry_drops |
| 54 | 4 | uart_errors |
| 58 | 4 | UART 发送超时次数 |
| 62 | 2 | CRC16，低字节在前 |

CRC 使用 **CRC-16/CCITT-FALSE**：多项式 0x1021，初值 0xFFFF，不反射，不做最终异或。覆盖偏移 **2..61** 的60字节，不包含同步头和CRC字段。标准校验字符串 `123456789` 的结果为 `0x29B1`。

flags 定义：bit0 样本有效；bit1 传感器原生坐标轴；bit2 样本过期；bit3 驱动故障；bit4 未校准，其余保留为0。状态字段描述打包时驱动状态，样本时间戳描述该样本的采集时刻。

`frame_sequence` 每 10 ms 时隙增加一次，忙、NULL、调度延迟跳过的时隙也计入，因此空洞可以反映发送机会丢失或链路接收丢帧，不能仅凭它区分两者。`telemetry_drops` 统计未启动发送的机会；已启动但中途失败的帧通过 UART 错误/超时计数反映。`sample_sequence` 是1kHz采样序号，在100Hz遥测中通常每帧增加约10，不表示漏采。

32位微秒时间戳约71.6分钟回绕，时间差应按无符号32位相减。接收器支持任意分包、杂字节、错误长度/版本和CRC错误后重新同步；主机摘要包含CRC错误与遗漏时隙计数。离线文件尾部不足一帧时，保留字节数显示为 `trailing_bytes`。

这些软件测试验证协议和串口缓冲状态逻辑，实际板卡仍需实测采样频率、时间戳抖动、供电、线路、电磁干扰和错误恢复；这里不代表整机完成飞行验收。
