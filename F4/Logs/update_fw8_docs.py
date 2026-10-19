"""Update current-entry docs, retaining FW6/FW7 historical evidence."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'README.md'
s=p.read_text(encoding='utf-8-sig')
start=s.index('本工程在')
end=s.index('## 已检查')
s=s[:start]+'''本工程在STM32F427上实现飞控传感器采集与九轴姿态融合：MPU6000约1 kHz SPI DMA采样，LSM303D提供约100 Hz三轴磁场；主循环逐样本预测四元数，200 Hz修正/发布姿态，USART2以100 Hz输出18通道FireWater。原7个原始值、原64字节CRC二进制格式保留。采样中断不打印，UART DMA发送不等待电脑。

当前FW8已完成本板真实三维磁标定、烧录及静止/降级验证。Yaw受磁北约束，磁异常会清除独立的航向有效标志；不能保证完全消除漂移。实际标定、18通道表和验收见[FW8九轴记录](../../docs/MPU6000_FW8_九轴磁融合实测.md)。[FW7六轴记录](../../docs/MPU6000_FW7_姿态解算移植.md)与[FW6片选修复](../../docs/MPU6000_FW6_实测修复记录.md)保留为历史依据。完整加速度/温漂标定、控制器、解锁和电机输出尚未实现。

'''+s[end:]
replacements={
'当前FW7已通过SWD下载':'当前FW8已通过SWD下载',
'即可显示11通道':'即可显示18通道',
'mpu6000:ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,temp_raw,roll_deg,pitch_deg,yaw_deg,att_valid\\n':'mpu6000:ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,temp_raw,roll_deg,pitch_deg,yaw_deg,att_valid,mx_uT,my_uT,mz_uT,heading_valid,mag_calibrated,mag_used,mag_fresh\\n',
'CH0..6仍为有符号16位原始计数；CH7/8/9为机体系Roll/Pitch/相对Yaw，单位度；CH10为姿态有效标志。':'CH0..6仍为原生轴有符号16位原始计数；CH7/8/9为FRD机体系Roll/Pitch/Yaw（度），CH10为姿态/RP有效。CH11..13为标定前原始机体系磁场µT；CH14为航向有效，CH15为磁标定参数有效，CH16为当前磁融合启用，CH17为磁数据健康新鲜。使用Yaw必须检查CH14；正常静止上述5个有效/健康通道均为1。磁失效时RP可继续有效，Yaw失去磁约束。',
'若增加其他SPI1传感器，必须先增加总线调度；目前SPI1由MPU6000独占。':'FW8的LSM303D已通过仲裁共享SPI1；只在MPU DMA空闲时间窗读磁场，意外DRDY保留原时间戳延后派发。其他SPI器件也必须接入该仲裁。',
'- `Core/Src/attitude_estimator.c`：静止校准、FRD轴映射、逐样本四元数预测、200 Hz重力修正/姿态发布、故障锁定。':'- `Core/Src/attitude_estimator.c`：静止校准、FRD轴映射、逐样本预测、200 Hz重力/航向融合与姿态发布、磁异常降级和故障锁定。\n- `Core/Src/lsm303d_mag.c`：LSM303D真实磁采集、完整硬铁/软铁补偿、CRC和配置检查。\n- `Core/Inc/mag_calibration_params.h`：本板实际三维转动拟合参数；生成工具为`Tools/calibrate_mag.py`。',
'state=2、error=0、flags bit0=1；内部角度rad，Yaw为相对角度':'state=2、error=0、heading_state=2；健康flags通常125；角度rad，Yaw为磁北航向',
'实测最大约14 µs，包含校准/预测/发布路径':'FW8实测最大18 µs，包含校准/预测/发布路径',
'#BOOT MPU6000 FW7 USART2=460800 FireWater11 AHRS6':'#BOOT MPU6000 FW8 USART2=460800 FireWater18 AHRS9',
'当前板上已运行FW7':'当前板上已运行FW8',
'重新下载时确认BOOT版本为FW7':'重新下载时确认BOOT版本为FW8',
'该探测只用于启动排障；采样期间不探测其他器件':'该只读探测只用于启动排障；运行中LSM303D通过专用驱动和仲裁读取',
'FW7默认11通道，前7项仍为原始值':'当前FW8默认18通道，前7项仍为原始值',
}
for old,new in replacements.items():
    assert old in s,old
    s=s.replace(old,new)
marker='## 验证和后续生成'
s=s.replace(marker,'''## FW8当前验收

66项测试通过，无跳过；Keil主目标及保留的二进制/7通道兼容构建均0错误、0警告。最终18通道固件实测60.005秒接收5999行，预测1001.24 Hz、发布200.00 Hz、磁融合96.70 Hz；Yaw首尾变化0.114°。IMU/SPI/串口错误与稳态丢样均0，最大解算18 µs。软件磁驱动故障注入会清除航向有效标志而保留RP，复位恢复正常；两轮复位通过。Flash与最终HEX逐字节核对通过。证据为`Logs/fw8_validation.json`和[完整记录](../../docs/MPU6000_FW8_九轴磁融合实测.md)。静态局部统计不代表绝对航向精度或飞行验收。

'''+marker)
p.write_text(s,encoding='utf-8')

p=ROOT/'Tools/README.md'
s=p.read_text(encoding='utf-8-sig')
start=s.index('选择上位机FireWater解析器')
end=s.index('需要原二进制字节发送时')
s=s[:start]+'''选择上位机FireWater解析器，串口使用COM5、460800/8N1/无流控。FW8默认每行`mpu6000:`加18个逗号分隔值，以真实LF结束、不发送NUL：

```text
mpu6000:ax,ay,az,gx,gy,gz,temp,roll,pitch,yaw,att_valid,mx,my,mz,heading_valid,mag_calibrated,mag_used,mag_fresh\\n
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

'''+s[end:]
p.write_text(s,encoding='utf-8')

p=ROOT.parents[1]/'docs/FMU_MPU6000_DMA_配置.md'
s=p.read_text(encoding='utf-8-sig')
s=s.replace('当前FW7在原FW6基础上接入六轴姿态解算，见[FW7移植记录](MPU6000_FW7_姿态解算移植.md)。',
'''当前FW8接入MPU6000与板载LSM303D磁力计的九轴融合及本板实际磁标定，见[FW8记录](MPU6000_FW8_九轴磁融合实测.md)。[FW7六轴记录](MPU6000_FW7_姿态解算移植.md)为历史版本。''')
s=s.replace('FW7默认FireWater输出原7项加Roll/Pitch/相对Yaw/有效标志，共11通道',
'''FW8默认FireWater共18通道：原7项、Roll/Pitch/Yaw、姿态有效、原始磁XYZ和4个磁/航向健康标志''')
s=s.replace('流程：PD15 EXTI',
'''板载LSM303D使用已有PC15片选，以100 Hz ODR连续测磁，主循环轮询新数据，不增加EXTI或DMA配置。SPI1必须经`mpu6000_port.c`仲裁共享，不能绕过仲裁直接访问磁力计。磁力计原生轴已匹配FRD，MPU原生轴映射[Y,-X,Z]；具体参数和18通道表见FW8记录。

流程：PD15 EXTI''')
p.write_text(s,encoding='utf-8')
print('Updated README, Tools README and CubeMX configuration guide')
