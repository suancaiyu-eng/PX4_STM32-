# 独立 PWM 台架测试工程

本目录是完整、独立的 STM32F103C8T6 工程，拥有自己的 `Core`、`Drivers`、启动文件、编译配置和输出目录。源码是一次性复制后独立维护的快照，没有符号链接、外部源码路径或自动同步脚本；构建不需要正式固件目录，也不包含遥控、板间协议或启动计数写入。

打开 **`MDK-ARM/F1_test.uvprojx`**，选择以下目标之一：

| 目标 | 行为 | HEX |
| --- | --- | --- |
| `F1_PWM_SWEEP`（默认） | 上电后 MAIN2、3、4、5 同时按10%、20%……90%，每档1秒循环 | `MDK-ARM/F1_PWM_SWEEP/F1_PWM_SWEEP.hex` |
| `F1_PWM_TEST` | 上电低电平，通过 Watch 显式触发一路限时/持续输出 | `MDK-ARM/F1_PWM_TEST/F1_PWM_TEST.hex` |

固定400 Hz、周期2500 µs、计时分辨率1 µs，使用内部HSI倍频64 MHz：

| 通道 | 引脚 | 定时器 |
| --- | --- | --- |
| MAIN2 | PA1 | TIM2_CH2 |
| MAIN3 | PB8 | TIM4_CH3 |
| MAIN4 | PB9 | TIM4_CH4 |
| MAIN5 | PA6 | TIM3_CH1 |

两路TIM4共享周期，比较值分别设置。其他MAIN通道不配置PWM。三组定时器之间不承诺严格相位同步。

## 自动四路扫描

Rebuild → Flash → Download，进入调试后按 F5 运行，或正常断电重启。不需要修改 Watch。四路高电平宽度依次为250、500、750、1000、1250、1500、1750、2000、2250 µs，每秒切换，9秒一轮。

当前四路采用相同占空比轨迹，便于依次换探头验证每个接口。若要验证不同脉宽及独立通道控制，使用下述单路Watch目标或修改本目录的本地测试调用。

可查看 `g_main_pwm_sweep`：`magic=0x53575031`、`duty_percent`、`pulse_us`、`transitions`。运行中将 `enabled=0x00000000` 可停止四路，改为1从10%重新开始。暂停CPU不等于停止硬件PWM。

扫描范围超出电调1000–2000 µs范围，**仅接示波器，不接电调或电机**。

## Watch 单路测试

选择 `F1_PWM_TEST` 构建/下载，保持 CPU 运行。设置 `g_main_pwm_test`，参数写完后最后递增 `sequence`：

| 字段 | 持续输出MAIN2的值 |
| --- | --- |
| `command` | `0x00000003` |
| `main_channel` | `0x00000002`（也可3、4、5） |
| `pulse_us` | `0x000003E8`（1000 µs，允许1000–2000） |
| `duration_ms` | `0x00000000`（持续模式忽略） |
| `sequence` | 复位后首次为1，以后递增 |

command=1为1–1000 ms的限时测试；command=2停止。一次只启用一路，其余三路低电平。修改参数但不递增序号不会触发；result=1运行、2停止、3限时完成、4参数非法、5驱动/超时错误。主循环服务异常后不自动恢复。

## 构建和隔离证据

- `Tools/prepare_keil.py` 仅校验本目录工程，断言全部编译源码、包含路径均位于本目录内部，不读写其他项目。
- `python -m unittest discover -s Tests -p test_main_pwm.py -v` 使用本目录自己的PWM源码和HAL桩验证四路映射、范围、超时、Watch及扫描。
- `Logs/separation_validation.json` 记录两目标构建和隔离检查；`Logs/*_relocation_build.log` 是整个工程移到临时目录后的重新构建证据，两个目标均不需要原工程源码。
- `Archive/main2_scope_verified_legacy.zip` 保存此前用户实测正确的单路MAIN2固件、源码、旧测试和日志，仅用于追溯，不参与当前工程构建。

实物证据边界：用户已经确认旧MAIN2扫描波形正确；本次四路版本尚未烧录、MAIN3–5波形尚待实测。
