# SuperCar2：STM32 双 X42S 下位机

基于已有 STM32F103C8T6 / HAL / Keil 工程实现，控制链路为 **香橙派 UART1 → STM32 USART2 → USART1 共享 TTL 总线 → 两台 X42S（Emm 固件）**。上位机处理 ROS、导航、差速换算；STM32 接收左右轮 RPM/位置脉冲数，执行驱动器通信、安全保护和反馈。当前实际接线为左轮地址 1、右轮地址 2。

电机协议仅保留 Emm 固件实现，速度使用 RPM，位置配置回包只接受 Emm 的 33 字节格式。

## 从这里开始

1. 打开 `MDK-ARM/SuperCar2.uvprojx`，选择 ARM Compiler 5，编译下载。
2. 阅读 [下位机实现与联调](docs/下位机实现与联调.md)，确认驱动器设置和接线。
3. `CAR_SINGLE_MOTOR_TEST`：0=双轮、1=仅左轮、2=仅右轮；修改后重新编译烧录。当前按仅接右轮地址2设为2，Python使用 `--single-right`；双轮运行改回0。
4. USART2 使用 115200 / 8N1，发送 `PING\r\n`，应返回 `OK PONG`。
5. 当前右轮模式等初始化完成，发送 `MOTOR ENABLE 2`，收到 `OK QUEUED` 后确认使能，再发送 `MOTOR VEL 2 20 10`。运动期间每100 ms发送 `HEARTBEAT`；发送 `STOP` 停止。也可使用下方Python程序自动完成这些步骤。

所有 ASCII 命令以换行结束。`OK QUEUED` 表示下位机接受请求；电机实际状态通过 `STATUS` 或二进制反馈确认。缺少心跳/有效运动控制超过 300 ms 时锁定停机，须 `CLEAR_FAULT` 恢复后重新使能。

## 模块分工

| 文件 | 职责 |
| --- | --- |
| `main.c` | CubeMX 硬件初始化；调用 `CarControl_Init` / `CarControl_Poll` |
| `car_uart.c/.h` | 两路 UART 中断收发、缓冲、非阻塞 TX、异常恢复；ASCII/二进制协议、CRC、序号门禁、ACK 与遥测 |
| `car_motor.c/.h` | Emm 帧和回包、配置门禁、双轮状态、共享总线事务、优先停止及异步恢复 |
| `car_control.c/.h` | 主循环调度、PB12 急停、PB13/14 可选报警、300 ms 保活、故障锁存、PC13 LED、独立看门狗 |
| `car_config.h` | 地址、方向、限速、周期、单轮模式、保护开关 |

## 工程文档

工作空间原 `docs` 已迁入本工程并删除。本工程 `docs` 是唯一文档目录，方案、协议及离线接线图已按当前代码统一更新，入口见 [文档索引](docs/README.md)。

- [下位机实现与联调](docs/下位机实现与联调.md)：实际接线、参数、状态机、故障恢复、验收步骤。
- [参数说明与调参指南](docs/参数说明与调参指南.md)：逐项配置解释、单轮/双轮调速、加速度、位置单位、周期调整及故障排查。
- [旧巡线协议对照与优化记录](docs/旧巡线协议对照与优化记录.md)：对照已实测的 CH343/Emm F6 帧、左右地址、5 ms 间隔和停止路径。
- [上位机协议 v1](docs/上位机协议_v1.md)：当前实现的协议准则；总体方案采用相同的消息编号。
- [接线与 CubeMX 总览](docs/共享串口接线与CubeMX配置总览.html)：当前引脚、接线、已实现模块与软件时序，包含 PB12/13/14、PC13。
- [X42S 实施方案](docs/X42S上位机-STM32电机控制实施方案_v1.md) 与 [ROS 自主探索方案](docs/ROS小车自主探索方案_v1.md)：明确已完成的下位机工作，以及待上板、标定和上位机实施的阶段。

2026-10-05 已完成 3 模块重构，旧的 10 个业务源文件及对应头文件已清理。接口统一为 `CarUart_*`、`CarMotor_*`、`CarControl_*`，协议和控制行为沿用原实现。实施记录见 [模块简化重构规划](docs/模块简化重构规划.md) 和 [项目交接文档](docs/项目交接文档.md)。

## 编译与软件测试

笔记本暂代香橙派时，USB-TTL接STM32 USART2（TX→PA3、RX←PA2、GND共地），运行根目录 `stm32_host_test.py`。接线、单轮模式、心跳与退出失能的注意点见 [笔记本上位机测试指南](docs/笔记本上位机测试指南.md)。

```powershell
python -m pip install pyserial
python stm32_host_test.py --list-ports
python stm32_host_test.py --port COM11 ping
python stm32_host_test.py --port COM11 --single-right status
python stm32_host_test.py --port COM11 --single-right right 20 --run 3
```

原 `test.py` 是电脑直连驱动器的Emm测试；新程序经STM32控制电机，运动期间自动心跳，正常结束/Ctrl+C/异常时尽力停止、失能并确认。双轮模式要求两个地址都在线；仅左轮用固件模式1和 `--single`，仅右轮用模式2和 `--single-right`。当前配置是右轮模式2，须重新烧录才能生效。

驱动器初始化/回包故障可用根目录 `motor_uart_diag.py` 做1A/42/3A/35只读查询并打印原始字节；**需先断开STM32电机串口，将USB-TTL直连驱动器**，再运行 `python motor_uart_diag.py --port COM11 --direct-driver --addr 2`。接线和诊断结果说明见上面的笔记本测试指南。

```powershell
python tools/build_firmware.py --toolchain E:/Keil5/Core/ARM/ARMCC/bin
python tools/test_firmware.py --cc C:/mingw64/bin/gcc.exe
python tools/host_link.py --type velocity --left 30 --right 30 --seq 1
```

命令行固件输出在 `build/firmware/SuperCar2.hex`、`.axf`、`.map`；Keil 自身编译输出在 `MDK-ARM/SuperCar2/`。自动化测试编译真实 `car_uart`、`car_motor`、`car_control` 模块（含板级 IO/看门狗初始化），通过模拟 HAL/驱动器验证协议、断联、急停、故障、位置去重、配置门禁和 TX 缓冲生命周期。模拟测试不覆盖真实电气连接、电机运动和看门狗硬件时序。

CubeMX 重新生成后，运行 `python tools/update_keil_project.py` 恢复应用模块的 Keil 分组。`main.c` 的应用调用位于 USER CODE 区。UART 初始化保留 CubeMX 原结构，GPIO 由 `CarControl_Init` 内部的板级初始化再次配置；USART3、I2C、PWM 和外部编码器未启用。

修改 Markdown 后运行 `python tools/sync_embedded_docs.py`，同步离线接线页的文档下载副本；`--check` 可验证一致性，四张内嵌 SVG 保留。
