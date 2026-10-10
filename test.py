# -*- coding: utf-8 -*-
"""USB-TTL直连Emm驱动器的速度发送工具（原来已测通的test.py）。

用途与连接：
  笔记本 -> USB-TTL -> 电机驱动器；本脚本发送F6二进制帧。
  USB-TTL TX接驱动器RX，RX接驱动器TX，GND共地，驱动器独立供电。
  直连前断开STM32 PA9/PA10与驱动器的通信连接，避免两个主机同时发送。
  通过STM32控制电机时，改用stm32_host_test.py并接STM32 PA2/PA3。

准备：
  在工程根目录运行；依赖安装：python -m pip install pyserial
  关闭占用COM11的串口助手；COM11替换为实际USB-TTL端口。
  默认115200/8N1，左轮地址1、右轮地址2，驱动器需使用Emm固件和固定6B校验。
  驱动器需已初始化并处于可运行状态；本脚本不发送F3使能指令。

当前仅接右轮地址2的用法：
  python test.py --port COM11 --acc 10 --run 2 right 10
  python test.py --port COM11 --acc 10 --run 2 right -10
  python test.py --port COM11 stop right
  python test.py --port COM11 --addr 2 --repeat 3 --run 2 right 40

其他用法（对应驱动器须已接线、上电）：
  python test.py --port COM11 --run 2 left 10
  python test.py --port COM11 stop both
  python test.py --help
  python test.py right --help

参数规则：
  --port/--baud/--acc/--addr/--repeat/--open-delay/--run都放在left/right/stop之前。
  speed单位为电机轴RPM；正数按车体前进方向映射，负数按后退方向映射。
  默认左轮正数dir=0、右轮正数dir=1；--addr只改地址，不改变所选轮的方向映射。
  --acc是Emm加速度档位，使用0..255；不是米每秒平方。--repeat只重复发送，不确认成功。
  --run默认0：发送后立即退出，不自动停止，电机可能继续转动！推荐明确指定--run 2。
  --run大于0时，等待指定秒数再发F6零速；没有反馈确认，也不会失能或松轴。
  Ctrl+C/串口异常没有finally停止流程，不保证发送零速；机械固定后再进行测试。
  本脚本没有转速上限校验，初次建议10 RPM；当前STM32上位机脚本上限为60 RPM。

协议与输出：
  8字节帧：地址 F6 方向 速度高字节 速度低字节 加速度 同步位 6B。
  右轮40 RPM、acc=10：02 F6 01 00 28 0A 00 6B。
  stop使用方向0、速度0、加速度0；right 0仍使用所选轮方向和--acc。
  本脚本只发指令，不读取驱动器回包；“已发送”不代表已接受或已停止。
  只读检查配置和反馈使用motor_uart_diag.py；完整STM32测试使用stm32_host_test.py。
"""

import argparse
import time

import serial

# ===== Emm速度帧常量；本工具绕过STM32，直接发送驱动器协议 =====
EMM_V5_VEL_CMD = 0xF6   # 速度模式功能码
CHECKSUM = 0x6B         # 固定校验字节
DEFAULT_ACC = 10        # 默认加速度档位，取值0..255

# 地址：左轮 0x01、右轮 0x02
MOTOR_ADDR = {"left": 0x01, "right": 0x02}
# 方向：左轮正转=0 反转=1；右轮正转=1 反转=0
FORWARD_DIR = {"left": 0, "right": 1}
REVERSE_DIR = {"left": 1, "right": 0}


def build_vel_frame(addr, direction, vel, acc=DEFAULT_ACC):
    """构造8字节F6帧；vel为非负RPM，direction为0/1，acc为0..255。

    本函数没有范围校验；速度字段为两字节大端，调用方应使用合理的小转速。
    """
    return bytes([
        addr,
        EMM_V5_VEL_CMD,
        direction,
        (vel >> 8) & 0xFF,   # 速度高字节
        vel & 0xFF,          # 速度低字节
        acc,
        0x00,                # 同步标志：0=立即执行
        CHECKSUM,
    ])


def hex_str(frame):
    """帧转成 01 F6 01 00 28 0A 00 6B 这种形式，便于和官方工具对比"""
    return " ".join("%02X" % b for b in frame)


def send_frame(ser, frame, repeat=1):
    """发送并等待本机串口输出完成；flush不是驱动器应答确认。"""
    for i in range(repeat):
        ser.write(frame)
        ser.flush()
        if i != repeat - 1:
            time.sleep(0.05)


def send_speed(ser, addr, motor, speed, acc, repeat):
    """将有符号车体速度转换为所选轮的驱动器方向和非负RPM。"""
    if speed < 0:
        direction, vel = REVERSE_DIR[motor], -speed
    else:
        direction, vel = FORWARD_DIR[motor], speed
    frame = build_vel_frame(addr, direction, vel, acc)
    send_frame(ser, frame, repeat)
    return frame


def send_stop(ser, addr, repeat=1):
    """发F6零速帧（dir=0/rpm=0/acc=0）；不读取确认，也不发送F3失能。"""
    frame = build_vel_frame(addr, 0, 0, 0)
    send_frame(ser, frame, repeat)
    return frame


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="串口号，如 COM11")
    p.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    p.add_argument("--acc", type=int, default=DEFAULT_ACC, help="加速度档位，默认 10")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=None,
                   help="覆盖电机地址（十进制或0x开头），默认左轮1 / 右轮2；方向仍由轮名决定")
    p.add_argument("--repeat", type=int, default=1, help="同一帧重复发送次数，默认 1")
    p.add_argument("--open-delay", type=float, default=0.2,
                   help="打开串口后等待秒数再发送，默认 0.2")
    p.add_argument("--run", type=float, default=0,
                   help="left/right发送后等待秒数再发零速，默认0不自动停止；无回包确认")

    sub = p.add_subparsers(dest="cmd", required=True)
    sl = sub.add_parser("left", help="控制左轮")
    sl.add_argument("speed", type=int, help="转速值（正=前进，负=反转，0=停止）")
    sr = sub.add_parser("right", help="控制右轮")
    sr.add_argument("speed", type=int, help="转速值（正=前进，负=反转，0=停止）")
    ss = sub.add_parser("stop", help="停止指定轮子")
    ss.add_argument("motor", choices=["left", "right", "both"], help="要停止的轮子")
    return p.parse_args()


def main():
    args = parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(args.open_delay)   # 给USB-TTL留稳定时间；不保证驱动器此时已启动完成

        if args.cmd in ("left", "right"):
            motor = args.cmd
            addr = args.addr if args.addr is not None else MOTOR_ADDR[motor]
            frame = send_speed(ser, addr, motor, args.speed, args.acc, args.repeat)
            print("地址=0x%02X 帧=%s 重复=%d 次" % (addr, hex_str(frame), args.repeat))

            if args.speed == 0:
                print("已发送: %s 停止" % motor)
                return
            print("已发送: %s 转速=%d 加速度=%d" % (motor, args.speed, args.acc))

            if args.run > 0:
                time.sleep(args.run)
                send_stop(ser, addr, args.repeat)
                print("已发送 %s 停止指令" % motor)

        elif args.cmd == "stop":
            motors = ["left", "right"] if args.motor == "both" else [args.motor]
            for m in motors:
                addr = args.addr if args.addr is not None else MOTOR_ADDR[m]
                frame = send_stop(ser, addr, args.repeat)
                print("地址=0x%02X 帧=%s" % (addr, hex_str(frame)))
                time.sleep(0.005)   # 两台驱动器之间留5 ms发送间隔；TTL直连也保留此间隔
            print("已发送停止指令: %s" % ",".join(motors))


if __name__ == "__main__":
    main()
