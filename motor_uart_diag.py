# -*- coding: utf-8 -*-
"""USB-TTL直连X42S/Emm驱动器的只读诊断工具。

用途与连接：
  笔记本 -> USB-TTL -> 驱动器；查询二进制回包，适用于排查STM32协议兼容性。
  先确认电机静止并固定；断开STM32 PA9/PA10与驱动器的通信连接。
  USB-TTL TX接驱动器RX，RX接驱动器TX，GND共地，驱动器独立供电。
  固定115200/8N1；驱动器使用固定6B校验。COM11替换为实际USB-TTL端口。
  --direct-driver只是表示已按上述方式接线，不会检测或切换物理连接。
  查询完成后恢复STM32接线，再用stm32_host_test.py测试整条控制链路。

安装与用法（在工程根目录运行）：
  python -m pip install pyserial
  python motor_uart_diag.py --port COM11 --direct-driver --addr 2
  python motor_uart_diag.py --port COM11 --direct-driver --addr 2 --repeat 1 --timeout 0.5
  python motor_uart_diag.py --dry-run --addr 2
  python motor_uart_diag.py --help

参数：
  --addr为十进制地址1..255，默认2（右轮）；左轮默认地址1。
  --repeat为完整查询轮数1..20，默认3；每轮依次查询1A/42/3A/35。
  --timeout为每条指令的接收窗口0.05..2秒，默认0.3秒，不是STM32总线超时参数。
  --dry-run只显示将发送的四条查询帧，不打开串口、不需要--direct-driver。

回包判读（按当前X42S_V1.0/Emm实测格式）：
  1A选项：02 1A 00 06 6B，共5字节；00 06为大端选项0x0006。
    不能按4字节解析；当前STM32要求bit1=1（Emm）、bit7=0（未缩小10倍输入）。
  42配置：当前实测33字节，UART接口2、波特率编码5、地址2、固定6B校验0、应答1。
    本工具只核对列出的门禁字段；其他长度会提示与当前STM32解析器不兼容。
  3A状态：bit0使能，bit1到位，bit2/3堵转/保护；0x03表示使能且到位。
  35速度：驱动器原始有符号RPM，未按车体右轮安装方向转换。
  [RX]始终保留原始字节；不以第一个6B截帧，因为数据字段也可能含6B。

本工具只发送查询，不使能、不运动、不停止、不清保护、不修改参数。
若电机原本在运动，本工具不会将它停下；Ctrl+C只中断查询。
本工具不读取STM32锁存故障；查询该故障请用stm32_host_test.py的status/diag。
"""
import argparse
import math
import sys
import time

QUERIES = ((0x1A, "选项"), (0x42, "配置"), (0x3A, "状态"), (0x35, "速度"))


def query_frame(addr, code):
    """构造白名单中的只读查询；42需额外辅助码6C，其余无辅助码。"""
    if not 1 <= addr <= 255 or code not in dict(QUERIES):
        raise ValueError("无效查询地址或功能码")
    return bytes((addr, code, 0x6C, 0x6B)) if code == 0x42 else bytes((addr, code, 0x6B))


def hex_text(data):
    return data.hex(" ").upper() or "(无数据)"


def describe_reply(data, addr, code):
    """按地址/功能码、已知长度和尾部校验分析回包，不修改原始接收数据。

    兼容检查以本工程实测帧为准；提示通过不等于整条STM32控制链路已通过测试。
    """
    offset = data.find(bytes((addr, code)))
    if offset < 0:
        return "未找到地址/功能码匹配的回包；检查接线、应答配置、串口噪声。"
    packet = data[offset:]
    fixed = {0x1A: 5, 0x3A: 4, 0x35: 6}  # 1A选项为2字节，不能沿用旧4字节假设
    if code == 0x42:
        if len(packet) < 3:
            return "配置回包不完整。"
        size = packet[2]
        if size < 4 or len(packet) < size:
            return f"42回包第三字节=0x{size:02X}，匹配位置后收到{len(packet)}字节；与当前固件期望不符。"
    else:
        size = fixed[code]
    if len(packet) < size or packet[size - 1] != 0x6B:
        return "回包长度或末尾6B不符；不要据此认为驱动器在线。"
    if code == 0x1A:
        option = int.from_bytes(packet[2:4], "big")
        return (f"1A选项=0x{option:04X}；当前固件要求bit1=1、bit7=0，"
                f"本次{'满足' if option & 2 and not option & 0x80 else '不满足'}。")
    if code == 0x3A:
        flags = packet[2]
        return (f"3A标志=0x{flags:02X}；使能={int(bool(flags & 1))}，"
                f"到位={int(bool(flags & 2))}，堵转/保护={int(bool(flags & 0x0C))}。")
    if code == 0x35:
        if packet[2] > 1:
            return "35速度方向字段无效。"
        speed = int.from_bytes(packet[3:5], "big") * (-1 if packet[2] else 1)
        return f"35驱动器原始速度={speed} RPM（尚未按车体左右方向转换）。"
    if size != 33:
        return (f"42回包声明长度={size}字节；当前STM32只接受33字节，"
                "本次格式会被固件丢弃并最终超时。保留完整原始回包核对版本。")
    expected = {3: 0x15, 6: 2, 18: 5, 20: addr, 21: 0, 22: 1}
    names = {3: "参数数量", 6: "接口", 18: "波特率编码", 20: "地址", 21: "校验方式", 22: "应答方式"}
    fields = ", ".join(f"{names[index]}={packet[index]}(期望{value})"
                       for index, value in expected.items())
    matches = all(packet[index] == value for index, value in expected.items())
    return f"42长度33字节；{fields}；{'这些字段通过' if matches else '有字段不匹配'}。"


def read_window(port, seconds):
    """在指定时间内收集全部字节，允许分段回包，并限制接收总量。"""
    deadline = time.monotonic() + seconds
    received = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(min(4096, max(1, port.in_waiting)))
        received.extend(chunk)
        if len(received) > 4096:
            raise RuntimeError("接收超过4096字节；检查是否开启驱动器主动回传或总线有其他发送者")
    return bytes(received)


def diagnose(port, addr, rounds, timeout):
    """依次查询选项、配置、状态和速度；每次查询前显示迟到或主动回包。"""
    for round_number in range(1, rounds + 1):
        print(f"\n--- 第{round_number}轮 ---")
        for code, name in QUERIES:
            # 显示迟到/主动帧，避免把上一次应答误归到本次查询。
            residual = read_window(port, 0.1)
            if residual:
                print(f"[查询前残留RX] {hex_text(residual)}")
            packet = query_frame(addr, code)
            print(f"[TX {code:02X} {name}] {hex_text(packet)}")
            if port.write(packet) != len(packet):
                raise OSError("查询指令没有完整发送")
            reply = read_window(port, timeout)
            print(f"[RX {len(reply)}字节] {hex_text(reply)}")
            print("[分析] " + describe_reply(reply, addr, code))


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="直连驱动器的USB-TTL串口，例如COM11")
    parser.add_argument("--direct-driver", action="store_true",
                        help="表示USB-TTL已直接接驱动器，不是STM32 USART2")
    parser.add_argument("--addr", type=int, default=2, help="十进制电机地址1..255，默认右轮2")
    parser.add_argument("--repeat", type=int, default=3, help="完整查询轮数1..20，默认3")
    parser.add_argument("--timeout", type=float, default=0.3, help="每条查询接收秒数0.05..2，默认0.3")
    parser.add_argument("--dry-run", action="store_true", help="仅预览查询帧，不打开串口")
    args = parser.parse_args(argv)
    if not 1 <= args.addr <= 255 or not 1 <= args.repeat <= 20:
        parser.error("addr必须为1..255，repeat必须为1..20")
    if not math.isfinite(args.timeout) or not 0.05 <= args.timeout <= 2:
        parser.error("timeout必须为0.05..2秒")
    if args.dry_run:
        for code, name in QUERIES:
            print(f"[预览 {name}] {hex_text(query_frame(args.addr, code))}")
        return 0
    if not args.port or not args.direct_driver:
        parser.error("请先按说明直连驱动器，再指定 --port COMxx --direct-driver")
    try:
        import serial
    except ImportError:
        print("请安装依赖: python -m pip install pyserial", file=sys.stderr)
        return 1
    try:
        with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.1,
                           xonxoff=False, rtscts=False, dsrdtr=False) as port:
            time.sleep(0.2)
            print("只读查询：不会使能、运动或修改驱动器参数。")
            diagnose(port, args.addr, args.repeat, args.timeout)
    except KeyboardInterrupt:
        print("诊断已中断。")
        return 130
    except (OSError, RuntimeError) as error:
        print(f"诊断失败: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
