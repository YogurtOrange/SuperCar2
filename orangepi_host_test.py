# -*- coding: utf-8 -*-
"""Orange Pi 5 双轮上位机测试工具，通过 STM32 控制左右 X42S/Emm 电机。

本文件基于 stm32_host_test.py，可单独复制到香橙派运行；只依赖 pyserial。

接线（Orange Pi 5 V1.2 的 26 针排针，均为物理针脚号）：
  8  UART0_TX_M2 -> STM32 PA3（USART2 RX）
  10 UART0_RX_M2 -> STM32 PA2（USART2 TX）
  14 GND        -> STM32 GND（也可用物理 6 脚 GND）
  香橙派和 STM32 各自供电，只连接 TX/RX/GND，使用 3.3V TTL。
  STM32 PA9（USART1 TX）接两台驱动器 RX，PA10（USART1 RX）接共享总线的回包线。
  左轮驱动器地址为 1，右轮为 2；两台均须上电并按工程文档配置共享 UART 总线。
  拆掉 UART0 回环测试时 8/10 脚之间的短接线后，再连接 STM32。

串口与协议：
  默认 /dev/ttyS0，115200/8N1，无流控，发送以 CRLF 结束的 ASCII 命令。
  串口须已启用且排针复用配置正确；本脚本不修改设备树或系统控制台。
  --port 可选择 /dev/ttyS1、/dev/ttyS3、/dev/ttyS4 或 USB-TTL 设备。
  Linux 下独占打开串口；运行时不要另开串口终端或第二个测试进程。

香橙派安装与首次通信测试：
  sudo apt install python3-serial
  python3 orangepi_host_test.py --list-ports
  python3 orangepi_host_test.py ping
  python3 orangepi_host_test.py status
  python3 orangepi_host_test.py diag
  python3 orangepi_host_test.py --help
  python3 orangepi_host_test.py wheels --help
  若系统不是 Debian/Ubuntu，可在自己的 Python 虚拟环境中 pip install pyserial。
  若没有串口权限：sudo usermod -aG dialout "$USER"，然后退出登录并重新登录。

当前双轮模式（固件 CAR_SINGLE_MOTOR_TEST=0，修改后重新编译、烧录）：
  python3 orangepi_host_test.py recover
  python3 orangepi_host_test.py --dry-run wheels 10 10 --acc 10 --run 2
  python3 orangepi_host_test.py wheels 10 10 --acc 10 --run 2       # 双轮前进
  python3 orangepi_host_test.py wheels -10 -10 --acc 10 --run 2   # 双轮后退
  python3 orangepi_host_test.py wheels 10 20 --acc 10 --run 2     # 左右不同速度
  python3 orangepi_host_test.py wheels -10 10 --acc 10 --run 2    # 原地左转
  python3 orangepi_host_test.py wheels 10 -10 --acc 10 --run 2    # 原地右转
  python3 orangepi_host_test.py stop
  python3 orangepi_host_test.py estop
  wheels 的两个参数依次为左轮 RPM、右轮 RPM；左右都用车体前进方向为正。
  默认同时检查、查询并在退出时失能两轮；status/diag/recover 同时处理两轮。
  也可只让其中一轮运动，另一轮仍需在线：
  python3 orangepi_host_test.py right 10 --acc 10 --run 2
  python3 orangepi_host_test.py position right 800 --rpm 20 --mode 2 --run 10

保留单轮调试选项，使用前先修改固件模式并重新编译、烧录：
  左轮单接：固件模式 1，使用 --single，例如：
  python3 orangepi_host_test.py --single left 10 --run 2
  右轮单接：固件模式 2，使用 --single-right，例如：
  python3 orangepi_host_test.py --single-right right 10 --run 2
  --single/--single-right 仅选择测试范围，不会修改 STM32 固件。
  未带单轮选项时会检查两台驱动器，即使只请求其中一轮运动。

参数与运动流程：
  --port/--single-right/--single/--verbose 等全局参数放在子命令之前。
  --acc/--run 放在 right/left/wheels/position 之后；--rpm/--mode 仅供 position。
  速度为整数电机轴 RPM，范围 -60..60，正数前进、负数后退。
  右轮方向由 STM32 转换，Python 不再次反转；加速度档位 --acc 为 0..255。
  速度测试 --run 默认 3 秒，先确认零速/失能，再使能并等待真实反馈后发目标。
  位置 pulses 为带符号 int32 细分脉冲数；--rpm 默认 20，--run 默认 10 秒。
  --mode：0 相对上一目标，1 绝对坐标，2 相对当前位置（默认）。
  位置指令只发一次；到位提前结束，超时未确认到位则停止并报错，不自动重发。
  活动测试每 100 ms 发心跳；正常结束/Ctrl+C/通信异常时尽力停止、失能并确认。
  失能后不保持锁轴力矩；运动测试前架空车轮并固定机械部分。

反馈与故障：
  ping/status/diag 不使能、不运动、不发心跳；ping 只验证香橙派到 STM32 的通信。
  ONLINE=1 CONFIG=1 AGE<250 表示在线、配置通过且反馈新鲜；FAULT=0 为无锁存故障。
  STATE：0 初始化，1 失能，2 就绪，3 速度，4 位置，5 停止中，6 故障。
  FLAGS：bit0 使能，bit1 到位，bit2/3 堵转/保护；退出须确认 RPM=0 且 bit0=0。
  OK QUEUED 仅说明请求已入队；执行结果以后续状态反馈为准。
  diag 的 INIT=5 表示初始化完成；TIMEOUT/REJECT 是最近一次历史错误回包。
  recover 不会重新运动；必须等待 FAULT=0、失能和零速才算恢复成功。
  estop 锁存急停故障，释放急停输入后需要手动 recover。
"""

import argparse
import glob
import os
import math
import re
import sys
import time

MOTOR_ADDR = {"left": 1, "right": 2}
MAX_RPM = 60                    # 与当前 car_config.h 一致
HEARTBEAT_SECONDS = 0.1
FEEDBACK_MAX_AGE_MS = 250
MOTOR_RE = re.compile(
    r"DATA MOTOR (\d+) STATE (\d+) RPM (-?\d+) FLAGS (\d+) FAULT (\d+)")
FEEDBACK_RE = re.compile(
    r"DATA FEEDBACK (\d+) ONLINE (\d+) CONFIG (\d+) AGE (\d+)")
FAULT_NAMES = {
    1: "急停/PB12", 2: "上位机心跳超时", 4: "电机通信/反馈超时",
    8: "驱动器保护/使能丢失", 16: "驱动器配置不匹配",
    32: "串口错误", 64: "驱动器拒绝指令/反馈非法",
}


def fault_text(mask):
    names = [name for bit, name in FAULT_NAMES.items() if mask & bit]
    return f"故障掩码={mask} (0x{mask:04X}): {', '.join(names) or '未知故障'}"


class Link:
    """单线程ASCII收发；heartbeat开启时，等待应答/运动/退出期间继续心跳。

    按换行组装分段数据，区分普通应答与故障事件；只读命令不启用heartbeat。
    """

    def __init__(self, serial_port, reply_timeout=0.8, verbose=False):
        self.serial = serial_port
        self.reply_timeout = reply_timeout
        self.verbose = verbose
        self.buffer = bytearray()
        self.heartbeat = False
        self.next_heartbeat = 0.0
        self.fault = 0

    def send(self, command):
        if command != "HEARTBEAT" or self.verbose:
            print(f"[TX] {command}")
        packet = (command + "\r\n").encode("ascii")
        if self.serial.write(packet) != len(packet):
            raise OSError("串口没有完整发送指令")

    def poll(self):
        now = time.monotonic()
        if self.heartbeat and now >= self.next_heartbeat:
            self.send("HEARTBEAT")
            self.next_heartbeat = time.monotonic() + HEARTBEAT_SECONDS
        self.buffer.extend(self.serial.read(min(4096, max(1, self.serial.in_waiting))))
        lines = []
        while b"\n" in self.buffer:
            raw, _, remaining = self.buffer.partition(b"\n")
            self.buffer = bytearray(remaining)
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            lines.append(line)
            if line != "OK HEARTBEAT" or self.verbose:
                print(f"[RX] {line}")
            match = MOTOR_RE.fullmatch(line)
            if match:
                self.fault = int(match[5])
            elif line.startswith("EVENT FAULT "):
                try:
                    self.fault = int(line.split()[-1])
                except ValueError:
                    raise RuntimeError(f"无法解析故障事件: {line}")
        if len(self.buffer) > 4096:
            raise RuntimeError("接收数据长期没有换行；检查波特率和是否混用了二进制协议")
        return lines

    def check_fault(self):
        if self.fault:
            raise RuntimeError(fault_text(self.fault) + "；修正原因后手动运行 recover")

    def wait(self, accept, timeout=None, allow_fault=False):
        deadline = time.monotonic() + (self.reply_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            lines = self.poll()
            if not allow_fault:
                self.check_fault()
            result = None
            for line in lines:
                if line.startswith("ERR "):
                    raise RuntimeError(f"STM32 拒绝指令: {line}")
                accepted = accept(line)
                if accepted is not None:
                    result = accepted
            if result is not None:
                return result
        raise TimeoutError("等待 STM32 应答超时；检查 USART2 接线、串口号、烧录程序")

    def command(self, command, expected, allow_fault=False):
        self.send(command)
        # HEARTBEAT 的 OK 不会被误当成当前命令的 OK。
        return self.wait(lambda line: line if line == expected else None,
                         allow_fault=allow_fault)

    def status(self, addr, allow_fault=False):
        """收齐指定地址的MOTOR和FEEDBACK两行，返回状态与速度反馈年龄。"""
        self.send(f"MOTOR STATUS {addr}")
        result = {}

        def accept(line):
            motor = MOTOR_RE.fullmatch(line)
            feedback = FEEDBACK_RE.fullmatch(line)
            if motor and int(motor[1]) == addr:
                result.update(zip(("state", "rpm", "flags", "fault"),
                                  map(int, motor.groups()[1:])))
            if feedback and int(feedback[1]) == addr:
                result.update(zip(("online", "config", "age"),
                                  map(int, feedback.groups()[1:])))
            if len(result) == 7:
                return result
            return None

        return self.wait(accept, allow_fault=allow_fault)

    def pause(self, seconds, allow_fault=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            lines = self.poll()
            if not allow_fault:
                self.check_fault()
            for line in lines:
                if line.startswith("ERR "):
                    raise RuntimeError(f"STM32 拒绝指令: {line}")

    def diagnostic(self, addr):
        """读取INIT及最近超时/拒绝记录三行；历史错误不会据此触发运动。"""
        self.send(f"MOTOR DIAG {addr}")
        result = {}

        def accept(line):
            for kind in ("DIAG", "TIMEOUT", "REJECT"):
                if line.startswith(f"DATA {kind} {addr} "):
                    result[kind] = line
            return result if len(result) == 3 else None

        return self.wait(accept, allow_fault=True)


def fresh(status):
    """速度反馈需同时在线、配置通过且年龄小于250 ms；对应当前固件门禁。"""
    return status["online"] == 1 and status["config"] == 1 and status["age"] < FEEDBACK_MAX_AGE_MS


def stopped(status):
    """检查新鲜零速及无堵转/保护；是否失能由调用方另外检查FLAGS bit0。"""
    return fresh(status) and status["rpm"] == 0 and not (status["flags"] & 0x0C)


def wait_states(link, addresses, predicate, timeout, allow_fault=False):
    deadline = time.monotonic() + timeout
    last = {}
    while time.monotonic() < deadline:
        last = {addr: link.status(addr, allow_fault) for addr in addresses}
        if all(predicate(status) for status in last.values()):
            return last
        link.pause(0.1, allow_fault)
    raise TimeoutError(f"等待电机状态超时，最后状态={last}；检查驱动器供电/回包/配置")


def prepare(link, configured, moving, timeout):
    """确认所有配置轮静止且失能，再只使能本次运动轮；每一步等待真实反馈。"""
    link.command("PING", "OK PONG")
    link.command("STOP", "OK STOP")
    wait_states(link, configured,
                lambda s: stopped(s) and s["state"] in (1, 2), timeout)
    for addr in configured:
        link.command(f"MOTOR ENABLE {addr} 0", "OK QUEUED")
    wait_states(link, configured,
                lambda s: stopped(s) and s["state"] == 1 and not (s["flags"] & 1), timeout)
    for addr in moving:
        link.command(f"MOTOR ENABLE {addr} 1", "OK QUEUED")
    wait_states(link, moving,
                lambda s: fresh(s) and s["state"] == 2 and bool(s["flags"] & 1), timeout)


def shutdown(link, configured, timeout):
    """不自动清故障；每一步失败仍尝试其余停止/失能操作，明确报告确认失败。"""
    failures = []
    for command, expected in [("STOP", "OK STOP")] + [
            (f"MOTOR ENABLE {addr} 0", "OK QUEUED") for addr in configured]:
        try:
            link.command(command, expected, allow_fault=True)
        except (OSError, RuntimeError, TimeoutError) as error:
            failures.append(f"{command}: {error}")
    try:
        wait_states(link, configured,
                    lambda s: stopped(s) and not (s["flags"] & 1),
                    timeout, allow_fault=True)
    except (OSError, RuntimeError, TimeoutError) as error:
        failures.append(str(error))
    if failures:
        print("[注意] 停止/失能未全部确认: " + "; ".join(failures), file=sys.stderr)
        print("请确认电机实际停止，必要时使用硬件急停。", file=sys.stderr)
        return False
    print("[完成] STM32 反馈：电机零速且已失能（退出后不保持锁轴力矩）。")
    return True


def motion_command(args):
    """生成STM32 ASCII请求；方向转换和F6/FD二进制组帧由STM32完成。"""
    if args.cmd in ("left", "right"):
        return f"MOTOR VEL {MOTOR_ADDR[args.cmd]} {args.speed} {args.acc}"
    if args.cmd == "wheels":
        return f"WHEELS {args.left} {args.right} {args.acc}"
    return (f"MOTOR POS {MOTOR_ADDR[args.motor]} {args.pulses} "
            f"{args.rpm} {args.acc} {args.mode}")


def configured_addresses(args):
    """默认同时验证左右地址1/2；显式单轮选项仅用于配套的单轮调试固件。"""
    return [2] if args.single_right else [1] if args.single else [1, 2]


def execute(link, args):
    """执行只读查询、恢复或运动；活动会话统一通过finally停止、失能并确认。"""
    configured = configured_addresses(args)
    if args.cmd == "ping":
        link.command("PING", "OK PONG", allow_fault=True)
        return
    if args.cmd == "status":
        for addr in configured:
            status = link.status(addr, allow_fault=True)
            if status["fault"]:
                print("[故障] " + fault_text(status["fault"]))
        return
    if args.cmd == "diag":
        for addr in configured:
            link.status(addr, allow_fault=True)
            link.diagnostic(addr)
        return
    if args.cmd == "estop":
        link.command("ESTOP", "OK ESTOP", allow_fault=True)
        print("[急停] 已请求停止并锁存故障；释放急停输入后手动 recover。")
        return

    link.heartbeat = True
    try:
        if args.cmd == "stop":
            return                         # finally 中停止、失能并验证
        if args.cmd == "recover":
            link.command("CLEAR_FAULT", "OK RECOVERY", allow_fault=True)
            wait_states(link, configured,
                        lambda s: not s["fault"] and stopped(s) and s["state"] == 1
                        and not (s["flags"] & 1), args.ready_timeout, allow_fault=True)
            print("[恢复] 故障已清除；本命令不会使能或重新运动。")
            return

        moving = configured if args.cmd == "wheels" else [
            MOTOR_ADDR[args.motor if args.cmd == "position" else args.cmd]]
        prepare(link, configured, moving, args.ready_timeout)
        link.command(motion_command(args), "OK QUEUED")  # 位置绝不自动重发
        deadline = time.monotonic() + args.run
        while time.monotonic() < deadline:
            states = {addr: link.status(addr) for addr in configured}
            if any(not fresh(s) for s in states.values()):
                raise RuntimeError("电机离线、配置无效或速度反馈过期，终止测试")
            if any(not (states[addr]["flags"] & 1) for addr in moving):
                raise RuntimeError("运动电机使能丢失，终止测试")
            if args.cmd == "position" and all(
                    states[addr]["state"] == 2 and states[addr]["flags"] & 2
                    and states[addr]["rpm"] == 0 for addr in moving):
                print("[到位] 驱动器反馈已到达目标且速度为零。")
                return
            link.pause(min(0.2, max(0.0, deadline - time.monotonic())))
        if args.cmd == "position":
            raise TimeoutError("位置动作在 --run 时间内未确认到位，停止；本次位置指令不会重发")
    finally:
        was_successful = sys.exc_info()[0] is None
        confirmed = shutdown(link, configured, args.ready_timeout)
        link.heartbeat = False
        if was_successful and not confirmed:
            raise RuntimeError("退出时未确认电机停止/失能")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyS0",
                        help="香橙派串口，默认 /dev/ttyS0；也可使用 /dev/ttyUSB0")
    parser.add_argument("--list-ports", action="store_true", help="列出串口，不发送指令")
    single = parser.add_mutually_exclusive_group()
    single.add_argument("--single", action="store_true", help="固件模式1：仅左地址1")
    single.add_argument("--single-right", action="store_true", help="固件模式2：仅右地址2")
    parser.add_argument("--dry-run", action="store_true", help="预览运动流程，不打开串口")
    parser.add_argument("--verbose", action="store_true", help="也打印心跳收发")
    parser.add_argument("--open-delay", type=float, default=0.2, help="打开串口后等待秒数，默认0.2")
    parser.add_argument("--ready-timeout", type=float, default=8.0, help="等待初始化/恢复/停止状态上限秒数，默认8")
    parser.add_argument("--reply-timeout", type=float, default=0.8, help="单条STM32请求应答超时秒数，默认0.8")
    subs = parser.add_subparsers(dest="cmd")
    motion = argparse.ArgumentParser(add_help=False)
    motion.add_argument("--acc", type=int, default=10, help="加速度档位0..255，首次建议10")
    motion.add_argument("--run", type=float, default=3.0, help="运行/观察上限秒数，必须大于0")
    for side in ("left", "right"):
        sub = subs.add_parser(side, parents=[motion], help=f"{side} 单轮速度测试")
        sub.add_argument("speed", type=int, help="整数RPM，正=前进，负=后退，0=零速")
    wheels = subs.add_parser("wheels", parents=[motion], help="双轮速度测试，左右参数均以车体前进为正")
    wheels.add_argument("left", type=int, help="左轮地址1的整数RPM，范围-60..60")
    wheels.add_argument("right", type=int, help="右轮地址2的整数RPM，范围-60..60，无需反转符号")
    position = subs.add_parser("position", parents=[motion], help="单轮位置测试，只发送一次")
    position.add_argument("motor", choices=MOTOR_ADDR)
    position.add_argument("pulses", type=int, help="带符号int32细分脉冲数")
    position.add_argument("--rpm", type=int, default=20, help="位置动作速度1..60 RPM，默认20")
    position.add_argument("--mode", type=int, choices=(0, 1, 2), default=2,
                          help="0相对上一目标，1绝对坐标，2相对当前位置（默认）")
    position.set_defaults(run=10.0)
    for name, help_text in (
            ("ping", "只确认香橙派与STM32通信"), ("status", "只读当前状态和锁存故障"),
            ("diag", "只读初始化进度及历史超时/拒绝回包"), ("stop", "停止、失能并确认反馈"),
            ("estop", "请求停止并锁存急停故障"), ("recover", "手动恢复，等待故障清零且失能/零速")):
        subs.add_parser(name, help=help_text)
    args = parser.parse_args(argv)
    if args.list_ports:
        return args
    if not args.cmd or (not args.port and not args.dry_run):
        parser.error("请指定测试命令，或使用 --list-ports / --dry-run")
    for name in ("open_delay", "ready_timeout", "reply_timeout", "run"):
        value = getattr(args, name, None)
        if value is not None and (not math.isfinite(value) or
                                 value < 0 or (name != "open_delay" and value == 0)):
            parser.error(f"{name.replace('_', '-')} 必须是有限的正数（open-delay可为0）")
    if hasattr(args, "acc") and not 0 <= args.acc <= 255:
        parser.error("acc 必须在0..255之间")
    speeds = [getattr(args, name, 0) for name in ("speed", "left", "right")]
    if any(abs(speed) > MAX_RPM for speed in speeds):
        parser.error(f"当前固件速度范围为 -{MAX_RPM}..{MAX_RPM} RPM")
    if args.cmd == "position" and (
            not -(2**31) <= args.pulses < 2**31 or not 1 <= args.rpm <= MAX_RPM):
        parser.error("位置脉冲必须是int32，位置速度必须在1..60 RPM之间")
    selected = "right" if args.single_right else "left" if args.single else None
    if selected and (args.cmd == "wheels" or
                     (args.cmd in MOTOR_ADDR and args.cmd != selected) or
                     (args.cmd == "position" and args.motor != selected)):
        parser.error(f"当前单轮参数只控制{selected}，请使用 {selected} / position {selected}")
    return args


def dry_run(args):
    configured = configured_addresses(args)
    print("[预览] 不打开串口，不代表电机实际可用；每条指令以 CRLF 结束。")
    print(f"[配置] 串口 {args.port}，115200/8N1；验证电机地址 {configured}")
    if args.cmd in ("left", "right", "wheels", "position"):
        print("PING -> 等 OK PONG；STOP -> 等零速/在线/配置有效")
        for addr in configured:
            print(f"MOTOR ENABLE {addr} 0")
        print("查询状态，确认初始化完成且已失能")
        moving = configured if args.cmd == "wheels" else [
            MOTOR_ADDR[args.motor if args.cmd == "position" else args.cmd]]
        for addr in moving:
            print(f"MOTOR ENABLE {addr} 1")
        print("查询状态，确认实际使能和 READY")
        print(motion_command(args))
        print(f"每100 ms HEARTBEAT；约每200 ms查询状态；运行/观察上限 {args.run:g} 秒")
    elif args.cmd in ("ping", "estop"):
        print(args.cmd.upper())
        return
    elif args.cmd == "status":
        for addr in configured:
            print(f"MOTOR STATUS {addr}")
        return
    elif args.cmd == "diag":
        for addr in configured:
            print(f"MOTOR STATUS {addr}")
            print(f"MOTOR DIAG {addr}")
        return
    elif args.cmd == "recover":
        print("CLEAR_FAULT -> 持续心跳并查询状态，等待故障=0且失能/零速")
    print("STOP")
    for addr in configured:
        print(f"MOTOR ENABLE {addr} 0")
    print("继续心跳，查询状态确认零速/失能后关闭串口")


def main(argv=None):
    # Windows重定向输出也使用UTF-8，避免终端/保存日志出现中文乱码。
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    args = parse_args(argv)
    if args.dry_run and not args.list_ports:
        dry_run(args)
        return 0
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("缺少 pyserial，请运行: sudo apt install python3-serial（Debian/Ubuntu），或在虚拟环境中 pip install pyserial", file=sys.stderr)
        return 1
    if args.list_ports:
        ports = {port.device: port.description for port in list_ports.comports()}
        # 补全板载节点；列出节点不代表已验证排针复用或物理通信。
        for pattern in ("/dev/ttyS*", "/dev/ttyAS*", "/dev/ttyAMA*",
                        "/dev/ttyUSB*", "/dev/ttyACM*"):
            for device in glob.glob(pattern):
                ports.setdefault(device, "系统串口设备节点")
        for device, description in sorted(ports.items()):
            access = "可读写" if os.access(device, os.R_OK | os.W_OK) else "需检查权限"
            print(f"{device}: {description} [{access}]")
        if not ports:
            print("未发现串口；检查 UART 启用配置或 USB-TTL 连接。")
        print("设备列表仅说明节点存在；串口占用、排针复用和收发需另行确认。")
        return 0
    try:
        # 短读/写超时保证心跳；Linux 独占锁避免另一脚本同时打开串口。
        options = {"exclusive": True} if sys.platform.startswith("linux") else {}
        print(f"[连接] {args.port}，115200/8N1；验证电机地址 {configured_addresses(args)}")
        with serial.Serial(args.port, 115200, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                           timeout=0.02, write_timeout=0.1,
                           xonxoff=False, rtscts=False, dsrdtr=False, **options) as port:
            time.sleep(args.open_delay)
            port.reset_input_buffer()
            execute(Link(port, args.reply_timeout, args.verbose), args)
    except KeyboardInterrupt:
        print("\n[中断] 请查看停止/失能的确认结果；未确认时检查电机实际状态。")
        return 130
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"[失败] {error}", file=sys.stderr)
        if sys.platform.startswith("linux") and isinstance(error, OSError):
            if not os.path.exists(args.port):
                print(f"串口节点不存在: {args.port}；检查 --port 和 UART 启用配置。",
                      file=sys.stderr)
            elif not os.access(args.port, os.R_OK | os.W_OK):
                print('串口权限不足；若设备属 dialout 组，执行：'
                      'sudo usermod -aG dialout "$USER"，然后退出登录并重新登录。',
                      file=sys.stderr)
            else:
                print("检查串口占用、设备状态，以及 TX/RX/GND 接线。", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
