# -*- coding: utf-8 -*-
"""笔记本替代香橙派，通过STM32控制X42S/Emm电机的上位机测试工具。

用途与连接：
  笔记本 -> USB-TTL -> STM32 USART2 -> USART1 -> 电机驱动器。
  USB-TTL TX接PA3（STM32 RX），RX接PA2（STM32 TX），GND共地，使用3.3V TTL。
  STM32 PA9（TX）接驱动器RX，PA10（RX）接驱动器TX，驱动器独立供电。
  串口为115200/8N1，向STM32发送以CRLF结束的ASCII命令。
  COM11是USB-TTL端口；ST-Link下载接口不等于USART2。
  test.py/motor_uart_diag.py用于直连驱动器，不能在这里代替本脚本。

安装与预览（在工程根目录运行）：
  python -m pip install pyserial
  python stm32_host_test.py --list-ports
  python stm32_host_test.py --dry-run --single-right right 10 --acc 10 --run 2
  python stm32_host_test.py --help
  python stm32_host_test.py right --help

当前仅接右轮地址2（固件CAR_SINGLE_MOTOR_TEST=2，修改后须重新编译烧录）：
  python stm32_host_test.py --port COM11 --single-right ping
  python stm32_host_test.py --port COM11 --single-right status
  python stm32_host_test.py --port COM11 --single-right diag
  python stm32_host_test.py --port COM11 --single-right recover
  python stm32_host_test.py --port COM11 --single-right right 10 --acc 10 --run 2
  python stm32_host_test.py --port COM11 --single-right right -10 --acc 10 --run 2
  python stm32_host_test.py --port COM11 --single-right position right 800 --rpm 20 --mode 2 --run 10
  python stm32_host_test.py --port COM11 --single-right stop
  python stm32_host_test.py --port COM11 --single-right estop

左轮单接：固件模式1，使用--single，例如：
  python stm32_host_test.py --port COM11 --single left 10 --run 2
双轮接线：固件模式0，两台驱动器都上电，省略单轮选项，例如：
  python stm32_host_test.py --port COM11 wheels 10 10 --acc 10 --run 2
  python stm32_host_test.py --port COM11 left 10 --run 2
  不带单轮选项会检查左右两台驱动器，即使只请求一轮运动也如此。
  --single/--single-right只选择脚本测试范围，不会修改STM32固件配置。

参数与执行流程：
  --port/--single-right/--single/--verbose等全局参数放在子命令之前。
  --acc/--run放在right/left/wheels/position之后；--rpm/--mode仅供position使用。
  speed为电机轴整数RPM，当前范围-60..60；正数前进、负数后退。
  右轮方向由STM32统一转换，Python不要再次反转；--acc为0..255加速度档位。
  速度--run默认3秒；先STOP并确认零速，再失能/使能并确认，然后发送速度目标。
  位置pulses为带符号细分脉冲数；--rpm为1..60，默认20，--run默认10秒。
  --mode：0相对上一目标，1绝对坐标，2相对当前位置（默认）。位置命令只发一次。
  位置到位可提前结束；观察时间到仍未确认到位则停止并报错，不自动重发。
  活动测试期间每100 ms心跳；正常结束/Ctrl+C/通信异常时尽力STOP、失能、确认零速。
  失能后不保持锁轴力矩；测试前架空车轮并固定机械部分，查看退出确认结果。

状态、诊断与恢复：
  ping只确认电脑与STM32通信；status/diag查询反馈，不使能、不运动、不发送心跳。
  ONLINE=1 CONFIG=1 AGE<250表示在线、配置通过、速度反馈较新；FAULT=0无锁存故障。
  STATE：0初始化，1失能，2就绪，3速度，4位置，5停止中，6故障。
  FLAGS：bit0使能，bit1到位，bit2/3堵转/保护；结束时RPM=0且bit0=0。
  OK QUEUED只表示STM32接受请求；执行结果看后续状态，不能单凭该行认定已运动。
  diag的INIT=5表示初始化完成；TIMEOUT/REJECT保存最近一次历史错误及原始回包。
  例如FAULT=68是超时4+指令拒绝64；02 F3 E2 6B表示驱动器拒绝使能/失能请求。
  仅凭E2回包不能确定具体拒绝原因；结合驱动器屏幕、供电和最新状态排查。
  recover排入停止、清保护、失能并等待安全反馈；OK RECOVERY不代表已完成。
  恢复成功须看到FAULT=0；历史diag记录仍保留，不代表故障再次出现，STM32重启会清记录。
  stop停止并失能；estop请求停止并锁存急停故障，释放急停输入后手动recover。
  同一COM只能由一个程序占用；运动时不要另开status/diag程序抢占串口。

完整接线与用法见docs/笔记本上位机测试指南.md。
"""

import argparse
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
    """选择需验证的轮：固件模式2右轮、模式1左轮、模式0双轮，必须与烧录配置一致。"""
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
    parser.add_argument("--port", help="USB-TTL 串口，例如 COM11")
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
    wheels = subs.add_parser("wheels", parents=[motion], help="双轮速度测试")
    wheels.add_argument("left", type=int)
    wheels.add_argument("right", type=int)
    position = subs.add_parser("position", parents=[motion], help="单轮位置测试，只发送一次")
    position.add_argument("motor", choices=MOTOR_ADDR)
    position.add_argument("pulses", type=int, help="带符号int32细分脉冲数")
    position.add_argument("--rpm", type=int, default=20, help="位置动作速度1..60 RPM，默认20")
    position.add_argument("--mode", type=int, choices=(0, 1, 2), default=2,
                          help="0相对上一目标，1绝对坐标，2相对当前位置（默认）")
    position.set_defaults(run=10.0)
    for name, help_text in (
            ("ping", "只确认电脑与STM32通信"), ("status", "只读当前状态和锁存故障"),
            ("diag", "只读初始化进度及历史超时/拒绝回包"), ("stop", "停止、失能并确认反馈"),
            ("estop", "请求停止并锁存急停故障"), ("recover", "手动恢复，等待故障清零且失能/零速")):
        subs.add_parser(name, help=help_text)
    args = parser.parse_args(argv)
    if args.list_ports:
        return args
    if not args.cmd or (not args.port and not args.dry_run):
        parser.error("请指定 --port 和测试命令，或使用 --list-ports / --dry-run")
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
        print("缺少 pyserial，请运行: python -m pip install pyserial", file=sys.stderr)
        return 1
    if args.list_ports:
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device}: {port.description}")
        if not ports:
            print("未发现串口；检查 USB-TTL 连接和驱动。")
        return 0
    try:
        # 短读超时保证等待期间仍可及时心跳；写超时防止 USB 异常永久阻塞。
        with serial.Serial(args.port, 115200, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                           timeout=0.02, write_timeout=0.1,
                           xonxoff=False, rtscts=False, dsrdtr=False) as port:
            time.sleep(args.open_delay)
            port.reset_input_buffer()
            execute(Link(port, args.reply_timeout, args.verbose), args)
    except KeyboardInterrupt:
        print("\n[中断] 请查看停止/失能的确认结果；未确认时检查电机实际状态。")
        return 130
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"[失败] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
