"""SuperCar2 binary protocol reference, usable by an Orange Pi ROS base driver.

Dry-run (no serial dependency/hardware):
  python tools/host_link.py --type velocity --left 30 --right 30 --seq 1
One-shot serial command (seq must increase within the current session):
  python tools/host_link.py --port COM5 --type hello
  python tools/host_link.py --port COM5 --type enable --addr 1 --seq 1
Continuous speed commissioning with automatic heartbeat/stop:
  python tools/host_link.py --port COM5 --type run --left 30 --right 30 --seconds 3
Requires pyserial only when --port is used. run stops in finally, including Ctrl+C.
"""
import argparse
import struct
import time

TYPES = {'velocity': 1, 'position': 2, 'stop': 3, 'estop': 4,
         'enable': 5, 'clear': 6, 'heartbeat': 0x10, 'status': 0x11, 'hello': 0x12}

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc

def frame(message_type, seq, payload=b''):
    if len(payload) > 64:
        raise ValueError('Payload exceeds 64 bytes')
    body = bytes((1, message_type, len(payload), seq & 255)) + payload
    return b'\xaa\x55' + body + struct.pack('<H', crc16(body))

class Decoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xaa\x55':
                del self.buffer[0]
                continue
            if len(self.buffer) < 6:
                break
            size = self.buffer[4] + 8
            if self.buffer[2] != 1 or size > 72:
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            data = bytes(self.buffer[:size])
            if crc16(data[2:-2]) != struct.unpack('<H', data[-2:])[0]:
                del self.buffer[0]
                continue
            messages.append((data[3], data[5], data[6:-2]))
            del self.buffer[:size]
        return messages

def describe(message):
    kind, seq, p = message
    if kind == 0x83 and len(p) == 2:
        return f'ACK seq={seq} type=0x{p[0]:02X} result={p[1]}'
    if kind == 0x80 and len(p) == 14:
        tick, fault = struct.unpack('<IH', p[:6])
        return f'STATUS t={tick} fault=0x{fault:04X} left={tuple(p[6:10])} right={tuple(p[10:14])}'
    if kind == 0x81 and len(p) == 40:
        tick = struct.unpack('<I', p[:4])[0]
        left = struct.unpack('<hqII', p[4:22])
        right = struct.unpack('<hqII', p[22:40])
        return f'FEEDBACK t={tick} left(rpm,ticks,speed_t,pos_t)={left} right={right}'
    return f'type=0x{kind:02X} seq={seq} payload={p.hex(" ")}'

class Link:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=0.02)
        self.seq = 0
        self.decoder = Decoder()

    def send(self, kind, payload=b''):
        seq = self.seq
        self.seq = (seq + 1) & 255
        self.serial.write(frame(kind, seq, payload))
        return seq

    def read(self):
        return self.decoder.feed(self.serial.read(max(1, self.serial.in_waiting)))

    def wait(self, kind, seq=None, timeout=1.5, predicate=lambda m: True):
        deadline = time.monotonic() + timeout
        next_heartbeat = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            if time.monotonic() >= next_heartbeat:
                self.send(0x10)
                next_heartbeat = time.monotonic() + 0.1
            for message in self.read():
                if message[0] == 0x80 and struct.unpack('<H', message[2][4:6])[0]:
                    raise RuntimeError(describe(message))
                if message[0] == kind and (seq is None or message[1] == seq) and predicate(message):
                    return message
        raise TimeoutError(f'No expected response type=0x{kind:02X}')

    def command(self, kind, payload=b''):
        seq = self.send(kind, payload)
        reply = self.wait(0x83, seq)
        if reply[2][1] != 0:
            raise RuntimeError(describe(reply))
        return reply

def run(link, args):
    try:
        link.command(0x12)
        # Wait for startup config and stop/disable completion before enabling.
        link.wait(0x80, predicate=lambda m: len(m[2]) == 14 and
                  m[2][8] and m[2][9] and m[2][12] and m[2][13] and
                  not (m[2][7] & 1) and not (m[2][11] & 1))
        # Current SuperCar2 wiring: left=1, right=2.
        link.command(5, bytes((1, 1)))
        link.command(5, bytes((2, 1)))
        link.wait(0x80, predicate=lambda m: bool(m[2][7] & m[2][11] & 1))
        payload = struct.pack('<hhB', args.left, args.right, args.acc)
        deadline = time.monotonic() + args.seconds
        next_velocity = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_velocity:
                link.send(1, payload)
                next_velocity = time.monotonic() + 0.1
            for message in link.read():
                if message[0] in (0x80, 0x81):
                    print(describe(message))
                if message[0] == 0x80 and struct.unpack('<H', message[2][4:6])[0]:
                    raise RuntimeError(describe(message))
                if message[0] == 0x83 and message[2][1]:
                    raise RuntimeError(describe(message))
    finally:
        # Physical disconnection may prevent this write; STM32 also has its watchdog.
        link.send(3)
        link.serial.flush()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--type', choices=[*TYPES, 'run'], default='status')
    parser.add_argument('--seq', type=int, default=0)
    parser.add_argument('--left', type=int, default=0)
    parser.add_argument('--right', type=int, default=0)
    parser.add_argument('--addr', type=int, choices=[1, 2], default=1)
    parser.add_argument('--enabled', type=int, choices=[0, 1], default=1)
    parser.add_argument('--rpm', type=int, default=30)
    parser.add_argument('--acc', type=int, default=10)
    parser.add_argument('--mode', type=int, choices=[0, 1, 2], default=2)
    parser.add_argument('--seconds', type=float, default=3)
    args = parser.parse_args()
    if not 0 <= args.acc <= 255 or not 0 <= args.seq <= 255:
        parser.error('acc and seq must be 0..255')
    if args.type in ('run', 'velocity') and (abs(args.left) > 60 or abs(args.right) > 60):
        parser.error('Initial firmware speed limit is 60 RPM')
    if args.type == 'run':
        if not args.port or args.seconds <= 0:
            parser.error('run requires --port and positive --seconds')
        link = Link(args.port)
        try:
            run(link, args)
        finally:
            link.serial.close()
        return
    kind = TYPES[args.type]
    payload = b''
    if kind == 1:
        payload = struct.pack('<hhB', args.left, args.right, args.acc)
    elif kind == 2:
        if not 1 <= args.rpm <= 60:
            parser.error('position rpm must be 1..60')
        payload = struct.pack('<iiHBB', args.left, args.right, args.rpm, args.acc, args.mode)
    elif kind == 5:
        payload = bytes((args.addr, args.enabled))
    packet = frame(kind, args.seq, payload)
    if not args.port:
        print(packet.hex(' ').upper())
        return
    link = Link(args.port)
    try:
        link.serial.write(packet)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            for message in link.read():
                print(describe(message))
    finally:
        link.serial.close()

if __name__ == '__main__':
    main()
