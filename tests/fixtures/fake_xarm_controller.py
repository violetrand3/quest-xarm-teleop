"""SYNTHETIC stand-in for an xArm controller, on loopback only. It is not a robot and proves
nothing about one. Its motion model (targets replaced in mode 7, queued in mode 0, state 4 clears
them, a target outside the boundary raises an error) is an ASSUMPTION used to exercise our code;
whether the real controller behaves that way is exactly what a supervised test has to find out. It answers the private Modbus protocol on a control port and streams rich
reports on a report port, and it records EVERY frame it receives so a test can assert that an
observer sent nothing but reads. Layouts follow the installed SDK 1.18.4 source."""
from __future__ import annotations

import math
import socket
import struct
import threading
import time

POSE = [299.83, 3.43, 461.18, math.radians(179.86), math.radians(-15.1), math.radians(-6.64)]
JOINTS = [math.radians(v) for v in (1.0, -30.0, -40.0, 0.5, 55.0, -6.0, 0.0)]
TCP_OFFSET = [0.0, 0.0, 172.0, 0.0, 0.0, 0.0]
WORLD_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
BOUNDARY = [502, 160, 270, -300, 585, 270]  # x_max, x_min, y_max, y_min, z_max, z_min


class FakeController:
    def __init__(self, err: int = 0, warn: int = 0, silent: bool = False, report: bool = True, move: bool = False,
                 report_period_s: float = 0.02, mode7_replaces: bool = True, pose: list | None = None):  # fmt: skip
        self.err, self.warn, self.silent, self.report, self.move, self.period = err, warn, silent, report, move, report_period_s
        self.mode7_replaces = mode7_replaces
        self.pose = list(pose or POSE)  # mm, rad
        self.mode, self.state, self.enabled = 0, 4, True
        self.target: tuple | None = None  # (xyz, speed mm/s)
        self.queue: list = []
        self.max_speed = 50.0  # reduced-mode cap
        self.lock = threading.Lock()
        self.moves_accepted = 0
        self.frames: list = []  # (protocol, register, payload) of every request received
        self.alive = True
        self.ctrl = self._listen()
        self.rep = self._listen() if report else None
        self.control_port = self.ctrl.getsockname()[1]
        self.report_port = self.rep.getsockname()[1] if self.rep else self._unused_port()
        self.threads = [threading.Thread(target=self._accept, args=(self.ctrl, self._control), daemon=True)]
        if self.rep:
            self.threads.append(threading.Thread(target=self._accept, args=(self.rep, self._reports), daemon=True))
        self.threads.append(threading.Thread(target=self._motion, daemon=True))
        for t in self.threads:
            t.start()

    @staticmethod
    def _listen() -> socket.socket:
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", 0))
        s.listen(2)
        s.settimeout(0.2)
        return s

    @staticmethod
    def _unused_port() -> int:
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    def close(self) -> None:
        self.alive = False
        for s in (self.ctrl, self.rep):
            if s:
                s.close()

    def _accept(self, srv: socket.socket, handler) -> None:  # type: ignore[no-untyped-def]
        while self.alive:
            try:
                conn, _ = srv.accept()
            except (socket.timeout, OSError):
                continue
            threading.Thread(target=handler, args=(conn,), daemon=True).start()

    def _motion(self) -> None:
        dt = 0.005
        while self.alive:
            time.sleep(dt)
            with self.lock:
                if self.state in (4, 5) or not self.enabled:
                    continue
                if self.target is None and self.queue:
                    self.target = self.queue.pop(0)
                if self.target is None:
                    self.state = 2
                    continue
                xyz, speed = self.target
                d = [xyz[k] - self.pose[k] for k in range(3)]
                n = math.sqrt(sum(v * v for v in d))
                step = min(speed, self.max_speed) * dt
                if n <= step:
                    self.pose[:3] = list(xyz)
                    self.target = None
                else:
                    self.state = 1
                    for k in range(3):
                        self.pose[k] += d[k] / n * step

    def _write(self, reg: int, payload: bytes) -> int:
        """Apply a write. Returns extra state bits for the reply (0x10 = not ready)."""
        with self.lock:
            if reg == 11:
                self.enabled = bool(payload[1])
            elif reg == 19:
                self.mode = payload[0]
            elif reg == 12:
                if payload[0] == 4:
                    self.target, self.queue, self.state = None, [], 4
                elif payload[0] == 0 and not self.err:
                    self.state = 2
            elif reg == 21:
                if self.err or self.state in (4, 5) or not self.enabled:
                    return 0x10
                vals = struct.unpack("<10f", payload[:40])
                xyz, speed = list(vals[:3]), vals[6]
                b = BOUNDARY
                if not (b[1] <= xyz[0] <= b[0] and b[3] <= xyz[1] <= b[2] and b[5] <= xyz[2] <= b[4]):
                    self.err, self.state, self.target, self.queue = 35, 4, None, []  # safety boundary
                    return 0x10
                self.moves_accepted += 1
                if self.mode == 7 and self.mode7_replaces:
                    self.target, self.queue = (xyz, speed), []
                elif self.target is None and not self.queue:
                    self.target = (xyz, speed)
                else:
                    self.queue.append((xyz, speed))
            return 0x10 if (self.state in (4, 5) or not self.enabled) else 0

    def _answer(self, reg: int) -> bytes:
        f = lambda vals: struct.pack("<%df" % len(vals), *vals)  # noqa: E731
        if reg == 1:
            return b"6,6,XI1305,AC1300,v2.5.5,fake".ljust(40, b"\0")
        if reg == 2:
            return (b"XI130500A00L00\0AC130000B00L00\0").ljust(40, b"\0")
        if reg == 13:
            return bytes([self.state])
        if reg == 14:
            return struct.pack(">H", len(self.queue) + (1 if self.target is not None and self.mode != 7 else 0))
        if reg == 15:
            return bytes([self.err, self.warn])
        if reg == 41:
            return f(self.pose)
        if reg == 42:
            return f(JOINTS)
        if reg == 49:
            return bytes([1])
        if reg == 53:
            return (bytes([1]) + struct.pack(">6h", *BOUNDARY) + f([250.0, math.radians(60.0)])
                    + f([math.radians(v) for v in (-360, 360) * 7]) + bytes([1, 0]))  # fmt: skip
        return b""

    def _control(self, conn: socket.socket) -> None:
        conn.settimeout(0.2)
        buf = b""
        while self.alive:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            buf += data
            while len(buf) >= 6:
                trans, prot, length = struct.unpack(">HHH", buf[:6])
                if len(buf) < 6 + length:
                    break
                body, buf = buf[6:6 + length], buf[6 + length:]
                self.frames.append((prot, body[0], bytes(body[1:])))
                if self.silent:
                    continue
                extra = self._write(body[0], bytes(body[1:])) if len(body) > 1 else 0
                state = (0x40 if self.err else 0x20 if self.warn else 0) | extra
                ans = self._answer(body[0])
                try:
                    conn.sendall(struct.pack(">HHH", trans, prot, len(ans) + 2) + bytes([body[0], state]) + ans)
                except OSError:
                    return

    def rich_report(self, counter: int) -> bytes:
        f = lambda vals: struct.pack("<%df" % len(vals), *vals)  # noqa: E731
        joints = list(JOINTS)
        if self.move:
            joints[0] += math.radians(0.05) * counter
        b = bytearray(587)
        struct.pack_into(">I", b, 0, 587)
        b[4] = (self.mode << 4) | self.state
        struct.pack_into(">H", b, 5, len(self.queue))
        b[7:35], b[35:59] = f(joints), f(self.pose)
        b[87], b[88], b[89], b[90] = 0x3F, 0x3F, self.err, self.warn
        b[91:115], b[115:131] = f(TCP_OFFSET), f([0.82, 0.0, 0.0, 48.0])
        b[131], b[132] = 5, 1
        b[133:145] = f([0.0, 0.0, -1.0])
        b[145:151] = bytes([6, 6, 1, 1, 1, 1])
        b[181:201], b[201:221], b[221:229] = f([1000, 1, 50000, 0.1, 1000]), f([20, 0.01, 20, 0.0001, 3.14]), f([2.3, 2.7])
        struct.pack_into(">I", b, 284, counter)
        b[288:312] = f(WORLD_OFFSET)
        b[314], b[315], b[316] = 0, 1, 0
        b[494] = 0b00011
        b[495] = 1
        struct.pack_into(">6h", b, 496, *BOUNDARY)
        b[508:512], b[512:516] = f([250.0]), f([math.radians(60.0)])
        b[516:572] = f([math.radians(v) for v in (-360, 360) * 7])
        b[572], b[573] = 1, 0
        return bytes(b)

    def _reports(self, conn: socket.socket) -> None:
        n = 100
        while self.alive:
            try:
                conn.sendall(self.rich_report(n))
            except OSError:
                return
            n += 1
            time.sleep(self.period)
