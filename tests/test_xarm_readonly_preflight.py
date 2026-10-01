"""Tests for the read-only xArm inspector.

NOTHING HERE TOUCHES A ROBOT. InjectedSdkTests use a fake SDK object. LoopbackSdkTests run the
REAL installed SDK code (or its copied source, via XARM_SDK_SRC) against a SYNTHETIC controller
on 127.0.0.1 that records every frame it receives. A fake controller proves what this program
sends; it proves nothing about how a real controller behaves.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "tests" / "fixtures"))

import xarm_readonly_preflight as rp  # noqa: E402
from fake_xarm_controller import BOUNDARY, POSE, TCP_OFFSET, FakeController  # noqa: E402

CHANGING = ("motion_enable", "set_mode", "set_state", "emergency_stop", "clean_error", "clean_warn", "reset",
            "move_gohome", "set_position", "set_tcp_offset", "set_reduced_mode", "set_gripper_position", "set_tgpio_digital")  # fmt: skip


class FakeCmd:
    """Injected stand-in for UxbusCmdTcp: records what would reach the wire."""

    wire: list = []
    _protocol_identifier = 2

    def send_modbus_request(self, unit_id, pdu_data, pdu_len, prot_id=-1, t_id=None, debug=False):  # type: ignore[no-untyped-def]
        FakeCmd.wire.append((unit_id, pdu_len))
        return 1


class FakeCore:
    """Injected stand-in for the SDK core. Its connect() copies the installed SDK's side effect:
    it calls clean_warn() by itself when a warning is present."""

    def __init__(self) -> None:
        self.executed: list = []  # robot-changing methods that REALLY ran
        self.connected = self.reported = False
        self.warn = 0
        self.fail: set = set()
        self.block_connect = threading.Event()
        self.block_connect.set()
        self.raise_in: str | None = None
        self.arm_cmd = FakeCmd()
        self._keep_heart = True
        self._first_report_over = False
        self.position_offset, self.world_offset, self.tcp_load = [0] * 6, [0] * 6, [0, [0, 0, 0]]  # SDK defaults
        self.mode, self.collision_sensitivity = 0, 0
        self.disconnects = 0

    def _handle_report_data(self, data):  # type: ignore[no-untyped-def]
        self._first_report_over = True

    def _connect_report(self) -> None:
        self.executed.append("report_reconnect")

    def connect(self, **kw):  # type: ignore[no-untyped-def]
        self.block_connect.wait()
        self.connected = self.reported = True
        self.get_version()
        if self.warn:
            self.clean_warn()  # exactly what x3/base.py _check_version does

    def disconnect(self) -> None:
        self.disconnects += 1
        self.connected = False

    def _get(self, name: str, reg: int, value):  # type: ignore[no-untyped-def]
        if self.raise_in == name:
            raise RuntimeError("synthetic SDK exception")
        sent = self.arm_cmd.send_modbus_request(reg, 0, 0)
        if sent == -1 or name in self.fail:
            return 3, value  # like the SDK: a non-zero code together with a stale/default value
        return 0, value

    def get_version(self): return self._get("get_version", 1, "6,6,XI130500A00L00,AC130000B00L00,v2.5.5")  # noqa: E704
    def get_robot_sn(self): return self._get("get_robot_sn", 2, "XI130500A00L00")  # noqa: E704
    def get_state(self): return self._get("get_state", 13, 4)  # noqa: E704
    def get_cmdnum(self): return self._get("get_cmdnum", 14, 0)  # noqa: E704
    def get_err_warn_code(self): return self._get("get_err_warn_code", 15, [0, self.warn])  # noqa: E704
    def get_position(self): return self._get("get_position", 41, [201.5, 0, 140.5, 180.0, 0, 0])  # noqa: E704
    def get_servo_angle(self): return self._get("get_servo_angle", 42, [0.0] * 7)  # noqa: E704
    def get_reduced_mode(self): return self._get("get_reduced_mode", 49, 1)  # noqa: E704
    def get_reduced_states(self): return self._get("get_reduced_states", 53, [1, list(BOUNDARY), 250.0, 60.0, [], 1, 0])  # noqa: E704
    def version_is_ge(self, *a): return True  # noqa: E704
    def set_timeout(self, t): return t  # noqa: E704
    def get_gripper_position(self): self.executed.append("get_gripper_position"); return 0, 0  # noqa: E702,E704


def _changing(name: str):  # type: ignore[no-untyped-def]
    def method(self, *a, **k):  # type: ignore[no-untyped-def]
        self.executed.append(name)
        self.arm_cmd.send_modbus_request(12, [0], 1)
        return 0
    method.__name__ = name
    return method


for _n in CHANGING:
    setattr(FakeCore, _n, _changing(_n))


class FakeAPI:
    last: "FakeAPI | None" = None
    prepare = staticmethod(lambda core: None)

    def __init__(self, port, **kw):  # type: ignore[no-untyped-def]
        self.port, self.kwargs, self._arm = port, kw, FakeCore()
        FakeAPI.last = self
        FakeAPI.prepare(self._arm)

    def __getattr__(self, name: str):  # type: ignore[no-untyped-def]
        return getattr(self._arm, name)


def run_fake(prepare=lambda core: None, **kw):  # type: ignore[no-untyped-def]
    FakeCmd.wire = []
    FakeAPI.prepare = staticmethod(prepare)
    insp = rp.Inspector(FakeAPI, FakeCmd, "10.0.0.1", connect_timeout_s=kw.pop("connect_timeout_s", 2.0), report_wait_s=0.2,
                        samples=2, sample_interval_s=0.01, **kw)  # fmt: skip
    snap = insp.run()
    return snap, FakeAPI.last._arm, insp


class InjectedSdkTests(unittest.TestCase):
    def test_startup_reads_and_cleanup_change_nothing(self) -> None:
        snap, core, insp = run_fake()
        self.assertEqual(snap["result"], "ok")
        self.assertEqual(FakeAPI.last.kwargs["do_not_open"], True)  # construction separate from connection
        self.assertEqual({k: FakeAPI.last.kwargs[k] for k in ("timed_comm", "enable_heartbeat", "baud_checkset", "forbid_uds")},
                         {"timed_comm": False, "enable_heartbeat": False, "baud_checkset": False, "forbid_uds": True})  # fmt: skip
        self.assertEqual(core.executed, [])
        self.assertTrue(all(reg in rp.READ_REGISTERS and n == 0 for reg, n in FakeCmd.wire), FakeCmd.wire)
        self.assertEqual(core.disconnects, 1)
        self.assertEqual(snap["refused_attempts_total"], 0)
        self.assertEqual(set(snap["ledger"]["sdk_api_calls"]) - {"XArmAPI.__init__(do_not_open=True)"}, set(rp.ALLOWED_API) & set(snap["ledger"]["sdk_api_calls"]))
        self.assertIs(FakeCmd.send_modbus_request.__name__, "send_modbus_request")  # wire guard removed again

    def test_sdk_internal_clean_warn_at_connect_is_refused(self) -> None:
        snap, core, _ = run_fake(prepare=lambda c: setattr(c, "warn", 11))
        self.assertEqual(snap["result"], "ok")
        self.assertEqual(core.executed, [])  # the SDK's own clean_warn() never ran
        self.assertEqual(snap["ledger"]["api_calls_refused"], {"clean_warn": 1})
        self.assertEqual(snap["polled_series"][0]["get_err_warn_code"]["value"], [0, 11])  # reported, not cleared

    def test_every_changing_method_is_refused_at_api_and_wire_level(self) -> None:
        led = rp.Ledger()
        core = FakeCore()
        undo = rp.install_wire_guard(FakeCmd, led)
        try:
            FakeCmd.wire = []
            for n in CHANGING:  # unguarded instance: only the wire guard stands in the way
                getattr(core, n)()
            self.assertEqual(FakeCmd.wire, [])
            self.assertEqual(len(led.summary()["wire_frames_refused"]), len(CHANGING))
            core2 = FakeCore()
            refused = rp.install_api_guard(core2, led)
            self.assertTrue(set(CHANGING) <= set(refused))
            self.assertIn("get_gripper_position", refused)  # not on the allowlist, so refused although it is a read
            for n in CHANGING:
                self.assertEqual(getattr(core2, n)(), -99)
            self.assertEqual(core2.executed, [])
            core.arm_cmd.send_modbus_request(41, [1], 1)  # an allowed register with a payload is refused too
            core.arm_cmd.send_modbus_request(41, 0, 0, prot_id=0)  # standard Modbus is refused
            self.assertEqual(FakeCmd.wire, [])
        finally:
            undo()

    def test_nonzero_return_code_is_never_accepted_as_a_value(self) -> None:
        snap, core, _ = run_fake(prepare=lambda c: c.fail.update({"get_position", "get_reduced_states"}))
        p = snap["polled_series"][0]["get_position"]
        self.assertEqual((p["ok"], p["value"], p["code"]), (False, None, 3))  # the SDK's default pose is not reported
        self.assertNotIn("reduced_states_decoded", snap)
        self.assertEqual(core.executed, [])

    def test_exception_during_reads_only_disconnects(self) -> None:
        snap, core, _ = run_fake(prepare=lambda c: setattr(c, "raise_in", "get_state"))
        self.assertEqual(snap["polled_series"][0]["get_state"]["code"], -101)
        self.assertEqual((core.executed, core.disconnects), ([], 1))

    def test_connect_timeout_is_bounded_and_cleanup_only_closes(self) -> None:
        t0 = time.monotonic()
        snap, core, _ = run_fake(prepare=lambda c: c.block_connect.clear(), connect_timeout_s=0.3)
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertEqual((snap["result"], snap["connect_error"]), ("connect_failed", "timeout"))
        self.assertEqual((core.executed, core.disconnects), ([], 1))
        self.assertEqual(FakeCmd.wire, [])
        core.block_connect.set()

    def test_without_reports_cached_defaults_are_not_reported(self) -> None:
        snap, core, _ = run_fake()
        self.assertFalse(snap["report"]["verified_fresh"])
        self.assertEqual(snap["report"]["received"], 0)
        self.assertNotIn("last", snap["report"])  # no tcp offset / load / motor state from SDK defaults
        self.assertIn("NOT accepted", snap["report"]["note"])

    def test_report_reconnect_is_disabled(self) -> None:
        snap, core, insp = run_fake()
        core._connect_report()
        self.assertEqual(core.executed, [])
        self.assertIn("SDK report reconnect attempt refused", insp.ledger.summary()["notes"])

    def test_inspector_cannot_call_a_non_allowed_operation(self) -> None:
        insp = rp.Inspector(FakeAPI, FakeCmd, "10.0.0.1")
        insp.api = FakeAPI("10.0.0.1")
        for n in rp.NEVER:
            with self.assertRaises(RuntimeError):
                insp._call(n)
        self.assertFalse(set(rp.NEVER) & rp.ALLOWED_API)

    def test_address_handling(self) -> None:
        with self.assertRaises(ValueError):
            rp.Inspector(FakeAPI, FakeCmd, "/dev/ttyUSB0")  # would select the SDK's serial path
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "s.py"
            p.write_text('arm = XArmAPI("10.1.2.3", is_radian=False)\nb = XArmAPI("10.1.2.3")\n')
            self.assertEqual(rp.address_from_file(p), "10.1.2.3")
            p.write_text('XArmAPI("10.1.2.3"); XArmAPI("10.1.2.4")')
            with self.assertRaises(SystemExit):
                rp.address_from_file(p)
        self.assertEqual(rp.redact("ROBOT_IP: 10.1.2.3 other 192.168.9.9", "10.1.2.3"), "ROBOT_IP: <xarm-ip> other <ip>")

    def test_boundary_order_and_report_decoder(self) -> None:
        b = rp.decode_boundary([502, 160, 270, -300, 585, 270])
        self.assertEqual((b["x_mm"], b["y_mm"], b["z_mm"]), ([160, 502], [-300, 270], [270, 585]))
        ctl = FakeController(report=False)
        ctl.close()
        d = rp.decode_rich_report(ctl.rich_report(7))
        self.assertEqual((d["state"], d["mode"], d["cmd_num"], d["report_counter"], d["arm_axis"]), (4, 0, 0, 7, 6))
        self.assertEqual(d["tcp_offset_mm_deg"][:3], TCP_OFFSET[:3])
        self.assertAlmostEqual(d["pose_mm_deg"][3], 179.86, places=3)
        self.assertEqual((d["tcp_load_kg"], d["tcp_load_cog_mm"]), (0.82, [0.0, 0.0, 48.0]))
        self.assertEqual((d["reduced_tcp_boundary_raw"], d["reduced_mode_on"], d["fence_on"]), (BOUNDARY, 1, 1))
        self.assertEqual(d["motor_enable_bits"][:6], [1] * 6)
        self.assertNotIn("world_offset_mm_deg", rp.decode_rich_report(ctl.rich_report(7)[:245]))  # short frame: field absent

    def test_main_without_connect_never_loads_the_sdk(self) -> None:
        before = set(sys.modules)
        self.assertEqual(rp.main(["--address", "10.0.0.1"]), 0)
        self.assertFalse([m for m in set(sys.modules) - before if m.split(".")[0] == "xarm"])
        self.assertEqual(rp.main(["--address", "10.0.0.1", "--connect"]), 2)  # no human confirmation


def _sdk_available() -> bool:
    src = os.environ.get("XARM_SDK_SRC")
    if src and src not in sys.path:
        sys.path.insert(0, src)
    return importlib.util.find_spec("xarm") is not None


@unittest.skipUnless(_sdk_available(), "xArm SDK not importable here (set XARM_SDK_SRC to a copy of its source)")
class LoopbackSdkTests(unittest.TestCase):
    """REAL SDK code against a SYNTHETIC loopback controller. Ports are patched to loopback ones."""

    @staticmethod
    def remap(ctl: FakeController):  # type: ignore[no-untyped-def]
        """Send the SDK's fixed controller ports (502, 30002) to the fake's loopback ports. Returns an undo."""
        import socket
        orig = socket.socket.connect
        ports = {502: ctl.control_port, 30002: ctl.report_port}

        def connect(sock, addr):  # type: ignore[no-untyped-def]
            if isinstance(addr, tuple) and addr[0] == "127.0.0.1" and addr[1] in ports:
                addr = ("127.0.0.1", ports[addr[1]])
            return orig(sock, addr)

        socket.socket.connect = connect  # type: ignore[method-assign]
        return lambda: setattr(socket.socket, "connect", orig)

    def run_real(self, ctl: FakeController, **kw):  # type: ignore[no-untyped-def]
        api_class, cmd_class, _ = rp.load_installed_sdk()
        undo = self.remap(ctl)
        out, real = None, sys.stdout
        sys.stdout = open(os.devnull, "w")
        try:
            insp = rp.Inspector(api_class, cmd_class, "127.0.0.1", samples=3, sample_interval_s=0.05,
                                report_wait_s=kw.pop("report_wait_s", 2.0), **kw)  # fmt: skip
            out = insp.run()
        finally:
            sys.stdout.close()
            sys.stdout = real
            undo()
            time.sleep(0.1)
            ctl.close()
        return out

    def assert_reads_only(self, ctl: FakeController) -> None:
        self.assertTrue(ctl.frames)
        for prot, reg, payload in ctl.frames:
            self.assertEqual(prot, 2, ctl.frames)  # plain private protocol, no heartbeat protocol 3
            self.assertIn(reg, rp.READ_REGISTERS, ctl.frames)
            self.assertEqual(payload, b"")

    def test_normal_snapshot_sends_only_reads_and_decodes_configuration(self) -> None:
        ctl = FakeController()
        s = self.run_real(ctl)
        self.assertEqual(s["result"], "ok", s.get("exception") or s.get("connect_error"))
        self.assert_reads_only(ctl)
        self.assertEqual(s["refused_attempts_total"], 0, s["ledger"])
        self.assertEqual((s["firmware"]["version"], s["firmware"]["axis"]), ("2.5.5", 6))
        last = s["polled_series"][-1]
        self.assertEqual((last["get_state"]["value"], last["get_cmdnum"]["value"], last["get_err_warn_code"]["value"]), (4, 0, [0, 0]))
        self.assertAlmostEqual(last["get_position"]["value"][0], POSE[0], places=2)
        self.assertAlmostEqual(last["get_position"]["value"][3], 179.86, places=2)  # degrees
        rs = s["reduced_states_decoded"]
        self.assertEqual((rs["reduced_mode_on"], rs["tcp_boundary"]["x_mm"], rs["tcp_boundary"]["raw"]), (1, [160, 502], BOUNDARY))
        r = s["report"]
        self.assertTrue(r["verified_fresh"], r)
        self.assertGreater(r["counter_last"], r["counter_first"])
        self.assertEqual(r["last"]["tcp_offset_mm_deg"][:3], [0.0, 0.0, 172.0])
        self.assertEqual(r["sdk_properties_cross_check"]["tcp_offset"][:3], [0.0, 0.0, 172.0])  # decoder agrees with the SDK
        self.assertEqual(r["last"]["reduced_tcp_boundary_raw"], BOUNDARY)
        self.assertFalse(r["stationary_check"]["movement_observed"])

    def test_warning_present_real_sdk_would_clean_it_but_nothing_is_sent(self) -> None:
        ctl = FakeController(warn=11)
        s = self.run_real(ctl)
        self.assertEqual(s["result"], "ok", s)
        self.assert_reads_only(ctl)  # register 17 (CLEAN_WAR) never arrived
        self.assertEqual(s["polled_series"][-1]["get_err_warn_code"]["value"], [0, 11])
        self.assertEqual(s["report"]["last"]["warn_code"], 11)

    def test_sdk_connect_clean_warn_hazard_and_both_guards(self) -> None:
        """The installed SDK's connect() clears a warning by itself on the path where warn_code is
        queried during the version check (enable_report=False). Shown on the fake controller only."""
        api_class, cmd_class, _ = rp.load_installed_sdk()

        def connect(api_guard: bool, wire_guard: bool):  # type: ignore[no-untyped-def]
            ctl, led = FakeController(warn=11, report=False), rp.Ledger()
            undo_map = self.remap(ctl)
            undo_wire = rp.install_wire_guard(cmd_class, led) if wire_guard else (lambda: None)
            real, sys.stdout = sys.stdout, open(os.devnull, "w")
            try:
                api = api_class("127.0.0.1", do_not_open=True, enable_report=False, timed_comm=False, baud_checkset=False,
                                check_robot_sn=True, forbid_uds=True)  # fmt: skip
                if api_guard:
                    rp.install_api_guard(api._arm, led)
                api._arm.connect()
                api._arm.disconnect()
            finally:
                sys.stdout.close()
                sys.stdout = real
                undo_wire()
                undo_map()
                time.sleep(0.1)
                ctl.close()
            return {reg for _, reg, _ in ctl.frames}, led.summary()

        regs, _ = connect(api_guard=False, wire_guard=False)
        self.assertIn(17, regs)  # unguarded: CLEAN_WAR really is sent by connect()
        regs, led = connect(api_guard=False, wire_guard=True)
        self.assertNotIn(17, regs)
        self.assertEqual([w["register"] for w in led["wire_frames_refused"]], [17])
        regs, led = connect(api_guard=True, wire_guard=True)
        self.assertNotIn(17, regs)
        self.assertEqual(led["api_calls_refused"].get("clean_warn"), 1)
        self.assertEqual(led["wire_frames_refused"], [])  # stopped before it reached the wire layer
        self.assertTrue(regs <= set(rp.READ_REGISTERS))

    def test_error_present_is_reported_not_cleared(self) -> None:
        ctl = FakeController(err=22)
        s = self.run_real(ctl)
        self.assert_reads_only(ctl)
        self.assertEqual(s["polled_series"][-1]["get_err_warn_code"]["value"], [22, 0])

    def test_silent_controller_times_out_and_only_version_reads_were_sent(self) -> None:
        ctl = FakeController(silent=True)
        t0 = time.monotonic()
        s = self.run_real(ctl, connect_timeout_s=3.0)
        self.assertLess(time.monotonic() - t0, 8.0)
        self.assertEqual((s["result"], s["connect_error"]), ("connect_failed", "timeout"))
        self.assertEqual({reg for _, reg, _ in ctl.frames}, {1})
        self.assert_reads_only(ctl)

    def test_no_controller_fails_fast(self) -> None:
        ctl = FakeController()
        ctl.close()
        time.sleep(0.3)
        s = self.run_real(ctl, connect_timeout_s=4.0)
        self.assertEqual(s["result"], "connect_failed")
        self.assertEqual(ctl.frames, [])

    def test_without_report_socket_report_backed_fields_are_unavailable(self) -> None:
        ctl = FakeController(report=False)
        s = self.run_real(ctl, report_wait_s=0.5)
        self.assertEqual(s["result"], "ok")
        self.assert_reads_only(ctl)
        self.assertFalse(s["report"]["verified_fresh"])
        self.assertNotIn("last", s["report"])
        self.assertTrue(s["polled_series"][-1]["get_position"]["ok"])

    def test_movement_during_the_window_is_flagged(self) -> None:
        ctl = FakeController(move=True)
        s = self.run_real(ctl)
        self.assertTrue(s["report"]["stationary_check"]["movement_observed"])
        self.assert_reads_only(ctl)


if __name__ == "__main__":
    unittest.main()
