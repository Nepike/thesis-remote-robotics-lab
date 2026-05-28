#!/usr/bin/env python3
"""
RemoteLab test suite.

    python test.py                    # unit tests only (no roscore needed)
    python test.py --integration      # + integration via real server + WebSocket
    python test.py --port 8765        # choose integration server port
"""

# ---------------------------------------------------------------------------
# ROS / hardware stubs — before any project import
# ---------------------------------------------------------------------------
import sys
import types

def _stub(name: str) -> types.ModuleType:
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m

_rospy = _stub("rospy")
_rospy.init_node  = lambda *a, **kw: None
_rospy.loginfo    = lambda *a, **kw: None
_rospy.logwarn    = lambda *a, **kw: None
_rospy.Publisher  = lambda *a, **kw: types.SimpleNamespace(
    publish=lambda m: None, unregister=lambda: None, calls=[])
_rospy.Subscriber = lambda *a, **kw: types.SimpleNamespace(unregister=lambda: None)

_geo = _stub("geometry_msgs"); _geo_msg = _stub("geometry_msgs.msg"); _geo.msg = _geo_msg
class _Twist:
    class _Vec:
        def __init__(self): self.x = self.y = self.z = 0.0
    def __init__(self): self.linear = _Twist._Vec(); self.angular = _Twist._Vec()
_geo_msg.Twist = _Twist

_yy = _stub("msg_yy"); _yy_msg = _stub("msg_yy.msg"); _yy.msg = _yy_msg
class _YYCmd:
    def __init__(self): self.command = 0; self.arg = []; self.angle = []; self.da = []
_yy_msg.cmd = _YYCmd; _yy_msg.sens = type("sens", (), {})

_sa = _stub("serial_asyncio"); _sa.open_serial_connection = None

# ---------------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------------
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import argparse, asyncio, base64, json, subprocess, traceback, uuid

from BasicClasses import Command, Device
from manager import RemoteLabManager

_cli = argparse.ArgumentParser()
_cli.add_argument("--integration", action="store_true")
_cli.add_argument("--port", type=int, default=8765)
_args = _cli.parse_args()

# ---------------------------------------------------------------------------
# Reporter
# ---------------------------------------------------------------------------
_passed = _failed = _skipped = 0

def _ok(name):
    global _passed; _passed += 1; print(f"  [OK]   {name}")

def _fail(name, detail=""):
    global _failed; _failed += 1; print(f"  [FAIL] {name}")
    for line in detail.strip().splitlines(): print(f"         {line}")

def _skip(name, reason):
    global _skipped; _skipped += 1; print(f"  [SKIP] {name}  ({reason})")

def _section(title):
    print(f"\n{'='*60}\n  {title}\n{'='*60}")

def _assert(name, cond, detail=""):
    if cond: _ok(name)
    else:    _fail(name, detail or "assertion is False")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _make_device(name="r1", shared=False, driver="mock") -> Device:
    return Device(name=name, ip="", port=0, driver=driver,
                  ros_namespace=f"/{name}", baud_rate=None,
                  shared=shared, active=True)


class _MockDriver:
    def __init__(self, device):
        self._device   = device
        self.executed: list = []
    async def execute_command(self, command: Command):
        self.executed.append(command.name)
        dur = float(command.args.get("duration", 0.0))
        if dur > 0:
            try:   await asyncio.sleep(dur)
            finally: pass
    async def start_transports(self): return ()
    async def start_adapters(self):   return ()
    async def setup_telemetry(self):  pass
    async def teardown_telemetry(self): pass


class _SlowDriver(_MockDriver):
    """Blocks for 60 s — used to test interrupt."""
    async def execute_command(self, command: Command):
        self.executed.append(command.name)
        try:   await asyncio.sleep(60.0)
        finally: pass


async def _make_manager(device_names=("r1",), shared=False,
                        driver_cls=_MockDriver) -> RemoteLabManager:
    """
    RemoteLabManager with in-process mock drivers.
    All devices share the same driver class (override with driver_cls).
    Bypasses load_config / DeviceSupervisor.run — no real hardware needed.
    """
    mgr = RemoteLabManager()
    mgr._devices.clear()
    mgr._drivers.clear()

    for name in device_names:
        dev = _make_device(name=name, shared=shared)
        drv = driver_cls(dev)
        mgr._devices[name] = dev
        mgr._drivers[name] = drv
        mgr._supervisor.load_device(dev, drv)
        mgr._scheduler.register_device(name)          # BEFORE start() → worker created
        mgr._access_controller.register_device(name, shared=shared)

    mgr._supervisor.on_device_down = mgr._scheduler.pause_device
    mgr._supervisor.on_device_up   = mgr._scheduler.resume_device
    mgr._supervisor._running = True

    await mgr._scheduler.start()
    return mgr


# ===========================================================================
# RemoteLabManager unit tests (was failing)
# ===========================================================================
async def test_manager_unit():
    _section("RemoteLabManager (unit, mock drivers)")

    # ── 1. submit without acquire → PermissionError ───────────────────────────
    mgr = await _make_manager(["r1"], shared=False)
    drv1: _MockDriver = mgr._drivers["r1"]

    try:
        await mgr.submit_command("cli", ["r1"], "move", priority=5)
        _fail("submit without acquire: should raise PermissionError")
    except PermissionError:
        _ok("submit without acquire: PermissionError raised")

    # ── 2. submit after acquire ───────────────────────────────────────────────
    mgr.acquire_device("r1", "cli")
    cid = await mgr.submit_command("cli", ["r1"], "go", priority=5)
    _assert("submit after acquire: non-empty command_id",
            isinstance(cid, str) and len(cid) > 0)

    # ── 3. submit_command_wait ────────────────────────────────────────────────
    # FIX: release "cli" first — otherwise "waiter" can't acquire the exclusive device.
    mgr.release_device("r1", "cli")
    mgr.acquire_device("r1", "waiter")
    drv1.executed.clear()
    wait_done = asyncio.Event()

    async def do_wait():
        await mgr.submit_command_wait("waiter", ["r1"], "waitcmd", priority=5)
        wait_done.set()

    asyncio.create_task(do_wait())
    await asyncio.sleep(0.3)
    _assert("submit_command_wait: resolves after execution", wait_done.is_set())
    _assert("submit_command_wait: command actually ran",     "waitcmd" in drv1.executed)

    # ── 4. cancel_all_commands unblocks _command_waiters ─────────────────────
    ev = asyncio.Event()
    fake_id = str(uuid.uuid4())
    mgr._command_waiters[fake_id] = (ev, 1)
    mgr.cancel_all_commands()
    _assert("cancel_all_commands: waiter event set",     ev.is_set())
    _assert("cancel_all_commands: _command_waiters empty", len(mgr._command_waiters) == 0)

    await mgr._scheduler.shutdown()

    # ── 5. interrupt → on_command_complete still fires ────────────────────────
    # FIX: slow_dev must be registered BEFORE scheduler.start() so a worker exists.
    # _make_manager registers all devices before start(), so pass driver_cls=_SlowDriver.
    mgr_int = await _make_manager(["slow_dev"], shared=True, driver_cls=_SlowDriver)

    complete_calls: list = []
    async def on_complete(cmd: Command, dev: str):
        complete_calls.append(dev)
    mgr_int.on_command_complete = on_complete

    await mgr_int.submit_command("cli", ["slow_dev"], "long", priority=5)
    await asyncio.sleep(0.05)       # let worker pick it up
    mgr_int.interrupt_device("slow_dev")
    await asyncio.sleep(0.1)
    _assert("interrupt: on_command_complete fires after interrupt",
            "slow_dev" in complete_calls,
            "Bug: on_command_complete not called after interrupt_device()")

    await mgr_int._scheduler.shutdown()

    # ── 6. on_client_disconnect: releases locks ───────────────────────────────
    mgr2 = await _make_manager(["dev_x"], shared=False)
    mgr2.acquire_device("dev_x", "dc")
    _assert("before disconnect: dc owns dev_x",
            mgr2._access_controller.check_access("dev_x", "dc"))
    mgr2.on_client_disconnect("dc")
    _assert("after disconnect: dc lost ownership",
            not mgr2._access_controller.check_access("dev_x", "dc"))
    await mgr2._scheduler.shutdown()

    # ── 7. get_devices / get_driver ───────────────────────────────────────────
    mgr3 = await _make_manager(["r1", "r2"], shared=False)
    names = [d.name for d in mgr3.get_devices()]
    _assert("get_devices: r1 present", "r1" in names)
    _assert("get_devices: r2 present", "r2" in names)
    _assert("get_driver: known device",  mgr3.get_driver("r1") is not None)
    _assert("get_driver: unknown → None", mgr3.get_driver("no_such") is None)

    # ── 8. cancel_command ────────────────────────────────────────────────────
    # FIX: fresh manager so "canceller" can cleanly acquire r1 without leftovers.
    mgr4 = await _make_manager(["r1"], shared=False)
    drv4: _MockDriver = mgr4._drivers["r1"]
    mgr4.acquire_device("r1", "canceller")

    mgr4._scheduler.pause_device("r1")
    await asyncio.sleep(0.02)
    cid_cancel = await mgr4.submit_command("canceller", ["r1"], "to_cancel", priority=5)
    mgr4.cancel_command(cid_cancel)
    mgr4._scheduler.resume_device("r1")
    await asyncio.sleep(0.1)
    _assert("cancel_command: cancelled cmd not executed",
            "to_cancel" not in drv4.executed)

    await mgr4._scheduler.shutdown()
    await mgr3._scheduler.shutdown()


# ===========================================================================
# Integration
# ===========================================================================
async def test_integration():
    _section("Integration (full server + WebSocket)")

    if not _args.integration:
        _skip("Integration tests",
              "pass --integration flag (requires roscore running)")
        return

    try:
        import websockets as _ws
    except ImportError:
        _fail("Integration tests", "websockets not installed — pip install websockets")
        return

    import urllib.request

    PORT = _args.port

    TEST_DEVICES = [
        {"name": "shared_mock", "ip": "", "port": 0, "driver": "mock",
         "ros_namespace": None, "shared": True,  "active": True},
        {"name": "excl_mock",   "ip": "", "port": 0, "driver": "mock",
         "ros_namespace": None, "shared": False, "active": True},
    ]

    INTEG_USER = "test_integ_runner"
    INTEG_PASS = "integ_XYZ_secure_123"
    USER2      = "test_integ_u2"
    PASS2      = "integ_pass2_ABC"

    from network.auth import _hash_password

    users_path   = Path("users.json")
    devices_path = Path("devices.json")
    users_bak    = users_path.read_text()   if users_path.exists()   else "{}"
    devices_bak  = devices_path.read_text() if devices_path.exists() else "[]"

    users_data = json.loads(users_bak)
    users_data[INTEG_USER] = _hash_password(INTEG_PASS)
    users_data[USER2]      = _hash_password(PASS2)
    users_path.write_text(json.dumps(users_data, indent=2))
    devices_path.write_text(json.dumps(TEST_DEVICES, indent=2))

    server_proc = None
    try:
        server_proc = subprocess.Popen(
            [sys.executable, "network/server.py", "--port", str(PORT)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )

        health_url = f"http://localhost:{PORT}/health"
        ready = False
        for _ in range(40):
            await asyncio.sleep(0.5)
            if server_proc.poll() is not None:
                out = server_proc.stdout.read().decode(errors="replace")
                _fail("Server startup", f"process exited early:\n{out[-3000:]}")
                return
            try:
                with urllib.request.urlopen(health_url, timeout=1) as r:
                    if r.status == 200: ready = True; break
            except Exception:
                pass

        if not ready:
            out = server_proc.stdout.read().decode(errors="replace")
            _fail("Server startup", f"no /health in 20 s:\n{out[-3000:]}")
            return
        _ok(f"Server started — /health OK  (port {PORT})")

        ws_url  = f"ws://localhost:{PORT}/ws"
        ok_hdr  = {"Authorization": "Basic " + base64.b64encode(
                       f"{INTEG_USER}:{INTEG_PASS}".encode()).decode()}
        ok2_hdr = {"Authorization": "Basic " + base64.b64encode(
                       f"{USER2}:{PASS2}".encode()).decode()}
        bad_hdr = {"Authorization": "Basic " + base64.b64encode(
                       b"nobody:wrongpass").decode()}

        # ── invalid credentials ───────────────────────────────────────────────
        rejected = False
        try:
            async with _ws.connect(ws_url, extra_headers=bad_hdr): pass
        except Exception:
            rejected = True
        _assert("Auth: invalid credentials → rejected", rejected)

        # ── UNKNOWN_MESSAGE ───────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send('{"type":"totally_unknown_type_xyz"}')
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("UNKNOWN_MESSAGE: error type", msg["type"] == "error")
            _assert("UNKNOWN_MESSAGE: error code", msg["code"] == "UNKNOWN_MESSAGE")

        # ── malformed JSON ────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send("not valid json {{{{")
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Malformed JSON: error returned", msg["type"] == "error")

        # ── get_devices ───────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send('{"type":"get_devices"}')
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("get_devices: type",           msg["type"] == "devices")
            names = [d["name"] for d in msg["data"]]
            _assert("get_devices: shared_mock",    "shared_mock" in names)
            _assert("get_devices: excl_mock",      "excl_mock"   in names)
            _assert("get_devices: shared flag",
                    next(d for d in msg["data"] if d["name"] == "shared_mock")["shared"] is True)
            _assert("get_devices: excl flag",
                    next(d for d in msg["data"] if d["name"] == "excl_mock")["shared"] is False)

        # ── shared: submit without acquire ────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "submit", "devices": ["shared_mock"],
                                      "name": "test_cmd", "priority": 5, "args": {}}))
            ack  = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Shared submit: ack",         ack["type"] == "ack")
            _assert("Shared submit: status",      ack.get("status") == "queued")
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Shared submit: done",        done["type"] == "done")
            _assert("Shared submit: device",      done["device"] == "shared_mock")
            _assert("Shared submit: matching id", done["command_id"] == ack["command_id"])

        # ── exclusive: submit without acquire → ACCESS_DENIED ─────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "submit", "devices": ["excl_mock"],
                                      "name": "x", "priority": 5, "args": {}}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Excl without acquire: ACCESS_DENIED",
                    err["type"] == "error" and err["code"] == "ACCESS_DENIED")

        # ── exclusive: acquire + submit + done + release ──────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)

            await ws.send(json.dumps({"type": "submit", "devices": ["excl_mock"],
                                      "name": "go", "priority": 5, "args": {}}))
            ack  = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Excl: ack",    ack["type"] == "ack")
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Excl: done",   done["type"] == "done")
            _assert("Excl: device", done["device"] == "excl_mock")
            _assert("Excl: id",     done["command_id"] == ack["command_id"])

            await ws.send(json.dumps({"type": "release", "device": "excl_mock"}))
            await asyncio.sleep(0.1)
            _ok("Excl: release sent")

        # ── ALREADY_OWNED ─────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws1:
            await ws1.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.2)
            async with _ws.connect(ws_url, extra_headers=ok2_hdr) as ws2:
                await ws2.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
                err = json.loads(await asyncio.wait_for(ws2.recv(), timeout=3.0))
                _assert("ALREADY_OWNED: error",
                        err["type"] == "error" and err["code"] == "ALREADY_OWNED")

        # ── UNKNOWN_DEVICE ────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "submit", "devices": ["no_such_device"],
                                      "name": "x", "priority": 5, "args": {}}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("UNKNOWN_DEVICE", err["code"] == "UNKNOWN_DEVICE")

        # ── cancel queued command ─────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)

            # slow cmd keeps worker busy
            await ws.send(json.dumps({"type": "submit", "devices": ["excl_mock"],
                                      "name": "slow", "priority": 5,
                                      "args": {"duration": 2.0}}))
            ack1 = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))

            # second cmd: enqueue then cancel immediately
            await ws.send(json.dumps({"type": "submit", "devices": ["excl_mock"],
                                      "name": "cancel_me", "priority": 3, "args": {}}))
            ack2 = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            await ws.send(json.dumps({"type": "cancel",
                                      "command_id": ack2["command_id"]}))

            done1 = json.loads(await asyncio.wait_for(ws.recv(), timeout=10.0))
            _assert("Cancel queued: first cmd completes", done1["type"] == "done")
            try:
                extra = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.5))
                _fail("Cancel queued: unexpected second done", str(extra))
            except asyncio.TimeoutError:
                _ok("Cancel queued: second cmd skipped (no extra done)")

        # ── interrupt executing command ───────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)

            await ws.send(json.dumps({"type": "submit", "devices": ["excl_mock"],
                                      "name": "very_long", "priority": 5,
                                      "args": {"duration": 30.0}}))
            ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            await asyncio.sleep(0.1)   # let it start

            await ws.send(json.dumps({"type": "interrupt", "device": "excl_mock"}))

            # done MUST arrive — this tests the try/finally fix in manager.py
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Interrupt: done received after interrupt",
                    done["type"] == "done",
                    "Bug: on_command_complete not called after interrupt_device()")
            _assert("Interrupt: matching id", done["command_id"] == ack["command_id"])

        # ── subscribe_telemetry / unsubscribe ─────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "subscribe_telemetry",
                                      "device": "shared_mock"}))
            await asyncio.sleep(0.2)
            await ws.send(json.dumps({"type": "unsubscribe_telemetry",
                                      "device": "shared_mock"}))
            _ok("Telemetry: subscribe/unsubscribe without crash")

        # ── subscribe unknown device → UNKNOWN_DEVICE ─────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "subscribe_telemetry",
                                      "device": "ghost_device"}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Subscribe unknown device: UNKNOWN_DEVICE",
                    err["type"] == "error" and err["code"] == "UNKNOWN_DEVICE")

        # ── run_procedure: stop_all ───────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "run_procedure",
                                      "name": "stop_all", "args": {}}))
            pa = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("stop_all: procedure_ack type",  pa["type"] == "procedure_ack")
            _assert("stop_all: has procedure_id",    "procedure_id" in pa)
            pd = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("stop_all: procedure_done type", pd["type"] == "procedure_done")
            _assert("stop_all: matching id",         pd["procedure_id"] == pa["procedure_id"])

        # ── unknown procedure → UNKNOWN_PROCEDURE ─────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "run_procedure",
                                      "name": "no_such_procedure", "args": {}}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Unknown procedure: error type",  err["type"] == "error")
            _assert("Unknown procedure: UNKNOWN_PROCEDURE", err["code"] == "UNKNOWN_PROCEDURE")

        # ── cancel_procedure with bogus id: no crash ──────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "cancel_procedure",
                                      "procedure_id": "bogus_id_xyz"}))
            await asyncio.sleep(0.2)
            _ok("cancel_procedure bogus id: no crash")

        # ── second connection evicts first ────────────────────────────────────
        ws1 = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws1.send('{"type":"get_devices"}')
        await asyncio.wait_for(ws1.recv(), timeout=3.0)

        ws2 = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws2.send('{"type":"get_devices"}')
        msg2 = json.loads(await asyncio.wait_for(ws2.recv(), timeout=3.0))
        _assert("Second connection works", msg2["type"] == "devices")

        evicted = False
        try:
            await asyncio.wait_for(ws1.recv(), timeout=1.0)
        except Exception:
            evicted = True
        _assert("First connection evicted", evicted)
        await ws1.close(); await ws2.close()

        # ── disconnect releases exclusive lock ────────────────────────────────
        ws_tmp = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws_tmp.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
        await asyncio.sleep(0.15)
        await ws_tmp.close()
        await asyncio.sleep(0.2)

        async with _ws.connect(ws_url, extra_headers=ok2_hdr) as ws_after:
            await ws_after.send(json.dumps({"type": "acquire",
                                            "device": "excl_mock"}))
            await asyncio.sleep(0.15)
            await ws_after.send(json.dumps({"type": "submit",
                                            "devices": ["excl_mock"],
                                            "name": "post_disconnect",
                                            "priority": 5, "args": {}}))
            msg = json.loads(await asyncio.wait_for(ws_after.recv(), timeout=3.0))
            _assert("Disconnect releases lock: other client can acquire",
                    msg["type"] == "ack", f"expected ack, got {msg}")

    except Exception:
        _fail("Integration", traceback.format_exc())

    finally:
        if server_proc and server_proc.poll() is None:
            server_proc.terminate()
            try:   server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired: server_proc.kill()

        if server_proc and server_proc.stdout:
            try:
                out = server_proc.stdout.read().decode(errors="replace")
                if out.strip():
                    _section("Server output (last 4000 chars)")
                    print(out[-4000:])
            except Exception:
                pass

        try:
            users_path.write_text(users_bak)
            devices_path.write_text(devices_bak)
        except Exception:
            pass


# ===========================================================================
# Main
# ===========================================================================
async def _run_all():
    await test_manager_unit()
    await test_integration()


try:
    asyncio.run(_run_all())
except Exception:
    _fail("async runner crashed", traceback.format_exc())

_section("Summary")
total = _passed + _failed + _skipped
print(f"  Passed:  {_passed}/{total}")
print(f"  Failed:  {_failed}/{total}")
print(f"  Skipped: {_skipped}/{total}")
print()
sys.exit(1 if _failed else 0)
