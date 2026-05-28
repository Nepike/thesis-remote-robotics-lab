#!/usr/bin/env python3
"""
RemoteLab comprehensive test suite.

    python test.py                    # unit tests only (no roscore needed)
    python test.py --integration      # + integration via real server + WebSocket
    python test.py --port 8765        # choose integration server port

Unit tests cover (no hardware / no ROS required):
    BasicClasses, AccessController, CommandScheduler,
    ProcedureManager, RemoteLabManager, wire protocol models.

Integration tests cover:
    Full HTTP+WebSocket server started as subprocess with mock devices.
    Auth, device listing, acquire/submit/done/release, cancel, interrupt,
    telemetry, procedures, error codes, connection eviction.
"""

# ---------------------------------------------------------------------------
# ROS / hardware stubs — MUST be installed before any project import that
# does `import rospy` / `import geometry_msgs.msg` at module level.
# ---------------------------------------------------------------------------
import sys
import types

def _stub(name: str) -> types.ModuleType:
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m

# rospy stub
_rospy = _stub("rospy")
_rospy.init_node  = lambda *a, **kw: None
_rospy.loginfo    = lambda *a, **kw: None
_rospy.logwarn    = lambda *a, **kw: None
_rospy.Publisher  = lambda *a, **kw: types.SimpleNamespace(
    publish=lambda m: None, unregister=lambda: None, calls=[])
_rospy.Subscriber = lambda *a, **kw: types.SimpleNamespace(unregister=lambda: None)

# geometry_msgs stub
_geo     = _stub("geometry_msgs")
_geo_msg = _stub("geometry_msgs.msg")
_geo.msg = _geo_msg

class _Twist:
    class _Vec:
        def __init__(self): self.x = self.y = self.z = 0.0
    def __init__(self):
        self.linear  = _Twist._Vec()
        self.angular = _Twist._Vec()

_geo_msg.Twist = _Twist

# msg_yy stub
_yy     = _stub("msg_yy")
_yy_msg = _stub("msg_yy.msg")
_yy.msg = _yy_msg

class _YYCmd:
    def __init__(self): self.command = 0; self.arg = []; self.angle = []; self.da = []

_yy_msg.cmd  = _YYCmd
_yy_msg.sens = type("sens", (), {})

# serial_asyncio stub (not actually called in unit tests)
_serial_asyncio = _stub("serial_asyncio")
_serial_asyncio.open_serial_connection = None

# ---------------------------------------------------------------------------
# Project path and imports
# ---------------------------------------------------------------------------
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import argparse
import asyncio
import base64
import json
import subprocess
import traceback
import uuid

from AccessController import AccessController
from BasicClasses import Command, Device
from CommandScheduler import CommandScheduler
from Logger import Logger
from Procedures import AbstractProcedure, AllGoHome, ProcedureManager, StopAll
from manager import RemoteLabManager
from network.models import (
    AckMessage, AcquireMessage, CancelMessage, CancelProcedureMessage,
    DevicesMessage, DoneMessage, ErrorMessage, GetDevicesMessage, IncomingMessage,
    InterruptMessage, ProcedureAckMessage, ProcedureDoneMessage, ProcedureErrorMessage,
    ReleaseMessage, RunProcedureMessage, SubmitMessage, SubscribeTelemetryMessage,
    TelemetryMessage, UnsubscribeTelemetryMessage,
)
from pydantic import TypeAdapter, ValidationError

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
_cli = argparse.ArgumentParser(description="RemoteLab test suite")
_cli.add_argument("--integration", action="store_true",
                  help="Run integration tests (requires roscore running)")
_cli.add_argument("--port", type=int, default=8765,
                  help="Port for the integration test server (default: 8765)")
_args = _cli.parse_args()

# ---------------------------------------------------------------------------
# Reporter
# ---------------------------------------------------------------------------
_passed = _failed = _skipped = 0


def _ok(name: str) -> None:
    global _passed
    _passed += 1
    print(f"  [OK]   {name}")


def _fail(name: str, detail: str = "") -> None:
    global _failed
    _failed += 1
    print(f"  [FAIL] {name}")
    for line in detail.strip().splitlines():
        print(f"         {line}")


def _skip(name: str, reason: str) -> None:
    global _skipped
    _skipped += 1
    print(f"  [SKIP] {name}  ({reason})")


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print("=" * 60)


def _assert(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        _ok(name)
    else:
        _fail(name, detail or "assertion is False")


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def _make_device(name: str = "r1", shared: bool = False,
                 driver: str = "mock") -> Device:
    return Device(
        name=name, ip="", port=0, driver=driver,
        ros_namespace=f"/{name}", baud_rate=None,
        shared=shared, active=True,
    )


class _MockDriver:
    """
    In-process mock driver used by unit tests.
    Tracks every execute_command() call; supports timed commands.
    """
    def __init__(self, device: Device):
        self._device    = device
        self.executed:  list = []   # list of command.name strings
        self.call_args: list = []   # list of command.args dicts

    async def execute_command(self, command: Command):
        self.executed.append(command.name)
        self.call_args.append(dict(command.args))
        dur = float(command.args.get("duration", 0.0))
        if dur > 0:
            try:
                await asyncio.sleep(dur)
            finally:
                pass  # nothing physical to stop in mock

    async def start_transports(self): return ()
    async def start_adapters(self):   return ()
    async def setup_telemetry(self):  pass
    async def teardown_telemetry(self): pass


async def _make_manager(device_names=("r1",), shared: bool = False) -> RemoteLabManager:
    """
    Build a RemoteLabManager with mock drivers, bypassing load_config() and
    DeviceSupervisor.run() so no real hardware or roscore is needed.
    """
    mgr = RemoteLabManager()          # RosInterface() → rospy.init_node → our stub
    mgr._devices.clear()
    mgr._drivers.clear()

    for name in device_names:
        dev = _make_device(name=name, shared=shared)
        drv = _MockDriver(dev)
        mgr._devices[name] = dev
        mgr._drivers[name] = drv
        mgr._supervisor.load_device(dev, drv)
        mgr._scheduler.register_device(name)
        mgr._access_controller.register_device(name, shared=shared)

    mgr._supervisor.on_device_down = mgr._scheduler.pause_device
    mgr._supervisor.on_device_up   = mgr._scheduler.resume_device
    mgr._supervisor._running = True   # skip re-entry guard in supervisor.run()

    await mgr._scheduler.start()
    return mgr


# ===========================================================================
# SECTION 1 — BasicClasses
# ===========================================================================
async def test_basic_classes():
    _section("BasicClasses")

    q: asyncio.PriorityQueue = asyncio.PriorityQueue()

    c_low  = Command(priority=1, client_id="c", devices=["d"], name="low")
    c_high = Command(priority=9, client_id="c", devices=["d"], name="high")
    c_med  = Command(priority=5, client_id="c", devices=["d"], name="med")
    await q.put(c_low);  await q.put(c_high);  await q.put(c_med)

    first  = await q.get()
    second = await q.get()
    third  = await q.get()
    _assert("Priority: highest number first",   first.name  == "high", f"got {first.name}")
    _assert("Priority: middle second",          second.name == "med")
    _assert("Priority: lowest last",            third.name  == "low")

    # FIFO at equal priority — earlier timestamp must go first
    await asyncio.sleep(0.002)
    c_a = Command(priority=5, client_id="c", devices=["d"], name="A")
    await asyncio.sleep(0.002)
    c_b = Command(priority=5, client_id="c", devices=["d"], name="B")
    await q.put(c_b); await q.put(c_a)   # enqueue in reverse order
    _assert("FIFO: earlier timestamp first at equal priority",
            (await q.get()).name == "A")

    # Equality and hashing based on command_id
    c1 = Command(priority=1, client_id="c", devices=["d"], name="x")
    c2 = Command(priority=1, client_id="c", devices=["d"], name="x")
    _assert("Equality: self == self",              c1 == c1)
    _assert("Equality: different ids not equal",   c1 != c2)
    _assert("Hash: consistent for same object",    hash(c1) == hash(c1))
    _assert("Hash: different for different ids",   hash(c1) != hash(c2))

    # Device is a frozen dataclass
    dev = _make_device()
    try:
        dev.name = "changed"          # type: ignore
        _fail("Device: frozen dataclass allows mutation")
    except (AttributeError, TypeError):
        _ok("Device: frozen (immutable)")

    # Command auto-generates unique ids
    ca = Command(priority=1, client_id="c", devices=["d"])
    cb = Command(priority=1, client_id="c", devices=["d"])
    _assert("Command: auto-generated ids are unique", ca.command_id != cb.command_id)
    _assert("Command: ids are non-empty strings",
            isinstance(ca.command_id, str) and len(ca.command_id) > 0)


# ===========================================================================
# SECTION 2 — AccessController
# ===========================================================================
async def test_access_controller():
    _section("AccessController")

    ac = AccessController()
    ac.register_device("shared",    shared=True)
    ac.register_device("excl",      shared=False)
    ac.register_device("excl2",     shared=False)

    # ── Shared device ────────────────────────────────────────────────────────
    _assert("Shared: acquire always True",            ac.acquire("shared", "c1"))
    _assert("Shared: check_access any client True",   ac.check_access("shared", "anyone"))
    _assert("Shared: get_owner is None",              ac.get_owner("shared") is None)

    # ── Exclusive: acquire free device ───────────────────────────────────────
    _assert("Excl: acquire free → True",              ac.acquire("excl", "c1"))
    _assert("Excl: check_access for owner → True",    ac.check_access("excl", "c1"))
    _assert("Excl: check_access for stranger → False",not ac.check_access("excl", "c2"))
    _assert("Excl: get_owner returns owner",          ac.get_owner("excl") == "c1")

    # ── Exclusive: idempotent re-acquire ─────────────────────────────────────
    _assert("Excl: re-acquire by same client → True", ac.acquire("excl", "c1"))

    # ── Exclusive: acquire blocked by another client ──────────────────────────
    _assert("Excl: acquire by other → False",         not ac.acquire("excl", "c2"))

    # ── Exclusive: release and reacquire ─────────────────────────────────────
    ac.release("excl", "c1")
    _assert("After release: owner check_access → False",  not ac.check_access("excl", "c1"))
    _assert("After release: get_owner is None",           ac.get_owner("excl") is None)
    _assert("After release: new client can acquire",      ac.acquire("excl", "c2"))

    # ── Non-owner release is a no-op ─────────────────────────────────────────
    ac.release("excl", "c1")  # c1 does not own — should be no-op
    _assert("Non-owner release: c2 still owns",           ac.check_access("excl", "c2"))

    # ── release_all ──────────────────────────────────────────────────────────
    ac.acquire("excl2", "c2")
    ac.release_all("c2")
    _assert("release_all: excl released",                 not ac.check_access("excl", "c2"))
    _assert("release_all: excl2 released",                not ac.check_access("excl2", "c2"))

    # ── register idempotency ──────────────────────────────────────────────────
    ac.register_device("excl", shared=False)  # re-register same device
    ac.acquire("excl", "c3")
    _assert("register idempotent: still exclusive",       not ac.acquire("excl", "c4"))


# ===========================================================================
# SECTION 3 — CommandScheduler
# ===========================================================================
async def test_command_scheduler():
    _section("CommandScheduler")

    # ── Basic execution ───────────────────────────────────────────────────────
    calls: list = []

    async def execute(cmd: Command, dev: str):
        calls.append((cmd.name, dev))
        dur = float(cmd.args.get("duration", 0.0))
        if dur > 0:
            try:   await asyncio.sleep(dur)
            finally: pass

    sched = CommandScheduler(execute)
    sched.register_device("d1")
    sched.register_device("d2")
    await sched.start()

    calls.clear()
    await sched.submit("cli", ["d1"], "hello", priority=5)
    await asyncio.sleep(0.1)
    _assert("Basic: command executed",        ("hello", "d1") in calls)
    _assert("Basic: not on other device",     ("hello", "d2") not in calls)

    # ── Priority ordering ─────────────────────────────────────────────────────
    order: list = []

    async def track_order(cmd: Command, dev: str):
        order.append(cmd.name)

    sched_prio = CommandScheduler(track_order)
    sched_prio.register_device("p1")
    await sched_prio.start()

    sched_prio.pause_device("p1")
    await asyncio.sleep(0.02)
    await sched_prio.submit("cli", ["p1"], "low",  priority=1)
    await sched_prio.submit("cli", ["p1"], "high", priority=9)
    await sched_prio.submit("cli", ["p1"], "med",  priority=5)
    sched_prio.resume_device("p1")
    await asyncio.sleep(0.15)

    _assert("Priority: high executed first", order[0] == "high" if order else False,
            f"order={order}")
    _assert("Priority: med second",          order[1] == "med"  if len(order) > 1 else False)
    _assert("Priority: low last",            order[2] == "low"  if len(order) > 2 else False)
    await sched_prio.shutdown()

    # ── Pause / Resume ────────────────────────────────────────────────────────
    calls.clear()
    sched.pause_device("d1")
    await asyncio.sleep(0.02)
    await sched.submit("cli", ["d1"], "while_paused", priority=5)
    await asyncio.sleep(0.1)
    _assert("Pause: cmd not executed while paused", ("while_paused", "d1") not in calls)
    sched.resume_device("d1")
    await asyncio.sleep(0.1)
    _assert("Resume: cmd executed after resume",    ("while_paused", "d1") in calls)

    # ── Cancel single command ─────────────────────────────────────────────────
    calls.clear()
    sched.pause_device("d1")
    await asyncio.sleep(0.02)
    cid = await sched.submit("cli", ["d1"], "cancel_me", priority=5)
    sched.cancel(cid)
    sched.resume_device("d1")
    await asyncio.sleep(0.1)
    _assert("Cancel: cancelled cmd not executed",   ("cancel_me", "d1") not in calls)

    # ── cancel_client_commands ────────────────────────────────────────────────
    calls.clear()
    sched.pause_device("d1")
    await asyncio.sleep(0.02)
    await sched.submit("client_x", ["d1"], "cmd_for_x", priority=5)
    sched.cancel_client_commands("client_x")
    sched.resume_device("d1")
    await asyncio.sleep(0.1)
    _assert("cancel_client: all client cmds skipped", ("cmd_for_x", "d1") not in calls)

    # ── Interrupt currently executing command ─────────────────────────────────
    interrupted = asyncio.Event()

    async def slow_cb(cmd: Command, dev: str):
        try:
            await asyncio.sleep(60.0)
        except asyncio.CancelledError:
            interrupted.set()
            raise

    sched_int = CommandScheduler(slow_cb)
    sched_int.register_device("int_dev")
    await sched_int.start()

    await sched_int.submit("cli", ["int_dev"], "long", priority=5)
    await asyncio.sleep(0.05)
    sched_int.interrupt_device("int_dev")
    await asyncio.sleep(0.1)
    _assert("Interrupt: CancelledError raised inside execute", interrupted.is_set())
    await sched_int.shutdown()

    # ── interrupt_all_devices ─────────────────────────────────────────────────
    cnt = {"n": 0}

    async def slow2(cmd: Command, dev: str):
        try:
            await asyncio.sleep(60.0)
        except asyncio.CancelledError:
            cnt["n"] += 1
            raise

    sched_ia = CommandScheduler(slow2)
    sched_ia.register_device("ia1")
    sched_ia.register_device("ia2")
    await sched_ia.start()
    await sched_ia.submit("cli", ["ia1"], "s1", priority=5)
    await sched_ia.submit("cli", ["ia2"], "s2", priority=5)
    await asyncio.sleep(0.05)
    sched_ia.interrupt_all_devices()
    await asyncio.sleep(0.1)
    _assert("interrupt_all: both commands interrupted", cnt["n"] == 2, f"n={cnt['n']}")
    await sched_ia.shutdown()

    # ── Multi-device command ──────────────────────────────────────────────────
    multi_calls: list = []

    async def multi_cb(cmd: Command, dev: str):
        multi_calls.append(dev)

    sched_m = CommandScheduler(multi_cb)
    sched_m.register_device("m1")
    sched_m.register_device("m2")
    await sched_m.start()
    await sched_m.submit("cli", ["m1", "m2"], "sync", priority=5)
    await asyncio.sleep(0.15)
    _assert("Multi-device: executed on m1", "m1" in multi_calls)
    _assert("Multi-device: executed on m2", "m2" in multi_calls)
    await sched_m.shutdown()

    # ── cancel_all_pending ────────────────────────────────────────────────────
    noop_calls: list = []

    async def noop(cmd: Command, dev: str):
        noop_calls.append(cmd.name)

    sched_cap = CommandScheduler(noop)
    sched_cap.register_device("cap")
    await sched_cap.start()
    sched_cap.pause_device("cap")
    await asyncio.sleep(0.02)
    await sched_cap.submit("cli", ["cap"], "p1", priority=1)
    await sched_cap.submit("cli", ["cap"], "p2", priority=2)
    sched_cap.cancel_all_pending()
    sched_cap.resume_device("cap")
    await asyncio.sleep(0.15)
    _assert("cancel_all_pending: no cmds executed", noop_calls == [])
    await sched_cap.shutdown()

    # ── Clean shutdown ────────────────────────────────────────────────────────
    sched_shut = CommandScheduler(execute)
    sched_shut.register_device("sd")
    await sched_shut.start()
    await sched_shut.submit("cli", ["sd"], "before_shut", priority=5)
    await asyncio.sleep(0.05)
    await sched_shut.shutdown()
    _assert("Shutdown: workers dict cleared", len(sched_shut._workers) == 0)

    # ── _pending_counts cleanup after multi-device completion ─────────────────
    sched_pc = CommandScheduler(execute)
    sched_pc.register_device("pc1")
    sched_pc.register_device("pc2")
    await sched_pc.start()
    cid_m = await sched_pc.submit("cli", ["pc1", "pc2"], "track", priority=5)
    await asyncio.sleep(0.2)
    _assert("Pending counts: cleaned up after all devices done",
            cid_m not in sched_pc._pending_counts)
    await sched_pc.shutdown()

    await sched.shutdown()


# ===========================================================================
# SECTION 4 — ProcedureManager
# ===========================================================================
async def test_procedure_manager():
    _section("ProcedureManager")

    class ImmediateProcedure(AbstractProcedure):
        name = "immediate"
        async def run(self, manager, client_id, args):
            pass  # completes instantly

    class ErrorProcedure(AbstractProcedure):
        name = "fail_proc"
        async def run(self, manager, client_id, args):
            raise RuntimeError("intentional failure")

    class SlowProcedure(AbstractProcedure):
        name = "slow_proc"
        async def run(self, manager, client_id, args):
            await asyncio.sleep(60)

    mgr = await _make_manager()
    pm  = mgr._procedure_manager
    pm.register(ImmediateProcedure())
    pm.register(ErrorProcedure())
    pm.register(SlowProcedure())

    done_evts:  dict = {}
    error_evts: dict = {}

    async def on_done(proc_id, client_id):
        if proc_id in done_evts:
            done_evts[proc_id].set()

    async def on_error(proc_id, client_id, msg):
        if proc_id in error_evts:
            error_evts[proc_id].set()

    pm.on_done  = on_done
    pm.on_error = on_error

    # ── run → on_done ─────────────────────────────────────────────────────────
    pid1 = await pm.run("immediate", "client1", {})
    done_evts[pid1] = asyncio.Event()
    await asyncio.wait_for(done_evts[pid1].wait(), timeout=2.0)
    _assert("on_done: fires after completion",      done_evts[pid1].is_set())
    _assert("on_done: proc removed from _running",  pid1 not in pm._running)

    # ── run → on_error ────────────────────────────────────────────────────────
    pid2 = await pm.run("fail_proc", "client1", {})
    error_evts[pid2] = asyncio.Event()
    await asyncio.wait_for(error_evts[pid2].wait(), timeout=2.0)
    _assert("on_error: fires on exception",         error_evts[pid2].is_set())
    _assert("on_error: proc removed from _running", pid2 not in pm._running)

    # ── cancel single procedure ───────────────────────────────────────────────
    pid3 = await pm.run("slow_proc", "client1", {})
    await asyncio.sleep(0.05)
    _assert("cancel: proc is running before cancel", pid3 in pm._running)
    pm.cancel(pid3)
    await asyncio.sleep(0.1)
    _assert("cancel: proc removed after cancel",     pid3 not in pm._running)

    # ── cancel_all_for_client ─────────────────────────────────────────────────
    pid4 = await pm.run("slow_proc", "clientX", {})
    pid5 = await pm.run("slow_proc", "clientX", {})
    pid6 = await pm.run("slow_proc", "clientY", {})  # different client
    await asyncio.sleep(0.05)
    pm.cancel_all_for_client("clientX")
    await asyncio.sleep(0.1)
    _assert("cancel_all_for_client: pid4 removed",  pid4 not in pm._running)
    _assert("cancel_all_for_client: pid5 removed",  pid5 not in pm._running)
    _assert("cancel_all_for_client: pid6 intact",   pid6 in pm._running)
    pm.cancel(pid6)
    await asyncio.sleep(0.05)

    # ── unknown procedure raises ValueError ───────────────────────────────────
    try:
        await pm.run("no_such_proc", "client1", {})
        _fail("Unknown procedure: should raise ValueError")
    except ValueError:
        _ok("Unknown procedure: raises ValueError")

    # ── proc_client_id is distinct from real client_id ───────────────────────
    # Confirm no cross-pollution between real client and procedure's internal id
    pid7 = await pm.run("immediate", "realclient", {})
    proc_cid = f"proc:{pid7[:8]}"
    # After proc completes, its internal client_id is cleaned up
    done_evts[pid7] = asyncio.Event()
    await asyncio.wait_for(done_evts[pid7].wait(), timeout=2.0)
    _assert("proc_client_id: internal id cleaned up on finish",
            mgr._access_controller.get_owner("r1") is None)

    # ── StopAll procedure ─────────────────────────────────────────────────────
    pid8 = await pm.run("stop_all", "client1", {})
    done_evts[pid8] = asyncio.Event()
    await asyncio.wait_for(done_evts[pid8].wait(), timeout=2.0)
    _assert("StopAll: completes without error", done_evts[pid8].is_set())

    await mgr._scheduler.shutdown()


# ===========================================================================
# SECTION 5 — RemoteLabManager (unit, mock drivers)
# ===========================================================================
async def test_manager_unit():
    _section("RemoteLabManager (unit, mock drivers)")

    mgr = await _make_manager(device_names=["r1", "r2"], shared=False)
    drv1: _MockDriver = mgr._drivers["r1"]
    drv2: _MockDriver = mgr._drivers["r2"]

    # ── submit without acquire → PermissionError ──────────────────────────────
    try:
        await mgr.submit_command("cli", ["r1"], "move", priority=5)
        _fail("submit without acquire: should raise PermissionError")
    except PermissionError:
        _ok("submit without acquire: PermissionError raised")

    # ── submit after acquire → OK ─────────────────────────────────────────────
    mgr.acquire_device("r1", "cli")
    cid = await mgr.submit_command("cli", ["r1"], "go", priority=5)
    _assert("submit after acquire: returns non-empty command_id",
            isinstance(cid, str) and len(cid) > 0)

    # ── submit_command_wait: blocks until execution completes ─────────────────
    mgr.acquire_device("r1", "waiter")
    drv1.executed.clear()
    wait_done = asyncio.Event()

    async def do_wait():
        await mgr.submit_command_wait("waiter", ["r1"], "waitcmd", priority=5)
        wait_done.set()

    asyncio.create_task(do_wait())
    await asyncio.sleep(0.3)
    _assert("submit_command_wait: resolves after execution",   wait_done.is_set())
    _assert("submit_command_wait: command actually ran",       "waitcmd" in drv1.executed)

    # ── cancel_all_commands unblocks _command_waiters ─────────────────────────
    ev = asyncio.Event()
    fake_id = str(uuid.uuid4())
    mgr._command_waiters[fake_id] = (ev, 1)
    mgr.cancel_all_commands()
    _assert("cancel_all_commands: waiter unblocked", ev.is_set())
    _assert("cancel_all_commands: _command_waiters cleared",
            len(mgr._command_waiters) == 0)

    # ── on_command_complete fired even after interrupt ─────────────────────────
    # (tests the bug-fix: _on_execute_command uses try/finally)
    complete_calls: list = []

    async def on_complete(cmd: Command, dev: str):
        complete_calls.append(dev)

    mgr.on_command_complete = on_complete

    # Use a slow mock driver so we can interrupt
    class _SlowDriver(_MockDriver):
        async def execute_command(self, command: Command):
            self.executed.append(command.name)
            try:
                await asyncio.sleep(60.0)
            finally:
                pass

    slow_dev = _make_device("slow_dev")
    slow_drv = _SlowDriver(slow_dev)
    mgr._devices["slow_dev"]  = slow_dev
    mgr._drivers["slow_dev"]  = slow_drv
    mgr._scheduler.register_device("slow_dev")
    mgr._access_controller.register_device("slow_dev", shared=True)

    complete_calls.clear()
    await mgr.submit_command("cli", ["slow_dev"], "long", priority=5)
    await asyncio.sleep(0.05)       # let it start
    mgr.interrupt_device("slow_dev")
    await asyncio.sleep(0.1)
    _assert("Interrupt + on_command_complete: callback fired after interrupt",
            "slow_dev" in complete_calls,
            "Bug: on_command_complete not called after interrupt_device()")

    mgr.on_command_complete = None

    # ── on_client_disconnect: releases locks + cancels cmds ───────────────────
    mgr2 = await _make_manager(["dev_x"], shared=False)
    mgr2.acquire_device("dev_x", "dc")
    _assert("Before disconnect: dc owns dev_x",
            mgr2._access_controller.check_access("dev_x", "dc"))
    mgr2.on_client_disconnect("dc")
    _assert("After disconnect: dc lost ownership",
            not mgr2._access_controller.check_access("dev_x", "dc"))
    await mgr2._scheduler.shutdown()

    # ── get_devices / get_driver ──────────────────────────────────────────────
    names = [d.name for d in mgr.get_devices()]
    _assert("get_devices: all registered devices present",
            "r1" in names and "r2" in names)
    _assert("get_driver: returns driver for known device",
            mgr.get_driver("r1") is not None)
    _assert("get_driver: None for unknown device",
            mgr.get_driver("no_such") is None)

    # ── cancel_command: specific command_id ──────────────────────────────────
    mgr.acquire_device("r1", "canceller")
    drv1.executed.clear()
    mgr._scheduler.pause_device("r1")
    await asyncio.sleep(0.02)
    cid_to_cancel = await mgr.submit_command("canceller", ["r1"], "to_cancel", priority=5)
    mgr.cancel_command(cid_to_cancel)
    mgr._scheduler.resume_device("r1")
    await asyncio.sleep(0.1)
    _assert("cancel_command: cancelled cmd not executed",
            "to_cancel" not in drv1.executed)

    await mgr._scheduler.shutdown()


# ===========================================================================
# SECTION 6 — Wire protocol models
# ===========================================================================
async def test_wire_models():
    _section("Wire protocol models")

    adapter = TypeAdapter(IncomingMessage)

    cases = [
        ('{"type":"submit","devices":["r1"],"name":"move","priority":5,"args":{}}',
         SubmitMessage),
        ('{"type":"acquire","device":"r1"}',
         AcquireMessage),
        ('{"type":"release","device":"r1"}',
         ReleaseMessage),
        ('{"type":"cancel","command_id":"abc-123"}',
         CancelMessage),
        ('{"type":"interrupt","device":"r1"}',
         InterruptMessage),
        ('{"type":"subscribe_telemetry","device":"r1"}',
         SubscribeTelemetryMessage),
        ('{"type":"unsubscribe_telemetry","device":"r1"}',
         UnsubscribeTelemetryMessage),
        ('{"type":"get_devices"}',
         GetDevicesMessage),
        ('{"type":"run_procedure","name":"stop_all","args":{}}',
         RunProcedureMessage),
        ('{"type":"cancel_procedure","procedure_id":"pid-xyz"}',
         CancelProcedureMessage),
    ]

    for raw, expected_cls in cases:
        try:
            msg = adapter.validate_json(raw)
            _assert(f"Parse {expected_cls.__name__}", isinstance(msg, expected_cls))
        except Exception as e:
            _fail(f"Parse {expected_cls.__name__}", str(e))

    # ── ValidationError on unknown type ──────────────────────────────────────
    try:
        adapter.validate_json('{"type":"no_such_type"}')
        _fail("Unknown type: should raise ValidationError")
    except ValidationError:
        _ok("Unknown type: raises ValidationError")

    # ── ValidationError on missing required fields ────────────────────────────
    try:
        adapter.validate_json('{"type":"submit"}')   # missing devices + name
        _fail("Missing fields: should raise ValidationError")
    except ValidationError:
        _ok("Missing fields: raises ValidationError")

    # ── Default field values ──────────────────────────────────────────────────
    msg = adapter.validate_json('{"type":"submit","devices":["r1"],"name":"go"}')
    _assert("SubmitMessage: default priority == 5",  msg.priority == 5)
    _assert("SubmitMessage: default args == {}",     msg.args == {})

    # ── Outgoing message serialization ────────────────────────────────────────
    ack = AckMessage(command_id="cid1")
    d   = json.loads(ack.model_dump_json())
    _assert("AckMessage: type field",     d["type"]       == "ack")
    _assert("AckMessage: command_id",     d["command_id"] == "cid1")
    _assert("AckMessage: status=queued",  d["status"]     == "queued")

    done = DoneMessage(command_id="cid1", device="r1")
    d    = json.loads(done.model_dump_json())
    _assert("DoneMessage: type",          d["type"]   == "done")
    _assert("DoneMessage: device",        d["device"] == "r1")

    err = ErrorMessage(code="ACCESS_DENIED", message="nope")
    d   = json.loads(err.model_dump_json())
    _assert("ErrorMessage: code",         d["code"]    == "ACCESS_DENIED")
    _assert("ErrorMessage: message",      d["message"] == "nope")

    tel = TelemetryMessage(device="r1", data={"speed": 1.5})
    d   = json.loads(tel.model_dump_json())
    _assert("TelemetryMessage: type",     d["type"] == "telemetry")
    _assert("TelemetryMessage: data",     d["data"]["speed"] == 1.5)

    pa = ProcedureAckMessage(procedure_id="pid1")
    _assert("ProcedureAckMessage: type",  json.loads(pa.model_dump_json())["type"] == "procedure_ack")

    pd = ProcedureDoneMessage(procedure_id="pid1")
    _assert("ProcedureDoneMessage: type", json.loads(pd.model_dump_json())["type"] == "procedure_done")

    pe = ProcedureErrorMessage(procedure_id="pid1", message="boom")
    d  = json.loads(pe.model_dump_json())
    _assert("ProcedureErrorMessage: type",    d["type"]    == "procedure_error")
    _assert("ProcedureErrorMessage: message", d["message"] == "boom")

    devs_msg = DevicesMessage(data=[])
    _assert("DevicesMessage: type",       json.loads(devs_msg.model_dump_json())["type"] == "devices")


# ===========================================================================
# SECTION 7 — Integration (full server + WebSocket)
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

    # Devices config: one shared mock + one exclusive mock
    TEST_DEVICES = [
        {
            "name": "shared_mock", "ip": "", "port": 0, "driver": "mock",
            "ros_namespace": None, "shared": True, "active": True,
        },
        {
            "name": "excl_mock", "ip": "", "port": 0, "driver": "mock",
            "ros_namespace": None, "shared": False, "active": True,
        },
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

    # Inject test users and devices
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

        # Poll /health until ready (max 20 s)
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
                    if r.status == 200:
                        ready = True
                        break
            except Exception:
                pass

        if not ready:
            out = server_proc.stdout.read().decode(errors="replace")
            _fail("Server startup", f"no /health response in 20 s:\n{out[-3000:]}")
            return
        _ok(f"Server started — /health OK  (port {PORT})")

        ws_url   = f"ws://localhost:{PORT}/ws"
        ok_hdr   = {"Authorization": "Basic " + base64.b64encode(
                        f"{INTEG_USER}:{INTEG_PASS}".encode()).decode()}
        ok2_hdr  = {"Authorization": "Basic " + base64.b64encode(
                        f"{USER2}:{PASS2}".encode()).decode()}
        bad_hdr  = {"Authorization": "Basic " + base64.b64encode(
                        b"nobody:wrongpass").decode()}

        # ── Invalid credentials rejected ──────────────────────────────────────
        rejected = False
        try:
            async with _ws.connect(ws_url, extra_headers=bad_hdr):
                pass
        except Exception:
            rejected = True
        _assert("Auth: invalid credentials → connection rejected", rejected)

        # ── UNKNOWN_MESSAGE error ─────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send('{"type":"totally_unknown_type_xyz"}')
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("UNKNOWN_MESSAGE: error type",  msg["type"] == "error")
            _assert("UNKNOWN_MESSAGE: error code",  msg["code"] == "UNKNOWN_MESSAGE")

        # ── Malformed JSON ────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send("not valid json {{{{")
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Malformed JSON: error returned", msg["type"] == "error")

        # ── get_devices ───────────────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send('{"type":"get_devices"}')
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("get_devices: type == devices",     msg["type"] == "devices")
            names = [d["name"] for d in msg["data"]]
            _assert("get_devices: shared_mock present",  "shared_mock" in names)
            _assert("get_devices: excl_mock present",    "excl_mock"   in names)
            shared_info = next(d for d in msg["data"] if d["name"] == "shared_mock")
            excl_info   = next(d for d in msg["data"] if d["name"] == "excl_mock")
            _assert("get_devices: shared flag correct",  shared_info["shared"] is True)
            _assert("get_devices: excl flag correct",    excl_info["shared"]   is False)

        # ── Shared device: submit without acquire ─────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({
                "type": "submit", "devices": ["shared_mock"],
                "name": "test_cmd", "priority": 5, "args": {},
            }))
            ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Shared submit: ack received",     ack["type"] == "ack")
            _assert("Shared submit: has command_id",   "command_id" in ack)
            _assert("Shared submit: status == queued", ack.get("status") == "queued")

            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Shared submit: done received",    done["type"] == "done")
            _assert("Shared submit: matching id",      done["command_id"] == ack["command_id"])
            _assert("Shared submit: correct device",   done["device"] == "shared_mock")

        # ── Exclusive: submit without acquire → ACCESS_DENIED ─────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "test_cmd", "priority": 5, "args": {},
            }))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Excl without acquire: error type",  err["type"] == "error")
            _assert("Excl without acquire: ACCESS_DENIED", err["code"] == "ACCESS_DENIED")

        # ── Exclusive: acquire + submit + done + release ──────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)  # silence = success

            await ws.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "go", "priority": 5, "args": {},
            }))
            ack  = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Excl acquire+submit: ack",         ack["type"] == "ack")
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Excl acquire+submit: done",        done["type"] == "done")
            _assert("Excl acquire+submit: device",      done["device"] == "excl_mock")

            await ws.send(json.dumps({"type": "release", "device": "excl_mock"}))
            await asyncio.sleep(0.1)
            _ok("Excl: release sent (silence = success)")

        # ── ALREADY_OWNED: second client gets rejected ────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws1:
            await ws1.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.2)

            async with _ws.connect(ws_url, extra_headers=ok2_hdr) as ws2:
                await ws2.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
                err = json.loads(await asyncio.wait_for(ws2.recv(), timeout=3.0))
                _assert("ALREADY_OWNED: error type",  err["type"] == "error")
                _assert("ALREADY_OWNED: error code",  err["code"] == "ALREADY_OWNED")

        # ── UNKNOWN_DEVICE: submit to non-existent device ─────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({
                "type": "submit", "devices": ["no_such_device"],
                "name": "test", "priority": 5, "args": {},
            }))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("UNKNOWN_DEVICE: error code", err["code"] == "UNKNOWN_DEVICE")

        # ── Cancel queued command ─────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)

            # First (slow) command keeps worker busy
            await ws.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "slow", "priority": 5, "args": {"duration": 2.0},
            }))
            ack1 = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Cancel setup: slow cmd acked", ack1["type"] == "ack")

            # Second command enqueued + immediately cancelled
            await ws.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "cancel_me", "priority": 3, "args": {},
            }))
            ack2 = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            await ws.send(json.dumps({"type": "cancel", "command_id": ack2["command_id"]}))

            # First command completes normally
            done1 = json.loads(await asyncio.wait_for(ws.recv(), timeout=10.0))
            _assert("Cancel queued: first cmd done", done1["type"] == "done")

            # No second done expected (cancelled)
            try:
                extra = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.5))
                _fail("Cancel queued: unexpected msg after cancel", str(extra))
            except asyncio.TimeoutError:
                _ok("Cancel queued: second cmd not executed (correctly skipped)")

        # ── Interrupt executing command ───────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)

            await ws.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "very_long", "priority": 5, "args": {"duration": 30.0},
            }))
            ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            await asyncio.sleep(0.1)  # let it start executing

            await ws.send(json.dumps({"type": "interrupt", "device": "excl_mock"}))

            # After interrupt, done must arrive (tests the bug-fix in manager.py)
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Interrupt: done received after interrupt",
                    done["type"] == "done",
                    "Bug: on_command_complete not called after interrupt")
            _assert("Interrupt: matching command_id",
                    done["command_id"] == ack["command_id"])

        # ── Subscribe telemetry (mock driver sends no data — verify no crash) ──
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "subscribe_telemetry",
                                      "device": "shared_mock"}))
            await asyncio.sleep(0.2)
            await ws.send(json.dumps({"type": "unsubscribe_telemetry",
                                      "device": "shared_mock"}))
            _ok("Telemetry: subscribe/unsubscribe without crash")

        # ── Subscribe unknown device → UNKNOWN_DEVICE ─────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "subscribe_telemetry",
                                      "device": "ghost_device"}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Subscribe unknown device: UNKNOWN_DEVICE",
                    err["type"] == "error" and err["code"] == "UNKNOWN_DEVICE")

        # ── run_procedure: stop_all ────────────────────────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "run_procedure",
                                      "name": "stop_all", "args": {}}))
            proc_ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Procedure stop_all: ack type",
                    proc_ack["type"] == "procedure_ack")
            _assert("Procedure stop_all: has procedure_id",
                    "procedure_id" in proc_ack)

            proc_done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Procedure stop_all: done type",
                    proc_done["type"] == "procedure_done")
            _assert("Procedure stop_all: matching id",
                    proc_done["procedure_id"] == proc_ack["procedure_id"])

        # ── run_procedure: unknown → UNKNOWN_PROCEDURE ─────────────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "run_procedure",
                                      "name": "no_such_procedure", "args": {}}))
            err = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Unknown procedure: error type",
                    err["type"] == "error")
            _assert("Unknown procedure: UNKNOWN_PROCEDURE code",
                    err["code"] == "UNKNOWN_PROCEDURE")

        # ── cancel_procedure with bogus id: no-op / no crash ──────────────────
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({"type": "cancel_procedure",
                                      "procedure_id": "non_existent_bogus_id"}))
            await asyncio.sleep(0.2)
            _ok("cancel_procedure bogus id: no crash")

        # ── Second connection evicts first ─────────────────────────────────────
        ws1 = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws1.send('{"type":"get_devices"}')
        await asyncio.wait_for(ws1.recv(), timeout=3.0)

        ws2 = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws2.send('{"type":"get_devices"}')
        msg2 = json.loads(await asyncio.wait_for(ws2.recv(), timeout=3.0))
        _assert("Second connection: new conn works",
                msg2["type"] == "devices")

        # ws1 should be dead now (closed by server)
        evicted = False
        try:
            await asyncio.wait_for(ws1.recv(), timeout=1.0)
        except Exception:
            evicted = True
        _assert("Second connection: first conn evicted", evicted)

        await ws1.close()
        await ws2.close()

        # ── Multi-device command (two shared devices) ──────────────────────────
        # For a real multi-device test we'd need two different shared devices.
        # We only have one shared mock, so verify the ack+done flow with it.
        async with _ws.connect(ws_url, extra_headers=ok_hdr) as ws:
            await ws.send(json.dumps({
                "type": "submit", "devices": ["shared_mock"],
                "name": "multi_test", "priority": 5, "args": {},
            }))
            ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            _assert("Multi-device flow: ack",  ack["type"] == "ack")
            done = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("Multi-device flow: done", done["type"] == "done")

        # ── Disconnect cleans up: exclusive lock released ─────────────────────
        ws_temp = await _ws.connect(ws_url, extra_headers=ok_hdr)
        await ws_temp.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
        await asyncio.sleep(0.15)
        await ws_temp.close()
        await asyncio.sleep(0.2)

        # Another client should now be able to acquire
        async with _ws.connect(ws_url, extra_headers=ok2_hdr) as ws2_after:
            await ws2_after.send(json.dumps({"type": "acquire", "device": "excl_mock"}))
            await asyncio.sleep(0.15)
            # Submit to verify access was granted (no ACCESS_DENIED)
            await ws2_after.send(json.dumps({
                "type": "submit", "devices": ["excl_mock"],
                "name": "post_disconnect", "priority": 5, "args": {},
            }))
            msg = json.loads(await asyncio.wait_for(ws2_after.recv(), timeout=3.0))
            _assert("Disconnect cleanup: lock released, other client can acquire",
                    msg["type"] == "ack",
                    f"Expected ack, got {msg}")

    except Exception:
        _fail("Integration", traceback.format_exc())

    finally:
        if server_proc and server_proc.poll() is None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()

        # Always print server output for diagnosis
        if server_proc and server_proc.stdout:
            try:
                out = server_proc.stdout.read().decode(errors="replace")
                if out.strip():
                    _section("Server output (last 4000 chars)")
                    print(out[-4000:])
            except Exception:
                pass

        # Restore original files
        try:
            users_path.write_text(users_bak)
            devices_path.write_text(devices_bak)
        except Exception:
            pass


# ===========================================================================
# MAIN — run all sections in one event loop
# ===========================================================================
async def _run_all():
    await test_basic_classes()
    await test_access_controller()
    await test_command_scheduler()
    await test_procedure_manager()
    await test_manager_unit()
    await test_wire_models()
    await test_integration()


try:
    asyncio.run(_run_all())
except Exception:
    _fail("async runner crashed", traceback.format_exc())

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
_section("Summary")
total = _passed + _failed + _skipped
print(f"  Passed:  {_passed}/{total}")
print(f"  Failed:  {_failed}/{total}")
print(f"  Skipped: {_skipped}/{total}")
print()
sys.exit(1 if _failed else 0)
