#!/usr/bin/env python3
"""
Smoke-test for the RemoteLab server-side code.

Run from the manager/ directory:
    python test.py

Covers all pure-Python modules. ROS-dependent modules (rospy, serial_asyncio,
msg_yy) are probed for availability and skipped gracefully if absent.
"""

import sys
import asyncio
import json
import traceback
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

# ---------------------------------------------------------------------------
# Minimal test reporter
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
    print(f"\n{'='*56}")
    print(f"  {title}")
    print("=" * 56)


def _assert(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        _ok(name)
    else:
        _fail(name, detail or "assertion is False")


def _try(name: str, fn) -> bool:
    """Run fn(), report ok/fail, return success flag."""
    try:
        fn()
        _ok(name)
        return True
    except Exception:
        _fail(name, traceback.format_exc())
        return False


# ---------------------------------------------------------------------------
# 1. Import probe
# ---------------------------------------------------------------------------
_section("Import probe")

_ok_imports: dict = {}

for _mod in ["BasicClasses", "AccessController", "Logger",
             "TelemetryTypes", "CommandScheduler"]:
    try:
        __import__(_mod)
        _ok(f"import {_mod}")
        _ok_imports[_mod] = True
    except Exception as exc:
        _fail(f"import {_mod}", str(exc))
        _ok_imports[_mod] = False

for _mod in ["network.models", "network.auth"]:
    try:
        __import__(_mod)
        _ok(f"import {_mod}")
        _ok_imports[_mod] = True
    except Exception as exc:
        _fail(f"import {_mod}", str(exc))
        _ok_imports[_mod] = False

_has_ros = _has_serial = _has_msg_yy = False
for _label, _modname in [
    ("rospy",         "rospy"),
    ("serial_asyncio","serial_asyncio"),
    ("msg_yy",        "msg_yy.msg"),
]:
    try:
        __import__(_modname)
        if _label == "rospy":          _has_ros    = True
        if _label == "serial_asyncio": _has_serial = True
        if _label == "msg_yy":         _has_msg_yy = True
        _ok(f"import {_modname}")
    except ImportError as exc:
        _skip(f"import {_modname}", str(exc))

_has_drivers = _has_ros and _has_serial and _has_msg_yy
if _has_drivers:
    try:
        import DeviceDrivers  # noqa: F401
        _ok("import DeviceDrivers")
        _ok_imports["DeviceDrivers"] = True
    except Exception:
        _fail("import DeviceDrivers", traceback.format_exc())
        _has_drivers = False
else:
    _skip("import DeviceDrivers", "ROS deps missing (expected outside ROS env)")


# ---------------------------------------------------------------------------
# 2. BasicClasses
# ---------------------------------------------------------------------------
_section("BasicClasses")

if _ok_imports.get("BasicClasses"):
    try:
        import heapq, time
        from BasicClasses import Device, Command

        d = Device(
            name="robot1", ip="127.0.0.1", port=9000,
            shared=True, active=True, driver="yarp13",
            ros_namespace="/robot1", baud_rate=None,
        )
        _assert("Device creation", d.name == "robot1")

        c_hi = Command(priority=10, client_id="alice", devices=["r1"], name="high")
        c_lo = Command(priority=5,  client_id="alice", devices=["r1"], name="low")
        _assert("Higher priority sorts first (c_hi < c_lo)", c_hi < c_lo)

        time.sleep(0.01)
        c_lo2 = Command(priority=5, client_id="alice", devices=["r1"], name="low2")
        _assert("FIFO at equal priority (earlier < later)", c_lo < c_lo2)

        heap = [c_lo, c_hi, c_lo2]
        heapq.heapify(heap)
        first = heapq.heappop(heap)
        _assert("Heap pops highest priority first", first.name == "high",
                f"got '{first.name}'")

        _assert("Command equality by command_id",
                c_hi == c_hi and c_hi != c_lo)
        _assert("Command is hashable (usable in set)",
                len({c_hi, c_hi, c_lo}) == 2)

    except Exception:
        _fail("BasicClasses", traceback.format_exc())
else:
    _skip("BasicClasses tests", "import failed")


# ---------------------------------------------------------------------------
# 3. AccessController
# ---------------------------------------------------------------------------
_section("AccessController")

if _ok_imports.get("AccessController"):
    try:
        from AccessController import AccessController

        ac = AccessController()
        ac.register_device("shared",    shared=True)
        ac.register_device("exclusive", shared=False)

        _assert("Shared: always accessible without acquire",
                ac.check_access("shared", "alice"))
        _assert("Shared: acquire always returns True",
                ac.acquire("shared", "alice"))

        _assert("Exclusive: inaccessible before acquire",
                not ac.check_access("exclusive", "alice"))

        _assert("Exclusive: acquire succeeds when free",
                ac.acquire("exclusive", "alice"))
        _assert("Exclusive: accessible after acquire",
                ac.check_access("exclusive", "alice"))

        _assert("Exclusive: other client is blocked",
                not ac.acquire("exclusive", "bob"))
        _assert("Exclusive: other client cannot access",
                not ac.check_access("exclusive", "bob"))

        _assert("Exclusive: re-acquire by owner is idempotent",
                ac.acquire("exclusive", "alice"))

        ac.release("exclusive", "alice")
        _assert("Exclusive: inaccessible after release",
                not ac.check_access("exclusive", "alice"))

        ac.acquire("exclusive", "bob")
        ac.release_all("bob")
        _assert("release_all clears all locks",
                ac.get_owner("exclusive") is None)

    except Exception:
        _fail("AccessController", traceback.format_exc())
else:
    _skip("AccessController tests", "import failed")


# ---------------------------------------------------------------------------
# 4. network.auth
# ---------------------------------------------------------------------------
_section("network.auth  (password hashing + UserStore)")

if _ok_imports.get("network.auth"):
    try:
        from network.auth import _hash_password, _verify_password, UserStore

        h = _hash_password("s3cr3t!")
        parts = h.split(":")
        _assert("Hash has exactly 4 colon-separated parts",
                len(parts) == 4, f"got {len(parts)}: {h}")
        _assert("Hash starts with 'pbkdf2_'",
                parts[0].startswith("pbkdf2_"), parts[0])

        _assert("Correct password verifies",      _verify_password("s3cr3t!", h))
        _assert("Wrong password rejected",         not _verify_password("wrong", h))
        _assert("Corrupted hash returns False",    not _verify_password("x", "garbage"))
        _assert("Too few parts returns False",     not _verify_password("x", "a:b:c"))
        _assert("Empty string returns False",      not _verify_password("x", ""))

        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False
        ) as f:
            json.dump({
                "_note": "ignored",
                "alice": _hash_password("pa$$word"),
            }, f)
            tmp = Path(f.name)

        store = UserStore(tmp)
        _assert("UserStore: correct credentials",  store.verify("alice", "pa$$word"))
        _assert("UserStore: wrong password",       not store.verify("alice", "wrong"))
        _assert("UserStore: unknown user",         not store.verify("nobody", "pa$$word"))
        _assert("UserStore: _ keys are ignored",   "_note" not in store._users)
        tmp.unlink()

    except Exception:
        _fail("network.auth", traceback.format_exc())
else:
    _skip("network.auth tests", "import failed")


# ---------------------------------------------------------------------------
# 5. network.models
# ---------------------------------------------------------------------------
_section("network.models  (Pydantic wire protocol)")

if _ok_imports.get("network.models"):
    try:
        from pydantic import TypeAdapter, ValidationError
        from network.models import (
            IncomingMessage,
            SubmitMessage, AcquireMessage, ReleaseMessage, CancelMessage,
            SubscribeTelemetryMessage, UnsubscribeTelemetryMessage,
            GetDevicesMessage,
            AckMessage, DoneMessage, TelemetryMessage,
            DevicesMessage, DeviceInfo, ErrorMessage,
        )

        adapter = TypeAdapter(IncomingMessage)

        incoming = [
            ('{"type":"submit","devices":["r1"],"name":"move"}',
             SubmitMessage),
            ('{"type":"submit","devices":["r1","r2"],"name":"go",'
             '"priority":9,"args":{"x":1.0}}',
             SubmitMessage),
            ('{"type":"acquire","device":"r1"}',
             AcquireMessage),
            ('{"type":"release","device":"r1"}',
             ReleaseMessage),
            ('{"type":"cancel","command_id":"abc-123"}',
             CancelMessage),
            ('{"type":"subscribe_telemetry","device":"r1"}',
             SubscribeTelemetryMessage),
            ('{"type":"unsubscribe_telemetry","device":"r1"}',
             UnsubscribeTelemetryMessage),
            ('{"type":"get_devices"}',
             GetDevicesMessage),
        ]

        for raw, cls in incoming:
            try:
                msg = adapter.validate_json(raw)
                _assert(f"Parse {cls.__name__}", isinstance(msg, cls))
            except Exception:
                _fail(f"Parse {cls.__name__}", traceback.format_exc())

        raised = False
        try:
            adapter.validate_json('{"type":"does_not_exist"}')
        except ValidationError:
            raised = True
        _assert("Unknown type raises ValidationError", raised)

        def _j(model):
            return json.loads(model.model_dump_json())

        d = _j(AckMessage(command_id="id1"))
        _assert("AckMessage  serializes correctly",
                d["type"] == "ack" and d["command_id"] == "id1" and d["status"] == "queued")

        d = _j(DoneMessage(command_id="id1", device="r1"))
        _assert("DoneMessage serializes correctly",
                d["type"] == "done" and d["device"] == "r1")

        d = _j(ErrorMessage(code="ACCESS_DENIED", message="nope"))
        _assert("ErrorMessage serializes correctly",
                d["type"] == "error" and d["code"] == "ACCESS_DENIED")

        d = _j(DevicesMessage(data=[DeviceInfo(name="r1", driver="yarp13", shared=False)]))
        _assert("DevicesMessage serializes correctly",
                d["data"][0]["name"] == "r1")

        d = _j(TelemetryMessage(device="r1", data={"speed": 1.5}))
        _assert("TelemetryMessage serializes correctly",
                d["type"] == "telemetry" and d["data"]["speed"] == 1.5)

        # SubmitMessage: args default to {} not None
        msg = adapter.validate_json('{"type":"submit","devices":["r1"],"name":"stop"}')
        _assert("SubmitMessage.args defaults to {}",
                isinstance(msg, SubmitMessage) and msg.args == {})

    except Exception:
        _fail("network.models", traceback.format_exc())
else:
    _skip("network.models tests", "import failed")


# ---------------------------------------------------------------------------
# 6. Async tests — all in ONE asyncio.run() to share the event loop
# ---------------------------------------------------------------------------
async def _run_async_tests():

    # --- Logger ---
    _section("Logger")
    try:
        from Logger import Logger

        logger = Logger.get()
        await logger.log("TEST", "Logger smoke test")  # should print a timestamped line

        _assert("Logger is a singleton", Logger.get() is logger)
        _ok("Logger.log() executes without error")
    except Exception:
        _fail("Logger", traceback.format_exc())

    # --- CommandScheduler ---
    _section("CommandScheduler")

    if not _ok_imports.get("CommandScheduler"):
        _skip("CommandScheduler tests", "import failed")
        return

    try:
        from CommandScheduler import CommandScheduler
        from BasicClasses import Command

        results: list = []

        async def _fake_exec(command: Command, device_name: str) -> None:
            results.append((command.name, device_name))

        sched = CommandScheduler(_fake_exec)
        sched.register_device("r1")
        sched.register_device("r2")
        await sched.start()

        # Basic submit + execute
        results.clear()
        await sched.submit("alice", ["r1"], "hello", priority=5)
        await asyncio.sleep(0.15)
        _assert("Basic submit executes", ("hello", "r1") in results)

        # Priority ordering: submit low then high while paused
        sched.pause_device("r1")
        results.clear()
        await sched.submit("alice", ["r1"], "low_cmd",  priority=1)
        await sched.submit("alice", ["r1"], "high_cmd", priority=9)
        sched.resume_device("r1")
        await asyncio.sleep(0.2)
        names_r1 = [name for name, dev in results if dev == "r1"]
        _assert("High priority executes first",
                names_r1[:1] == ["high_cmd"],
                f"execution order: {names_r1}")

        # Command cancellation
        sched.pause_device("r1")
        results.clear()
        cid = await sched.submit("alice", ["r1"], "to_cancel", priority=1)
        await sched.submit("alice", ["r1"], "to_run",    priority=9)
        sched.cancel(cid)
        sched.resume_device("r1")
        await asyncio.sleep(0.2)
        names = [name for name, _ in results]
        _assert("Cancelled command does not execute",   "to_cancel" not in names)
        _assert("Non-cancelled command still executes", "to_run"    in names)

        # Multi-device submit
        results.clear()
        await sched.submit("alice", ["r1", "r2"], "broadcast", priority=5)
        await asyncio.sleep(0.2)
        _assert("Multi-device: executes on r1", ("broadcast", "r1") in results)
        _assert("Multi-device: executes on r2", ("broadcast", "r2") in results)

        # cancel_client_commands (called on client disconnect)
        sched.pause_device("r1")
        results.clear()
        await sched.submit("bob", ["r1"], "bob_cmd", priority=5)
        sched.cancel_client_commands("bob")
        sched.resume_device("r1")
        await asyncio.sleep(0.15)
        _assert("cancel_client_commands: command does not execute",
                ("bob_cmd", "r1") not in results)

        # interrupt_device (cancel currently executing command)
        results.clear()
        executing = asyncio.Event()
        interrupted = asyncio.Event()

        async def _slow_exec(command: Command, device_name: str) -> None:
            if command.name == "slow":
                executing.set()
                try:
                    await asyncio.sleep(10)  # will be interrupted
                except asyncio.CancelledError:
                    interrupted.set()
                    raise
            else:
                results.append(command.name)

        sched2 = CommandScheduler(_slow_exec)
        sched2.register_device("r1")
        await sched2.start()
        await sched2.submit("alice", ["r1"], "slow",     priority=5)
        await sched2.submit("alice", ["r1"], "after_interrupt", priority=5)
        await asyncio.wait_for(executing.wait(), timeout=1.0)
        sched2.interrupt_device("r1")
        await asyncio.wait_for(interrupted.wait(), timeout=1.0)
        await asyncio.sleep(0.15)
        _assert("interrupt_device: interrupted command raises CancelledError",
                interrupted.is_set())
        _assert("interrupt_device: next command executes after interrupt",
                "after_interrupt" in results)
        await sched2.shutdown()

        await sched.shutdown()
        _ok("Scheduler shutdown clean")

    except Exception:
        _fail("CommandScheduler", traceback.format_exc())


_section("Async tests (Logger + CommandScheduler)")
try:
    asyncio.run(_run_async_tests())
except Exception:
    _fail("async test runner", traceback.format_exc())


# ---------------------------------------------------------------------------
# 7. Summary
# ---------------------------------------------------------------------------
_section("Summary")
total = _passed + _failed + _skipped
print(f"  Passed:  {_passed}")
print(f"  Failed:  {_failed}")
print(f"  Skipped: {_skipped}  (expected on non-ROS machine)")
print(f"  Total:   {total}\n")
if _failed == 0:
    print("  All testable components OK.")
else:
    print(f"  {_failed} test(s) FAILED.")
sys.exit(0 if _failed == 0 else 1)
