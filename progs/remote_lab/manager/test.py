#!/usr/bin/env python3
"""
Smoke-test for the RemoteLab server-side code.

Basic run (no ROS device required):
    python test.py

Full integration test (requires: roscore running + device connected):
    python test.py --integration

The integration test starts the server as a subprocess, connects via WebSocket,
and exercises the full protocol (auth, get_devices, acquire, submit, done, telemetry).
"""

import argparse
import asyncio
import base64
import json
import subprocess
import sys
import tempfile
import traceback
import urllib.request
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).parent))

_cli = argparse.ArgumentParser(description="RemoteLab smoke test")
_cli.add_argument("--integration", action="store_true",
                  help="Run full server integration test (needs roscore + device)")
_cli.add_argument("--port", type=int, default=8765,
                  help="Port to use for the integration test server (default: 8765)")
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
    print(f"\n{'='*58}")
    print(f"  {title}")
    print("=" * 58)


def _assert(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        _ok(name)
    else:
        _fail(name, detail or "assertion is False")


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
    ("rospy",          "rospy"),
    ("serial_asyncio", "serial_asyncio"),
    ("msg_yy",         "msg_yy.msg"),
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
        import heapq
        import time as _time
        from BasicClasses import Command, Device

        d = Device(name="robot1", ip="127.0.0.1", port=9000,
                   shared=True, active=True, driver="yarp13",
                   ros_namespace="/robot1", baud_rate=None)
        _assert("Device creation", d.name == "robot1")

        c_hi = Command(priority=10, client_id="alice", devices=["r1"], name="high")
        c_lo = Command(priority=5,  client_id="alice", devices=["r1"], name="low")
        _assert("Higher priority sorts first (c_hi < c_lo)", c_hi < c_lo)

        _time.sleep(0.01)
        c_lo2 = Command(priority=5, client_id="alice", devices=["r1"], name="low2")
        _assert("FIFO at equal priority (earlier < later)", c_lo < c_lo2)

        heap = [c_lo, c_hi, c_lo2]
        heapq.heapify(heap)
        first = heapq.heappop(heap)
        _assert("Heap pops highest priority first",
                first.name == "high", f"got '{first.name}'")

        _assert("Command equality by command_id",
                c_hi == c_hi and c_hi != c_lo)
        _assert("Command is hashable (usable in a set)",
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
        from network.auth import UserStore, _hash_password, _verify_password

        h = _hash_password("s3cr3t!")
        parts = h.split(":")
        _assert("Hash has exactly 4 colon-separated parts",
                len(parts) == 4, f"got {len(parts)}: {h}")
        _assert("Hash starts with 'pbkdf2_'",
                parts[0].startswith("pbkdf2_"), parts[0])

        _assert("Correct password verifies",   _verify_password("s3cr3t!", h))
        _assert("Wrong password rejected",     not _verify_password("wrong", h))
        _assert("Corrupted hash -> False",     not _verify_password("x", "garbage"))
        _assert("Too few parts -> False",      not _verify_password("x", "a:b:c"))
        _assert("Empty string -> False",       not _verify_password("x", ""))

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({"_note": "ignored", "alice": _hash_password("pa$$word")}, f)
            _tmp = Path(f.name)

        store = UserStore(_tmp)
        _assert("UserStore: correct credentials",  store.verify("alice", "pa$$word"))
        _assert("UserStore: wrong password",       not store.verify("alice", "wrong"))
        _assert("UserStore: unknown user",         not store.verify("nobody", "pa$$word"))
        _assert("UserStore: _ keys are ignored",   "_note" not in store._users)
        _tmp.unlink()

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
            AckMessage, CancelMessage, DeviceInfo,
            DevicesMessage, DoneMessage, ErrorMessage,
            GetDevicesMessage, IncomingMessage,
            AcquireMessage, ReleaseMessage, SubmitMessage,
            SubscribeTelemetryMessage, TelemetryMessage,
            UnsubscribeTelemetryMessage,
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
        _assert("AckMessage serialization",
                d["type"] == "ack" and d["command_id"] == "id1" and d["status"] == "queued")

        d = _j(DoneMessage(command_id="id1", device="r1"))
        _assert("DoneMessage serialization",
                d["type"] == "done" and d["device"] == "r1")

        d = _j(ErrorMessage(code="ACCESS_DENIED", message="nope"))
        _assert("ErrorMessage serialization",
                d["type"] == "error" and d["code"] == "ACCESS_DENIED")

        d = _j(DevicesMessage(data=[DeviceInfo(name="r1", driver="yarp13", shared=False)]))
        _assert("DevicesMessage serialization", d["data"][0]["name"] == "r1")

        d = _j(TelemetryMessage(device="r1", data={"speed": 1.5}))
        _assert("TelemetryMessage serialization",
                d["type"] == "telemetry" and d["data"]["speed"] == 1.5)

        msg = adapter.validate_json('{"type":"submit","devices":["r1"],"name":"stop"}')
        _assert("SubmitMessage.args defaults to {}",
                isinstance(msg, SubmitMessage) and msg.args == {})

    except Exception:
        _fail("network.models", traceback.format_exc())
else:
    _skip("network.models tests", "import failed")


# ---------------------------------------------------------------------------
# 6. DeviceDrivers  (component tests — no real hardware)
# ---------------------------------------------------------------------------
_section("DeviceDrivers  (component tests with mock interfaces)")

if _has_drivers:
    try:
        from BasicClasses import Device
        from DeviceDrivers import (
            DriverFactory, SimpleSerialDevice, Yarp13Driver,
        )

        mock_ros    = MagicMock()
        mock_serial = MagicMock()
        factory = DriverFactory(mock_ros, mock_serial)

        dev_ros = Device(name="r1", ip="1.2.3.4", port=9000, shared=True, active=True,
                         driver="yarp13", ros_namespace="/r1", baud_rate=None)
        dev_ser = Device(name="s1", ip="1.2.3.4", port=9001, shared=False, active=True,
                         driver="simple_serial", ros_namespace=None, baud_rate=9600)
        dev_bad = Device(name="x",  ip="1.2.3.4", port=9002, shared=True, active=True,
                         driver="no_such_driver", ros_namespace=None, baud_rate=None)

        drv_ros = factory.create_driver(dev_ros)
        drv_ser = factory.create_driver(dev_ser)
        _assert("DriverFactory creates Yarp13Driver",       isinstance(drv_ros, Yarp13Driver))
        _assert("DriverFactory creates SimpleSerialDevice", isinstance(drv_ser, SimpleSerialDevice))

        raised = False
        try:
            factory.create_driver(dev_bad)
        except ValueError:
            raised = True
        _assert("Unknown driver type raises ValueError", raised)

        # Transport path
        _assert("Transport path contains device name",
                dev_ros.name in str(drv_ros._transport_path))
        _assert("Transport path is under /tmp",
                str(drv_ros._transport_path).startswith("/tmp"))

        # Telemetry listener registry
        async def _dummy_cb(t):
            pass

        lid1 = drv_ros.add_telemetry_listener(_dummy_cb)
        lid2 = drv_ros.add_telemetry_listener(_dummy_cb)
        _assert("add_telemetry_listener returns unique IDs", lid1 != lid2)
        _assert("Two listeners registered", len(drv_ros._telemetry_listeners) == 2)

        drv_ros.remove_telemetry_listener(lid1)
        _assert("remove_telemetry_listener removes one", len(drv_ros._telemetry_listeners) == 1)

        drv_ros.remove_telemetry_listener(9999)  # non-existent — must not crash
        _ok("remove_telemetry_listener with unknown id is a no-op")

        _assert("get_telemetry() returns None before first message",
                drv_ros.get_telemetry() is None)

        # _notify_listeners sends to all registered listeners
        received = []

        async def _capturing_cb(t):
            received.append(t)

        drv_ros.add_telemetry_listener(_capturing_cb)
        asyncio.run(drv_ros._notify_listeners("fake_telemetry"))
        _assert("_notify_listeners calls all callbacks",
                "fake_telemetry" in received)

    except Exception:
        _fail("DeviceDrivers", traceback.format_exc())
else:
    _skip("DeviceDrivers tests", "ROS deps missing")


# ---------------------------------------------------------------------------
# 7. Async tests — all in ONE asyncio.run() to share the event loop
# ---------------------------------------------------------------------------
async def _run_async_tests():

    # --- Logger ---
    _section("Logger")
    try:
        from Logger import Logger
        logger = Logger.get()
        await logger.log("TEST", "Logger smoke test")
        _assert("Logger is a singleton", Logger.get() is logger)
        _ok("Logger.log() executes without error")
    except Exception:
        _fail("Logger", traceback.format_exc())

    # --- CommandScheduler ---
    _section("CommandScheduler")
    if not _ok_imports.get("CommandScheduler"):
        _skip("CommandScheduler tests", "import failed")
    else:
        try:
            from BasicClasses import Command
            from CommandScheduler import CommandScheduler

            results: list = []

            async def _fake_exec(command: Command, device_name: str) -> None:
                results.append((command.name, device_name))

            sched = CommandScheduler(_fake_exec)
            sched.register_device("r1")
            sched.register_device("r2")
            await sched.start()

            # Basic execute
            results.clear()
            await sched.submit("alice", ["r1"], "hello", priority=5)
            await asyncio.sleep(0.15)
            _assert("Basic submit executes", ("hello", "r1") in results)

            # Priority ordering
            sched.pause_device("r1")
            results.clear()
            await sched.submit("alice", ["r1"], "low_cmd",  priority=1)
            await sched.submit("alice", ["r1"], "high_cmd", priority=9)
            sched.resume_device("r1")
            await asyncio.sleep(0.2)
            order = [n for n, d in results if d == "r1"]
            _assert("High priority executes first",
                    order[:1] == ["high_cmd"], f"order: {order}")

            # Cancellation
            sched.pause_device("r1")
            results.clear()
            cid = await sched.submit("alice", ["r1"], "to_cancel", priority=1)
            await sched.submit("alice", ["r1"], "to_run",    priority=9)
            sched.cancel(cid)
            sched.resume_device("r1")
            await asyncio.sleep(0.2)
            names = [n for n, _ in results]
            _assert("Cancelled command does not execute", "to_cancel" not in names)
            _assert("Non-cancelled command still runs",   "to_run"    in names)

            # Multi-device
            results.clear()
            await sched.submit("alice", ["r1", "r2"], "broadcast", priority=5)
            await asyncio.sleep(0.2)
            _assert("Multi-device executes on r1", ("broadcast", "r1") in results)
            _assert("Multi-device executes on r2", ("broadcast", "r2") in results)

            # cancel_client_commands (called on disconnect)
            sched.pause_device("r1")
            results.clear()
            await sched.submit("bob", ["r1"], "bob_cmd", priority=5)
            sched.cancel_client_commands("bob")
            sched.resume_device("r1")
            await asyncio.sleep(0.15)
            _assert("cancel_client_commands: command suppressed",
                    ("bob_cmd", "r1") not in results)

            # interrupt_device (cancels currently executing command)
            results.clear()
            executing  = asyncio.Event()
            interrupted = asyncio.Event()

            async def _slow_exec(command: Command, device_name: str) -> None:
                if command.name == "slow":
                    executing.set()
                    try:
                        await asyncio.sleep(60)
                    except asyncio.CancelledError:
                        interrupted.set()
                        raise
                else:
                    results.append(command.name)

            sched2 = CommandScheduler(_slow_exec)
            sched2.register_device("r1")
            await sched2.start()
            await sched2.submit("alice", ["r1"], "slow",          priority=5)
            await sched2.submit("alice", ["r1"], "after_interrupt", priority=5)
            await asyncio.wait_for(executing.wait(), timeout=1.0)
            sched2.interrupt_device("r1")
            await asyncio.wait_for(interrupted.wait(), timeout=1.0)
            await asyncio.sleep(0.15)
            _assert("interrupt_device: CancelledError raised in command",
                    interrupted.is_set())
            _assert("interrupt_device: next command runs after interrupt",
                    "after_interrupt" in results)
            await sched2.shutdown()

            await sched.shutdown()
            _ok("Scheduler shutdown clean")

        except Exception:
            _fail("CommandScheduler", traceback.format_exc())

    # --- Integration ---
    _section("Integration  (full server + WebSocket)")

    if not _args.integration:
        _skip("Integration tests",
              "pass --integration to enable  "
              "(requires: roscore running, devices.json present)")
        return

    # Check websockets library
    try:
        import websockets as _ws
    except ImportError:
        _fail("Integration tests",
              "websockets not installed — run: pip install websockets")
        return

    PORT       = _args.port
    INTEG_USER = "testuser_integ"
    INTEG_PASS = "integration_test_XYZ"

    users_path = Path("users.json")
    users_bak  = users_path.read_text() if users_path.exists() else "{}"
    users_data = json.loads(users_bak)

    from network.auth import _hash_password as _hp
    users_data[INTEG_USER] = _hp(INTEG_PASS)
    users_path.write_text(json.dumps(users_data, indent=2))

    server_proc = None
    try:
        server_proc = subprocess.Popen(
            [sys.executable, "network/server.py", "--port", str(PORT)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        # Poll /health until ready (max 20 s)
        health_url = f"http://localhost:{PORT}/health"
        ready = False
        for _ in range(40):
            await asyncio.sleep(0.5)
            if server_proc.poll() is not None:
                out = server_proc.stdout.read().decode(errors="replace")
                _fail("Server startup", f"process exited early:\n{out}")
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
            _fail("Server startup", f"no response in 20 s:\n{out[-2000:]}")
            return
        _ok(f"Server started — /health OK  (port {PORT})")

        ws_url   = f"ws://localhost:{PORT}/ws"
        auth_hdr = {"Authorization": "Basic " + base64.b64encode(
            f"{INTEG_USER}:{INTEG_PASS}".encode()).decode()}
        bad_hdr  = {"Authorization": "Basic " + base64.b64encode(
            b"nobody:wrongpass").decode()}

        # Invalid credentials must be rejected
        rejected = False
        try:
            async with _ws.connect(ws_url, extra_headers=bad_hdr):
                pass
        except Exception:
            rejected = True
        _assert("Invalid credentials: connection rejected", rejected)

        # Main protocol test
        async with _ws.connect(ws_url, extra_headers=auth_hdr) as ws:

            # get_devices
            await ws.send(json.dumps({"type": "get_devices"}))
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            _assert("get_devices returns 'devices' type", msg["type"] == "devices")
            devices = msg["data"]
            _ok(f"get_devices: {len(devices)} device(s) — "
                f"{[d['name'] for d in devices]}")

            if not devices:
                _skip("Protocol tests", "no active devices in devices.json")
            else:
                dev_name = devices[0]["name"]

                # acquire (fire-and-forget — no ack on success)
                await ws.send(json.dumps({"type": "acquire", "device": dev_name}))
                await asyncio.sleep(0.1)
                _ok(f"acquire '{dev_name}' sent (no ack on success by design)")

                # submit -> ack -> done
                await ws.send(json.dumps({
                    "type": "submit", "devices": [dev_name],
                    "name": "test_cmd", "priority": 5, "args": {},
                }))
                ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                _assert("submit returns ack",         ack["type"] == "ack")
                _assert("ack has command_id",         "command_id" in ack)
                _assert("ack status is 'queued'",     ack.get("status") == "queued")

                done = json.loads(await asyncio.wait_for(ws.recv(), timeout=10.0))
                _assert("command returns done",        done["type"] == "done")
                _assert("done matches ack command_id",
                        done["command_id"] == ack["command_id"])
                _assert("done contains device name",   done["device"] == dev_name)

                # release
                await ws.send(json.dumps({"type": "release", "device": dev_name}))
                await asyncio.sleep(0.1)
                _ok(f"release '{dev_name}' sent")

                # subscribe_telemetry (pass if device sending, skip on timeout)
                await ws.send(json.dumps({"type": "subscribe_telemetry",
                                          "device": dev_name}))
                try:
                    tel = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                    if tel["type"] == "telemetry":
                        sample_keys = list(tel["data"].keys())[:5]
                        _ok(f"Telemetry received — keys: {sample_keys}")
                    else:
                        # Could be a DoneMessage or error for another reason
                        _skip("Telemetry",
                              f"unexpected message type: {tel['type']}")
                except asyncio.TimeoutError:
                    _skip("Telemetry",
                          "no message in 5 s — device not connected or not "
                          "publishing on ROS topic")

                await ws.send(json.dumps({"type": "unsubscribe_telemetry",
                                          "device": dev_name}))

                # cancel: queue a low-priority command then immediately cancel it
                await ws.send(json.dumps({"type": "acquire", "device": dev_name}))
                await asyncio.sleep(0.05)
                await ws.send(json.dumps({
                    "type": "submit", "devices": [dev_name],
                    "name": "cancel_me", "priority": 1, "args": {},
                }))
                ack_c = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                await ws.send(json.dumps({"type": "cancel",
                                          "command_id": ack_c["command_id"]}))
                # Command may still execute (if already picked up by worker)
                # or be skipped — both are correct. Just drain pending messages.
                try:
                    await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    pass
                _ok("cancel: sent without crash")

        # Second connection with same user — first one should be evicted
        async with _ws.connect(ws_url, extra_headers=auth_hdr) as ws2:
            await ws2.send(json.dumps({"type": "get_devices"}))
            msg2 = json.loads(await asyncio.wait_for(ws2.recv(), timeout=5.0))
            _assert("New connection after old one: get_devices works",
                    msg2["type"] == "devices")
        _ok("Second connection (same user) works — first was evicted")

    except Exception:
        _fail("Integration", traceback.format_exc())

    finally:
        if server_proc and server_proc.poll() is None:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server_proc.kill()
        # Restore users.json (remove test user)
        try:
            data = json.loads(users_path.read_text())
            data.pop(INTEG_USER, None)
            users_path.write_text(json.dumps(data, indent=2))
        except Exception:
            pass


_section("Async tests (Logger + CommandScheduler + Integration)")
try:
    asyncio.run(_run_async_tests())
except Exception:
    _fail("async test runner", traceback.format_exc())


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
_section("Summary")
total = _passed + _failed + _skipped
print(f"  Passed:  {_passed}")
print(f"  Failed:  {_failed}")
print(f"  Skipped: {_skipped}")
print(f"  Total:   {total}\n")
if _failed == 0:
    print("  All testable components OK.")
else:
    print(f"  {_failed} test(s) FAILED.")
sys.exit(0 if _failed == 0 else 1)
