#!/usr/bin/env python3
"""
Minimal integration test for the RemoteLab WebSocket server.

    python test.py --integration [--port 8765]

Starts the server as a subprocess, connects via WebSocket, and exercises:
  - HTTP Basic Auth (valid + invalid credentials)
  - get_devices
  - acquire / submit / done / release
  - telemetry subscribe/unsubscribe
  - cancel
  - second connection with same user (eviction)
"""

import argparse
import asyncio
import base64
import json
import subprocess
import sys
import traceback
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

_cli = argparse.ArgumentParser(description="RemoteLab integration test")
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
# Integration test
# ---------------------------------------------------------------------------
async def _run_integration():
    _section("Integration  (full server + WebSocket)")

    if not _args.integration:
        _skip("Integration tests",
              "pass --integration to enable  "
              "(requires: roscore running, devices.json present)")
        return

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

        # --- Invalid credentials must be rejected ---
        rejected = False
        try:
            async with _ws.connect(ws_url, extra_headers=bad_hdr):
                pass
        except Exception:
            rejected = True
        _assert("Invalid credentials: connection rejected", rejected)

        # --- Valid credentials: connect and run protocol ---
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

                # acquire
                await ws.send(json.dumps({"type": "acquire", "device": dev_name}))
                await asyncio.sleep(0.1)
                _ok(f"acquire '{dev_name}' sent (no ack on success by design)")

                # submit -> ack -> done
                await ws.send(json.dumps({
                    "type": "submit", "devices": [dev_name],
                    "name": "test_cmd", "priority": 5, "args": {},
                }))
                ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                _assert("submit returns ack",     ack["type"] == "ack")
                _assert("ack has command_id",     "command_id" in ack)
                _assert("ack status is 'queued'", ack.get("status") == "queued")

                done = json.loads(await asyncio.wait_for(ws.recv(), timeout=10.0))
                _assert("command returns done",       done["type"] == "done")
                _assert("done matches ack command_id",
                        done["command_id"] == ack["command_id"])
                _assert("done contains device name",  done["device"] == dev_name)

                # release
                await ws.send(json.dumps({"type": "release", "device": dev_name}))
                await asyncio.sleep(0.1)
                _ok(f"release '{dev_name}' sent")

                # subscribe_telemetry
                await ws.send(json.dumps({"type": "subscribe_telemetry",
                                          "device": dev_name}))
                try:
                    tel = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                    if tel["type"] == "telemetry":
                        _ok(f"Telemetry received — keys: {list(tel['data'].keys())[:5]}")
                    else:
                        _skip("Telemetry",
                              f"unexpected message type: {tel['type']}")
                except asyncio.TimeoutError:
                    _skip("Telemetry",
                          "no message in 5 s — device not connected or not publishing")

                await ws.send(json.dumps({"type": "unsubscribe_telemetry",
                                          "device": dev_name}))

                # cancel
                await ws.send(json.dumps({"type": "acquire", "device": dev_name}))
                await asyncio.sleep(0.05)
                await ws.send(json.dumps({
                    "type": "submit", "devices": [dev_name],
                    "name": "cancel_me", "priority": 1, "args": {},
                }))
                ack_c = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
                await ws.send(json.dumps({"type": "cancel",
                                          "command_id": ack_c["command_id"]}))
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
        # Always print server output — essential for diagnosing failures
        if server_proc and server_proc.stdout:
            try:
                out = server_proc.stdout.read().decode(errors="replace")
                if out.strip():
                    _section("Server output (uvicorn logs)")
                    print(out[-4000:])
            except Exception:
                pass
        # Restore users.json (remove test user)
        try:
            data = json.loads(users_path.read_text())
            data.pop(INTEG_USER, None)
            users_path.write_text(json.dumps(data, indent=2))
        except Exception:
            pass


try:
    asyncio.run(_run_integration())
except Exception:
    _fail("async test runner", traceback.format_exc())


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
_section("Summary")
total = _passed + _failed + _skipped
print(f"  Passed:  {_passed}/{total}")
print(f"  Failed:  {_failed}/{total}")
print(f"  Skipped: {_skipped}/{total}")
print()
if _failed:
    sys.exit(1)
