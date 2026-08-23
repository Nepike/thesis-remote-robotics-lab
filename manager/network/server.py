"""
FastAPI application entry point.

Run from the manager/ directory:
    python network/server.py
    python network/server.py --host 0.0.0.0 --port 8000

Or via uvicorn directly (same effect):
    uvicorn network.server:app --host 0.0.0.0 --port 8000

Note: do not use uvicorn --workers N > 1. All state lives in one process.
"""

import socket
import sys
from pathlib import Path

# Add manager/ to sys.path so that manager-level modules (BasicClasses, manager, DeviceDrivers, etc.)
# can be imported from inside the network/ package.
# Must be done before any project imports.
sys.path.insert(0, str(Path(__file__).parent.parent))

import argparse
import asyncio
import json
from contextlib import asynccontextmanager
from typing import Callable, Optional

import uvicorn
from fastapi import FastAPI

from manager import RemoteLabManager
from network import ws_handler
from network.auth import init_user_store


_BASE_DIR     = Path(__file__).parent.parent  # manager/
_DEVICES_JSON = _BASE_DIR / "devices.json"
_USERS_JSON   = _BASE_DIR / "users.json"


# devices.json / users.json are gitignored (they hold per-deployment data that
# must survive code updates). If they are absent on a fresh checkout, the server
# writes these templates so the operator sees the expected schema, then stops
# because an empty config has nothing to serve.
_DEVICES_TEMPLATE = [
    {
        "name": "example-robot",
        "ip": "192.168.0.100",
        "port": 2000,
        "driver": "yarp13",
        "ros_namespace": "robot1/example",
        "baud_rate": None,
        "shared": True,
        "active": False,
    }
]

_USERS_TEMPLATE = {
    "_note": "Manage users with: python network/auth.py register|delete|list"
}


def _devices_usable(data) -> bool:
    """Usable once the config lists at least one ACTIVE device."""
    return isinstance(data, list) and any(
        isinstance(d, dict) and d.get("active") for d in data
    )


def _users_usable(data) -> bool:
    """Usable once the config has at least one real (non-comment) user."""
    return isinstance(data, dict) and any(not k.startswith("_") for k in data)


def _ensure_config(path: Path, template, usable: Callable[[object], bool]) -> bool:
    """
    Create `path` from `template` if missing. Return True if the config is usable
    (non-empty), False otherwise — missing-then-created, empty, or unreadable —
    printing the reason in each case.
    """
    created = False
    if not path.exists():
        path.write_text(json.dumps(template, indent=2) + "\n", encoding="utf-8")
        created = True
        print(f"[server] Created template config: {path}")

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as e:
        print(f"[server] Cannot read {path.name}: {e}")
        return False

    if not usable(data):
        print(f"[server] {path.name} {'was just created and is empty' if created else 'is empty'}.")
        return False
    return True


def _roscore_running(uri: str = "http://localhost:11311") -> bool:
    """Return True if a ROS master is already accepting connections."""
    try:
        host, port = uri.replace("http://", "").rsplit(":", 1)
        with socket.create_connection((host, int(port)), timeout=1):
            return True
    except OSError:
        return False


async def _start_roscore() -> Optional[asyncio.subprocess.Process]:
    """
    Start roscore if no ROS master is running yet.

    Returns the Process object so the caller can terminate it on shutdown,
    or None if roscore was already running externally.
    Raises RuntimeError if roscore fails to come up within 5 seconds.
    """
    if _roscore_running():
        print("[server] roscore already running — skipping auto-start")
        return None

    print("[server] Starting roscore...")
    proc = await asyncio.create_subprocess_exec(
        "roscore",
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
    )

    # Wait up to 5 seconds for the master to become reachable
    for _ in range(50):
        await asyncio.sleep(0.1)
        if _roscore_running():
            print("[server] roscore is up")
            return proc

    proc.terminate()
    raise RuntimeError("roscore did not start within 5 seconds")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Server lifecycle: everything before yield runs on startup,
    everything after yield runs on shutdown.
    FastAPI guarantees the shutdown block runs even on SIGTERM.
    """

    # Bootstrap config: create templates if missing, and refuse to start on an
    # empty configuration (no active devices / no users) — there would be nothing
    # to serve. Both files are gitignored so deployment data survives code updates.
    devices_ok = _ensure_config(_DEVICES_JSON, _DEVICES_TEMPLATE, _devices_usable)
    users_ok   = _ensure_config(_USERS_JSON,   _USERS_TEMPLATE,   _users_usable)
    if not (devices_ok and users_ok):
        print("[server] Empty configuration — nothing to serve. Stopping.")
        print(f"[server]   1. Add at least one active device to {_DEVICES_JSON.name}")
        print(f"[server]   2. Register at least one user:  python network/auth.py register <username>")
        raise RuntimeError("Empty configuration: fill devices.json and users.json, then restart.")

    # Start roscore before rospy.init_node() is called inside RemoteLabManager.__init__()
    roscore_proc = await _start_roscore()

    # Load user credentials before the first connection arrives
    init_user_store(_USERS_JSON)

    # Start the manager: parses devices.json, brings up device processes,
    # starts command scheduler workers
    manager = RemoteLabManager()
    await manager.load_config(_DEVICES_JSON)
    await manager.start()

    # Wire WebSocket layer to the manager
    ws_handler.init(manager)

    # Expose manager on app.state in case future REST endpoints need it
    app.state.manager = manager

    yield  # server is live here

    # Graceful shutdown: drain command queues, stop device processes, release ROS
    await manager.shutdown()

    # Shut down roscore only if we started it
    if roscore_proc and roscore_proc.returncode is None:
        print("[server] Stopping roscore...")
        roscore_proc.terminate()
        try:
            await asyncio.wait_for(roscore_proc.wait(), timeout=5)
        except asyncio.TimeoutError:
            roscore_proc.kill()


app = FastAPI(title="RemoteLab Server", lifespan=lifespan)

app.include_router(ws_handler.router)


@app.get("/health")
async def health():
    """Connectivity check - no auth required."""
    return {"status": "ok"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RemoteLab WebSocket server")
    parser.add_argument("--host",   default="0.0.0.0",  help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port",   default=8000, type=int, help="Bind port (default: 8000)")
    parser.add_argument("--reload", action="store_true",    help="Auto-reload on code changes (dev only)")
    args = parser.parse_args()

    uvicorn.run(
        "network.server:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
    )
