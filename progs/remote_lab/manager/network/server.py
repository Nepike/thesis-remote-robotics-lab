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
from contextlib import asynccontextmanager
from typing import Optional

import uvicorn
from fastapi import FastAPI

from manager import RemoteLabManager
from network import ws_handler
from network.auth import init_user_store


_BASE_DIR     = Path(__file__).parent.parent  # manager/
_DEVICES_JSON = _BASE_DIR / "devices.json"
_USERS_JSON   = _BASE_DIR / "users.json"


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

    # Fail fast if config files are missing
    for path in (_DEVICES_JSON, _USERS_JSON):
        if not path.exists():
            raise FileNotFoundError(f"Required config file not found: {path}")

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
