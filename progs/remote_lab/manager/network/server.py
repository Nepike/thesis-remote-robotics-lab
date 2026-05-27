"""
FastAPI application entry point.

Run from the manager/ directory:
    python network/server.py
    python network/server.py --host 0.0.0.0 --port 8000

Or via uvicorn directly (same effect):
    uvicorn network.server:app --host 0.0.0.0 --port 8000

Note: do not use uvicorn --workers N > 1. All state lives in one process.
"""

import sys
from pathlib import Path

# Add manager/ to sys.path so that manager-level modules (BasicClasses, manager, DeviceDrivers, etc.)
# can be imported from inside the network/ package.
# Must be done before any project imports.
sys.path.insert(0, str(Path(__file__).parent.parent))

import argparse
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI

from manager import RemoteLabManager
from network import ws_handler
from network.auth import init_user_store


_BASE_DIR     = Path(__file__).parent.parent  # manager/
_DEVICES_JSON = _BASE_DIR / "devices.json"
_USERS_JSON   = _BASE_DIR / "users.json"


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
