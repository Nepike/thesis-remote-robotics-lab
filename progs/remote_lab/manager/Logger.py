import asyncio
from datetime import datetime
from pathlib import Path
from typing import Optional, TextIO


class Logger:
    _instance: Optional["Logger"] = None

    # Subprocess output (socat, rosserial, ...) is written here instead of the
    # console, so high-level events stay readable. Directory lives next to the
    # manager code regardless of the current working directory.
    _LOG_DIR = Path(__file__).parent / "logs"

    def __init__(self):
        self._lock = asyncio.Lock()
        self._LOG_DIR.mkdir(parents=True, exist_ok=True)

    @classmethod
    def get(cls) -> "Logger":
        if cls._instance is None:
            cls._instance = Logger()
        return cls._instance

    async def log(self, prefix: str, message: str):
        """High-level event log — goes to the console."""
        async with self._lock:
            now = datetime.now().strftime("%H:%M:%S")
            print(f"[{now}] [{prefix}] {message}")

    def attach_stream(self, prefix: str, stream: asyncio.StreamReader):
        """
        Pipe a subprocess output stream into logs/<prefix>.log (NOT the console).
        Keeps noisy socat / rosserial output off-screen but available on disk.
        """
        asyncio.create_task(self._log_stream(prefix, stream))

    async def _log_stream(self, prefix: str, stream: asyncio.StreamReader):
        log_path = self._LOG_DIR / f"{prefix}.log"
        f: TextIO = open(log_path, "a", buffering=1)  # line-buffered
        try:
            while True:
                line = await stream.readline()
                if not line:
                    break
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"[{now}] {line.decode(errors='ignore').rstrip()}\n")
        finally:
            f.close()
