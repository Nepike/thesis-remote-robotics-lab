import asyncio
from datetime import datetime
from typing import Optional


class Logger:
    _instance: Optional["Logger"] = None

    def __init__(self):
        self._lock = asyncio.Lock()

    @classmethod
    def get(cls) -> "Logger":
        if cls._instance is None:
            cls._instance = Logger()
            cls._instance.log("LOGGER", "Logger singleton created")
        return cls._instance

    async def log(self, prefix: str, message: str):
        async with self._lock:
            now = datetime.now().strftime("%H:%M:%S")
            print(f"[{now}] [{prefix}] {message}")

    def attach_stream(self, prefix: str, stream: asyncio.StreamReader):
        asyncio.create_task(self._log_stream(prefix, stream))

    async def _log_stream(self, prefix: str, stream: asyncio.StreamReader):
        while True:
            line = await stream.readline()
            if not line:
                break

            await self.log(prefix, line.decode(errors="ignore").rstrip())