import asyncio
import os
import json
import tempfile
import subprocess
import pty

from channels.generic.websocket import AsyncWebsocketConsumer
from django.conf import settings


SSH_HOST = "inbicst.ru"
SSH_PORT = 22


class TerminalConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        user = self.scope["user"]

        if not user.is_authenticated:
            await self.close()
            return

        if not user.ssh_private_key:
            await self.close()
            return

        self.username = user.username

        # 1. временный файл для ключа
        self.key_file = tempfile.NamedTemporaryFile(
            mode="w",
            delete=False,
            prefix="sshkey_",
        )
        self.key_file.write(user.ssh_private_key)
        self.key_file.close()
        os.chmod(self.key_file.name, 0o600)

        # 2. PTY
        self.master_fd, self.slave_fd = pty.openpty()

        # 3. SSH процесс
        self.process = subprocess.Popen(
            [
                "ssh",
                "-i", self.key_file.name,
                "-p", str(SSH_PORT),
                "-o", "StrictHostKeyChecking=no",
                f"{self.username}@{SSH_HOST}",
            ],
            stdin=self.slave_fd,
            stdout=self.slave_fd,
            stderr=self.slave_fd,
            start_new_session=True,
        )

        os.close(self.slave_fd)

        await self.accept()

        # 4. чтение stdout
        self.reader_task = asyncio.create_task(self._read_from_pty())

    async def receive(self, text_data):
        # resize поддержка
        try:
            data = json.loads(text_data)
            if data.get("type") == "resize":
                return
        except json.JSONDecodeError:
            pass

        os.write(self.master_fd, text_data.encode())

    async def _read_from_pty(self):
        try:
            while True:
                data = os.read(self.master_fd, 1024)
                if not data:
                    break
                await self.send(text_data=data.decode(errors="ignore"))
        except Exception:
            pass
        finally:
            await self.close()

    async def disconnect(self, close_code):
        try:
            self.process.terminate()
        except Exception:
            pass

        try:
            os.close(self.master_fd)
        except Exception:
            pass

        try:
            os.unlink(self.key_file.name)
        except Exception:
            pass
