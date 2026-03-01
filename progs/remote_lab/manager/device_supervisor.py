import asyncio
from typing import Dict, Optional
from pathlib import Path
import json
import os
from dataclasses import dataclass


@dataclass
class Device:
    name: str
    ip: str
    port: int
    type: str # IDEA: enum?
    namespace: str
    shared: bool
    active: bool


class DeviceProcess:
    def __init__(self, device: Device):
        self.device = device
        self.socat_proc: Optional[asyncio.subprocess.Process] = None
        self.rosserial_proc: Optional[asyncio.subprocess.Process] = None

    @property
    def tty_path(self):
        return f"/tmp/remote_lab_tty/ttyDEVICE-{self.device.name}"


class DeviceSupervisor:
    def __init__(self, config_path: str):
        self.config_path = Path(config_path)
        self.devices: Dict[str, DeviceProcess] = {}
        self.running = False

    def load_devices(self):
        with open(self.config_path, "r") as f:
            raw = json.load(f)

        # IDEA: not load if not active?
        for item in raw:
            device = Device(
                name=item["name"],
                ip=item["ip"],
                port=item["port"],
                type=item["type"],
                namespace=item["namespace"],
                shared=item["shared"],
                active=item["active"],
            )
            self.devices[device.name] = DeviceProcess(device)
            print(f"[DeviceSupervisor] Loaded {device.name}")

    async def __start_device(self, device_proc: DeviceProcess):
        device = device_proc.device

        # TODO: normal logging...
        print(f"[DeviceSupervisor] Starting {device.name} ...")

        if os.path.exists(device_proc.tty_path):
            os.remove(device_proc.tty_path)

        # socat pty,link=/tmp/remote_lab_tty/ttyDEVICE-Test-yarp13,raw,echo=0,waitslave,mode=666 tcp:192.168.0.101:2000
        device_proc.socat_proc = await asyncio.create_subprocess_exec(
            "socat",
            f"pty,link={device_proc.tty_path},raw,echo=0,waitslave,mode=666",
            f"tcp:{device.ip}:{device.port}",
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL
        )
        await asyncio.sleep(1) # God help us

        # roslaunch yyctl rosserial.launch port:=/dev/ttyESP32
        cmd = (
            f"source /opt/ros/noetic/setup.bash && "
            f"source {Path.home()}/ros/devel/setup.bash && "
            f"roslaunch yyctl rosserial.launch "
            f"port:={device_proc.tty_path} "
            f"__ns:={device.namespace}"
        )
        device_proc.rosserial_proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL
        )
        await asyncio.sleep(1)  # God help us 2

    async def __stop_device(self, device_proc: DeviceProcess):
        for proc in [device_proc.socat_proc, device_proc.rosserial_proc]:
            if proc and proc.returncode is None:
                proc.terminate()
                try:
                    await asyncio.wait_for(proc.wait(), timeout=5)
                except asyncio.TimeoutError:
                    proc.kill()

        device = device_proc.device
        print(f"[DeviceSupervisor] Stopped {device.name}")

    async def __restart_device(self, device_proc: DeviceProcess):
        await self.__stop_device(device_proc)
        await asyncio.sleep(1)
        await self.__start_device(device_proc)

    async def __watchdog_loop(self):
        while self.running:
            for device_proc in self.devices.values():
                # socat
                if device_proc.socat_proc and device_proc.socat_proc.returncode is not None:
                    print(f"[DeviceSupervisor] socat died for {device_proc.device.name}, restarting...")
                    await self.__restart_device(device_proc)
                # rosserial
                if device_proc.rosserial_proc and device_proc.rosserial_proc.returncode is not None:
                    print(f"[DeviceSupervisor] rosserial died for {device_proc.device.name}, restarting...")
                    await self.__restart_device(device_proc)

            await asyncio.sleep(10)

    async def start_all(self):
        self.running = True
        for device_proc in self.devices.values():
            await self.__start_device(device_proc)
        asyncio.create_task(self.__watchdog_loop())

    async def stop_all(self):
        print("[DeviceSupervisor] Shutting down ...")
        self.running = False

        for device_proc in self.devices.values():
            await self.__stop_device(device_proc)