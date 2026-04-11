import asyncio
import time
from typing import List, Optional, Dict, Callable
from pathlib import Path
import json
import os

from BasicClasses import Device
from Logger import Logger

#
# class DeviceInstance:
#     """
#     Bonds the device and required processes.
#
#     transport_proc: socat - emulates a wired connection
#     adapter_proc: protocol-required process (e.g. roslaunch)
#     """
#     def __init__(self, device: Device):
#         self.device = device
#
#         self.transport_proc: Optional[asyncio.subprocess.Process] = None
#         self.adapter_proc: Optional[asyncio.subprocess.Process] = None
#
#         self._restart_lock = asyncio.Lock()
#
#
#
#
#     async def start(self):
#         await self._start_transport_proc()
#         await self._start_adapter_proc()
#
#     async def stop(self):
#         for proc in [self.transport_proc, self.adapter_proc]:
#             if proc and proc.returncode is None:
#                 proc.terminate()
#                 try:
#                     await asyncio.wait_for(proc.wait(), timeout=5)
#                 except asyncio.TimeoutError:
#                     proc.kill()
#
#     async def restart(self):
#         async with self._restart_lock:
#             await self.stop()
#             await asyncio.sleep(1)
#             await self.start()


# TODO redo with drivers or smth
class DeviceSupervisor:
    """
    Provides methods for monitoring devices' processes
    """
    class _DeviceProcess:
        """
        Just an auxiliary class for storing additional processes info
        """
        def __init__(self, name: str, process: asyncio.subprocess.Process):
            self.name = name




    def __init__(self):
        self._procs: Dict[Device, List[DeviceSupervisor._DeviceProcess]] = {}
        self._running = False


    def load_device(self, device: Device):
        self._procs[device] = list()
        logger = Logger.get()
        logger.log("DEVICE_SUPERVISOR", f"Device '{device.name}' loaded")


    def load_from_config(self, config_path: Path):
        with open(config_path, "r") as f:
            raw = json.load(f)

        try:
            for item in raw:
                device = Device(
                    name=item["name"],
                    ip=item["ip"],
                    port=item["port"],
                    driver=item["driver"],
                    ros_namespace=item.get("ros_namespace", None),
                    baud_rate=item.get("baud_rate", None),
                    shared=item["shared"],
                    active=item["active"],
                )

                if not device.active:
                    continue
                self.load_device(device)

        except KeyError as e:
            raise ValueError(f"Config file doesn't fit required structure (some fields are missing): {e})")


    async def _watchdog_loop(self):
        while self.running:
            for device_instance in self._devices:

                if device_instance.transport_proc and device_instance.transport_proc.returncode is not None:
                    print(f"[DeviceSupervisor] transport is dead for {device_instance.device.name}, restarting...")
                    asyncio.create_task(device_instance.restart())
                    continue

                if device_instance.adapter_proc and device_instance.adapter_proc.returncode is not None:
                    print(f"[DeviceSupervisor] adapter is dead for {device_instance.device.name}, restarting...")
                    asyncio.create_task(device_instance.restart())
                    continue

            await asyncio.sleep(2)


    async def run(self):
        self.running = True
        for device_instance in self._devices:
            print(f"[DeviceSupervisor] Starting {device_instance.device.name}...")
            await device_instance.start()
        asyncio.create_task(self._watchdog_loop())


    async def shutdown(self):
        print("[DeviceSupervisor] Shutting down ...")
        self.running = False

        for device_instance in self._devices:
            await device_instance.stop()