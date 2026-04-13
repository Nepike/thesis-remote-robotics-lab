import asyncio
import time
from typing import List, Optional, Dict, Callable, Tuple, Awaitable
from pathlib import Path

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

    # TODO make iterable make strict typization add override for driver overrides
    class _DeviceProcesses:
        """
        An auxiliary class for storing processes
        """
        def __init__(self,
                     transport_starter: Callable[[], Awaitable[Tuple[asyncio.subprocess.Process, ...]]],
                     adapter_starter: Callable[[], Awaitable[Tuple[asyncio.subprocess.Process, ...]]]):
            self._transport_starter = transport_starter
            self._adapter_starter = adapter_starter
            self.processes: List[asyncio.subprocess.Process] = []

        async def start(self):
            transport_procs = await self._transport_starter()
            adapter_procs = await self._adapter_starter()

            self.processes.extend(transport_procs + adapter_procs)

    def __init__(self):
        self._procs: Dict[Device, List[asyncio.subprocess.Process]] = {}
        self._running = False


    def load_device(self, device: Device, transport_starter: Callable[[], Tuple[asyncio.subprocess.Process, ...]],
                    adapter_starter: Callable[[], Tuple[asyncio.subprocess.Process, ...]]):
        self._processes[device] = list()
        self._starters[device] = DeviceSupervisor._DeviceStarters(transport_starter, adapter_starter)
        logger = Logger.get()
        logger.log("DEVICE_SUPERVISOR", f"Device '{device.name}' loaded")


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