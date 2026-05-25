import asyncio
from typing import Callable, Dict, Optional, Tuple

from BasicClasses import Device
from DeviceDrivers import AbstractDriver
from Logger import Logger


class DeviceSupervisor:
    """
    Monitors device processes (transports and adapters) and restarts them on failure.
    """

    class _DeviceState:
        def __init__(self, driver: AbstractDriver):
            self.driver = driver
            self.transport_procs: Tuple[asyncio.subprocess.Process, ...] = ()
            self.adapter_procs: Tuple[asyncio.subprocess.Process, ...] = ()
            self.restart_lock = asyncio.Lock()

        def all_procs(self) -> Tuple[asyncio.subprocess.Process, ...]:
            return self.transport_procs + self.adapter_procs

        def transports_alive(self) -> bool:
            return bool(self.transport_procs) and all(p.returncode is None for p in self.transport_procs)

        def adapters_alive(self) -> bool:
            # Empty tuple means no adapters required (e.g. SerialBasedDriver) - counts as alive.
            return not self.adapter_procs or all(p.returncode is None for p in self.adapter_procs)

    def __init__(self):
        self._devices: Dict[Device, DeviceSupervisor._DeviceState] = {}
        self._running = False
        self._watchdog_task: Optional[asyncio.Task] = None

        # Optional hooks wired up by RemoteLabManager to integrate with CommandScheduler.
        # Called with device.name when a device goes down / comes back up.
        # Typical usage:
        #   supervisor.on_device_down = scheduler.pause_device
        #   supervisor.on_device_up  = scheduler.resume_device
        self.on_device_down: Optional[Callable[[str], None]] = None
        self.on_device_up:   Optional[Callable[[str], None]] = None

    def load_device(self, device: Device, driver: AbstractDriver):
        self._devices[device] = DeviceSupervisor._DeviceState(driver)

    @staticmethod
    async def _start_device(device: Device, state: _DeviceState):
        logger = Logger.get()
        await logger.log("SUPERVISOR", f"Starting '{device.name}'...")
        state.transport_procs = await state.driver.start_transports()
        state.adapter_procs = await state.driver.start_adapters()
        await state.driver.setup_telemetry()
        await logger.log("SUPERVISOR", f"'{device.name}' started")

    @staticmethod
    async def _stop_procs(procs: Tuple[asyncio.subprocess.Process, ...]):
        for proc in procs:
            if proc.returncode is None:
                proc.terminate()
                try:
                    await asyncio.wait_for(proc.wait(), timeout=5)
                except asyncio.TimeoutError:
                    proc.kill()

    async def run(self):
        if self._running:
            return

        self._running = True

        for device, state in self._devices.items():
            await self._start_device(device, state)

        self._watchdog_task = asyncio.create_task(self._watchdog_loop())

    async def shutdown(self):
        logger = Logger.get()
        await logger.log("SUPERVISOR", "Shutting down...")

        self._running = False

        if self._watchdog_task:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass

        for _, state in self._devices.items():
            await state.driver.teardown_telemetry()
            await self._stop_procs(state.all_procs())

        await logger.log("SUPERVISOR", "Shutdown complete")

    
    async def _restart_device(self, device: Device, state: _DeviceState):
        async with state.restart_lock:
            logger = Logger.get()
            await logger.log("SUPERVISOR", f"Restarting '{device.name}'...")

            # Pause the command worker so no new commands start while the device is down
            if self.on_device_down:
                self.on_device_down(device.name)

            await state.driver.teardown_telemetry()
            await DeviceSupervisor._stop_procs(state.all_procs())
            await asyncio.sleep(1)
            state.transport_procs = await state.driver.start_transports()
            state.adapter_procs = await state.driver.start_adapters()
            await state.driver.setup_telemetry()

            # Device is fully back - allow the command worker to continue
            if self.on_device_up:
                self.on_device_up(device.name)

            await logger.log("SUPERVISOR", f"'{device.name}' restarted")

    async def _watchdog_loop(self):
        logger = Logger.get()
        while self._running:
            for device, state in self._devices.items():
                if state.restart_lock.locked():
                    continue

                if not state.transports_alive() or not state.adapters_alive():
                    await logger.log("SUPERVISOR", f"Process died for '{device.name}', restarting...")
                    asyncio.create_task(self._restart_device(device, state))

            await asyncio.sleep(2)


