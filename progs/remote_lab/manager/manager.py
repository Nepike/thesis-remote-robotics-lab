import asyncio
import json
from pathlib import Path
from typing import Awaitable, Callable, Dict, List, Optional

from AccessController import AccessController
from BasicClasses import Command, Device
from CommandScheduler import CommandScheduler
from DeviceDrivers import AbstractDriver, DriverFactory
from DeviceSupervisor import DeviceSupervisor
from HardwareInterfaces import RosInterface, SerialInterface
from Logger import Logger


class RemoteLabManager:
    """
    Top-level orchestrator.  Owns all subsystems and wires them together.

    Startup sequence (must be respected):
        manager = RemoteLabManager()
        await manager.load_config()   # parse devices.json, create drivers
        await manager.start()         # bring devices up, then start scheduler workers

    Public API (called by the network layer once it exists):
        submit_command / acquire_device / release_device / on_client_disconnect
        get_devices / get_driver
    """

    def __init__(self):
        # Hardware interfaces - shared across all drivers of the same type
        self._ros    = RosInterface()
        self._serial = SerialInterface()

        self._driver_factory     = DriverFactory(self._ros, self._serial)
        self._supervisor         = DeviceSupervisor()
        self._scheduler          = CommandScheduler(self._on_execute_command)
        self._access_controller  = AccessController()

        # device_name -> driver / Device (populated by load_config)
        self._drivers: Dict[str, AbstractDriver] = {}
        self._devices: Dict[str, Device]         = {}

        # Wired by the network layer after startup.
        # Called with (command, device_name) after every execute_command() returns.
        self.on_command_complete: Optional[Callable[[Command, str], Awaitable[None]]] = None


    async def load_config(self, config_path: Path = Path("./devices.json")):
        """
        Parse devices.json and register every active device with all subsystems.
        Must be called before start().
        """
        with open(config_path, "r") as f:
            raw = json.load(f)

        try:
            for item in raw:
                device = Device(
                    name=item["name"],
                    ip=item["ip"],
                    port=item["port"],
                    driver=item["driver"],
                    ros_namespace=item.get("ros_namespace"),
                    baud_rate=item.get("baud_rate"),
                    shared=item["shared"],
                    active=item["active"],
                )

                if not device.active:
                    continue

                driver = self._driver_factory.create_driver(device)

                self._devices[device.name] = device
                self._drivers[device.name] = driver

                # Each subsystem gets its own registration call
                self._supervisor.load_device(device, driver)
                self._scheduler.register_device(device.name)
                self._access_controller.register_device(device.name, device.shared)

        except KeyError as e:
            raise ValueError(f"devices.json is missing required field: {e}")

    async def start(self):
        logger = Logger.get()

        # Wire supervisor lifecycle hooks to scheduler pause/resume
        self._supervisor.on_device_down = self._scheduler.pause_device
        self._supervisor.on_device_up   = self._scheduler.resume_device

        # Bring all devices up first, then start accepting commands
        await self._supervisor.run()
        await self._scheduler.start()

        await logger.log("MANAGER", "Started")

    async def shutdown(self):
        logger = Logger.get()
        await logger.log("MANAGER", "Shutting down...")

        # Stop accepting and executing commands first
        await self._scheduler.shutdown()
        # Then tear down device processes
        await self._supervisor.shutdown()
        # Finally release ROS resources
        self._ros.shutdown()

        await logger.log("MANAGER", "Shutdown complete")


    async def _on_execute_command(self, command: Command, device_name: str):
        driver = self._drivers.get(device_name)
        if driver is None:
            raise RuntimeError(f"No driver registered for '{device_name}'")
        await driver.execute_command(command)
        if self.on_command_complete:
            await self.on_command_complete(command, device_name)


    async def submit_command(self, client_id: str, devices: List[str], command_name: str, priority: int, args: Optional[dict] = None) -> str:
        """
        Validate access and enqueue a command.

        Returns command_id. Raises PermissionError if the client does not have
        access to one or more of the requested devices.
        """
        for device_name in devices:
            if not self._access_controller.check_access(device_name, client_id):
                raise PermissionError(
                    f"Client '{client_id}' does not have access to '{device_name}'. "
                    f"Call acquire_device() first."
                )

        return await self._scheduler.submit(client_id, devices, command_name, priority, args)

    def acquire_device(self, device_name: str, client_id: str) -> bool:
        """
        Attempt to acquire exclusive ownership of a device.
        Always succeeds for shared devices. Returns False if the device is already owned by another client.
        """
        return self._access_controller.acquire(device_name, client_id)

    def release_device(self, device_name: str, client_id: str):
        """Release exclusive ownership of a single device."""
        self._access_controller.release(device_name, client_id)

    def on_client_disconnect(self, client_id: str):
        """
        Clean up all state associated with a disconnected client:
        cancels pending commands and releases exclusive device locks.
        """
        self._scheduler.cancel_client_commands(client_id)
        self._access_controller.release_all(client_id)

    def get_devices(self) -> List[Device]:
        """Return a list of all registered active devices."""
        return list(self._devices.values())

    def get_driver(self, device_name: str) -> Optional[AbstractDriver]:
        """Return the driver for a device, or None if not found."""
        return self._drivers.get(device_name)

    def cancel_command(self, command_id: str):
        """Cancel a specific queued command. No effect if already executing."""
        self._scheduler.cancel(command_id)


async def main():
    manager = RemoteLabManager()
    await manager.load_config()
    await manager.start()

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        await manager.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
