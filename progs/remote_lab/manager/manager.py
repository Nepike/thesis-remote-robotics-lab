import asyncio
from pathlib import Path
import json

from BasicClasses import Device
from HardwareInterfaces import RosInterface, SerialInterface
from DeviceDrivers import DriverFactory
from DeviceSupervisor import DeviceSupervisor



class RemoteLabManager:
    def __init__(self):
        self.ros_interface = RosInterface()
        self.serial_interface = SerialInterface()

        self.driver_factory: DriverFactory = DriverFactory(self.ros_interface, self.serial_interface)

        self.device_supervisor = DeviceSupervisor()

    async def load_config(self, config_path:Path = Path("./devices.json")):
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

                # TODO

        except KeyError as e:
            raise ValueError(f"Config file doesn't fit required structure (some fields are missing): {e})")

    # async def start(self):
    #     self.device_supervisor.load_devices()
    #     await self.device_supervisor.run()
    #
    #     print("[Manager] started")
    #
    #
    # async def shutdown(self):
    #     await self.device_supervisor.shutdown()
    #     print("[Manager] stopped")


async def main():
    manager = RemoteLabManager()
    await manager.start()

    try:
        while True:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        await manager.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
