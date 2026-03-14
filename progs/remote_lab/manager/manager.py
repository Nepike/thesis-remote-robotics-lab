import asyncio

from device_supervisor import DeviceSupervisor
from ros_interface import RosInterface
from access_controller import AccessController


class RemoteLabManager:
    def __init__(self):
        self.device_supervisor = DeviceSupervisor(
            config_path="devices.json",
            tty_root_path="/tmp/remote_lab_tty"
        )

        self.ros = RosInterface()
        self.access = AccessController()

    async def start(self):
        self.device_supervisor.load_devices()

        for device_proc in self.device_supervisor.devices.values():
            device = device_proc.device
            self.access.register_device(
                device_name=device.name,
                shared=device.shared
            )
        await self.device_supervisor.start_all()

        print("[Manager] started")



    async def shutdown(self):
        await self.device_supervisor.stop_all()
        self.ros.shutdown()
        print("[Manager] stopped")


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
