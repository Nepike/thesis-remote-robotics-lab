import asyncio
from device_supervisor import DeviceSupervisor
from ros_interface import RosInterface


class RemoteLabManager:
    def __init__(self):
        self.device_supervisor = DeviceSupervisor("devices.json","/tmp/remotelab_tty")
        self.ros = RosInterface()

    async def start(self):
        self.device_supervisor.load_devices()
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