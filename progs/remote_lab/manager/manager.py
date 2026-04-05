import asyncio

from DeviceSupervisor import DeviceSupervisor



class RemoteLabManager:
    def __init__(self):
        self.device_supervisor = DeviceSupervisor(config_path="../devices.json")

    async def start(self):
        self.device_supervisor.load_devices()
        await self.device_supervisor.run()

        print("[Manager] started")


    async def shutdown(self):
        await self.device_supervisor.shutdown()
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
