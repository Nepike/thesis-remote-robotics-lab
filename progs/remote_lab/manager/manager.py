import asyncio
from device_supervisor import DeviceSupervisor


async def main():
    supervisor = DeviceSupervisor("../devices.json")
    supervisor.load_devices()
    await supervisor.start_all()

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        await supervisor.stop_all()


if __name__ == "__main__":
    asyncio.run(main())
