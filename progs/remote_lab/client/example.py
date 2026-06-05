import asyncio
import os
import sys
from contextlib import AsyncExitStack

# Поддержка запуска как из корня проекта, так и из папки client/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from client import RemoteLab, AcquireError, ConnectionLostError

# Настройки подключения
SERVER_URL = "localhost:8000"
USERNAME   = "test"
PASSWORD   = "1234"


def on_telemetry(data: dict):
    print(data)


async def main():
    print(f"Connecting to {SERVER_URL} as '{USERNAME}'...")

    try:
        async with RemoteLab(SERVER_URL, USERNAME, PASSWORD) as lab:
            print("Connected.\n")

            devices = await lab.get_devices()
            print(f"Active devices ({len(devices)}):")
            for d in devices:
                shared_tag = "shared" if d["shared"] else "exclusive"
                print(f"  • {d['name']}  driver={d['driver']}  [{shared_tag}]")
            print()

            if not devices:
                print("No active devices.")
                return

            device_names = [d["name"] for d in devices]

            # Телеметрия с первого робота — просто чтобы наблюдать поток
            await lab.device(devices[0]["name"]).subscribe_telemetry(on_telemetry)

            # Захватываем только эксклюзивные устройства (shared не требуют lock)
            async with AsyncExitStack() as stack:
                for d in devices:
                    if not d["shared"]:
                        await stack.enter_async_context(lab.device(d["name"]).lock())

                # 1. Все роботы пищат 1 секунду
                print("group beep (1s)...")
                cmd = await lab.submit(device_names, "beep", priority=5, duration=1.0)
                await cmd

                # 2. Все крутятся влево на месте
                print("group spin LEFT (2s)...")
                cmd = await lab.submit(
                    device_names, "move", priority=5,
                    speed_lin=0.0, speed_ang=1.0, duration=4.0,
                )
                await cmd

                # 3. Все крутятся вправо на месте
                print("group spin RIGHT (2s)...")
                cmd = await lab.submit(
                    device_names, "move", priority=5,
                    speed_lin=0.0, speed_ang=-1.0, duration=4.0,
                )
                await cmd

                print("done.")

    except AcquireError as e:
        print(f"Could not acquire a device: {e}")
        sys.exit(1)
    except ConnectionLostError as e:
        print(f"\nConnection lost: {e}")
        sys.exit(1)
    except OSError as e:
        print(f"\nCould not connect to server ({SERVER_URL}): {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
