"""
RemoteLab — пример использования клиентской библиотеки.

Запуск:
    # Из корня проекта (рядом с папкой client/):
    python client/example.py

    # Или напрямую из client/:
    cd client && python example.py

Переменные окружения (опционально):
    RL_URL       — адрес сервера (по умолчанию ws://localhost:8000)
    RL_USER      — логин         (по умолчанию nepike)
    RL_PASSWORD  — пароль        (по умолчанию нужно задать)
"""

import asyncio
import os
import sys

# Поддержка запуска как из корня проекта, так и из папки client/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from client import RemoteLab, AcquireError, ConnectionLostError

# ── Настройки подключения ──────────────────────────────────────────────────────
SERVER_URL = os.environ.get("RL_URL",      "ws://localhost:8000")
USERNAME   = os.environ.get("RL_USER",     "nepike")
PASSWORD   = os.environ.get("RL_PASSWORD", "CHANGE_ME")


# ── Телеметрия-коллбек ────────────────────────────────────────────────────────
def on_telemetry(data: dict):
    print(
        f"  [telemetry]  "
        f"enc_l={data.get('enc_left', '?'):6}  enc_r={data.get('enc_right', '?'):6}  "
        f"spd_l={data.get('speed_left', '?'):4}  spd_r={data.get('speed_right', '?'):4}  "
        f"U={data.get('acc_voltage', '?'):.2f}V"
    )


# ── Основной сценарий ─────────────────────────────────────────────────────────
async def main():
    print(f"Connecting to {SERVER_URL} as '{USERNAME}'...")

    try:
        async with RemoteLab(SERVER_URL, USERNAME, PASSWORD) as lab:
            print("Connected.\n")

            # 1. Список устройств
            devices = await lab.get_devices()
            print(f"Active devices ({len(devices)}):")
            for d in devices:
                shared_tag = "shared" if d["shared"] else "exclusive"
                print(f"  • {d['name']}  driver={d['driver']}  [{shared_tag}]")
            print()

            if not devices:
                print("No active devices. Check devices.json on the server.")
                return

            device_name = devices[0]["name"]
            robot = lab.device(device_name)
            print(f"Using device: '{device_name}'\n")

            # 2. Телеметрия (подписываемся до команд, чтобы сразу видеть данные)
            print("Subscribing to telemetry (5 seconds)...")
            await robot.subscribe_telemetry(on_telemetry)
            await asyncio.sleep(5)
            await robot.unsubscribe_telemetry()
            print()

            # 3. Команды — нужен exclusive lock (если устройство не shared)
            is_shared = devices[0]["shared"]
            if is_shared:
                # shared-устройства не требуют acquire
                await _run_commands(robot)
            else:
                try:
                    async with robot.lock():
                        print("Lock acquired.")
                        await _run_commands(robot)
                    print("Lock released.")
                except AcquireError as e:
                    print(f"Could not acquire lock: {e}")
                    print("(Another client may be using the device — try again later.)")

    except ConnectionLostError as e:
        print(f"\nConnection lost: {e}")
        sys.exit(1)
    except OSError as e:
        print(f"\nCould not connect to server ({SERVER_URL}): {e}")
        print("Make sure the server is running and the address is correct.")
        sys.exit(1)


async def _run_commands(robot):
    print("\n--- Sending commands ---")

    # Бип 0.3 с — ждём завершения
    print("beep (0.3s)...")
    cmd = await robot.submit("beep", priority=5, duration=0.3)
    await cmd
    print("  done.")

    await asyncio.sleep(0.5)

    # Движение вперёд 1 с — ждём
    print("move forward (1.0s)...")
    cmd = await robot.submit("move", priority=5, speed_lin=0.4, speed_ang=0.0, duration=1.0)
    await cmd
    print("  done.")

    await asyncio.sleep(0.3)

    # Стоп (мгновенно)
    print("stop...")
    await robot.submit("stop", priority=999)
    print("  sent.")

    print("--- Done ---\n")


if __name__ == "__main__":
    asyncio.run(main())
