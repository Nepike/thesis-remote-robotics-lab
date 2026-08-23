"""
Диагностика надёжности ДВИЖЕНИЯ и ОСТАНОВКИ (после перехода на потоковую модель).

Проверяет самое критичное: после команды move робот действительно
  (а) начинает двигаться (pwm != 0 во время move) и
  (б) ГАРАНТИРОВАННО останавливается (pwm -> 0 после move).

pwm_left/pwm_right в телеметрии — это командное значение ШИМ, которое прошивка
выставляет независимо от того, подключены ли моторы. Поэтому тест работает и на
тестовых девайсах без моторов: «крутимся на месте» => pwm = ±250, «стоп» => 0.

Запуск:  python client/diag.py
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from client import RemoteLab

SERVER_URL = "localhost:8000"
USERNAME   = "test"
PASSWORD   = "1234"

N_TRIALS   = 15
MOVE_SEC   = 2.0   # длительность move (вращение на месте)
SETTLE     = 1.0   # ждать после move, чтобы стоп-хвост точно отработал
QUIET_GAP  = 1.5   # пауза между попытками


def _pwm(robot):
    t = robot.get_latest_telemetry()
    if t is None:
        return None
    return t.get("pwm_left"), t.get("pwm_right")


async def main():
    print(f"Connecting to {SERVER_URL} as '{USERNAME}'...")
    async with RemoteLab(SERVER_URL, USERNAME, PASSWORD) as lab:
        devices = await lab.get_devices()
        if not devices:
            print("No active devices.")
            return
        name = devices[0]["name"]
        robot = lab.device(name)
        print(f"Device: {name}\n")

        await robot.subscribe_telemetry(lambda d: None)
        await asyncio.sleep(2.0)
        if _pwm(robot) is None:
            print("Телеметрия не пришла.")
            return

        moved_ok = 0
        stopped_ok = 0
        for i in range(1, N_TRIALS + 1):
            # Старт вращения на месте
            cmd = await robot.submit(
                "move", priority=5, speed_lin=0.0, speed_ang=1.0, duration=MOVE_SEC,
            )

            # Замер В ПРОЦЕССЕ движения
            await asyncio.sleep(MOVE_SEC * 0.5)
            pl, pr = _pwm(robot)
            moving = (pl != 0 or pr != 0)

            await cmd                      # дождаться завершения move
            await asyncio.sleep(SETTLE)    # дать стоп-хвосту отработать

            # Замер ПОСЛЕ остановки
            pl2, pr2 = _pwm(robot)
            stopped = (pl2 == 0 and pr2 == 0)

            moved_ok += 1 if moving else 0
            stopped_ok += 1 if stopped else 0

            mv = "движется" if moving else "НЕ ДВИЖЕТСЯ"
            st = "ОСТАНОВИЛСЯ" if stopped else "!!! НЕ ОСТАНОВИЛСЯ !!!"
            print(f"#{i:2d}: во время move pwm=({pl},{pr}) [{mv}]  ->  после: pwm=({pl2},{pr2}) [{st}]")

            await asyncio.sleep(QUIET_GAP)

        print("\n================ ИТОГ ================")
        print(f"Движение запустилось: {moved_ok}/{N_TRIALS}")
        print(f"ОСТАНОВКА сработала:  {stopped_ok}/{N_TRIALS}  <-- критично для безопасности")
        if stopped_ok == N_TRIALS and moved_ok == N_TRIALS:
            print("\nОК: и движение, и остановка надёжны.")
        else:
            print("\nЕсть отказы — присылай вывод.")


if __name__ == "__main__":
    asyncio.run(main())
