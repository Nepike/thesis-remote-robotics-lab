"""
example.py — путеводитель по возможностям виртуальной лаборатории RemoteLab.

Этот файл одновременно и рабочий пример, и документация: он показывает ВСЕ
основные сценарии работы клиента и перечисляет, какие команды понимает каждый
тип устройства. Запустите его при поднятом сервере — он сам расскажет, что
подключено и что этому можно отправить.

    python client/example.py

═══════════════════════════════════════════════════════════════════════════════
КАК ДОБАВИТЬ НОВЫЙ ТИП УСТРОЙСТВА В ЭТОТ ПРИМЕР
───────────────────────────────────────────────────────────────────────────────
Каталог команд ниже (DRIVER_COMMANDS / DRIVER_TELEMETRY) — data-driven: чтобы
показать команды нового драйвера, не нужно трогать логику примера. Достаточно:

  1. Реализовать драйвер на сервере (DeviceDrivers.py) и зарегистрировать его в
     DriverFactory под строковым ключом, например "my_robot".
  2. Добавить сюда по образцу один блок:
         DRIVER_COMMANDS["my_robot"] = [
             Cmd("go",   "Поехать: dist (см), speed (0..100)", dict(dist=50, speed=30)),
             Cmd("stop", "Остановиться", {}),
         ]
         DRIVER_TELEMETRY["my_robot"] = "battery, x, y, heading"
  3. (Опционально) дополнить демо-функции, если хотите вживую показать новые
     команды. Базовое поведение примера при этом уже работает: каталог нового
     типа распечатается автоматически.
═══════════════════════════════════════════════════════════════════════════════
"""

import asyncio
import os
import sys
from contextlib import AsyncExitStack
from dataclasses import dataclass, field

# Поддержка запуска как из корня проекта, так и из папки client/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from client import (
    RemoteLab,
    AcquireError,
    SubmitError,
    ProcedureError,
    ConnectionLostError,
)

# ─────────────────────────────── Настройки ────────────────────────────────────
SERVER_URL = "localhost:8000"   # можно "ws://host:port" или просто "host:port"
USERNAME   = "test"
PASSWORD   = "1234"


# ══════════════════════════ КАТАЛОГ КОМАНД ПО ДРАЙВЕРАМ ════════════════════════
# Это «справочник», который печатается в начале работы. Каждый драйвер на сервере
# понимает свой набор команд (см. execute_command соответствующего драйвера в
# manager/DeviceDrivers.py). Имена и аргументы здесь должны совпадать с теми, что
# ждёт execute_command.

@dataclass
class Cmd:
    name: str                       # имя команды (первый аргумент submit)
    desc: str                       # человекочитаемое описание + аргументы
    example: dict = field(default_factory=dict)  # пример аргументов для submit(**example)


# Команды устройств yarp-13 (полный набор из Yarp13Driver.execute_command).
# Аргументы передаются как именованные: robot.submit("move", speed_lin=0.2, ...)
DRIVER_COMMANDS = {
    "yarp13": [
        Cmd("move",   "Движение: speed_lin (м/с, +вперёд), speed_ang (рад/с, +влево), duration (с)",
            dict(speed_lin=0.0, speed_ang=1.0, duration=3.0)),
        Cmd("stop",   "Немедленная остановка колёс", {}),
        Cmd("dctl",   "Прямой ШИМ на колёса: w_l, w_r (-255..255), duration (с)",
            dict(w_l=120, w_r=120, duration=2.0)),
        Cmd("pidctl", "Скорость колёс через ПИД: w_l, w_r, duration (с)",
            dict(w_l=80, w_r=80, duration=2.0)),
        Cmd("beep",   "Пищать duration секунд", dict(duration=1.0)),
        Cmd("beep_on",  "Включить пищалку (до beep_off)", {}),
        Cmd("beep_off", "Выключить пищалку", {}),
        Cmd("gun_on",   "Включить «пушку»/реле нагрузки", {}),
        Cmd("gun_off",  "Выключить «пушку»/реле нагрузки", {}),
        Cmd("set_servo", "Углы 3 серв: a0, a1, a2 (град)", dict(a0=90, a1=90, a2=90)),
        Cmd("set_enc",   "Сброс энкодеров: left, right", dict(left=0, right=0)),
        Cmd("set_pid",   "Коэффициенты ПИД: kp, ki, kd", dict(kp=0.5, ki=0.02, kd=0.2)),
        Cmd("set_pid_left",  "ПИД только левого колеса: kp, ki, kd", dict(kp=0.5, ki=0.02, kd=0.2)),
        Cmd("set_pid_right", "ПИД только правого колеса: kp, ki, kd", dict(kp=0.5, ki=0.02, kd=0.2)),
        Cmd("set_refl_dist", "Дистанции рефлекс-стопа, см: center, left, right",
            dict(center=20, left=20, right=20)),
        Cmd("set_motors_ratio", "Балансировка моторов: left, right", dict(left=1.0, right=1.0)),
        Cmd("set_calibr_speed", "Калибровка компаса: speed (ШИМ), max_cnt (тиков)",
            dict(speed=40, max_cnt=600)),
        Cmd("compass_calibr", "Запустить калибровку компаса", {}),
        Cmd("set_klpf", "ФНЧ скорости привода: k (0..1)", dict(k=1.0)),
    ],

    # Пример минимального последовательного устройства (SimpleSerialDevice).
    # Протокол — строки "<ИМЯ> [аргументы]\\n"; аргументы идут как значения kwargs.
    "simple_serial": [
        Cmd("ON",  "Включить", {}),
        Cmd("OFF", "Выключить", {}),
        Cmd("SET_THRESHOLD", "Установить порог: value", dict(value=50)),
    ],

    # <- сюда добавляйте блок нового драйвера по образцу (см. шапку файла).
}

# Короткое описание полей телеметрии каждого типа (что приходит в callback / snapshot).
DRIVER_TELEMETRY = {
    "yarp13": "enc_left/right, speed_left/right, compass, pitch, roll, acc_voltage, "
              "rf_* (дальномеры), pwm_left/right, bumpers, status, cmd_count",
    "simple_serial": "uptime, value, status",
    # <- и сюда строку про телеметрию нового драйвера.
}


# ════════════════════════════════ ХЕЛПЕРЫ ═════════════════════════════════════

def print_devices(devices: list):
    """Печатает таблицу подключённых устройств и поясняет модель доступа."""
    print(f"Активные устройства ({len(devices)}):")
    for d in devices:
        access = "shared (общий — lock не нужен)" if d["shared"] else "exclusive (нужен lock)"
        print(f"  • {d['name']:<18} driver={d['driver']:<14} {access}")
    print()


def print_command_catalog(devices: list):
    """
    Самодокументация: для КАЖДОГО типа драйвера среди подключённых устройств
    печатает список доступных команд и поля телеметрии. Именно здесь видно, чем
    отличается yarp-13 от других типов.
    """
    driver_types = sorted({d["driver"] for d in devices})
    print("Что можно отправлять (каталог команд по типам устройств):")
    for dt in driver_types:
        print(f"\n  ── Драйвер «{dt}» " + "─" * (40 - len(dt)))
        commands = DRIVER_COMMANDS.get(dt)
        if not commands:
            print("     (команды для этого типа не описаны в примере — дополните DRIVER_COMMANDS)")
            continue
        for c in commands:
            print(f"     {c.name:<16} {c.desc}")
        tele = DRIVER_TELEMETRY.get(dt, "— не описана —")
        print(f"     · телеметрия: {tele}")
    print()


# ════════════════════════════════ ДЕМО-СЦЕНАРИИ ═══════════════════════════════
# Каждый сценарий самодостаточен и снабжён пояснением. Можно закомментировать
# ненужные вызовы в main().

async def demo_telemetry(lab: RemoteLab, device_names: list):
    """
    Телеметрия двумя способами:
      push   — подписка с callback (вызывается на каждое новое сообщение);
      pull   — разовое чтение последнего снимка get_latest_telemetry().
    """
    print("[Телеметрия] подписываемся на push-поток первого устройства...")
    name = device_names[0]
    robot = lab.device(name)

    # callback получает обычный dict с полями телеметрии (см. DRIVER_TELEMETRY)
    await robot.subscribe_telemetry(lambda data: None)  # тихо копим снимки

    await asyncio.sleep(1.0)  # дать потоку прийти

    snap = robot.get_latest_telemetry()  # pull: последний снимок без callback
    if snap is None:
        print(f"  ⚠ от '{name}' пока нет телеметрии")
    else:
        # показываем пару характерных полей (если есть)
        keys = [k for k in ("compass", "acc_voltage", "cmd_count") if k in snap]
        shown = ", ".join(f"{k}={snap[k]}" for k in keys) or "снимок получен"
        print(f"  '{name}': {shown}")
    print()


async def demo_single_device(lab: RemoteLab, devices: list):
    """
    Одиночные команды одному роботу + правильная работа с эксклюзивным доступом.

    lock() безопасен и для shared-устройств (там это no-op), поэтому шаблон
    `async with robot.lock()` универсален.
    """
    d = devices[0]
    name = d["name"]
    robot = lab.device(name)
    print(f"[Одиночное устройство] работаем с '{name}'...")

    # lock() гарантирует release даже при исключении
    async with robot.lock():
        # CommandHandle можно дождаться (await) либо «выстрелить и забыть»
        print("  beep 0.5с (ждём завершения)...")
        cmd = await robot.submit("beep", duration=0.5)
        await cmd                                  # блокируемся до выполнения

        print("  короткий поворот на месте (fire-and-forget + await)...")
        cmd = await robot.submit("move", speed_lin=0.0, speed_ang=1.0, duration=1.0)
        await cmd
        # на всякий случай явный стоп (потоковая остановка на сервере и так есть)
        stop = await robot.submit("stop", priority=9)
        await stop
    print()


async def demo_group_direct(lab: RemoteLab, devices: list):
    """
    Групповая команда НАПРЯМУЮ с клиента: одна submit на список устройств.
    Хэндл завершится, когда КАЖДОЕ устройство отчитается 'done'.

    Берём только shared-устройства: для exclusive нужен предварительный lock
    каждого (см. demo_single_device).
    """
    shared = [d["name"] for d in devices if d["shared"]]
    if not shared:
        print("[Группа] нет shared-устройств для прямой групповой команды.\n")
        return

    print(f"[Группа] синхронный beep на {shared}...")
    cmd = await lab.submit(shared, "beep", priority=5, duration=0.7)
    await cmd
    print("  готово.\n")


async def demo_procedures(lab: RemoteLab):
    """
    Серверные групповые ПРОЦЕДУРЫ — сценарии, исполняемые на сервере целиком.
    Клиент лишь запускает их по имени и ждёт результат.

    Зарегистрированные процедуры:
      sync_test   — синхронный тест-парад всей группы (пищат, крутятся влево/
                      вправо) + проверка живой телеметрии каждого робота;
      stop_all    — аварийная остановка всей лаборатории;
      all_go_home — возврат роботов на базу (камера + ArUco + A*;).

    Важно: процедура сама захватывает нужные устройства, поэтому НЕ держите на
    клиенте lock на те же устройства во время её запуска.
    """
    print("[Процедура] запускаем sync_test (beep → влево 3с → вправо 3с → проверка телеметрии)...")
    try:
        proc = await lab.run_procedure("sync_test")
        await proc  # ждём завершения; при провале телеметрии бросит ProcedureError
        print("  sync_test: УСПЕХ — все роботы живы и синхронны.\n")
    except ProcedureError as e:
        print(f"  sync_test: ПРОВАЛ — {e}\n")


async def demo_emergency_stop(lab: RemoteLab, device_names: list):
    """
    Шаблон аварийной остановки одного устройства:
      1. interrupt() — прерывает команду, выполняемую ПРЯМО СЕЙЧАС;
      2. submit("stop", priority=...) — высокоприоритетный стоп следующим.
    Для остановки ВСЕЙ лаборатории используйте процедуру stop_all.
    """
    name = device_names[0]
    robot = lab.device(name)
    print(f"[Авария] демонстрация interrupt+stop на '{name}'...")

    # запускаем длинное движение и тут же прерываем его
    await robot.submit("move", speed_lin=0.0, speed_ang=1.0, duration=10.0)
    await asyncio.sleep(0.5)
    await robot.interrupt()                       # прервать текущее
    await robot.submit("stop", priority=999)      # и сразу стоп
    print("  движение прервано и остановлено.\n")


# ════════════════════════════════════ MAIN ═══════════════════════════════════

async def main():
    print(f"Подключаемся к {SERVER_URL} под пользователем '{USERNAME}'...\n")
    try:
        async with RemoteLab(SERVER_URL, USERNAME, PASSWORD) as lab:
            print("Подключено.\n")

            # 1. Какие устройства есть и что им можно слать
            devices = await lab.get_devices()
            if not devices:
                print("Активных устройств нет — нечего демонстрировать.")
                return
            print_devices(devices)
            print_command_catalog(devices)

            device_names = [d["name"] for d in devices]

            # 2. Телеметрия (push + pull)
            await demo_telemetry(lab, device_names)

            # 3. Одиночные команды + эксклюзивный доступ
            await demo_single_device(lab, devices)

            # 4. Прямая групповая команда с клиента
            await demo_group_direct(lab, devices)

            # 5. Серверная групповая процедура (синхронный тест группы)
            await demo_procedures(lab)

            # 6. Аварийная остановка (interrupt + stop)
            await demo_emergency_stop(lab, device_names)

            print("Все сценарии отработаны.")

    except AcquireError as e:
        print(f"Не удалось захватить устройство: {e}")
        sys.exit(1)
    except SubmitError as e:
        print(f"Сервер отклонил команду: {e}")
        sys.exit(1)
    except ConnectionLostError as e:
        print(f"\nСоединение потеряно: {e}")
        sys.exit(1)
    except OSError as e:
        print(f"\nНе удалось подключиться к серверу ({SERVER_URL}): {e}")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
