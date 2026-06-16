"""
controller.py — управление роботом лаборатории с игрового геймпада (телеоп).

Подключи джойстик/геймпад к компу, запусти скрипт — двигаешь левым стиком, робот
едет в ту же сторону; держишь кнопку — робот пищит. Команды уходят на сервер
обычными средствами клиентской библиотеки (submit/interrupt), ничего серверного
трогать не нужно.

    pip install pygame
    python client/controller.py

Как это работает (чтобы не копить очередь команд и не «терять» робота):
  Драйв: при изменении стика (с зоной нечувствительности) или раз в KEEPALIVE с
    шлём interrupt() + move(speed_lin, speed_ang, duration=HOLD). interrupt снимает
    текущий поток скорости и сразу запускает новый — низкая задержка, очередь не
    растёт. duration работает как «deadman»: если скрипт упадёт, робот сам встанет.
  Beep: по фронту кнопки — beep_on при нажатии, beep_off при отпускании (реле
    «липкое», тон держится между move-командами; короткая пауза драйва на фронте —
    норма, т.к. устройство выполняет команды по одной).
  Центр стика → один stop, дальше команды не шлём (нет лишнего трафика).

Подбор индексов осей/кнопок под твой геймпад: запусти с PROBE = True (внизу) — скрипт
выведет живые значения осей и кнопок, не отправляя команд. Найди нужные и впиши в
AXIS_FWD / AXIS_TURN / BEEP_BUTTON / STOP_BUTTON.
"""

import asyncio
import os
import sys
import time

# Поддержка запуска как из корня проекта, так и из папки client/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from client import RemoteLab, AcquireError, SubmitError, ConnectionLostError

try:
    import pygame
except ImportError:
    pygame = None

# ─────────────────────────────── Настройки ────────────────────────────────────
SERVER_URL = "localhost:8000"
USERNAME   = "test"
PASSWORD   = "1234"
DEVICE     = None          # имя устройства; None -> первое активное

# Раскладка геймпада (если не подходит — включи PROBE и посмотри индексы)
AXIS_FWD    = 1            # левый стик: вперёд/назад
AXIS_TURN   = 0           # левый стик: влево/вправо
INVERT_FWD  = True        # у большинства геймпадов «вверх» = отрицательное значение
INVERT_TURN = True        # «влево» = отрицательное -> хотим это в +угловую (CCW)
BEEP_BUTTON = 0           # кнопка «пищать» (Xbox A)
STOP_BUTTON = 1           # кнопка аварийной остановки (Xbox B)

# Динамика и темп
MAX_LIN   = 0.5           # м/с при полном отклонении стика
MAX_ANG   = 1.5           # рад/с при полном отклонении
DEADZONE  = 0.12          # зона нечувствительности стика (0..1)
LOOP_HZ   = 20            # частота опроса геймпада
KEEPALIVE = 0.3           # как часто обновлять удерживаемую ненулевую команду, с
HOLD      = 0.5           # длительность move (deadman); > KEEPALIVE, чтобы без разрывов
DEADBAND  = 0.04          # мин. относительное изменение (lin/ang), чтобы переслать
MIN_SEND_DT = 0.08        # не чаще этого слать команды, с

PROBE = False             # True -> только печатать оси/кнопки, команды не слать


def _apply_deadzone(v: float) -> float:
    """Зона нечувствительности с плавным стартом за её краем."""
    if abs(v) < DEADZONE:
        return 0.0
    return (v - (DEADZONE if v > 0 else -DEADZONE)) / (1.0 - DEADZONE)


class Gamepad:
    """Тонкая обёртка над pygame.joystick: оси с дедзоной + фронты кнопок."""

    def __init__(self):
        if pygame is None:
            sys.exit("Нужен pygame:  pip install pygame")
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            sys.exit("Геймпад не найден. Подключи его и проверь, что ОС его видит.")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self._prev_buttons = {}
        print(f"Геймпад: '{self.js.get_name()}'  оси={self.js.get_numaxes()}  кнопок={self.js.get_numbuttons()}")

    def poll(self):
        """Прокачать события (без этого значения не обновляются)."""
        pygame.event.pump()

    def axis(self, i: int) -> float:
        if i < 0 or i >= self.js.get_numaxes():
            return 0.0
        return _apply_deadzone(self.js.get_axis(i))

    def button(self, i: int) -> bool:
        if i < 0 or i >= self.js.get_numbuttons():
            return False
        return bool(self.js.get_button(i))

    def pressed_edge(self, i: int) -> bool:
        """True ровно в тот опрос, когда кнопку нажали (фронт нажатия)."""
        now = self.button(i)
        was = self._prev_buttons.get(i, False)
        self._prev_buttons[i] = now
        return now and not was

    def edge(self, i: int):
        """Вернуть 'down'/'up'/None — фронт нажатия/отпускания кнопки."""
        now = self.button(i)
        was = self._prev_buttons.get(i, False)
        self._prev_buttons[i] = now
        if now and not was:
            return "down"
        if was and not now:
            return "up"
        return None

    def close(self):
        pygame.quit()


def _axes_to_cmd(pad: Gamepad):
    """Левый стик -> (speed_lin м/с, speed_ang рад/с)."""
    fwd = pad.axis(AXIS_FWD) * (-1 if INVERT_FWD else 1)
    turn = pad.axis(AXIS_TURN) * (-1 if INVERT_TURN else 1)
    return fwd * MAX_LIN, turn * MAX_ANG


async def _probe(pad: Gamepad):
    """Печатать живые оси/кнопки, чтобы подобрать индексы. Команды не шлём."""
    print("PROBE: двигай стиками и жми кнопки. Ctrl+C для выхода.\n")
    try:
        while True:
            pad.poll()
            axes = [f"{pad.js.get_axis(i):+.2f}" for i in range(pad.js.get_numaxes())]
            btns = [i for i in range(pad.js.get_numbuttons()) if pad.js.get_button(i)]
            print(f"  оси: {axes}   нажато: {btns}        ", end="\r", flush=True)
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        print()


async def _teleop(robot, pad: Gamepad):
    """Главный цикл: стик -> move, кнопка -> beep, центр -> stop."""
    last_lin = last_ang = 0.0
    last_send = 0.0
    moving = False
    print("\nЕзжай! Левый стик — движение, кнопка A — beep, B — аварийный стоп, Ctrl+C — выход.\n")

    while True:
        pad.poll()
        now = time.monotonic()

        # Аварийный стоп — наивысший приоритет
        if pad.pressed_edge(STOP_BUTTON):
            await robot.interrupt()
            await robot.submit("stop", priority=999)
            moving = False
            last_lin = last_ang = 0.0
            last_send = now
            print("  [STOP]")

        # Beep — по фронтам кнопки (липкое реле держит тон между move-командами)
        be = pad.edge(BEEP_BUTTON)
        if be == "down":
            await robot.submit("beep_on", priority=9)
        elif be == "up":
            await robot.submit("beep_off", priority=9)

        # Драйв
        lin, ang = _axes_to_cmd(pad)
        nonzero = (lin != 0.0 or ang != 0.0)
        changed = (abs(lin - last_lin) > DEADBAND * MAX_LIN or
                   abs(ang - last_ang) > DEADBAND * MAX_ANG)
        due = (now - last_send) > KEEPALIVE

        if (now - last_send) >= MIN_SEND_DT and (changed or (nonzero and due) or (not nonzero and moving)):
            if nonzero:
                # interrupt снимает текущий поток скорости, новый move стартует сразу
                await robot.interrupt()
                await robot.submit("move", priority=5,
                                   speed_lin=round(lin, 3), speed_ang=round(ang, 3), duration=HOLD)
                moving = True
            else:
                # стик в центре -> один стоп, дальше молчим
                await robot.interrupt()
                await robot.submit("stop", priority=8)
                moving = False
            last_lin, last_ang, last_send = lin, ang, now

        await asyncio.sleep(1.0 / LOOP_HZ)


async def main():
    pad = Gamepad()
    if PROBE:
        await _probe(pad)
        pad.close()
        return

    print(f"Подключаюсь к {SERVER_URL} как '{USERNAME}'...")
    try:
        async with RemoteLab(SERVER_URL, USERNAME, PASSWORD) as lab:
            devices = await lab.get_devices()
            if not devices:
                print("Нет активных устройств.")
                return
            name = DEVICE or devices[0]["name"]
            robot = lab.device(name)
            print(f"Управляю устройством '{name}'.")

            # lock() безопасен и для shared-устройств (там это no-op)
            async with robot.lock():
                try:
                    await _teleop(robot, pad)
                finally:
                    # Гарантированно гасим движение и пищалку при выходе
                    try:
                        await robot.interrupt()
                        await robot.submit("stop", priority=999)
                        await robot.submit("beep_off", priority=999)
                    except Exception:
                        pass

    except AcquireError as e:
        print(f"Не удалось захватить устройство: {e}")
    except SubmitError as e:
        print(f"Сервер отклонил команду: {e}")
    except ConnectionLostError as e:
        print(f"\nСоединение потеряно: {e}")
    except OSError as e:
        print(f"\nНе удалось подключиться к серверу ({SERVER_URL}): {e}")
    finally:
        pad.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nВыход.")
