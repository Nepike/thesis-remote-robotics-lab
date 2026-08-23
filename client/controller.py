"""
controller.py — управление роботом лаборатории с игрового геймпада (телеоп).

Подключи джойстик/геймпад к компу, запусти скрипт — двигаешь левым стиком, робот
едет в ту же сторону; держишь кнопку — робот пищит. Команды уходят на сервер
обычными средствами клиентской библиотеки (submit/interrupt).

    pip install pygame
    python client/controller.py

Как это работает (плавно, без рывков):
  Драйв: левый стик -> ШИМ левого/правого колеса (микс вперёд/поворот). Шлём команду
    `dctl` БЕЗ duration — это HOLD-режим: драйвер задаёт скорость и НЕ ставит стоп-хвост,
    робот держит её до следующей команды. Поэтому соседние сетпоинты не дерутся со
    стопами -> плавно. Шлём только при изменении стика + раз в KEEPALIVE (keepalive
    «кормит» прошивочный watchdog, чтобы ровный ход не сбрасывался).
  Стоп: при отпускании/центре/кнопке стоп шлём `dctl 0 0` С duration — это
    стримит ноль + стоп-хвост, т.е. надёжная остановка поверх теряющего линка.
  Beep: по фронтам кнопки (отдельный канал, на драйв не влияет).

⚠️ Безопасность HOLD: пока клиент шлёт keepalive — робот едет; перестал (упал/завис/
  разрыв) — его должен остановить watchdog ПРОШИВКИ по таймауту команды
  (Wait_cmd_time, сейчас ~2 c и под тумблером SW_CONNECT — см. TODO в DeviceDrivers).
  Убедись, что SW_CONNECT включён; в идеале укороти Wait_cmd_time и снизь KEEPALIVE.

Подбор индексов осей/кнопок под твой геймпад: PROBE = True (внизу) — печатает живые
оси/кнопки без отправки команд. Найди нужные и впиши в AXIS_* / *_BUTTON.
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
AXIS_FWD    = 1           # левый стик: вперёд/назад
AXIS_TURN   = 0           # левый стик: влево/вправо
INVERT_FWD  = True        # у большинства геймпадов «вверх» = отрицательное значение
INVERT_TURN = True        # «влево» = отрицательное -> хотим поворот влево (CCW)
BEEP_BUTTON = 0           # кнопка «пищать» (Xbox A)
STOP_BUTTON = 1           # кнопка остановки (Xbox B)

# Драйв
DRIVE_CMD    = "dctl"     # "dctl" = сырой ШИМ (ПИД off), "pidctl" = замкнутая скорость колёс
MAX_PWM      = 200        # ШИМ при полном отклонении стика (|.| <= 255)
DEADZONE     = 0.12       # зона нечувствительности стика (0..1)
LOOP_HZ      = 20         # частота опроса геймпада, Гц
KEEPALIVE    = 0.5        # как часто обновлять удерживаемую скорость (кормить watchdog), с
                          #   ВАЖНО: < Wait_cmd_time прошивки, иначе ровный ход будет сбрасываться
PWM_DEADBAND = 6          # мин. изменение ШИМ, чтобы переслать
STOP_DUR     = 0.3        # длительность стрима стопа при отпускании (надёжная остановка), с

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


def _axes_to_pwm(pad: Gamepad):
    """Левый стик -> (w_l, w_r) ШИМ колёс в [-MAX_PWM, +MAX_PWM] (дифф-привод)."""
    fwd = pad.axis(AXIS_FWD) * (-1 if INVERT_FWD else 1)
    turn = pad.axis(AXIS_TURN) * (-1 if INVERT_TURN else 1)
    left = fwd - turn          # +turn -> правое колесо быстрее -> поворот влево (CCW)
    right = fwd + turn
    m = max(1.0, abs(left), abs(right))   # нормируем, чтобы не вылезти за MAX_PWM
    return (left / m) * MAX_PWM, (right / m) * MAX_PWM


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


async def _drive_hold(robot, w_l: float, w_r: float):
    """HOLD-сетпоинт: задать ШИМ колёс без стоп-хвоста (держится до следующей команды)."""
    await robot.interrupt()   # снять предыдущий сетпоинт/стрим; у HOLD нет хвоста -> без блипа
    await robot.submit(DRIVE_CMD, priority=5, w_l=round(w_l), w_r=round(w_r))


async def _drive_stop(robot):
    """Надёжная остановка: стрим нуля + стоп-хвост (переживает потерю кадров)."""
    await robot.interrupt()
    await robot.submit(DRIVE_CMD, priority=8, w_l=0, w_r=0, duration=STOP_DUR)


async def _teleop(robot, pad: Gamepad):
    """Главный цикл: стик -> HOLD-сетпоинт, центр/стоп -> остановка, кнопка -> beep."""
    last_l = last_r = 0.0
    last_send = 0.0
    moving = False
    print("\nЕзжай! Левый стик — движение, кнопка A — beep, B — стоп, Ctrl+C — выход.\n")

    while True:
        pad.poll()
        now = time.monotonic()

        # Beep — по фронтам кнопки
        be = pad.edge(BEEP_BUTTON)
        if be == "down":
            await robot.submit("beep_on", priority=9)
        elif be == "up":
            await robot.submit("beep_off", priority=9)

        # Стик -> ШИМ (или ноль по кнопке стоп)
        if pad.button(STOP_BUTTON):
            w_l, w_r = 0.0, 0.0
        else:
            w_l, w_r = _axes_to_pwm(pad)

        nonzero = (round(w_l) != 0 or round(w_r) != 0)
        changed = (abs(w_l - last_l) > PWM_DEADBAND or abs(w_r - last_r) > PWM_DEADBAND)
        due = (now - last_send) > KEEPALIVE     # keepalive держит watchdog робота сытым

        if nonzero and (changed or due):
            await _drive_hold(robot, w_l, w_r)
            last_l, last_r, last_send, moving = w_l, w_r, now, True
        elif not nonzero and moving:
            # отпустили / центр / кнопка стоп -> надёжно гасим один раз
            await _drive_stop(robot)
            last_l = last_r = 0.0
            last_send, moving = now, False

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
                        await _drive_stop(robot)
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
