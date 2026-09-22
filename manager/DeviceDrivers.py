from abc import ABC, abstractmethod
from typing import Any, Awaitable, Callable, Dict, Optional, Set, Tuple
import asyncio
from pathlib import Path
import os

# Available once the package has been built via catkin and the ROS environment has been loaded (source devel/setup.bash).
import geometry_msgs.msg
import msg_yy.msg

from BasicClasses import Device, Command
from HardwareInterfaces import RosInterface, SerialInterface, TcpInterface
from Logger import Logger
from TelemetryTypes import Yarp13Telemetry, SimpleSerialTelemetry, MicrobotTelemetry


class AbstractDriver(ABC):
    """
    Base class for all device drivers.

    Each driver must implement command execution logic and interface-required processes (transports, adapters) start logic
    """

    # Command names this driver accepts, in the order they are worth showing to
    # a human. Served to clients through get_devices(), so a new device type no
    # longer means editing the client to teach it what the type can do — and
    # procedures that run across a HETEROGENEOUS fleet can ask instead of
    # assuming every robot has, say, a beeper.
    COMMANDS: Tuple[str, ...] = ()

    def __init__(self, device: Device):
        self._device: Device = device

        transport_root = Path("/tmp/remote_lab_tty")
        transport_root.mkdir(parents=True, exist_ok=True)
        self._transport_path: Path = transport_root / f"ttyDEVICE-{self._device.name}"

        # For pull-telemetry
        self._latest_telemetry: Optional[Any] = None

        # True once telemetry has started flowing — i.e. the physical device is
        # actually connected and talking. Reset on teardown so a reconnect re-logs.
        self._online: bool = False

        #  When a new message arrives from the device, all the functions are called.
        self._telemetry_listeners: Dict[int, Callable[[Any], Awaitable[None]]] = {}

        # Counter for generating unique listener IDs
        self._next_listener_id: int = 0

    @abstractmethod
    async def start_transports(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts transport processes (usually just socat) and returns a tuple of them.
        """
        pass

    @abstractmethod
    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts interface-required processes (e.g. rosserial) and returns a tuple of them.
        """
        pass

    @abstractmethod
    async def execute_command(self, command: Command):
        """
        Execute a command on the physical device.

        CONTRACT for commands that take time (movement, timed beep, etc.):
          - Use `await asyncio.sleep(duration)` — NOT time.sleep() - so the
            event loop stays unblocked and CancelledError can arrive.
          - Wrap the sleep in try/finally and send a physical stop in the
            finally block.  This guarantees the device halts when the command
            is interrupted via interrupt_device() or stop_all.

        Instantaneous commands (beep_on, set_servo, …) have no await and need
        no finally — CancelledError cannot reach them mid-execution.

        Example (correct):
            twist_move = ...
            self._ros.publish(topic, Twist, twist_move)
            try:
                await asyncio.sleep(command.args["duration"])
            finally:
                self._ros.publish(topic, Twist, Twist())  # stop

        Example (wrong - blocks the event loop, cannot be interrupted):
            time.sleep(command.args["duration"])
        """

    async def setup_telemetry(self):
        """Subscribe to device telemetry sources. Called after adapters are started."""
        pass

    async def setup_publishers(self):
        """
        Pre-create (warm up) command publishers and wait for their subscribers to
        connect. Called after setup_telemetry, on every (re)start of the device.

        Removes the rospy create+publish race: without warm-up, the first command
        after startup / after a device restart is published before the publisher-
        subscriber connection is established and is silently lost.
        """
        pass

    async def teardown_telemetry(self):
        """Unsubscribe / close telemetry connections. Called before processes are stopped."""
        # Mark offline so the next successful connection re-logs the 'online' event.
        # Subclasses that override this must call super().teardown_telemetry().
        self._online = False

    def emergency_stop(self) -> None:
        """
        Bring EVERY actuator of this device to a halt, right now.

        Called by the stop_all procedure as the final step. Unlike a "stop" command
        this does NOT go through the scheduler: the queue may be paused (device
        restarting) or the actuator may be latched by a command that has already
        returned — a HOLD setpoint being the obvious case, where there is no running
        coroutine left to interrupt yet the wheels keep turning.

        Must be synchronous and must not raise: it runs on the emergency path.
        Default is a no-op so a driver with no actuators needs no implementation.
        """
        pass

    def supports(self, command_name: str) -> bool:
        """
        True if this driver implements the named command.

        A driver that leaves COMMANDS empty has not declared a catalogue, so we
        answer True and let execute_command() be the judge — that keeps the
        check backwards-compatible with any driver written before COMMANDS existed.
        """
        return (not self.COMMANDS) or (command_name in self.COMMANDS)

    def get_telemetry(self) -> Optional[Any]:
        """Return the latest received telemetry snapshot, or None if nothing has arrived yet."""
        return self._latest_telemetry

    def add_telemetry_listener(self, callback: Callable[[Any], Awaitable[None]]) -> int:
        """Register an async callback invoked on every new telemetry message. Returns listener id."""
        lid = self._next_listener_id
        self._telemetry_listeners[lid] = callback
        self._next_listener_id += 1
        return lid

    def remove_telemetry_listener(self, listener_id: int):
        self._telemetry_listeners.pop(listener_id, None)

    async def _notify_listeners(self, telemetry: Any):
        # First telemetry after (re)connect means the device is actually online and talking.
        if not self._online:
            self._online = True
            await Logger.get().log("DEVICE", f"'{self._device.name}' online")

        # It iterates over a copy of the dictionary because the listener could theoretically remove itself during the call
        for callback in list(self._telemetry_listeners.values()):
            await callback(telemetry)



# ---------------------------- STARTERS POOL GO HERE ----------------------------
# TODO - сейчас драйверы могут дублировать код запуска некоторых процессов
#  наверное, хорошо бы сделать некий пулл функций-стартеров, и просто вызывать внутри драйвера функцию оттуда
#  UPD: kinda-done - mb exists a better way (i don't like current realisation),
#  I'll just leave it here

async def _start_socat(tty_path: Path, tcp_path: str, logger_prefix: str) -> asyncio.subprocess.Process:
    if os.path.exists(tty_path):
        os.remove(tty_path)

    proc = await asyncio.create_subprocess_exec(
        "socat",
        f"pty,link={tty_path},raw,echo=0,waitslave,mode=666",
        f"tcp:{tcp_path},nodelay,retry=3",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    logger = Logger.get()
    await logger.log(logger_prefix, f"socat started")
    logger.attach_stream(logger_prefix, proc.stdout)
    logger.attach_stream(logger_prefix, proc.stderr)

    for _ in range(20):
        if os.path.exists(tty_path):
            break
        await asyncio.sleep(0.1)

    return proc

async def _start_rosserial(tty_path: Path, namespace: str, logger_prefix: str) -> asyncio.subprocess.Process:
    cmd = (
        f"bash -c 'source /opt/ros/noetic/setup.bash && "
        f"source {Path.home()}/ros/devel/setup.bash && "
        f"roslaunch yyctl rosserial.launch "
        f"port:={tty_path} __ns:={namespace}'"
    )

    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    logger = Logger.get()
    await logger.log(logger_prefix, f"rosserial started")
    logger.attach_stream(logger_prefix, proc.stdout)
    logger.attach_stream(logger_prefix, proc.stderr)

    await asyncio.sleep(1)  # God help us

    return proc

# ---------------------------- STARTERS POOL GO ABOVE ----------------------------


# ---------------------------- BASIC INTERFACE-BASED DRIVERS GO HERE ----------------------------
class RosBasedDriver(AbstractDriver):
    """
    Base class for all ros-based devices.
    """
    def __init__(self, device: Device, ros: RosInterface):
        super().__init__(device)
        self._ros: RosInterface = ros

    async def start_transports(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts socat.
        """
        proc = await _start_socat(tty_path=self._transport_path,
                                  tcp_path=f"{self._device.ip}:{self._device.port}",
                                  logger_prefix=f"{self._device.name}-SOCAT")
        return (proc,)

    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts rosserial.
        """
        proc = await _start_rosserial(tty_path=self._transport_path,
                                      namespace=self._device.ros_namespace,
                                      logger_prefix=f"{self._device.name}-ROSSERIAL")
        return (proc,)

    @abstractmethod
    async def execute_command(self, command: Command):
        """
        Executes a command via RosInterface.

        You must override this method for every device type.
        """
        pass


class SerialBasedDriver(AbstractDriver):
    """
    Base class for all serial-based devices.
    """
    def __init__(self, device: Device, serial: SerialInterface):
        super().__init__(device)
        self._serial: SerialInterface = serial

    async def start_transports(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts socat.
        """
        proc = await _start_socat(tty_path=self._transport_path,
                                  tcp_path=f"{self._device.ip}:{self._device.port}",
                                  logger_prefix=f"{self._device.name}-SOCAT")
        return (proc,)

    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Does nothing. (No additional processes are required for this interface)
        """
        return ()

    @abstractmethod
    async def execute_command(self, command: Command):
        """
        Executes a command via SerialInterface.

        You must override this method for every device type.
        """
        pass


class TcpBasedDriver(AbstractDriver):
    """
    Base class for devices that speak a line protocol over TCP directly.

    Use this when the network endpoint IS the robot's controller (the ESP32 runs
    the control loop itself) rather than a transparent bridge to a second MCU.
    Such a device needs neither socat nor rosserial, so both process hooks return
    empty tuples.

    That has a consequence worth stating: DeviceSupervisor restarts a device when
    one of its PROCESSES dies, and here there are none — `transports_alive()` and
    `adapters_alive()` treat an empty tuple as alive, so the supervisor will never
    restart this device. It doesn't need to: TcpInterface reconnects the socket on
    its own, and `_online` flips back the moment telemetry resumes.
    """

    def __init__(self, device: Device, tcp: TcpInterface):
        super().__init__(device)
        self._tcp: TcpInterface = tcp

    async def start_transports(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """Nothing to start: the connection is a plain socket, opened in setup_telemetry."""
        return ()

    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """Nothing to start: the device speaks its own protocol, no adapter in between."""
        return ()

    @abstractmethod
    async def execute_command(self, command: Command):
        """
        Executes a command via TcpInterface.

        You must override this method for every device type.
        """
        pass


# ---------------------------- BASIC INTERFACE-BASED DRIVERS GO ABOVE ----------------------------

# ---------------------------- CUSTOM DRIVERS GO HERE ----------------------------
# To add a new custom driver,
# you must first ensure that a base driver-class associated with the corresponding interface
# has been implemented (similar to those in the block above).
# After that, you inherit from it and implement the `execute_command` method.
#
# The class will have the following fields available: `self._device`, `self._transport_path`, `self._<interface>`


class Yarp13Driver(RosBasedDriver):
    """
    Driver for yarp-13 devices.

    Supports:
    - move
    - beep
    - ...
    """

    COMMANDS: Tuple[str, ...] = (
        "move", "stop", "dctl", "pidctl",
        "beep", "beep_on", "beep_off",
        "gun_on", "gun_off",
        "set_servo", "set_enc", "set_refl_dist", "set_motors_ratio",
        "set_pid", "set_pid_left", "set_pid_right",
        "compass_calibr", "set_calibr_speed", "set_klpf",
    )

    # Command codes for msg_yy::cmd (from y13cmd.h)
    _CMD: Dict[str, int] = {
        "beep_on":          1,
        "beep_off":         2,
        "gun_on":           3,
        "gun_off":          4,
        "set_servo":        5,
        "set_refl_dist":    6,
        "set_enc":          7,
        "set_pid":          8,
        "compass_calibr":   9,
        "set_calibr_speed": 10,
        "set_motors_ratio": 11,
        "set_pid_left":     12,
        "set_pid_right":    13,
        "usr":              14,
        "dctl":             15,
        "pidctl":           16,
    }

    # Subcodes for CMD_USR (da[0])
    _SUBCMD: Dict[str, int] = {
        "set_rc5":  1,
        "set_klpf": 2,
    }

    # --- Reliable delivery over a lossy rosserial link --------------------------
    # Instead of sending edge commands we
    # STREAM the desired setpoint at _STREAM_HZ: a dropped frame is corrected by the
    # next one ~100 ms later.  Motion stop is streamed for a short tail in the BACKGROUND so it survives even if the
    # command coroutine is cancelled (E-stop).
    _STREAM_HZ:   float = 10.0   # setpoint streaming rate, Hz
    _ONESHOT_S:   float = 0.3    # how long to stream a one-shot command (~3 frames)
    _STOP_TAIL_S: float = 0.6    # how long to keep streaming stop after motion ends
    _DEADMAN_S:   float = 0.5    # set_velocity deadman: stop if no new setpoint within this

    # Each command targets one logical ACTUATOR CHANNEL. A command cancels the
    # running stop/off tail ONLY on its own channel, so independent actuators that
    # share the yy_command topic (beep relay, gun relay, servos, encoders, PID,
    # motors, ...) never cancel each other's tails
    #
    # Only commands that share a channel with another command need listing here.
    # Anything not listed is its own channel (its name) — inert, since only "held"
    # commands (move/dctl/pidctl -> drive, beep -> beep) ever create a tail.
    _CHANNEL: Dict[str, str] = {
        "move": "drive", "stop": "drive", "dctl": "drive", "pidctl": "drive",
        "beep": "beep",  "beep_on": "beep", "beep_off": "beep",
        "gun_on": "gun", "gun_off": "gun",
    }

    def _channel_of(self, name: str) -> str:
        return self._CHANNEL.get(name, name)

    def __init__(self, device: Device, ros: RosInterface):
        super().__init__(device, ros)
        # channel -> background task that keeps streaming the latest stop/off value.
        self._tails: Dict[str, asyncio.Task] = {}

    async def _stream(self, topic: str, msg_type, message, duration: float):
        """Publish `message` at _STREAM_HZ for `duration` seconds (at least once)."""
        period = 1.0 / self._STREAM_HZ
        n = max(1, int(round(duration / period)))
        for _ in range(n):
            self._ros.publish(topic, msg_type, message)
            await asyncio.sleep(period)

    def _cancel_tail(self, channel: str):
        t = self._tails.pop(channel, None)
        if t is not None and not t.done():
            t.cancel()

    def _start_tail(self, channel: str, topic: str, msg_type, message):
        """
        Stream a stop/off value for the given actuator `channel` in the BACKGROUND
        for _STOP_TAIL_S seconds. Fire-and-forget so it completes even when the
        owning command coroutine is cancelled (interrupt / E-stop). Replaces any
        previous tail on the same channel.
        """
        self._cancel_tail(channel)
        self._tails[channel] = asyncio.create_task(
            self._stream(topic, msg_type, message, self._STOP_TAIL_S)
        )

    def _topic(self, suffix: str) -> str:
        ns = self._device.ros_namespace.strip("/")
        return f"/{ns}/{suffix}"

    def set_velocity(self, v: float, omega: float):
        """
        Publish ONE cmd_vel setpoint immediately, for closed-loop procedures that
        run their own control loop (e.g. AllGoHome): the procedure calls this at a
        fixed rate with fresh (v, omega).

        Safety deadman: each call re-arms a background timer on the "drive" channel.
        If new setpoints stop arriving (procedure crashes / is cancelled), the timer
        fires after _DEADMAN_S and streams a stop — the robot never runs away.
        Going through this method (not the scheduler) keeps the loop low-latency;
        E-stop still works because stop_all cancels the owning procedure, whose
        teardown stops the robot, and the deadman is the final backstop.
        """
        Twist = geometry_msgs.msg.Twist
        topic = self._topic("cmd_vel")
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(omega)
        self._ros.publish(topic, Twist, twist)
        self._arm_deadman("drive", topic, Twist, Twist(), self._DEADMAN_S)

    def _arm_deadman(self, channel: str, topic: str, msg_type, stop_message, delay: float):
        """
        (Re)arm a deadman on `channel`: after `delay` with no refresh, stream the
        stop value. Cancelled/replaced by the next set_velocity, so during active
        control it never fires. Reuses the per-channel tail slot.
        """
        self._cancel_tail(channel)

        async def _deadman():
            try:
                await asyncio.sleep(delay)
            except asyncio.CancelledError:
                return
            await self._stream(topic, msg_type, stop_message, self._STOP_TAIL_S)

        self._tails[channel] = asyncio.create_task(_deadman())

    # Reserved tail slot for the E-stop burst. Not a real actuator channel, so no
    # incoming command can ever cancel it via _channel_of() — an emergency stop
    # must not be revoked by whatever arrives next.
    _ESTOP_CHANNEL: str = "__estop__"

    def emergency_stop(self) -> None:
        """
        Halt every actuator: wheels (both the cmd_vel and the direct-PWM paths),
        the beeper and the gun. See AbstractDriver.emergency_stop.

        Kills all running tails/deadmen first so nothing re-arms motion behind us,
        publishes one stop frame per actuator immediately, then keeps re-publishing
        them in the background for _STOP_TAIL_S — a single dropped frame on the
        rosserial link must not be what leaves a motor running.
        """
        Twist = geometry_msgs.msg.Twist
        Cmd   = msg_yy.msg.cmd
        cmd_vel = self._topic("cmd_vel")
        yy_cmd  = self._topic("yy_command")

        for channel in list(self._tails):
            self._cancel_tail(channel)

        stops = [(cmd_vel, Twist, Twist())]
        for name in ("dctl", "pidctl"):          # zero the wheels on the direct paths too
            msg = Cmd()
            msg.command = self._CMD[name]
            msg.arg     = [0.0, 0.0]
            stops.append((yy_cmd, Cmd, msg))
        for name in ("beep_off", "gun_off"):
            msg = Cmd()
            msg.command = self._CMD[name]
            stops.append((yy_cmd, Cmd, msg))

        for topic, msg_type, message in stops:
            self._ros.publish(topic, msg_type, message)

        self._tails[self._ESTOP_CHANNEL] = asyncio.create_task(self._estop_tail(stops))

    async def _estop_tail(self, stops):
        """Re-publish the E-stop frames at _STREAM_HZ for _STOP_TAIL_S."""
        period = 1.0 / self._STREAM_HZ
        for _ in range(max(1, int(round(self._STOP_TAIL_S / period)))):
            await asyncio.sleep(period)
            for topic, msg_type, message in stops:
                self._ros.publish(topic, msg_type, message)

    async def setup_publishers(self):
        ns = self._device.ros_namespace.strip("/")
        # Create both command publishers ahead of any command and wait (best-effort,
        # in parallel) for the rosserial subscriber to connect. Runs before the
        # scheduler is resumed after a restart, so the first command after a device
        # comes back isn't lost to the publisher-connection race.
        self._ros.register_publisher(f"/{ns}/yy_command", msg_yy.msg.cmd)
        self._ros.register_publisher(f"/{ns}/cmd_vel", geometry_msgs.msg.Twist)
        await asyncio.gather(
            self._ros.wait_for_publisher(f"/{ns}/yy_command", timeout=2.0),
            self._ros.wait_for_publisher(f"/{ns}/cmd_vel", timeout=2.0),
        )

    async def setup_telemetry(self):
        if not self._device.ros_namespace:
            raise RuntimeError(
                f"Device '{self._device.name}' has no ros_namespace configured"
            )
        self._ros.subscribe_async(
            f"/{self._device.ros_namespace.strip('/')}/yy_sensors",
            msg_yy.msg.sens,
            self._on_sensors_msg,
        )

    async def teardown_telemetry(self):
        await super().teardown_telemetry()
        # Stop every background stop/off tail before the device goes away, so no
        # lingering task keeps publishing (and silently re-creating publishers) on
        # a ROS node that is restarting or shutting down.
        for channel in list(self._tails):
            self._cancel_tail(channel)
        if self._device.ros_namespace:
            self._ros.unsubscribe(
                f"/{self._device.ros_namespace.strip('/')}/yy_sensors"
            )

    async def _on_sensors_msg(self, msg):
        telemetry = Yarp13Telemetry(
            enc_left=msg.enc_left,
            enc_right=msg.enc_right,
            speed_left=int(msg.data[2]),
            speed_right=int(msg.data[3]),
            compass=msg.compass,
            pitch=msg.cpitch,
            roll=msg.croll,
            acc_voltage=msg.acc_voltage,
            rf_center=msg.rf_center,
            rf_left=msg.rf_left,
            rf_right=msg.rf_right,
            rf_side_left_fwd=msg.rf_side_left_fwd,
            rf_side_right_fwd=msg.rf_side_right_fwd,
            rf_side_left_bck=msg.rf_side_left_bck,
            rf_side_right_bck=msg.rf_side_right_bck,
            rf_bck_center=msg.rf_bck_center,
            pwm_left=float(msg.data[4]),
            pwm_right=float(msg.data[5]),
            bumpers=int(msg.data[1]),
            status=msg.status,
            cmd_count=int(msg.data[0]),
        )
        self._latest_telemetry = telemetry
        await self._notify_listeners(telemetry)

    async def execute_command(self, command: Command):
        ns      = self._device.ros_namespace.strip("/")
        a       = command.args or {}
        cmd_vel = f"/{ns}/cmd_vel"
        yy_cmd  = f"/{ns}/yy_command"
        Twist   = geometry_msgs.msg.Twist
        Cmd     = msg_yy.msg.cmd

        # A new command overrides any running stop/off tail on its OWN actuator
        # channel. Independent actuators never cancel each other: set_enc does not
        # touch the beep tail, but a new beep / beep_on does.
        channel = self._channel_of(command.name)
        self._cancel_tail(channel)

        if command.name == "move":
            # Stream the velocity setpoint for `duration`; on completion or interrupt,
            # stream a stop tail in the background (survives E-stop). Robust to frame loss.
            twist = Twist()
            twist.linear.x  = float(a.get("speed_lin", 0.0))
            twist.angular.z = float(a.get("speed_ang", 0.0))
            try:
                await self._stream(cmd_vel, Twist, twist, float(a.get("duration", 0.0)))
            finally:
                self._start_tail(channel, cmd_vel, Twist, Twist())

        elif command.name == "stop":
            # Stream zero Twist for the stop tail — reliably halts the robot.
            await self._stream(cmd_vel, Twist, Twist(), self._STOP_TAIL_S)

        elif command.name in ("dctl", "pidctl"):
            # Direct PWM or PID wheel speed.  arg: w_l, w_r in [-255, +255], optional duration.
            #
            # With `duration`: drive that long, then a background stop tail halts the robot
            # (timed move). WITHOUT `duration`: HOLD — set the wheel speed and do NOT stop;
            # the caller keeps it alive (streams the setpoint) and stops it explicitly
            # (e.g. dctl w_l=w_r=0 with a duration). Used by the joystick teleop so
            # consecutive setpoints don't fight a stop tail.
            #
            # Безопасность HOLD. Явная остановка теперь есть: stop_all зовёт
            # emergency_stop(), который шлёт нулевой dctl/pidctl напрямую, минуя
            # очередь, — раньше сорванный HOLD не останавливало вообще ничто на
            # сервере (отменять и прерывать было нечего: команда уже завершилась).
            #
            # Чего ещё НЕТ: автоматического стопа, когда клиент замолчал, но связь
            # цела, — его некому инициировать, для этого нужен серверный дедман
            # (control plane, фаза 1). Плюс стоп на дисконнект клиента.
            # Последний рубеж, если умрёт сам сервер, — прошивочный watchdog:
            # Wait_cmd_time ~2 c, и он под тумблером SW_CONNECT (при разомкнутом
            # входе INPUT_PULLUP читается как «выключено», то есть отказ проводки
            # тихо снимает защиту). Прошивка в рамках работы не менялась.
            yy = Cmd()
            yy.command = self._CMD[command.name]
            yy.arg     = [float(a.get("w_l", 0.0)), float(a.get("w_r", 0.0))]
            stop = Cmd()
            stop.command = self._CMD[command.name]
            stop.arg     = [0.0, 0.0]
            try:
                await self._stream(yy_cmd, Cmd, yy, float(a.get("duration", 0.0)))
            finally:
                if float(a.get("duration", 0.0)):   # HOLD (no duration) -> no auto-stop
                    self._start_tail(channel, yy_cmd, Cmd, stop)

        elif command.name == "beep":
            # Stream beep_on for `duration`, then stream beep_off as the tail.
            on = Cmd();  on.command  = self._CMD["beep_on"]
            off = Cmd(); off.command = self._CMD["beep_off"]
            try:
                await self._stream(yy_cmd, Cmd, on, float(a.get("duration", 0.5)))
            finally:
                self._start_tail(channel, yy_cmd, Cmd, off)

        elif command.name in ("beep_on", "beep_off", "gun_on", "gun_off", "compass_calibr"):
            # One-shot toggle — stream a short burst so a dropped frame doesn't lose it.
            # Its channel's tail was already cancelled above (beep_on cancels a running
            # beep-off tail; gun_on/off cancel the gun channel; compass_calibr is its own).
            yy = Cmd()
            yy.command = self._CMD[command.name]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_servo":
            # arg: a0, a1, a2 — angles in degrees for each of the 3 servos.
            yy = Cmd()
            yy.command = self._CMD["set_servo"]
            yy.angle   = [int(a.get("a0", 90)), int(a.get("a1", 90)), int(a.get("a2", 90))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name in ("set_pid", "set_pid_left", "set_pid_right"):
            # arg: kp, ki, kd
            yy = Cmd()
            yy.command = self._CMD[command.name]
            yy.arg     = [float(a.get("kp", 0.5)), float(a.get("ki", 0.02)), float(a.get("kd", 0.2))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_enc":
            # arg: left, right — reset encoder counters to these values.
            yy = Cmd()
            yy.command = self._CMD["set_enc"]
            yy.arg     = [float(a.get("left", 0)), float(a.get("right", 0))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_refl_dist":
            # arg: center, left, right — obstacle reflex distances in cm.
            yy = Cmd()
            yy.command = self._CMD["set_refl_dist"]
            yy.arg     = [float(a.get("center", 20)), float(a.get("left", 20)), float(a.get("right", 20))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_motors_ratio":
            # arg: left, right — scaling factors to balance drive motors.
            yy = Cmd()
            yy.command = self._CMD["set_motors_ratio"]
            yy.arg     = [float(a.get("left", 1.0)), float(a.get("right", 1.0))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_calibr_speed":
            # arg: speed (PWM during calibration spin), max_cnt (number of ticks).
            yy = Cmd()
            yy.command = self._CMD["set_calibr_speed"]
            yy.arg     = [float(a.get("speed", 40)), float(a.get("max_cnt", 600))]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        elif command.name == "set_klpf":
            # arg: k — low-pass filter coefficient for drive speed (0..1).
            yy = Cmd()
            yy.command = self._CMD["usr"]
            yy.arg     = [float(a.get("k", 1.0))]
            yy.da      = [self._SUBCMD["set_klpf"], 0, 0, 0]
            await self._stream(yy_cmd, Cmd, yy, self._ONESHOT_S)

        else:
            raise ValueError(f"Unknown command '{command.name}' for Yarp13Driver")


class SimpleSerialDevice(SerialBasedDriver):
    """
    Example driver for a minimal custom serial device.

    Protocol (newline-terminated ASCII):
      Telemetry from device: "uptime=<int>,value=<float>,status=<str>"
      Commands to device:    "<name> [arg1 arg2 ...]\n"

    Example commands:  "ON\n",  "OFF\n",  "SET_THRESHOLD 50\n"
    """

    # Left empty on purpose: this driver forwards whatever name it is given
    # straight to the device, so there is no fixed catalogue to declare.
    COMMANDS: Tuple[str, ...] = ()

    async def setup_telemetry(self):
        await self._serial.open(self._transport_path, self._device.baud_rate)
        self._serial.subscribe_async(self._transport_path, self._on_line)

    async def teardown_telemetry(self):
        await super().teardown_telemetry()
        await self._serial.close(self._transport_path)

    async def _on_line(self, line: bytes):
        try:
            parts = dict(kv.split("=", 1) for kv in line.decode().strip().split(","))
            telemetry = SimpleSerialTelemetry(
                uptime=int(parts["uptime"]),
                value=float(parts["value"]),
                status=parts["status"],
            )
        except Exception:
            return  # malformed line — silently skip

        self._latest_telemetry = telemetry
        await self._notify_listeners(telemetry)

    async def execute_command(self, command: Command):
        line = command.name
        if command.args:
            line += " " + " ".join(str(v) for v in command.args.values())
        await self._serial.write(self._transport_path, (line + "\n").encode())


class MicrobotDriver(TcpBasedDriver):
    """
    Driver for the three-wheeled micro-robot (firmware/esp32-microbot).

    Unlike yarp-13, there is no Arduino and no rosserial: the ESP32 is the whole
    robot. We open a socket to it and exchange newline-terminated ASCII.

    Commands:
        move     speed_lin [m/s], speed_ang [rad/s], duration [s]
        stop
        set_pid  kp, ki, kd
        arm / disarm   — enable / disable the power stage

    The setpoint is STREAMED for the duration of a move, exactly as for yarp-13
    and for the same reason: a dropped frame is corrected ~100 ms later instead
    of leaving the robot with a stale command. Here it is also what keeps the
    firmware deadman fed — the ESP32 zeroes its setpoint after CMD_TIMEOUT_MS
    without a TWIST, so a server that dies mid-move stops the robot rather than
    launching it across the room. That is the guarantee yarp-13 never had.
    """

    COMMANDS: Tuple[str, ...] = ("move", "stop", "set_pid", "arm", "disarm")

    _STREAM_HZ:   float = 10.0   # setpoint streaming rate, Hz
    _ONESHOT_S:   float = 0.3    # how long to repeat a one-shot command
    _STOP_TAIL_S: float = 0.6    # how long to keep streaming stop after motion ends
    _DEADMAN_S:   float = 0.5    # set_velocity deadman (mirrors the firmware timeout)

    # The robot has exactly one actuator group, so unlike Yarp13Driver there is
    # no channel map to maintain — every tail belongs to the drive.
    _DRIVE:  str = "drive"
    _ESTOP:  str = "__estop__"   # reserved slot: no command may cancel an E-stop

    def __init__(self, device: Device, tcp: TcpInterface):
        super().__init__(device, tcp)
        self._tails: Dict[str, asyncio.Task] = {}
        # Strong refs for the sync fire-and-forget paths (set_velocity,
        # emergency_stop). asyncio only keeps a weak one.
        self._bg_tasks: Set[asyncio.Task] = set()

    # --- plumbing ------------------------------------------------------------

    def _spawn(self, coro) -> asyncio.Task:
        task = asyncio.ensure_future(coro)
        self._bg_tasks.add(task)
        task.add_done_callback(self._bg_tasks.discard)
        return task

    async def _send(self, line: str):
        await self._tcp.write(self._device.name, (line + "\n").encode())

    async def _send_quiet(self, line: str):
        """Send, swallowing a lost connection. For background tails and E-stop."""
        try:
            await self._send(line)
        except (ConnectionError, RuntimeError):
            pass

    async def _stream(self, line: str, duration: float):
        """Send `line` at _STREAM_HZ for `duration` seconds (at least once)."""
        period = 1.0 / self._STREAM_HZ
        n = max(1, int(round(duration / period)))
        for _ in range(n):
            await self._send(line)
            await asyncio.sleep(period)

    def _cancel_tail(self, channel: str):
        task = self._tails.pop(channel, None)
        if task is not None and not task.done():
            task.cancel()

    def _start_stop_tail(self):
        """
        Keep streaming STOP in the BACKGROUND after a move ends, so the halt
        survives cancellation of the owning command coroutine (interrupt/E-stop).
        """
        self._cancel_tail(self._DRIVE)
        self._tails[self._DRIVE] = asyncio.ensure_future(
            self._stream_quiet("STOP", self._STOP_TAIL_S)
        )

    async def _stream_quiet(self, line: str, duration: float):
        period = 1.0 / self._STREAM_HZ
        for _ in range(max(1, int(round(duration / period)))):
            await self._send_quiet(line)
            await asyncio.sleep(period)

    # --- direct control, for server-side procedures ---------------------------

    def set_velocity(self, v: float, omega: float):
        """
        Push ONE twist setpoint immediately, for procedures that run their own
        control loop (AllGoHome). Duck-typed: Procedures looks for this method by
        name, so implementing it is what makes the micro-robot a first-class
        participant in group navigation alongside yarp-13.

        Synchronous by contract (the procedure loop calls it without awaiting),
        so the write is fired off as a task. Each call re-arms a deadman: if
        setpoints stop arriving the robot is stopped from here too, not only by
        the firmware timeout.
        """
        self._cancel_tail(self._DRIVE)
        self._spawn(self._send_quiet(f"TWIST {float(v):.4f} {float(omega):.4f}"))
        self._tails[self._DRIVE] = asyncio.ensure_future(self._deadman())

    async def _deadman(self):
        try:
            await asyncio.sleep(self._DEADMAN_S)
        except asyncio.CancelledError:
            return
        await self._stream_quiet("STOP", self._STOP_TAIL_S)

    def emergency_stop(self) -> None:
        """
        Halt the drive right now and keep saying so. See AbstractDriver.emergency_stop.

        Also disarms the power stage: unlike the yarp-13 path, this robot has a
        firmware-level enable, so an E-stop can actually cut the motors instead
        of only commanding zero speed.
        """
        for channel in list(self._tails):
            self._cancel_tail(channel)
        self._spawn(self._send_quiet("STOP"))
        self._spawn(self._send_quiet("ARM 0"))
        self._tails[self._ESTOP] = asyncio.ensure_future(
            self._stream_quiet("STOP", self._STOP_TAIL_S)
        )

    # --- lifecycle -----------------------------------------------------------

    async def setup_telemetry(self):
        await self._tcp.open(self._device.name, self._device.ip, self._device.port, self._on_line)

    async def teardown_telemetry(self):
        await super().teardown_telemetry()
        for channel in list(self._tails):
            self._cancel_tail(channel)
        await self._tcp.close(self._device.name)

    async def _on_line(self, line: bytes):
        text = line.decode(errors="ignore").strip()
        if not text or "=" not in text:
            return  # PONG, boot banners, anything that isn't a telemetry frame

        try:
            kv = dict(part.split("=", 1) for part in text.split(",") if "=" in part)
            telemetry = MicrobotTelemetry(
                uptime_ms=int(kv["t"]),
                rf_left=int(kv["rf_l"]),
                rf_center=int(kv["rf_c"]),
                rf_right=int(kv["rf_r"]),
                vbat=float(kv["vbat"]),
                speed_lin=float(kv["v"]),
                speed_ang=float(kv["w"]),
                pwm_left=int(kv["pwm_l"]),
                pwm_right=int(kv["pwm_r"]),
                armed=kv["armed"] == "1",
                kp=float(kv["kp"]),
                ki=float(kv["ki"]),
                kd=float(kv["kd"]),
            )
        except (KeyError, ValueError):
            return  # malformed frame — skip it, the next one is 100 ms away

        self._latest_telemetry = telemetry
        await self._notify_listeners(telemetry)

    # --- commands ------------------------------------------------------------

    async def execute_command(self, command: Command):
        a = command.args or {}

        if command.name == "move":
            # Stream the twist for `duration`, then hand over to a background
            # stop tail (which outlives cancellation of this coroutine).
            line = (
                f"TWIST {float(a.get('speed_lin', 0.0)):.4f} "
                f"{float(a.get('speed_ang', 0.0)):.4f}"
            )
            self._cancel_tail(self._DRIVE)
            try:
                await self._stream(line, float(a.get("duration", 0.0)))
            finally:
                self._start_stop_tail()

        elif command.name == "stop":
            self._cancel_tail(self._DRIVE)
            await self._stream("STOP", self._STOP_TAIL_S)

        elif command.name == "set_pid":
            # Gains are accepted and stored by the firmware today, but the loop
            # there is still open — there are no encoders to close it with. The
            # values come back in telemetry, so a client can verify they landed.
            await self._stream(
                f"PID {float(a.get('kp', 0.8)):.4f} "
                f"{float(a.get('ki', 0.0)):.4f} "
                f"{float(a.get('kd', 0.0)):.4f}",
                self._ONESHOT_S,
            )

        elif command.name in ("arm", "disarm"):
            if command.name == "disarm":
                self._cancel_tail(self._DRIVE)
            await self._stream(f"ARM {1 if command.name == 'arm' else 0}", self._ONESHOT_S)

        else:
            raise ValueError(f"Unknown command '{command.name}' for MicrobotDriver")


# ---------------------------- CUSTOM DRIVERS GO ABOVE ----------------------------


class DriverFactory:
    """
    Factory for creating drivers.
    """
    def __init__(self, ros: RosInterface, serial: SerialInterface, tcp: TcpInterface):
        self._ros: RosInterface = ros
        self._serial: SerialInterface = serial
        self._tcp: TcpInterface = tcp

    def create_driver(self, device: Device) -> AbstractDriver:
        if device.driver == "yarp13":
            return Yarp13Driver(device, self._ros)
        elif device.driver == "microbot":
            return MicrobotDriver(device, self._tcp)
        elif device.driver == "simple_serial":
            return SimpleSerialDevice(device, self._serial)
        else:
            raise ValueError(f"Unknown driver: {device.driver}")
