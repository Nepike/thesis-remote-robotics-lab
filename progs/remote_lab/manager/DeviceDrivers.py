from abc import ABC, abstractmethod
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple
import asyncio
from pathlib import Path
import os

# Available once the package has been built via catkin and the ROS environment has been loaded (source devel/setup.bash).
import geometry_msgs.msg
import msg_yy.msg

from BasicClasses import Device, Command
from HardwareInterfaces import RosInterface, SerialInterface
from Logger import Logger
from TelemetryTypes import Yarp13Telemetry, SimpleSerialTelemetry


class AbstractDriver(ABC):
    """
    Base class for all device drivers.

    Each driver must implement command execution logic and interface-required processes (transports, adapters) start logic
    """

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
            yy = Cmd()
            yy.command = self._CMD[command.name]
            yy.arg     = [float(a.get("w_l", 0.0)), float(a.get("w_r", 0.0))]
            stop = Cmd()
            stop.command = self._CMD[command.name]
            stop.arg     = [0.0, 0.0]
            try:
                await self._stream(yy_cmd, Cmd, yy, float(a.get("duration", 0.0)))
            finally:
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


# ---------------------------- CUSTOM DRIVERS GO ABOVE ----------------------------


class DriverFactory:
    """
    Factory for creating drivers.
    """
    def __init__(self, ros: RosInterface, serial: SerialInterface):
        self._ros: RosInterface = ros
        self._serial: SerialInterface = serial

    def create_driver(self, device: Device) -> AbstractDriver:
        if device.driver == "yarp13":
            return Yarp13Driver(device, self._ros)
        elif device.driver == "simple_serial":
            return SimpleSerialDevice(device, self._serial)
        else:
            raise ValueError(f"Unknown driver: {device.driver}")
