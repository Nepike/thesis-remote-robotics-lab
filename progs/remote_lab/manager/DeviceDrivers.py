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

    # One-shot commands can be lost or corrupted on the (flaky) serial link, and a
    # single drop swallows the whole command. Sending each command a few times
    # back-to-back makes a transient drop far less likely to lose it entirely.
    # Especially important for the stop/off messages in `finally` blocks (safety).
    _CMD_REPEAT: int = 3

    def _publish_repeated(self, topic, msg_type, message, times: Optional[int] = None):
        """Publish the same command several times (see _CMD_REPEAT) for robustness."""
        n = self._CMD_REPEAT if times is None else times
        for _ in range(n):
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
        ns = self._device.ros_namespace.strip("/")
        a  = command.args or {}

        if command.name == "move":
            # Publish velocity and hold for `duration` seconds.
            # On interrupt (CancelledError), the finally block sends a stop before propagating.
            twist = geometry_msgs.msg.Twist()
            twist.linear.x  = float(a.get("speed_lin", 0.0))
            twist.angular.z = float(a.get("speed_ang", 0.0))
            self._publish_repeated(f"/{ns}/cmd_vel", geometry_msgs.msg.Twist, twist)
            try:
                await asyncio.sleep(float(a.get("duration", 0.0)))
            finally:
                self._publish_repeated(
                    f"/{ns}/cmd_vel", geometry_msgs.msg.Twist, geometry_msgs.msg.Twist()
                )

        elif command.name == "stop":
            # Zero Twist stops both motors regardless of current control mode.
            self._publish_repeated(
                f"/{ns}/cmd_vel", geometry_msgs.msg.Twist, geometry_msgs.msg.Twist()
            )

        elif command.name in ("dctl", "pidctl"):
            # Direct PWM or PID wheel speed.  arg: w_l, w_r in [-255, +255], optional duration.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD[command.name]
            yy.arg     = [float(a.get("w_l", 0.0)), float(a.get("w_r", 0.0))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)
            try:
                await asyncio.sleep(float(a.get("duration", 0.0)))
            finally:
                stop = msg_yy.msg.cmd()
                stop.command = self._CMD[command.name]
                stop.arg     = [0.0, 0.0]
                self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, stop)

        elif command.name in ("beep_on", "beep_off", "gun_on", "gun_off", "compass_calibr"):
            # Instantaneous toggle commands — no duration.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD[command.name]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "beep":
            # Convenience: beep_on → sleep → beep_off.  arg: duration (seconds).
            on = msg_yy.msg.cmd()
            on.command = self._CMD["beep_on"]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, on)
            try:
                await asyncio.sleep(float(a.get("duration", 0.5)))
            finally:
                off = msg_yy.msg.cmd()
                off.command = self._CMD["beep_off"]
                self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, off)

        elif command.name == "set_servo":
            # arg: a0, a1, a2 — angles in degrees for each of the 3 servos.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["set_servo"]
            yy.angle   = [int(a.get("a0", 90)), int(a.get("a1", 90)), int(a.get("a2", 90))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name in ("set_pid", "set_pid_left", "set_pid_right"):
            # arg: kp, ki, kd
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD[command.name]
            yy.arg     = [float(a.get("kp", 0.5)), float(a.get("ki", 0.02)), float(a.get("kd", 0.2))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "set_enc":
            # arg: left, right — reset encoder counters to these values.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["set_enc"]
            yy.arg     = [float(a.get("left", 0)), float(a.get("right", 0))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "set_refl_dist":
            # arg: center, left, right — obstacle reflex distances in cm.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["set_refl_dist"]
            yy.arg     = [float(a.get("center", 20)), float(a.get("left", 20)), float(a.get("right", 20))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "set_motors_ratio":
            # arg: left, right — scaling factors to balance drive motors.
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["set_motors_ratio"]
            yy.arg     = [float(a.get("left", 1.0)), float(a.get("right", 1.0))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "set_calibr_speed":
            # arg: speed (PWM during calibration spin), max_cnt (number of ticks).
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["set_calibr_speed"]
            yy.arg     = [float(a.get("speed", 40)), float(a.get("max_cnt", 600))]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

        elif command.name == "set_klpf":
            # arg: k — low-pass filter coefficient for drive speed (0..1).
            yy = msg_yy.msg.cmd()
            yy.command = self._CMD["usr"]
            yy.arg     = [float(a.get("k", 1.0))]
            yy.da      = [self._SUBCMD["set_klpf"], 0, 0, 0]
            self._publish_repeated(f"/{ns}/yy_command", msg_yy.msg.cmd, yy)

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
