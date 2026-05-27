from abc import ABC, abstractmethod
from typing import Any, Awaitable, Callable, Dict, Optional, Tuple
import asyncio
from pathlib import Path
import os

# Available once the package has been built via catkin and the ROS environment has been loaded (source devel/setup.bash).
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
        Executes a command via some HardwareInterface.

        You must override this method for every device type.
        """

    # ПРИМЕР
    # Правильно - можно прервать:
    # async def execute_command(self, command):
    #     self._ros.publish("/cmd_vel", Twist, move_msg)
    #     await asyncio.sleep(command.args["duration"])  # <- CancelledError сюда
    #     self._ros.publish("/cmd_vel", Twist, stop_msg)
    #
    # Неправильно - невозможно прервать:
    # async def execute_command(self, command):
    #     time.sleep(command.args["duration"])  # blocking — заблокирует весь event loop
    # pass
    # крч, длинные команды обязаны быть асинхронными, чтобы их можно было прервать...

    async def setup_telemetry(self):
        """Subscribe to device telemetry sources. Called after adapters are started."""
        pass

    async def teardown_telemetry(self):
        """Unsubscribe / close telemetry connections. Called before processes are stopped."""
        pass

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
        # TODO
        pass


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
