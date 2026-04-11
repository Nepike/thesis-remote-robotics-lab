from abc import ABC, abstractmethod
from typing import Tuple
import asyncio
from pathlib import Path
import os

from BasicClasses import Device, Command
from HardwareInterfaces import RosInterface, SerialInterface
from Logger import Logger


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
        pass


    # TODO telemetry
    # def register_telemetry(...):
    #     pass


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
    async def execute_command(self, command: Command):
        # TODO
        pass


# ---------------------------- CUSTOM DRIVERS GO ABOVE ----------------------------


class DriverFactory:
    def __init__(self, ros: RosInterface, serial: SerialInterface):
        self._ros: RosInterface = ros
        self._serial: SerialInterface = serial

    def create_driver(self, device: Device) -> AbstractDriver:
        if device.driver == "yarp13":
            return Yarp13Driver(device, self._ros)
        elif ...:
            return ...
        else:
            raise ValueError(f"Unknown driver: {device.driver}")
