from abc import ABC, abstractmethod
from typing import Tuple
import asyncio
from pathlib import Path

from BasicClasses import Device, Command
from HardwareInterfaces import RosInterface, SerialInterface
from Logger import Logger


class AbstractDriver(ABC):
    """
    Base class for all device drivers.

    Each driver must implement command execution logic and interface-required processes (adapters) start logic
    """

    def __init__(self, device: Device):
        self._device: Device = device

    @abstractmethod
    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts interface-required processes (e.g. roslaunch) and returns a tuple of them.
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

# ---------------------------- BASIC INTERFACE-BASED DRIVERS GO HERE ----------------------------
class RosBasedDriver(AbstractDriver):
    """
    Base class for all ros-based devices.
    """
    def __init__(self, device: Device, ros: RosInterface):
        super().__init__(device)
        self._ros: RosInterface = ros

    async def start_adapters(self) -> Tuple[asyncio.subprocess.Process, ...]:
        """
        Starts roslaunch.
        """
        cmd = (
            f"bash -c 'source /opt/ros/noetic/setup.bash && "
            f"source {Path.home()}/ros/devel/setup.bash && "
            f"roslaunch yyctl rosserial.launch "
            f"port:={self._device.tty_path} __ns:={self._device.ros_namespace}'"
        )

        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        await asyncio.sleep(1)  # God help us

        logger = Logger.get()
        await logger.log("DRIVER",  f"roslaunch for {self._device.name} started")
        logger.attach_stream("DRIVER", proc.stdout)
        logger.attach_stream("DRIVER_ERR", proc.stderr)

        return tuple([proc])

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
# The class will have the following fields available: `self._device`, `self._<interface>`


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
