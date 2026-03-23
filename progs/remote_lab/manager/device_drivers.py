from abc import ABC, abstractmethod
from typing import Dict, Callable


class BaseDeviceDriver(ABC):
    """
    Base class for all device drivers.

    Each driver must implement command execution logic.
    """

    def __init__(self):
        self._commands: Dict[str, Callable] = {}


    @abstractmethod
    async def execute(self, ros, namespace: str, command):
        """
        Execute command.

        Parameters
        ----------
        ros : RosInterface
        namespace : str
        command : Command
        """
        pass


    def register_telemetry(self, ros, namespace: str, callback_map: Dict[str, Callable]):
        """
        Register telemetry subscriptions.

        callback_map:
            topic_name → callback
        """
        # optional override
        pass



class DriverRegistry:
    """
    Stores mapping: device_type → driver instance
    """

    def __init__(self):
        self._drivers: Dict[str, BaseDeviceDriver] = {}



    def register(self, device_type: str, driver: BaseDeviceDriver):

        if device_type in self._drivers:
            raise ValueError(f"Driver already registered for type '{device_type}'")

        self._drivers[device_type] = driver



    def get(self, device_type: str) -> BaseDeviceDriver:

        driver = self._drivers.get(device_type)

        if driver is None:
            raise ValueError(f"No driver for device type '{device_type}'")

        return driver




# ROS message import (пример)
from geometry_msgs.msg import Twist


class MotorDriver(BaseDeviceDriver):
    """
    Driver for motor devices.

    Supports:
    - move
    - stop
    """

    def __init__(self):
        super().__init__()



    async def execute(self, ros, namespace: str, command):

        if command.name == "move":
            await self._move(ros, namespace, command.args)

        elif command.name == "stop":
            await self._stop(ros, namespace)

        else:
            raise ValueError(f"Unknown motor command: {command.name}")



    async def _move(self, ros, namespace: str, args: dict):

        speed = args.get("speed", 0.0)

        msg = Twist()
        msg.linear.x = speed

        ros.publish(
            f"{namespace}/cmd_vel",
            Twist,
            msg
        )



    async def _stop(self, ros, namespace: str):

        msg = Twist()
        msg.linear.x = 0.0

        ros.publish(
            f"{namespace}/cmd_vel",
            Twist,
            msg
        )



    def register_telemetry(self, ros, namespace: str, callback_map):

        topic = f"{namespace}/state"

        if topic in callback_map:

            ros.subscribe(
                topic,
                Twist,  # пример
                callback_map[topic]
            )