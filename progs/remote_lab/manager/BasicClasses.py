from dataclasses import dataclass, field
from pathlib import Path
import uuid
from typing import Optional, List, Dict
import time


@dataclass(frozen=True)
class Device:
    name: str
    ip: str
    port: int
    shared: bool
    active: bool
    driver: str
    # driver-specific fields
    ros_namespace: Optional[str] # for ros-based drivers
    baud_rate: Optional[int] # for serial-based drivers
    # ... for <smth>-based drivers


class User:
    def __init__(self):
        self._id = str(uuid.uuid4())

    def __hash__(self):
        return hash(self._id)

    def __eq__(self, other):
        return self._id == other._id # I am SO sorry, I wish Python 3.8 had friend-functions


# TODO
@dataclass(order=True)
class Command:
    """
    Command object stored in device priority queues.

    Ordering is based on (priority, timestamp) so PriorityQueue
    executes higher priority commands first and keeps FIFO
    ordering for equal priorities.
    """

    priority: int
    timestamp: float = field(init=False, compare=True)

    # command_id: str = field(default_factory=lambda: str(uuid.uuid4()), compare=False)
    # client_id: str = field(default="", compare=False)
    #
    # devices: List[str] = field(default_factory=list, compare=False)

    name: str = field(default="", compare=False)
    args: dict = field(default_factory=dict, compare=False)

    def __post_init__(self):
        self.timestamp = time.time()


