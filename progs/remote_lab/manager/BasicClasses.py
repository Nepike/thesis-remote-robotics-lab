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


@dataclass(eq=False)
class Command:
    """
    Command object stored in device priority queues.

    Ordering: higher priority number -> executed first.
    Equal priority -> FIFO by submission timestamp.
    """

    priority: int
    client_id: str
    devices: List[str]
    name: str = ""
    args: dict = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: float = field(init=False)

    def __post_init__(self):
        self.timestamp = time.time()

    # Identity is determined solely by command_id
    def __eq__(self, other: object) -> bool:
        return isinstance(other, Command) and self.command_id == other.command_id

    def __hash__(self) -> int:
        return hash(self.command_id)

    # PriorityQueue (min-heap): __lt__ returning True means "I go first"
    def __lt__(self, other: "Command") -> bool:
        if self.priority != other.priority:
            return self.priority > other.priority   # higher number -> first
        return self.timestamp < other.timestamp     # earlier submission -> FIFO


