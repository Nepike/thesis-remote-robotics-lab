from dataclasses import dataclass
from pathlib import Path
import uuid
from typing import Optional

DEVICES_TTY_ROOT = "/tmp/remote_lab_tty"
Path(DEVICES_TTY_ROOT).mkdir(parents=True, exist_ok=True)


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

    @property
    def tty_path(self) -> str:
        return str(Path(DEVICES_TTY_ROOT) / f"ttyDEVICE-{self.name}")


class User:
    def __init__(self):
        self._id = str(uuid.uuid4())

    def __hash__(self):
        return hash(self._id)

    def __eq__(self, other):
        return self._id == other._id # I am SO sorry, I wish Python 3.8 had friend-functions


