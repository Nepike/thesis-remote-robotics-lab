from dataclasses import dataclass
from pathlib import Path

DEVICES_TTY_ROOT = "/tmp/remote_lab_tty"
Path(DEVICES_TTY_ROOT).mkdir(parents=True, exist_ok=True)


@dataclass
class Device:
    name: str
    ip: str
    port: int

    type: str # see DeviceDrivers.py
    protocol: str # "ros", "serial", ...
    config: dict # protocol-specific config (e.g. ros_namespace, seral_baud)

    shared: bool
    active: bool

    @property
    def tty_path(self):
        return str(Path(DEVICES_TTY_ROOT) / f"ttyDEVICE-{self.name}")



class User:
    ...