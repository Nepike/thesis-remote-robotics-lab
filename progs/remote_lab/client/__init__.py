"""
RemoteLab client package.

Import everything you need from here:
    from client import RemoteLab, AcquireError, SubmitError
"""

from .client import AcquireContext, CommandHandle, DeviceProxy, RemoteLab
from ._connection import (
    AcquireError,
    ConnectionLostError,
    RemoteLabError,
    SubmitError,
)

__all__ = [
    "RemoteLab",
    "DeviceProxy",
    "CommandHandle",
    "AcquireContext",
    "RemoteLabError",
    "ConnectionLostError",
    "AcquireError",
    "SubmitError",
]
