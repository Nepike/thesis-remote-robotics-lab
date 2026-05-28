"""
RemoteLab client package.

Import everything you need from here:
    from client import RemoteLab, AcquireError, SubmitError, ProcedureError
"""

from .client import AcquireContext, CommandHandle, DeviceProxy, ProcedureHandle, RemoteLab
from ._connection import (
    AcquireError,
    ConnectionLostError,
    ProcedureError,
    RemoteLabError,
    SubmitError,
)

__all__ = [
    "RemoteLab",
    "DeviceProxy",
    "CommandHandle",
    "AcquireContext",
    "ProcedureHandle",
    "RemoteLabError",
    "ConnectionLostError",
    "AcquireError",
    "SubmitError",
    "ProcedureError",
]
