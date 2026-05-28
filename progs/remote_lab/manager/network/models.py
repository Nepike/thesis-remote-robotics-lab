"""
Wire protocol models.

All WebSocket messages are JSON objects with a "type" discriminator field.

Client -> Server: IncomingMessage (discriminated union, parse with TypeAdapter)
Server -> Client: individual Outgoing* models, serialized with .model_dump_json()
"""

from __future__ import annotations

from typing import Any, Dict, List, Literal, Union

try:
    from typing import Annotated          # Python 3.9+
except ImportError:
    from typing_extensions import Annotated  # Python 3.8

from pydantic import BaseModel, Field


# Client -> Server

class SubmitMessage(BaseModel):
    """Submit a command to one or more devices."""
    type: Literal["submit"] = "submit"
    devices: List[str]
    name: str
    priority: int = 5
    args: Dict[str, Any] = Field(default_factory=dict)


class AcquireMessage(BaseModel):
    """Acquire exclusive ownership of a device before sending commands to it."""
    type: Literal["acquire"] = "acquire"
    device: str


class ReleaseMessage(BaseModel):
    """Release exclusive ownership of a device."""
    type: Literal["release"] = "release"
    device: str


class CancelMessage(BaseModel):
    """Cancel a queued command by its id. No effect if already executing."""
    type: Literal["cancel"] = "cancel"
    command_id: str


class InterruptMessage(BaseModel):
    """
    Interrupt the command currently executing on a device.

    Unlike CancelMessage (which removes a command from the queue),
    InterruptMessage raises CancelledError inside the running execute_command()
    coroutine. The worker then immediately picks up the next command in the queue.
    No effect if the device is idle.
    """
    type: Literal["interrupt"] = "interrupt"
    device: str


class SubscribeTelemetryMessage(BaseModel):
    """Start receiving telemetry pushes for a device."""
    type: Literal["subscribe_telemetry"] = "subscribe_telemetry"
    device: str


class UnsubscribeTelemetryMessage(BaseModel):
    """Stop receiving telemetry pushes for a device."""
    type: Literal["unsubscribe_telemetry"] = "unsubscribe_telemetry"
    device: str


class GetDevicesMessage(BaseModel):
    """Request the list of registered active devices."""
    type: Literal["get_devices"] = "get_devices"


class RunProcedureMessage(BaseModel):
    """Start a named group procedure on the server."""
    type: Literal["run_procedure"] = "run_procedure"
    name: str
    args: Dict[str, Any] = Field(default_factory=dict)


class CancelProcedureMessage(BaseModel):
    """Cancel a running procedure by its id."""
    type: Literal["cancel_procedure"] = "cancel_procedure"
    procedure_id: str


# Discriminated union — parse any incoming message with:
#   TypeAdapter(IncomingMessage).validate_json(raw)
IncomingMessage = Annotated[
    Union[
        SubmitMessage,
        AcquireMessage,
        ReleaseMessage,
        CancelMessage,
        InterruptMessage,
        SubscribeTelemetryMessage,
        UnsubscribeTelemetryMessage,
        GetDevicesMessage,
        RunProcedureMessage,
        CancelProcedureMessage,
    ],
    Field(discriminator="type"),
]


# Server -> Client

class AckMessage(BaseModel):
    """Command was validated and placed in the queue."""
    type: Literal["ack"] = "ack"
    command_id: str
    status: Literal["queued"] = "queued"


class DoneMessage(BaseModel):
    """
    Command finished executing on a specific device.

    For multi-device commands the client receives one DoneMessage per device.
    The client-side robot.move() unblocks when it has received Done from all
    targeted devices.
    """
    type: Literal["done"] = "done"
    command_id: str
    device: str


class TelemetryMessage(BaseModel):
    """
    Telemetry snapshot pushed to subscribed clients.

    `data` mirrors the fields of the driver's telemetry dataclass
    (e.g. Yarp13Telemetry), serialized to a plain dict.
    """
    type: Literal["telemetry"] = "telemetry"
    device: str
    data: Dict[str, Any]


class DeviceInfo(BaseModel):
    """Lightweight device descriptor sent to clients."""
    name: str
    driver: str
    shared: bool


class DevicesMessage(BaseModel):
    """Response to GetDevicesMessage."""
    type: Literal["devices"] = "devices"
    data: List[DeviceInfo]


class ProcedureAckMessage(BaseModel):
    """Procedure was accepted and started; procedure_id identifies it from now on."""
    type: Literal["procedure_ack"] = "procedure_ack"
    procedure_id: str


class ProcedureDoneMessage(BaseModel):
    """Procedure finished successfully."""
    type: Literal["procedure_done"] = "procedure_done"
    procedure_id: str


class ProcedureErrorMessage(BaseModel):
    """Procedure failed with an exception."""
    type: Literal["procedure_error"] = "procedure_error"
    procedure_id: str
    message: str


class ErrorMessage(BaseModel):
    """
    Server-side error that the client should surface to the caller.

    Codes (string enum, intentionally not a Python Enum to stay wire-stable):
        UNKNOWN_DEVICE     — device name not registered
        ACCESS_DENIED      — client does not hold exclusive lock
        ALREADY_OWNED      — acquire failed, device owned by another client
        UNKNOWN_PROCEDURE  — procedure name not registered on the server
        UNKNOWN_MESSAGE    — unrecognized message type
        INTERNAL           — unexpected server-side exception
    """
    type: Literal["error"] = "error"
    code: str
    message: str
