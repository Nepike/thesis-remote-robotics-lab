"""
WebSocket connection handler.

Endpoint:      ws://<host>/ws
Auth:          HTTP Basic Auth on the upgrade request (see auth.py).
Message format: JSON with a "type" discriminator (see models.py).

Startup wiring (call once before accepting connections):
    from network.ws_handler import init
    init(manager)
"""

import asyncio
import dataclasses
from typing import Dict, Optional

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from pydantic import TypeAdapter, ValidationError

from BasicClasses import Command
from DeviceDrivers import AbstractDriver
from manager import RemoteLabManager
from network.auth import get_client_id
from network.models import (
    AckMessage,
    AcquireMessage,
    CancelMessage,
    DeviceInfo,
    DevicesMessage,
    DoneMessage,
    ErrorMessage,
    GetDevicesMessage,
    IncomingMessage,
    ReleaseMessage,
    SubmitMessage,
    SubscribeTelemetryMessage,
    TelemetryMessage,
    UnsubscribeTelemetryMessage,
)

# Pre-compiled parser for all incoming message types.
_incoming_adapter = TypeAdapter(IncomingMessage)

# FastAPI router - collects route definitions from this module.
router = APIRouter()

# Both vars are None at import time and filled in by init() at startup.
_manager:      Optional[RemoteLabManager]    = None
_conn_manager: Optional["ConnectionManager"] = None


class ClientSession:
    """
    Holds all mutable state for one connected WebSocket client
    """
    def __init__(self, client_id: str, websocket: WebSocket):
        self.client_id = client_id
        self.websocket = websocket
        self._telemetry_subs: Dict[str, int] = {}
        # WebSocket.send_text() is a coroutine but is not safe to call. The lock serializes sends
        self._send_lock = asyncio.Lock()

    async def send(self, msg) -> None:
        """
        Serialize a Pydantic model to JSON and send it over the WebSocket.
        Silently drops the message if the socket is already closed -
        the main receive loop will notice the disconnect on the next iteration.
        """
        async with self._send_lock:
            try:
                await self.websocket.send_text(msg.model_dump_json())
            except Exception:
                pass

    async def subscribe_telemetry(self, device_name: str, driver: AbstractDriver) -> None:
        """
        Register an async callback on the driver so that every new telemetry
        snapshot is forwarded to this client as a TelemetryMessage.
        Calling this twice for the same device is a no-op.
        """
        if device_name in self._telemetry_subs:
            return

        async def _on_telemetry(telemetry) -> None:
            await self.send(TelemetryMessage(device=device_name, data=dataclasses.asdict(telemetry)))

        lid = driver.add_telemetry_listener(_on_telemetry)
        self._telemetry_subs[device_name] = lid

    def unsubscribe_telemetry(self, device_name: str, driver: AbstractDriver) -> None:
        lid = self._telemetry_subs.pop(device_name, None)
        if lid is not None:
            driver.remove_telemetry_listener(lid)

    def unsubscribe_all_telemetry(self) -> None:
        """
        Remove all telemetry listeners for this client.
        (disconnection)
        """
        for device_name, lid in list(self._telemetry_subs.items()):
            driver = _manager.get_driver(device_name)
            if driver:
                driver.remove_telemetry_listener(lid)
        self._telemetry_subs.clear()


class ConnectionManager:
    """
    Global registry of all live WebSocket sessions.

    Responsibilities:
    - Accept new connections and close old ones for the same user.
    - Route command-completion events to the client that submitted the command.

    One session per username: if the same user opens a second connection, the first one is forcibly closed.
    """

    def __init__(self):
        self._sessions: Dict[str, ClientSession] = {} # username -> active session
        self._command_owners: Dict[str, str] = {} # command_id -> username
        self._command_pending: Dict[str, int] = {} # command_id -> number of workers

    async def connect(self, client_id: str, websocket: WebSocket) -> ClientSession:
        """
        Accept the WebSocket handshake, replace any existing session for this user, and return a new ClientSession
        """
        await websocket.accept()
        old = self._sessions.get(client_id)
        if old:
            await old.websocket.close(code=1008, reason="Replaced by a new connection")
        session = ClientSession(client_id, websocket)
        self._sessions[client_id] = session
        return session

    def disconnect(self, client_id: str) -> None:
        self._sessions.pop(client_id, None)

    def register_command(self, command_id: str, client_id: str, num_devices: int) -> None:
        """
        Called right after a command is queued.
        Records which client owns it and how many devices will execute it
        (so on_command_complete() knows where to send DoneMessages)
        """
        self._command_owners[command_id]  = client_id
        self._command_pending[command_id] = num_devices

    async def on_command_complete(self, command: Command, device_name: str) -> None:
        """
        Called by RemoteLabManager._on_execute_command() after execute_command() returns.

        Sends a DoneMessage to the client that submitted the command.
        When all targeted devices have finished, cleans up the tracking entries.
        """
        client_id = self._command_owners.get(command.command_id)
        if client_id:
            session = self._sessions.get(client_id)
            if session:
                await session.send(DoneMessage(command_id=command.command_id, device=device_name))

        remaining = self._command_pending.get(command.command_id, 1) - 1
        if remaining <= 0:
            self._command_owners.pop(command.command_id, None)
            self._command_pending.pop(command.command_id, None)
        else:
            self._command_pending[command.command_id] = remaining


def init(manager: RemoteLabManager) -> None:
    """
    Wire the WebSocket layer to the manager.
    Must be called after manager.start() and before the server starts accepting connections.
    """
    global _manager, _conn_manager
    _manager = manager
    _conn_manager = ConnectionManager()

    _manager.on_command_complete = _conn_manager.on_command_complete


# Message handlers.
# Each function handles one specific incoming message type.
# They receive the session (so they can send responses) and the parsed message

async def _handle_submit(session: ClientSession, msg: SubmitMessage) -> None:
    try:
        command_id = await _manager.submit_command(
            client_id=session.client_id,
            devices=msg.devices,
            command_name=msg.name,
            priority=msg.priority,
            args=msg.args,
        )
    except PermissionError as e:
        await session.send(ErrorMessage(code="ACCESS_DENIED", message=str(e)))
        return
    except ValueError as e:
        await session.send(ErrorMessage(code="UNKNOWN_DEVICE", message=str(e)))
        return

    _conn_manager.register_command(command_id, session.client_id, len(msg.devices))
    await session.send(AckMessage(command_id=command_id))


async def _handle_acquire(session: ClientSession, msg: AcquireMessage) -> None:
    ok = _manager.acquire_device(msg.device, session.client_id)
    if not ok:
        await session.send(ErrorMessage(
            code="ALREADY_OWNED",
            message=f"Device '{msg.device}' is already owned by another client.",
        ))


async def _handle_release(session: ClientSession, msg: ReleaseMessage) -> None:
    _manager.release_device(msg.device, session.client_id)


async def _handle_cancel(session: ClientSession, msg: CancelMessage) -> None:
    _manager.cancel_command(msg.command_id)


async def _handle_subscribe(session: ClientSession, msg: SubscribeTelemetryMessage) -> None:
    driver = _manager.get_driver(msg.device)
    if driver is None:
        await session.send(ErrorMessage(code="UNKNOWN_DEVICE", message=f"Unknown device: '{msg.device}'"))
        return
    await session.subscribe_telemetry(msg.device, driver)


async def _handle_unsubscribe(session: ClientSession, msg: UnsubscribeTelemetryMessage) -> None:
    driver = _manager.get_driver(msg.device)
    if driver:
        session.unsubscribe_telemetry(msg.device, driver)


async def _handle_get_devices(session: ClientSession, _msg: GetDevicesMessage) -> None:
    infos = [
        DeviceInfo(name=d.name, driver=d.driver, shared=d.shared)
        for d in _manager.get_devices()
    ]
    await session.send(DevicesMessage(data=infos))


async def _dispatch(session: ClientSession, raw: str) -> None:
    """
    Parse one raw JSON string into a typed message and call the right handler.
    Sends ErrorMessage back to the client on parse failures or handler exceptions.
    """
    try:
        msg = _incoming_adapter.validate_json(raw)
    except ValidationError as e:
        await session.send(ErrorMessage(code="UNKNOWN_MESSAGE", message=str(e)))
        return

    try:
        if isinstance(msg, SubmitMessage):
            await _handle_submit(session, msg)
        elif isinstance(msg, AcquireMessage):
            await _handle_acquire(session, msg)
        elif isinstance(msg, ReleaseMessage):
            await _handle_release(session, msg)
        elif isinstance(msg, CancelMessage):
            await _handle_cancel(session, msg)
        elif isinstance(msg, SubscribeTelemetryMessage):
            await _handle_subscribe(session, msg)
        elif isinstance(msg, UnsubscribeTelemetryMessage):
            await _handle_unsubscribe(session, msg)
        elif isinstance(msg, GetDevicesMessage):
            await _handle_get_devices(session, msg)
    except Exception as e:
        await session.send(ErrorMessage(code="INTERNAL", message=str(e)))


@router.websocket("/ws") # ws://server/ws
async def websocket_endpoint(websocket: WebSocket,client_id: str = Depends(get_client_id)) -> None:
    """
    Main WebSocket endpoint. One persistent connection per client session.

    Lifecycle:
        connect (HTTP upgrade + Basic Auth check)
        -> receive messages in a loop -> dispatch to handlers
        -> disconnect (network drop or explicit close)
        -> unsubscribe telemetry, cancel pending commands, release device locks
    """
    session = await _conn_manager.connect(client_id, websocket)
    try:
        while True:
            raw = await websocket.receive_text()
            await _dispatch(session, raw)
    except WebSocketDisconnect:
        pass
    finally:
        # Always runs - even on unexpected disconnects or exceptions above
        session.unsubscribe_all_telemetry()
        _manager.on_client_disconnect(client_id)
        _conn_manager.disconnect(client_id)
