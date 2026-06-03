"""
Internal WebSocket connection manager.

Not intended to be used directly — import RemoteLab from client.py instead.

Responsibilities:
  - Open / close the WebSocket
  - Automatic reconnect (3 tries)
  - Dispatch incoming messages to the right waiters:
      ack      -> unblocks the submit() that sent the command
      done     -> decrements the per-command counter; fires the CommandHandle event
      error    -> routes to the pending acquire or the pending submit
      telemetry -> updates the latest snapshot and calls the registered callback
      devices  -> fulfills the get_devices() future
"""

import asyncio
import base64
import json
import logging
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Set

import websockets

log = logging.getLogger(__name__)


# Public exceptions
class RemoteLabError(Exception):
    """Base class for all RemoteLab client errors."""


class ConnectionLostError(RemoteLabError):
    """Connection was lost and could not be restored within the retry limit."""


class AcquireError(RemoteLabError):
    """acquire() failed — another client already holds the device."""


class SubmitError(RemoteLabError):
    """Server rejected the submitted command (ACCESS_DENIED or UNKNOWN_DEVICE)."""


class ProcedureError(RemoteLabError):
    """Server rejected or failed a procedure (UNKNOWN_PROCEDURE or runtime error)."""


class _PendingCommand:
    """
    Tracks one submitted command until all targeted devices report 'done'.

    A multi-device command (devices=["r1","r2"]) receives two separate 'done'
    messages from the server (one per device).  The handle fires only when all
    of them have arrived.
    """

    def __init__(self, command_id: str, num_devices: int):
        self.command_id = command_id
        self._remaining = num_devices
        self._event = asyncio.Event()
        self._error: Optional[Exception] = None

    def notify_done(self) -> bool:
        """
        Called each time one device sends 'done'.
        Returns True when ALL devices are done (event is set).
        """
        self._remaining -= 1
        if self._remaining <= 0:
            self._event.set()
            return True
        return False

    def fail(self, exc: Exception):
        """Unblock any waiter with an error (called on unexpected disconnect)."""
        self._error = exc
        self._event.set()

    @property
    def done(self) -> bool:
        return self._event.is_set()

    async def wait(self, timeout: Optional[float] = None):
        """
        Block until all devices report done.
        Raises the stored error if the command failed (e.g. connection lost).
        """
        if timeout is not None:
            await asyncio.wait_for(self._event.wait(), timeout=timeout)
        else:
            await self._event.wait()
        if self._error:
            raise self._error


class _PendingProcedure:
    """Tracks one running server-side procedure until it completes or errors."""

    def __init__(self, procedure_id: str):
        self.procedure_id = procedure_id
        self._event = asyncio.Event()
        self._error: Optional[str] = None

    def notify_done(self):
        self._event.set()

    def notify_error(self, message: str):
        self._error = message
        self._event.set()

    def fail(self, exc: Exception):
        """Unblock any waiter with an error (called on unexpected disconnect)."""
        self._error = str(exc)
        self._event.set()

    @property
    def done(self) -> bool:
        return self._event.is_set() and not self._error

    async def wait(self, timeout: Optional[float] = None):
        if timeout is not None:
            await asyncio.wait_for(self._event.wait(), timeout=timeout)
        else:
            await self._event.wait()
        if self._error:
            raise ProcedureError(self._error)


class Connection:
    """
    Persistent WebSocket connection to the RemoteLab server.

    Usage (managed by RemoteLab in client.py - do not instantiate directly!):
        conn = Connection(url, username, password)
        await conn.connect()
        ...
        await conn.disconnect()
    """

    _RETRY_DELAYS: List[float] = [1.0, 2.0, 4.0]  # backoff between reconnect attempts
    _ACK_TIMEOUT:  float = 5.0   # seconds to wait for server ack after submit / get_devices
    _ACQUIRE_TIMEOUT: float = 1.0  # seconds to wait for ALREADY_OWNED error after acquire

    def __init__(self, url: str, username: str, password: str):
        # Accept bare host[:port] or domain - add ws:// if no scheme given
        if "://" not in url:
            url = "ws://" + url
        self._url = url
        self._auth_header = {
            "Authorization": "Basic " + base64.b64encode(
                f"{username}:{password}".encode()
            ).decode()
        }

        self._ws = None
        self._reader_task: Optional[asyncio.Task] = None
        self._connected = False

        # Serializes all sends - prevents interleaved frames from concurrent callers.
        self._send_lock = asyncio.Lock()

        # While reconnecting, senders wait on this event before proceeding.
        self._ready = asyncio.Event()
        self._ready.set()  # initially ready

        # FIFO deque of futures, one per submit currently waiting for an ack.
        # The server sends acks in the same order it receives submits (single WS - single async handler), so FIFO correlation is safe.
        self._pending_acks: Deque[asyncio.Future] = deque()

        # command_id -> _PendingCommand (waiting for done messages)
        self._pending_commands: Dict[str, _PendingCommand] = {}

        # get_devices
        self._devices_future: Optional[asyncio.Future] = None

        # Telemetry
        self._telemetry_callbacks: Dict[str, Callable[[dict], None]] = {}
        self._latest_telemetry: Dict[str, dict] = {}

        # Procedures
        self._pending_proc_acks: Deque[asyncio.Future] = deque()
        self._pending_procedures: Dict[str, _PendingProcedure] = {}

        # Acquire tracking
        # Devices this client currently holds (auto re-acquired after reconnect).
        self._acquired_devices: Set[str] = set()

        # Serializes acquires
        self._acquire_lock = asyncio.Lock()
        self._pending_acquire_event: Optional[asyncio.Event] = None
        self._pending_acquire_error: Optional[str] = None


    async def connect(self):
        """Open the WebSocket and start the background reader task."""
        await self._open_ws()
        self._connected = True
        self._reader_task = asyncio.create_task(self._reader_loop(), name="remoteLab-reader")

    async def disconnect(self):
        """Close the connection gracefully."""
        self._connected = False
        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass
        if self._ws:
            await self._ws.close()
            self._ws = None

    async def _open_ws(self):
        """Low-level: open the WebSocket (no reader task)."""
        url = self._url.rstrip("/") + "/ws"
        self._ws = await websockets.connect(url, extra_headers=self._auth_header)



    async def _reader_loop(self):
        """Reads messages from the server forever; triggers reconnect on failure."""
        try:
            while True:
                raw = await self._ws.recv()
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    log.warning("RemoteLab: received non-JSON frame: %r", raw)
                    continue
                self._dispatch(msg)
        except asyncio.CancelledError:
            raise  # normal shutdown path
        except Exception as exc:
            if self._connected:
                log.warning("RemoteLab: connection lost (%s), reconnecting...", exc)
                asyncio.create_task(self._reconnect(exc), name="remoteLab-reconnect")

    def _dispatch(self, msg: dict):
        """Route one server message to the appropriate handler."""
        t = msg.get("type")

        if t == "ack":
            if self._pending_acks:
                fut = self._pending_acks.popleft()
                if not fut.done():
                    fut.set_result(msg)

        elif t == "done":
            command_id = msg.get("command_id", "")
            pending = self._pending_commands.get(command_id)
            if pending:
                fully_done = pending.notify_done()
                if fully_done:
                    self._pending_commands.pop(command_id, None)

        elif t == "telemetry":
            device = msg.get("device", "")
            data   = msg.get("data", {})
            self._latest_telemetry[device] = data
            cb = self._telemetry_callbacks.get(device)
            if cb:
                try:
                    cb(data)
                except Exception:
                    log.exception(
                        "RemoteLab: telemetry callback for '%s' raised", device
                    )

        elif t == "devices":
            if self._devices_future and not self._devices_future.done():
                self._devices_future.set_result(msg.get("data", []))

        elif t == "procedure_ack":
            if self._pending_proc_acks:
                fut = self._pending_proc_acks.popleft()
                if not fut.done():
                    fut.set_result(msg)

        elif t == "procedure_done":
            procedure_id = msg.get("procedure_id", "")
            pending = self._pending_procedures.pop(procedure_id, None)
            if pending:
                pending.notify_done()

        elif t == "procedure_error":
            procedure_id = msg.get("procedure_id", "")
            pending = self._pending_procedures.pop(procedure_id, None)
            if pending:
                pending.notify_error(msg.get("message", "Procedure failed"))

        elif t == "error":
            self._handle_server_error(msg)

        else:
            log.debug("RemoteLab: unrecognised message type %r", t)

    def _handle_server_error(self, msg: dict):
        code    = msg.get("code", "")
        message = msg.get("message", "")

        if code == "ALREADY_OWNED" and self._pending_acquire_event:
            # Failure response for the in-flight acquire
            self._pending_acquire_error = message
            self._pending_acquire_event.set()

        elif code == "UNKNOWN_PROCEDURE" and self._pending_proc_acks:
            fut = self._pending_proc_acks.popleft()
            if not fut.done():
                fut.set_exception(ProcedureError(f"{code}: {message}"))

        elif code in ("ACCESS_DENIED", "UNKNOWN_DEVICE") and self._pending_acks:
            # Server responded with an error instead of an ack for the in-flight submit
            fut = self._pending_acks.popleft()
            if not fut.done():
                fut.set_exception(SubmitError(f"{code}: {message}"))

        else:
            log.warning("RemoteLab server error [%s]: %s", code, message)




    async def _reconnect(self, cause: Exception):
        """Re-establish the connection after an unexpected disconnect."""
        # Block senders until we're back up
        self._ready.clear()

        # Immediately fail everything waiting on the dead connection
        self._fail_all_pending(ConnectionLostError(f"Connection lost: {cause}"))

        last_exc: Exception = cause
        for attempt, delay in enumerate(self._RETRY_DELAYS, start=1):
            log.info(
                "RemoteLab: reconnect attempt %d/%d in %.0f s...",
                attempt, len(self._RETRY_DELAYS), delay,
            )
            await asyncio.sleep(delay)
            try:
                await self._open_ws()
                self._reader_task = asyncio.create_task(
                    self._reader_loop(), name="remoteLab-reader"
                )
                # Restore state: re-acquire devices and re-subscribe telemetry
                for device in list(self._acquired_devices):
                    await self._ws.send(json.dumps({"type": "acquire", "device": device}))
                for device in list(self._telemetry_callbacks):
                    await self._ws.send(json.dumps({"type": "subscribe_telemetry", "device": device}))
                log.info("RemoteLab: reconnected.")
                self._ready.set()
                return
            except Exception as exc:
                last_exc = exc
                log.warning("RemoteLab: attempt %d failed: %s", attempt, exc)

        self._connected = False
        self._ready.set()  # unblock senders so they fail fast instead of hanging
        log.error("RemoteLab: gave up reconnecting after %d attempts.", len(self._RETRY_DELAYS))

    def _fail_all_pending(self, exc: Exception):
        """Cancel all in-flight operations with an error."""
        while self._pending_acks:
            fut = self._pending_acks.popleft()
            if not fut.done():
                fut.set_exception(exc)

        for pending in list(self._pending_commands.values()):
            pending.fail(exc)
        self._pending_commands.clear()

        if self._devices_future and not self._devices_future.done():
            self._devices_future.set_exception(exc)

        while self._pending_proc_acks:
            fut = self._pending_proc_acks.popleft()
            if not fut.done():
                fut.set_exception(exc)

        for pending in list(self._pending_procedures.values()):
            pending.fail(exc)
        self._pending_procedures.clear()


    async def _send(self, msg: dict):
        """
        Send a JSON message, waiting for reconnect if one is in progress.
        Raises ConnectionLostError if the connection is permanently dead.
        """
        await self._ready.wait()
        if not self._connected:
            raise ConnectionLostError("Not connected to RemoteLab server")
        async with self._send_lock:
            await self._ws.send(json.dumps(msg))




    async def acquire(self, device: str):
        """
        Acquire exclusive access to a device.

        On success: returns normally (server sends no ack - silence = success).
        On failure: raises AcquireError.
        No-op for shared devices (server always succeeds, no error sent).
        """
        async with self._acquire_lock:
            self._pending_acquire_event = asyncio.Event()
            self._pending_acquire_error = None

            await self._send({"type": "acquire", "device": device})

            try:
                # Wait briefly for an ALREADY_OWNED error.  Timeout = success.
                await asyncio.wait_for(self._pending_acquire_event.wait(),timeout=self._ACQUIRE_TIMEOUT)
                # Event fired before timeout -> server sent ALREADY_OWNED
                raise AcquireError(self._pending_acquire_error or f"Failed to acquire '{device}'")
            except asyncio.TimeoutError:
                self._acquired_devices.add(device)
            finally:
                self._pending_acquire_event = None

    async def release(self, device: str):
        """Release exclusive access to a device."""
        self._acquired_devices.discard(device)
        await self._send({"type": "release", "device": device})

    async def submit(self, devices: List[str], name: str, priority: int, args: dict) -> "_PendingCommand":
        """
        Submit a command for one or more devices.

        Returns a _PendingCommand that can be awaited for completion.
        Raises SubmitError if the server rejects the command.

        The ack future is registered inside the send lock so that the FIFO order between concurrent submits is preserved.
        """
        loop = asyncio.get_running_loop()
        ack_future: asyncio.Future = loop.create_future()

        await self._ready.wait()
        if not self._connected:
            raise ConnectionLostError("Not connected to RemoteLab server")

        async with self._send_lock:
            # Append BEFORE sending (на всякий)
            self._pending_acks.append(ack_future)
            await self._ws.send(json.dumps({
                "type":     "submit",
                "devices":  devices,
                "name":     name,
                "priority": priority,
                "args":     args,
            }))

        try:
            ack = await asyncio.wait_for(ack_future, timeout=self._ACK_TIMEOUT)
        except asyncio.TimeoutError:
            raise SubmitError(f"No ack from server within {self._ACK_TIMEOUT} s")

        # ack_future may have been failed by _handle_server_error (SubmitError) - asyncio.wait_for re-raises exceptions set on the future, so we're fine.

        command_id = ack["command_id"]
        pending = _PendingCommand(command_id=command_id, num_devices=len(devices))
        self._pending_commands[command_id] = pending
        return pending

    async def cancel(self, command_id: str):
        """
        Cancel a queued command (no effect if already executing).
        Use interrupt() to stop a command that is currently running.
        """
        await self._send({"type": "cancel", "command_id": command_id})

    async def interrupt(self, device: str):
        """
        Interrupt the command currently executing on a device.

        Raises CancelledError inside the running execute_command() on the server.
        The server worker catches it and immediately picks up the next queued command.
        No effect if the device is idle.

        Typical pattern for emergency stop:
            await robot.interrupt()                                            # stop whatever is running
            await robot.submit("STOP", priority=999999999999999999999)         # send a stop command next
        """
        await self._send({"type": "interrupt", "device": device})

    async def get_devices(self) -> List[dict]:
        """Request the list of active devices from the server."""
        loop = asyncio.get_running_loop()
        self._devices_future = loop.create_future()
        await self._send({"type": "get_devices"})
        try:
            return await asyncio.wait_for(self._devices_future, timeout=self._ACK_TIMEOUT)
        except asyncio.TimeoutError:
            raise RemoteLabError(f"No devices response from server within {self._ACK_TIMEOUT} s")
        finally:
            self._devices_future = None

    async def subscribe_telemetry(self, device: str, callback: Callable[[dict], None]):
        """Register a callback and ask the server to start pushing telemetry."""
        self._telemetry_callbacks[device] = callback
        await self._send({"type": "subscribe_telemetry", "device": device})

    async def unsubscribe_telemetry(self, device: str):
        """Remove the callback and ask the server to stop pushing telemetry."""
        self._telemetry_callbacks.pop(device, None)
        await self._send({"type": "unsubscribe_telemetry", "device": device})

    def get_latest_telemetry(self, device: str) -> Optional[dict]:
        """Return the most recent telemetry snapshot for a device, or None."""
        return self._latest_telemetry.get(device)

    async def run_procedure(self, name: str, args: dict) -> "_PendingProcedure":
        """
        Ask the server to start a group procedure.

        Returns a _PendingProcedure that can be awaited for completion.
        Raises ProcedureError if the server does not recognise the procedure name.
        """
        loop = asyncio.get_running_loop()
        ack_future: asyncio.Future = loop.create_future()

        await self._ready.wait()
        if not self._connected:
            raise ConnectionLostError("Not connected to RemoteLab server")

        async with self._send_lock:
            self._pending_proc_acks.append(ack_future)
            await self._ws.send(json.dumps({"type": "run_procedure", "name": name, "args": args}))

        try:
            ack = await asyncio.wait_for(ack_future, timeout=self._ACK_TIMEOUT)
        except asyncio.TimeoutError:
            raise ProcedureError(f"No ack from server within {self._ACK_TIMEOUT} s")

        procedure_id = ack["procedure_id"]
        pending = _PendingProcedure(procedure_id=procedure_id)
        self._pending_procedures[procedure_id] = pending
        return pending

    async def cancel_procedure(self, procedure_id: str):
        """Ask the server to cancel a running procedure."""
        await self._send({"type": "cancel_procedure", "procedure_id": procedure_id})
