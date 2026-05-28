import asyncio
from typing import Callable, Dict, List, Optional

from ._connection import (
    AcquireError,       # noqa: F401 - re-exported for user convenience
    Connection,
    ConnectionLostError,  # noqa: F401
    ProcedureError,     # noqa: F401
    RemoteLabError,     # noqa: F401
    SubmitError,        # noqa: F401
    _PendingCommand,
    _PendingProcedure,
)


class CommandHandle:
    """
    Handle for a submitted command. Returned by submit().

    The handle is "fire-and-forget" by default - you don't have to await it.
    Awaiting blocks until all targeted devices report 'done'.

        cmd = await robot.submit("move", priority=5, duration=10.0)

        # Option A: don't wait - move on immediately
        await asyncio.sleep(5)
        cmd.cancel()

        # Option B: wait for completion
        await cmd
        # or with timeout:
        await cmd.wait(timeout=30)
    """

    def __init__(self, pending: _PendingCommand, conn: Connection):
        self._pending = pending
        self._conn    = conn

    @property
    def command_id(self) -> str:
        """Server-assigned unique ID for this command."""
        return self._pending.command_id

    @property
    def done(self) -> bool:
        """True if all targeted devices have reported 'done'."""
        return self._pending.done

    def cancel(self):
        """
        Cancel this command if it is still in the queue (no await).
        Has no effect if the command is already executing.
        To stop an executing command use device.interrupt() instead.
        """
        asyncio.create_task(self._conn.cancel(self._pending.command_id))

    async def wait(self, timeout: Optional[float] = None):
        """
        Wait for all targeted devices to finish executing this command.
        Raises asyncio.TimeoutError  if the timeout is exceeded.
        Raises ConnectionLostError   if the connection was lost while waiting.
        """
        await self._pending.wait(timeout=timeout)

    def __await__(self):
        """Enables: await cmd  (equivalent to await cmd.wait())."""
        return self._pending.wait().__await__()



class ProcedureHandle:
    """
    Handle for a running server-side group procedure. Returned by lab.run_procedure().

        proc = await lab.run_procedure("all_go_home")

        # Option A: fire-and-forget
        # (the procedure keeps running independently on the server)

        # Option B: wait for completion
        await proc

        # Option C: wait with timeout
        await proc.wait(timeout=60.0)

        # Option D: cancel
        proc.cancel()
    """

    def __init__(self, pending: _PendingProcedure, conn: Connection):
        self._pending = pending
        self._conn    = conn

    @property
    def procedure_id(self) -> str:
        return self._pending.procedure_id

    @property
    def done(self) -> bool:
        return self._pending.done

    def cancel(self):
        """Ask the server to cancel this procedure (no await)."""
        asyncio.create_task(self._conn.cancel_procedure(self._pending.procedure_id))

    async def wait(self, timeout: Optional[float] = None):
        """
        Block until the procedure finishes.
        Raises ProcedureError if the procedure fails on the server.
        Raises asyncio.TimeoutError if timeout is exceeded.
        """
        await self._pending.wait(timeout=timeout)

    def __await__(self):
        return self._pending.wait().__await__()


class AcquireContext:
    """
    Async context manager for exclusive device access.
    Do not instantiate directly!

        async with robot.lock():
            await robot.submit("move", priority=5)
        # device is released here automatically
    """

    def __init__(self, device: "DeviceProxy"):
        self._device = device

    async def __aenter__(self) -> "DeviceProxy":
        await self._device.acquire()
        return self._device

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self._device.release()
        return False  # never suppress exceptions


class DeviceProxy:
    """
    Proxy for one registered device.

    Obtained via lab.device("name") - do not instantiate directly.
    The same proxy object is returned on repeated calls with the same name.
    """

    def __init__(self, lab: "RemoteLab", name: str):
        self._lab  = lab
        self._name = name

    @property
    def name(self) -> str:
        return self._name


    def lock(self) -> AcquireContext:
        return AcquireContext(self)

    async def acquire(self):
        """
        Manually acquire exclusive access to the device.
        Raises AcquireError if another client already holds it.
        Prefer lock() — it guarantees release even on exceptions.
        """
        await self._lab._conn.acquire(self._name)

    async def release(self):
        """Manually release exclusive access."""
        await self._lab._conn.release(self._name)



    async def submit(self, command: str, priority: int = 5, **args) -> CommandHandle:
        """
        Submit a command to this device.

        Returns a CommandHandle immediately after the server acks the command.
        The handle can be awaited later to block until execution completes.

        Args:
            command:  Command name as expected by the device driver
                      (e.g. "move", "stop", "beep").
            priority: Scheduling priority — higher number runs sooner.
                      Default is 5; use 999 for emergency stops.
            **args:   Keyword arguments forwarded to execute_command() on the driver
                      (e.g. duration=10.0, speed=50, angle=90).

        Raises:
            SubmitError:         Server rejected the command (no access / unknown device).
            ConnectionLostError: Connection was lost before the ack arrived.

        Examples:
            cmd = await robot.submit("move", priority=5, duration=10.0)
            await cmd                             # wait for done
            cmd2 = await robot.submit("beep")    # fire-and-forget
        """
        pending = await self._lab._conn.submit(
            devices=[self._name],
            name=command,
            priority=priority,
            args=args,
        )
        return CommandHandle(pending, self._lab._conn)

    async def interrupt(self):
        """
        Interrupt the command currently executing on this device.

        The running execute_command() on the server receives CancelledError.
        The device worker then immediately picks up the next queued command.
        No effect if the device is currently idle.

        Typical emergency-stop pattern:
            await robot.interrupt()
            await robot.submit("STOP", priority=999)
        """
        await self._lab._conn.interrupt(self._name)


    async def subscribe_telemetry(self, callback: Callable[[dict], None]):
        """
        Register a push callback and ask the server to start sending telemetry.

        The callback receives a plain dict matching the device's telemetry dataclass
        (e.g. Yarp13Telemetry fields: speed_left, enc_right, compass, …).
        Calling this again with a new callback replaces the previous one.

        Example:
            def on_telemetry(data: dict):
                print(f"left={data['speed_left']}  right={data['speed_right']}")

            await robot.subscribe_telemetry(on_telemetry)
        """
        await self._lab._conn.subscribe_telemetry(self._name, callback)

    async def unsubscribe_telemetry(self):
        """Stop receiving telemetry pushes for this device."""
        await self._lab._conn.unsubscribe_telemetry(self._name)

    def get_latest_telemetry(self) -> Optional[dict]:
        """
        Return the most recent telemetry snapshot received for this device, or None.

        Works without calling subscribe_telemetry() — the snapshot is updated
        whenever a telemetry message arrives (even from another subscriber on
        the same connection).  Use for occasional polling.

        Example:
            snapshot = robot.get_latest_telemetry()
            if snapshot:
                print(snapshot["acc_voltage"])
        """
        return self._lab._conn.get_latest_telemetry(self._name)




# МАГНУМ ОПУС ОПУС МАГНУМ МАГНУМ
class RemoteLab:
    """
    Main entry point for the RemoteLab client.

    Manages the WebSocket connection and exposes device proxies.

    Recommended usage — async context manager:
        async with RemoteLab("ws://server:8000", "USER", "PASSWORD") as lab:
            robot = lab.device("robot1")
            ...

    Manual lifecycle (e.g. when the context manager is inconvenient):
        lab = RemoteLab("ws://server:8000", "alice", "secret")
        await lab.connect()
        try:
            ...
        finally:
            await lab.disconnect()
    """

    def __init__(self, url: str, username: str, password: str):
        """
        Args:
            url:      WebSocket server URL, e.g. "ws://192.168.1.100:8000"
            username: HTTP Basic Auth username (registered in users.json)
            password: HTTP Basic Auth password
        """
        self._conn    = Connection(url, username, password)
        self._proxies: Dict[str, DeviceProxy] = {}


    async def connect(self):
        """Open the WebSocket connection. Called automatically by __aenter__."""
        await self._conn.connect()

    async def disconnect(self):
        """Close the connection gracefully. Called automatically by __aexit__."""
        await self._conn.disconnect()

    async def __aenter__(self) -> "RemoteLab":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()
        return False


    def device(self, name: str) -> DeviceProxy:
        """
        Return the DeviceProxy for the named device (created on first call).
        """
        if name not in self._proxies:
            self._proxies[name] = DeviceProxy(self, name)
        return self._proxies[name]

    async def get_devices(self) -> List[dict]:
        """
        Request the list of active devices from the server.

        Returns a list of dicts, each with keys:
            name   (str)  - device name
            driver (str)  - driver type ("yarp13", "simple_serial", …)
            shared (bool) - True if no acquire needed

        Example:
            for d in await lab.get_devices():
                print(d["name"], "shared=" + str(d["shared"]))
        """
        return await self._conn.get_devices()


    async def run_procedure(self, name: str, **args) -> ProcedureHandle:
        """
        Start a named group procedure on the server.

        Returns a ProcedureHandle immediately after the server accepts the request.
        The procedure runs independently on the server - the client can await the handle or fire-and-forget.

        Args:
            name:  Procedure name registered on the server (e.g. "all_go_home").
            **args: Keyword arguments forwarded to the procedure's run() method.

        Raises:
            ProcedureError:      Server does not recognize the procedure name.
            ConnectionLostError: Connection was lost before the ack arrived.

        Examples:
            proc = await lab.run_procedure("all_go_home")
            await proc                                  # wait for completion
            await proc.wait(timeout=120.0)              # wait with timeout
            proc.cancel()                               # cancel mid-run
        """
        pending = await self._conn.run_procedure(name, args)
        return ProcedureHandle(pending, self._conn)

    async def submit(self, devices: List[str], command: str, priority: int = 5, **args) -> CommandHandle:
        """
        Submit a single command to multiple devices simultaneously.

        The server enqueues the command independently on each device's worker.
        The returned CommandHandle fires only when ALL targeted devices have
        finished executing (i.e. all 'done' messages received).

        Example:
            cmd = await lab.submit(["robot1", "robot2"], "sync_move", priority=5, x=1.0)
            await cmd   # blocks until both robots finish
        """
        pending = await self._conn.submit(
            devices=devices,
            name=command,
            priority=priority,
            args=args,
        )
        return CommandHandle(pending, self._conn)
