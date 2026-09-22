import asyncio
import json
from pathlib import Path
from typing import Awaitable, Callable, Dict, List, Optional, Set, Tuple

from AccessController import AccessController
from BasicClasses import Command, Device
from CommandScheduler import CommandScheduler
from DeviceDrivers import AbstractDriver, DriverFactory
from DeviceSupervisor import DeviceSupervisor
from HardwareInterfaces import RosInterface, SerialInterface
from Logger import Logger
from Procedures import AllGoHome, StopAll, SyncTest, ProcedureManager, shutdown_navigation


class RemoteLabManager:
    """
    Top-level orchestrator.  Owns all subsystems and wires them together.

    Startup sequence (must be respected):
        manager = RemoteLabManager()
        await manager.load_config()   # parse devices.json, create drivers
        await manager.start()         # bring devices up, then start scheduler workers

    Public API (called by the network layer once it exists):
        submit_command / submit_command_wait / acquire_device / release_device
        run_procedure / cancel_procedure / on_client_disconnect
        get_devices / get_driver
    """

    def __init__(self):
        # Hardware interfaces - shared across all drivers of the same type
        self._ros    = RosInterface()
        self._serial = SerialInterface()

        self._driver_factory     = DriverFactory(self._ros, self._serial)
        self._supervisor         = DeviceSupervisor()
        self._scheduler          = CommandScheduler(self._on_execute_command)
        self._access_controller  = AccessController()

        # Release server-side waiters even for commands that never execute
        # (skipped because cancelled). See submit_command_wait / _on_command_settled.
        self._scheduler.on_command_settled = self._on_command_settled

        # device_name -> driver / Device (populated by load_config)
        self._drivers: Dict[str, AbstractDriver] = {}
        self._devices: Dict[str, Device]         = {}

        # command_id -> (done_event, remaining_device_count)
        # Used by procedures to await command completion server-side.
        self._command_waiters: Dict[str, Tuple[asyncio.Event, int]] = {}

        # Strong references to fire-and-forget tasks. asyncio keeps only a WEAK
        # reference to a running task, so a task nobody holds can be garbage
        # collected mid-flight and silently never finish.
        self._bg_tasks: Set[asyncio.Task] = set()

        self._procedure_manager = ProcedureManager(self)
        self._procedure_manager.register(StopAll())
        self._procedure_manager.register(AllGoHome())
        self._procedure_manager.register(SyncTest())

        # Wired by the network layer after startup. Called once per (command,
        # device) when the command SETTLES — executed, skipped or errored alike —
        # with executed=False for a command that never ran.
        self.on_command_complete: Optional[Callable[[Command, str, bool], Awaitable[None]]] = None


    async def load_config(self, config_path: Path = Path("./devices.json")):
        """
        Parse devices.json and register every active device with all subsystems.
        Must be called before start().
        """
        with open(config_path, "r") as f:
            raw = json.load(f)

        try:
            for item in raw:
                device = Device(
                    name=item["name"],
                    ip=item["ip"],
                    port=item["port"],
                    driver=item["driver"],
                    ros_namespace=item.get("ros_namespace"),
                    baud_rate=item.get("baud_rate"),
                    shared=item["shared"],
                    active=item["active"],
                )

                if not device.active:
                    continue

                driver = self._driver_factory.create_driver(device)

                self._devices[device.name] = device
                self._drivers[device.name] = driver

                # Each subsystem gets its own registration call
                self._supervisor.load_device(device, driver)
                self._scheduler.register_device(device.name)
                self._access_controller.register_device(device.name, device.shared)

        except KeyError as e:
            raise ValueError(f"devices.json is missing required field: {e}")

    async def start(self):
        logger = Logger.get()

        # Wire supervisor lifecycle hooks to scheduler pause/resume
        self._supervisor.on_device_down = self._scheduler.pause_device
        self._supervisor.on_device_up   = self._scheduler.resume_device

        # Bring all devices up first, then start accepting commands
        await self._supervisor.run()
        await self._scheduler.start()

        await logger.log("MANAGER", "Started")

    async def shutdown(self):
        logger = Logger.get()
        await logger.log("MANAGER", "Shutting down...")

        # Stop accepting and executing commands first
        await self._scheduler.shutdown()
        # Drain the fire-and-forget notification tasks before the loop goes away.
        # Snapshot first: each task's done-callback removes it from the set.
        pending = list(self._bg_tasks)
        for task in pending:
            task.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        # Then tear down device processes
        await self._supervisor.shutdown()
        # Release navigation resources (ArUco camera capture threads), if any
        shutdown_navigation()
        # Finally release ROS resources
        self._ros.shutdown()

        await logger.log("MANAGER", "Shutdown complete")


    def _spawn(self, coro):
        """Run a coroutine fire-and-forget, keeping a strong reference to the task."""
        task = asyncio.ensure_future(coro)
        self._bg_tasks.add(task)
        task.add_done_callback(self._bg_tasks.discard)
        return task

    async def _on_execute_command(self, command: Command, device_name: str):
        driver = self._drivers.get(device_name)
        if driver is None:
            raise RuntimeError(f"No driver registered for '{device_name}'")
        await driver.execute_command(command)
        # NB: the client is notified from _on_command_settled, not here. The
        # scheduler settles a command even when it never runs, and awaiting a
        # notification in a finally block while this coroutine is being cancelled
        # (interrupt / E-stop) is fragile.

    def _on_command_settled(self, command: Command, device_name: str, executed: bool):
        """
        Called by the scheduler once per (command, device) when the command leaves
        the queue for good — executed, skipped (cancelled), or errored alike.

        Two things happen here:
          1. Any procedure awaiting this command in submit_command_wait() is
             unblocked. Because settlement fires even for skipped commands, a
             cancelled command no longer leaves submit_command_wait() hanging.
          2. The network layer is notified, so the REMOTE client is unblocked on
             exactly the same terms. Notifying only on execution used to leave
             `await cmd` on the client hanging forever whenever a command was
             cancelled, and leaked the per-command tracking entries with it.
        """
        waiter = self._command_waiters.get(command.command_id)
        if waiter:
            event, remaining = waiter
            remaining -= 1
            if remaining <= 0:
                self._command_waiters.pop(command.command_id, None)
                event.set()
            else:
                self._command_waiters[command.command_id] = (event, remaining)

        # Sync callback (the scheduler calls it from a finally block), so hand the
        # async notification off to a task rather than awaiting it here.
        if self.on_command_complete:
            self._spawn(self.on_command_complete(command, device_name, executed))


    def _check_access(self, client_id: str, devices: List[str]):
        """Raise PermissionError if the client cannot command any of the devices."""
        for device_name in devices:
            if not self._access_controller.check_access(device_name, client_id):
                raise PermissionError(
                    f"Client '{client_id}' does not have access to '{device_name}'. "
                    f"Call acquire_device() first."
                )

    async def submit_command(self, client_id: str, devices: List[str], command_name: str, priority: int, args: Optional[dict] = None) -> str:
        """
        Validate access and enqueue a command.

        Returns command_id. Raises PermissionError if the client does not have
        access to one or more of the requested devices.
        """
        self._check_access(client_id, devices)
        return await self._scheduler.submit(client_id, devices, command_name, priority, args)

    async def submit_command_wait(self, client_id: str, devices: List[str], command_name: str, priority: int, args: Optional[dict] = None):
        """
        Submit a command and block until all targeted devices settle it.

        Intended for use inside procedures, where the procedure coroutine needs to
        know when a movement has physically completed before issuing the next one.

        The waiter is registered BEFORE the command is enqueued, so it cannot be
        missed by a worker that picks the command up immediately. It is released on
        settlement (execute / skip / error), so a cancelled command won't hang here.
        """
        self._check_access(client_id, devices)
        command = self._scheduler.make_command(client_id, devices, command_name, priority, args)

        # Register the waiter synchronously (no await) before enqueue, so no worker
        # can settle the command before we are listening.
        event = asyncio.Event()
        self._command_waiters[command.command_id] = (event, len(devices))

        await self._scheduler.enqueue(command)
        await event.wait()

    def acquire_device(self, device_name: str, client_id: str) -> bool:
        """
        Attempt to acquire exclusive ownership of a device.
        Always succeeds for shared devices. Returns False if the device is already owned by another client.
        """
        return self._access_controller.acquire(device_name, client_id)

    def release_device(self, device_name: str, client_id: str):
        """Release exclusive ownership of a single device."""
        self._access_controller.release(device_name, client_id)

    def on_client_disconnect(self, client_id: str):
        """
        Clean up all state associated with a disconnected client:
        cancels pending commands, releases device locks, and cancels procedures.
        """
        self._scheduler.cancel_client_commands(client_id)
        self._access_controller.release_all(client_id)
        self._procedure_manager.cancel_all_for_client(client_id)

    def get_devices(self) -> List[Device]:
        """Return a list of all registered active devices."""
        return list(self._devices.values())

    def get_driver(self, device_name: str) -> Optional[AbstractDriver]:
        """Return the driver for a device, or None if not found."""
        return self._drivers.get(device_name)

    def cancel_command(self, command_id: str):
        """Cancel a specific queued command. No effect if already executing."""
        self._scheduler.cancel(command_id)

    def interrupt_device(self, device_name: str):
        """
        Interrupt the command currently executing on a device.

        Raises CancelledError inside the running execute_command() coroutine.
        The worker catches it, logs it, and immediately picks up the next command.
        No effect if the device is idle.
        """
        self._scheduler.interrupt_device(device_name)

    def cancel_all_commands(self):
        """
        Cancel every command currently waiting in any device queue.
        Also unblocks any procedure suspended in submit_command_wait() so it
        does not hang after its command was canceled.
        """
        self._scheduler.cancel_all_pending()
        for event, _ in list(self._command_waiters.values()):
            event.set()
        self._command_waiters.clear()

    def interrupt_all_devices(self):
        """Interrupt the command currently executing on every device."""
        self._scheduler.interrupt_all_devices()

    def emergency_stop_all(self):
        """
        Publish a physical stop on every actuator channel of every device.

        The last line of defence in stop_all. Cancelling queued commands and
        interrupting running ones only stops work the scheduler owns; an actuator
        left latched by a command that already returned (a HOLD setpoint, say)
        keeps running and nothing above would touch it. This reaches the hardware
        directly, bypassing the queue — which may itself be paused by a device
        restart at that very moment.
        """
        for device_name, driver in self._drivers.items():
            try:
                driver.emergency_stop()
            except Exception as e:
                # Never let one unhappy driver stop us from halting the rest.
                self._spawn(Logger.get().log(
                    "MANAGER", f"emergency_stop failed for '{device_name}': {e}"
                ))

    async def run_procedure(self, name: str, client_id: str, args: dict) -> str:
        """
        Start a group procedure by name. Returns procedure_id immediately.
        Raises ValueError if the procedure name is not registered.
        """
        return await self._procedure_manager.run(name, client_id, args)

    def cancel_procedure(self, procedure_id: str):
        """Cancel a running procedure. No-op if already finished."""
        self._procedure_manager.cancel(procedure_id)

    def cancel_all_procedures(self, except_client: Optional[str] = None):
        """
        Cancel every running procedure. Used by the emergency stop_all to halt
        procedures that drive devices outside the scheduler (e.g. AllGoHome's
        direct velocity loop). `except_client` is the caller's proc-client-id so
        stop_all does not cancel itself.
        """
        self._procedure_manager.cancel_all(except_proc_client_id=except_client)

    @property
    def procedure_manager(self) -> ProcedureManager:
        """Expose ProcedureManager so ws_handler can wire its callbacks."""
        return self._procedure_manager

# No __main__ here on purpose: the server is started through network/server.py,
# which also brings up roscore, the user store and the WebSocket layer.
