import asyncio
from collections import defaultdict
from typing import Awaitable, Callable, Dict, List, Optional, Set

from BasicClasses import Command
from Logger import Logger


class CommandScheduler:
    """
    Priority-based multi-device command scheduler.

    Each registered device gets its own priority queue and a dedicated worker task.
    Workers respect the device's ready state, so they pause automatically during
    device restarts (call pause_device / resume_device from DeviceSupervisor hooks).
    """

    def __init__(self, execute_callback: Callable[[Command, str], Awaitable[None]]):
        """
        execute_callback(command, device_name) is called by a worker whenever
        a command is due for execution on the given device.
        """
        self._execute_callback = execute_callback

        self._device_queues: Dict[str, asyncio.PriorityQueue] = {}
        self._workers: Dict[str, asyncio.Task] = {}

        # asyncio.Event per device: set = ready to run, cleared = paused
        self._ready_events: Dict[str, asyncio.Event] = {}

        # currently running execute_callback sub-task per device (None if idle)
        self._current_exec_tasks: Dict[str, Optional[asyncio.Task]] = {}

        # command_id -> number of device queues still holding this command
        # (a multi-device command lands in N queues simultaneously)
        self._pending_counts: Dict[str, int] = {}

        # client_id -> set of command_ids currently enqueued
        self._client_commands: Dict[str, Set[str]] = defaultdict(set)

        # command_ids that workers must skip when they dequeue them
        self._cancelled_ids: Set[str] = set()

        self._running = False


    def register_device(self, device_name: str):
        """
        Create a queue and a ready-event for a device.
        Must be called before start(). (Ide.. I.. Idempotented? - well idk - идемпотентность крч)
        """
        if device_name in self._device_queues:
            return
        self._device_queues[device_name] = asyncio.PriorityQueue()
        event = asyncio.Event()
        event.set()  # devices start in the ready state
        self._ready_events[device_name] = event
        self._current_exec_tasks[device_name] = None

    async def start(self):
        """Spawn worker tasks for all registered devices."""
        if self._running:
            return
        self._running = True
        for device_name in self._device_queues:
            self._workers[device_name] = asyncio.create_task(self._device_worker(device_name), name=f"cmd-worker-{device_name}")

    async def shutdown(self):
        """Cancel all workers and stop the scheduler."""
        self._running = False
        # Unblock workers that are waiting on a paused device so they can exit
        for event in self._ready_events.values():
            event.set()
        for worker in self._workers.values():
            worker.cancel()
        await asyncio.gather(*self._workers.values(), return_exceptions=True)
        self._workers.clear()


    async def submit(self, client_id: str, devices: List[str], command_name: str, priority: int, args: Optional[dict] = None) -> str:
        """
        Enqueue a command for one or more devices.

        The same Command object is placed in each device's queue, so all
        targeted devices will eventually execute it independently.

        Returns the command_id. Raises ValueError for unknown device names.
        """
        unknown = [d for d in devices if d not in self._device_queues]
        if unknown:
            raise ValueError(f"Unknown device(s): {unknown}")

        command = Command(
            priority=priority,
            client_id=client_id,
            devices=devices,
            name=command_name,
            args=args or {},
        )

        self._pending_counts[command.command_id] = len(devices)
        self._client_commands[client_id].add(command.command_id)

        for device_name in devices:
            await self._device_queues[device_name].put(command)

        return command.command_id


    def cancel(self, command_id: str):
        """
        Cancel a specific command by id.
        Has no effect if the command is already executing.
        """
        self._cancelled_ids.add(command_id)

    def cancel_client_commands(self, client_id: str):
        """
        Cancel all queued commands from a client (e.g. on disconnect).
        Has no effect on a command that is currently executing.
        """
        command_ids = self._client_commands.pop(client_id, set())
        self._cancelled_ids.update(command_ids)


    def pause_device(self, device_name: str):
        """
        Prevent the worker from starting the next command for this device.
        The command currently executing (if any) will complete normally.
        Intended to be called by DeviceSupervisor before restarting a device.
        """
        event = self._ready_events.get(device_name)
        if event:
            event.clear()

    def resume_device(self, device_name: str):
        """
        Allow the worker to resume execution after a pause.
        Intended to be called by DeviceSupervisor after a device has restarted.
        """
        event = self._ready_events.get(device_name)
        if event:
            event.set()


    def interrupt_device(self, device_name: str):
        """
        Cancel the command currently executing on a device.

        The worker will catch the cancellation, log it, and immediately pick up
        the next command from the queue — so a high-priority command submitted
        before this call will run next.

        Has no effect if the device is idle.
        """
        task = self._current_exec_tasks.get(device_name)
        if task and not task.done():
            task.cancel()

    def interrupt_all_devices(self):
        """Interrupt the currently executing command on every device."""
        for device_name in self._current_exec_tasks:
            self.interrupt_device(device_name)

    def cancel_all_pending(self):
        """
        Cancel every command that is waiting in any queue.

        Combine with interrupt_all_devices() for a full emergency stop:
            scheduler.cancel_all_pending()
            scheduler.interrupt_all_devices()
            await scheduler.submit(..., "STOP_ALL", priority=MAX_PRIORITY)
        """
        for command_ids in self._client_commands.values():
            self._cancelled_ids.update(command_ids)
        self._client_commands.clear()


    async def _device_worker(self, device_name: str):
        queue = self._device_queues[device_name]
        ready = self._ready_events[device_name]
        logger = Logger.get()

        while self._running:
            # 1. Wait for a command to arrive in the queue
            try:
                command: Command = await queue.get()
            except asyncio.CancelledError:
                break

            # 2. Hold the dequeued command until the device is ready.
            #    This covers the restart window: the command is already out of
            #    the queue (priority order is preserved) but won't execute
            #    until the device comes back up.
            try:
                await ready.wait()

                if command.command_id in self._cancelled_ids:
                    await logger.log(
                        "SCHEDULER",
                        f"Skipping cancelled '{command.name}' on '{device_name}'",
                    )
                else:
                    # Run as a sub-task so it can be cancelled via interrupt_device()
                    # without cancelling the worker itself.
                    exec_task = asyncio.create_task(self._execute_callback(command, device_name))
                    self._current_exec_tasks[device_name] = exec_task
                    try:
                        await exec_task
                    except asyncio.CancelledError:
                        if exec_task.done() and exec_task.cancelled():
                            # exec_task was cancelled by interrupt_device() - expected.
                            # The worker loop continues normally.
                            await logger.log(
                                "SCHEDULER",
                                f"Interrupted '{command.name}' on '{device_name}'",
                            )
                        else:
                            # The worker task itself is being cancelled (shutdown).
                            # Cancel the sub-task too and propagate.
                            exec_task.cancel()
                            try:
                                await exec_task
                            except asyncio.CancelledError:
                                pass
                            raise
                    finally:
                        self._current_exec_tasks[device_name] = None

            except asyncio.CancelledError:
                break

            except Exception as e:
                await logger.log(
                    "SCHEDULER",
                    f"Error executing '{command.name}' on '{device_name}': {e}",
                )

            finally:
                queue.task_done()
                self._on_command_done(command)

    def _on_command_done(self, command: Command):
        """
        Decrement the pending counter for this command.
        When all device queues have processed it, clean up all tracking state.
        """
        remaining = self._pending_counts.get(command.command_id, 1) - 1
        if remaining <= 0:
            self._pending_counts.pop(command.command_id, None)
            self._cancelled_ids.discard(command.command_id)
            self._client_commands.get(command.client_id, set()).discard(command.command_id)
        else:
            self._pending_counts[command.command_id] = remaining
