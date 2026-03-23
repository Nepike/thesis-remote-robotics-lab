import asyncio
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Callable, Awaitable


@dataclass(order=True)
class Command:
    """
    Command object stored in device priority queues.

    Ordering is based on (priority, timestamp) so PriorityQueue
    executes higher priority commands first and keeps FIFO
    ordering for equal priorities.
    """

    priority: int
    timestamp: float = field(init=False, compare=True)

    command_id: str = field(default_factory=lambda: str(uuid.uuid4()), compare=False)
    client_id: str = field(default="", compare=False)

    devices: List[str] = field(default_factory=list, compare=False)

    name: str = field(default="", compare=False)
    args: dict = field(default_factory=dict, compare=False)

    def __post_init__(self):
        self.timestamp = time.time()


class CommandScheduler:
    """
    Priority-based multi-device command scheduler.

    Each device has its own priority queue and worker task.
    """

    def __init__(self, execute_callback: Callable[[Command, str], Awaitable[None]]):
        """
        execute_callback(command, device_name) will be called
        by workers when command should be executed.
        """

        self._device_queues: Dict[str, asyncio.PriorityQueue] = {}
        self._workers: Dict[str, asyncio.Task] = {}

        self._execute_callback = execute_callback

        self._running = False


    def register_device(self, device_name: str):
        """
        Create queue and worker for a device.
        """

        if device_name in self._device_queues:
            return

        self._device_queues[device_name] = asyncio.PriorityQueue()


    async def _device_worker(self, device_name: str):
        """
        Worker processing commands for a single device.
        """
        queue = self._device_queues[device_name]
        while self._running:
            try:
                command: Command = await queue.get()
                await self._execute_callback(command, device_name)

            except asyncio.CancelledError:
                break

            except Exception as e:
                print(
                    f"[Scheduler] error executing command "
                    f"{command.command_id} on {device_name}: {e}"
                )

            finally:
                queue.task_done()


    async def start(self):
        """
        Start worker tasks for all devices.
        """

        if self._running:
            return

        self._running = True

        for device in self._device_queues:
            worker = asyncio.create_task(self._device_worker(device))
            self._workers[device] = worker


    async def shutdown(self):
        """
        Stop scheduler and cancel workers.
        """

        self._running = False

        for worker in self._workers.values():
            worker.cancel()

        await asyncio.gather(*self._workers.values(), return_exceptions=True)

        self._workers.clear()


    async def submit(self, client_id: str, devices: List[str], command_name: str, priority: int, args: dict) -> str:
        """
        Submit command affecting one or multiple devices.

        Returns command_id.
        """

        command = Command(priority=priority, client_id=client_id, devices=devices, name=command_name, args=args)

        for device in devices:
            queue = self._device_queues.get(device)

            if queue is None:
                raise ValueError(f"Unknown device {device}")

            await queue.put(command)

        return command.command_id


    async def cancel_client_commands(self, client_id: str):
        """
        Remove pending commands from a client.

        Note: only affects queued commands,
        not currently executing ones.
        """

        for device, queue in self._device_queues.items():
            new_queue = asyncio.PriorityQueue()

            while not queue.empty():
                cmd = await queue.get()

                if cmd.client_id != client_id:
                    await new_queue.put(cmd)

            self._device_queues[device] = new_queue


