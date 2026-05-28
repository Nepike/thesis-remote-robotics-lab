import asyncio
import uuid
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Awaitable, Callable, Dict, Optional

from Logger import Logger

if TYPE_CHECKING:
    from manager import RemoteLabManager


class AbstractProcedure(ABC):
    """
    Base class for all group procedures.

    A procedure is a server-side coroutine with access to the full RemoteLabManager.

    Instantiate once, register via ProcedureManager.register() - the same instance
    handles all invocations (it must be stateless; per-run state lives in run()).
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique string key used to invoke this procedure (e.g. 'all_go_home')."""

    @abstractmethod
    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        """
        Execute the procedure.

        `client_id` is a unique ID minted for this procedure run - use it with
        manager.acquire_device() / manager.submit_command_wait() / manager.release_device()
        exactly as a regular client would.  Release all devices in a finally block.

        asyncio.CancelledError is raised here when cancel_procedure() is called.
        """


# -------------------------------------------------------------------------
# СОБСТВЕННО, PROCEDURES
# -------------------------------------------------------------------------

class StopAll(AbstractProcedure):
    """
    Emergency stop for the entire lab.

    Step 1 — cancel_all_commands: every command waiting in device queues is
    discarded so nothing new starts executing after the interrupt.

    Step 2 — interrupt_all_devices: CancelledError is raised inside each
    currently running execute_command(). The driver is responsible for what
    happens next — Yarp13 publishes a zero Twist in its finally block.

    # Maybe Step 3 - send STOP to every device, so we are not thinking about drivers and stuff?
    #

    No device locks are acquired - this procedure is intentionally privileged.
    """

    name = "stop_all"

    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        manager.cancel_all_commands()
        manager.interrupt_all_devices()


class AllGoHome(AbstractProcedure):
    """Return all active robots to their home positions using camera + ArUco localization."""

    name = "all_go_home"

    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        devices = [d.name for d in manager.get_devices()]
        # TODO: read current robot poses from camera (ArUco marker detection)
        # TODO: for each robot, compute a collision-free path to its home pose
        # TODO: acquire devices, drive each robot along its path, release on exit

        # UPD: kinda done - need to implement it here using real camera data
        # (see github.com/nepike/allgohome)
        #
        #   for name in devices:
        #       manager.acquire_device(name, client_id)
        #   try:
        #       await asyncio.gather(*[
        #           _drive_to_home(manager, client_id, name, home_poses[name])
        #           for name in devices
        #       ])
        #   finally:
        #       for name in devices:
        #           manager.release_device(name, client_id)


class ProcedureManager:
    """
    Registry and executor for group procedures.

    Typical setup (in RemoteLabManager.start / ws_handler.init):
        pm.register(AllGoHome())
        pm.on_done  = conn_manager.on_procedure_done
        pm.on_error = conn_manager.on_procedure_error
    """

    def __init__(self, manager: "RemoteLabManager"):
        self._manager = manager
        self._registry: Dict[str, AbstractProcedure] = {}
        self._running:  Dict[str, asyncio.Task] = {}   # procedure_id -> Task
        self._owners:   Dict[str, str] = {}             # procedure_id -> initiating client_id

        # Wired by ws_handler after startup
        self.on_done:  Optional[Callable[[str, str], Awaitable[None]]] = None
        self.on_error: Optional[Callable[[str, str, str], Awaitable[None]]] = None

    def register(self, procedure: AbstractProcedure) -> None:
        self._registry[procedure.name] = procedure

    async def run(self, name: str, client_id: str, args: dict) -> str:
        """
        Start a procedure by name. Returns procedure_id immediately.
        Raises ValueError for unknown procedure names.
        """
        if name not in self._registry:
            raise ValueError(f"Unknown procedure: '{name}'")

        procedure_id = str(uuid.uuid4())
        proc_client_id = f"proc:{procedure_id[:8]}"   # not real client_id - so client and procedure don't ruin each other's access
        self._owners[procedure_id] = client_id

        task = asyncio.create_task(self._run_wrapper(procedure_id, name, proc_client_id, args), name=f"proc-{name}-{procedure_id[:8]}")
        self._running[procedure_id] = task
        return procedure_id

    async def _run_wrapper(self, procedure_id: str, name: str, proc_client_id: str, args: dict):
        logger = Logger.get()
        client_id = self._owners.get(procedure_id, "?")
        try:
            await logger.log("PROC", f"Start '{name}' ({procedure_id[:8]}) for '{client_id}'")
            await self._registry[name].run(self._manager, proc_client_id, args)
            await logger.log("PROC", f"Done  '{name}' ({procedure_id[:8]})")
            if self.on_done:
                await self.on_done(procedure_id, client_id)
        except asyncio.CancelledError:
            await logger.log("PROC", f"Cancelled '{name}' ({procedure_id[:8]})")
        except Exception as exc:
            await logger.log("PROC", f"Error in '{name}' ({procedure_id[:8]}): {exc}")
            if self.on_error:
                await self.on_error(procedure_id, client_id, str(exc))
        finally:
            self._manager.on_client_disconnect(proc_client_id)
            _ = self._running.pop(procedure_id, None)
            self._owners.pop(procedure_id, None)

    def cancel(self, procedure_id: str) -> None:
        """Cancel a running procedure. No-op if already finished."""
        task = self._running.get(procedure_id)
        if task:
            task.cancel()

    def cancel_all_for_client(self, client_id: str) -> None:
        """Cancel all procedures started by a disconnecting client."""
        for proc_id, owner in list(self._owners.items()):
            if owner == client_id:
                self.cancel(proc_id)
