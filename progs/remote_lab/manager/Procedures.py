import asyncio
import uuid
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Awaitable, Callable, Dict, Optional

from Logger import Logger

# Navigation deps are optional at import time: if numpy / the navigation package is
# unavailable, only AllGoHome is disabled — the rest of the server still starts.
try:
    import numpy as np
    from navigation import (
        UKF,
        WaypointController,
        a_star,
        world_to_cell,
        build_localization_provider,
        load_nav_config,
    )
    _NAV_AVAILABLE = True
    _NAV_IMPORT_ERROR = ""
except Exception as _e:  # pragma: no cover
    _NAV_AVAILABLE = False
    _NAV_IMPORT_ERROR = str(_e)

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

    Step 3 — cancel_all_procedures: any other running procedure (e.g. AllGoHome,
    which drives robots via a direct velocity loop outside the scheduler) is
    cancelled, so its teardown stops the robots. This procedure excludes itself.

    No device locks are acquired - this procedure is intentionally privileged.
    """

    name = "stop_all"

    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        manager.cancel_all_commands()
        manager.interrupt_all_devices()
        manager.cancel_all_procedures(except_client=client_id)


class AllGoHome(AbstractProcedure):
    """
    Return all configured robots to their home cells.

    Robots are driven home ONE AT A TIME (sequentially), not simultaneously: A*
    plans against static obstacles only, so a moving robot must not become an
    unmodelled (moving) obstacle for another. While one robot drives, the others
    stand still, and their current cells are added to that robot's A* obstacle set,
    so it routes around the parked ones too. Those cells are known exactly: a robot
    that has not started yet sits at its start cell, one that finished sits at its
    home cell.

    Pipeline per robot:
        localization (pose) -> UKF (state estimate) -> A* (path, avoiding static
        obstacles + the other robots' current cells) -> waypoint controller ->
        direct velocity stream to the robot.

    Pose source is pluggable (see navigation.localization):
      now — SimulatedLocalizationProvider (ground truth advanced from the
        commanded velocities + Gaussian noise, exactly like the simulator camera);
      later — ArucoLocalizationProvider (real ceiling cameras + ArUco). Switch by
        setting "provider": "aruco" in nav_config.json. The navigation logic below
        does NOT change — only the pose source.

    Map, homes, markers and noise live in manager/nav_config.json (auto-created).

    Control uses driver.set_velocity() (direct, low-latency). On cancel/E-stop the
    finally block stops every robot and releases the locks; the driver deadman is
    the final backstop.
    """

    name = "all_go_home"

    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        logger = Logger.get()

        if not _NAV_AVAILABLE:
            raise RuntimeError(f"navigation unavailable: {_NAV_IMPORT_ERROR}")

        cfg = load_nav_config()
        if cfg is None:
            raise RuntimeError("nav_config.json is invalid — see server log")
        if not cfg.robots:
            await logger.log("PROC", "all_go_home: nav_config.json has no robots — configure homes")
            return

        loc = build_localization_provider(cfg)
        await logger.log("PROC", f"all_go_home: localization = {loc.describe()}")

        # Only active devices that are configured with a home cell.
        names = [d.name for d in manager.get_devices() if d.name in cfg.robots]
        if not names:
            await logger.log("PROC", "all_go_home: no active devices listed in nav_config.json")
            return

        acquired = []
        for name in names:
            if manager.acquire_device(name, client_id):
                acquired.append(name)
                sc = cfg.robots[name].start  # (cell_x, cell_y, theta)
                start_m = ((sc[0] + 0.5) * cfg.cell_size_m, (sc[1] + 0.5) * cfg.cell_size_m, sc[2])
                loc.register(name, start_m)  # cells -> metres; seeds simulated truth (no-op for ArUco)
            else:
                await logger.log("PROC", f"all_go_home: skipping '{name}' (busy/owned)")
        if not acquired:
            await logger.log("PROC", "all_go_home: no devices available")
            return

        # Current cell of every robot, used as a dynamic obstacle set while another
        # robot moves. Initially everyone is at its start cell.
        positions = {
            n: (int(cfg.robots[n].start[0]), int(cfg.robots[n].start[1])) for n in acquired
        }

        try:
            await logger.log("PROC", f"all_go_home: navigating {acquired} (one at a time)")
            home, failed = [], []
            for name in acquired:
                # All OTHER robots are stationary now -> treat their cells as obstacles.
                others = {positions[o] for o in acquired if o != name}
                ok = await self._drive_robot(manager, loc, cfg, name, others, logger)
                (home if ok else failed).append(name)
                # This robot is now parked at its home cell (an obstacle for the rest).
                positions[name] = tuple(cfg.robots[name].home)
            await logger.log("PROC", f"all_go_home: reached={home} failed={failed}")
            if failed:
                raise RuntimeError(f"Did not reach home: {failed}")
        finally:
            # Stop every robot and release, regardless of how we exit (success,
            # failure, or cancellation via E-stop).
            for name in acquired:
                drv = manager.get_driver(name)
                if drv is not None and hasattr(drv, "set_velocity"):
                    try:
                        drv.set_velocity(0.0, 0.0)
                    except Exception:
                        pass
                manager.release_device(name, client_id)

    async def _first_fix(self, loc, name, dt, tries=20):
        """Wait for the first non-dropped localization measurement."""
        for _ in range(tries):
            pose = await loc.get_pose(name)
            if pose is not None:
                return pose
            await asyncio.sleep(dt)
        return None

    async def _drive_robot(self, manager, loc, cfg, name, other_cells, logger):
        """Drive one robot home, avoiding static obstacles + `other_cells` (the
        current cells of the other, stationary robots). Returns (name, reached)."""
        driver = manager.get_driver(name)
        if driver is None or not hasattr(driver, "set_velocity"):
            await logger.log("PROC", f"all_go_home: '{name}' has no velocity control — skipped")
            return (name, False)

        fix = await self._first_fix(loc, name, cfg.dt)
        if fix is None:
            await logger.log("PROC", f"all_go_home: '{name}' no localization fix")
            return (name, False)

        Q = np.diag(cfg.q_diag) ** 2
        R = np.diag([cfg.r_pos, cfg.r_pos, cfg.r_theta]) ** 2
        ukf = UKF(np.array(fix, dtype=float), np.eye(3) * 0.3, Q, R)

        start_cell = world_to_cell(fix[0], fix[1], cfg)
        home_cell = cfg.robots[name].home
        # Static obstacles + the other robots' cells; never block our own start.
        blocked = cfg.blocked | set(other_cells)
        blocked.discard(start_cell)
        path = a_star(start_cell, home_cell, blocked, cfg.grid_w, cfg.grid_h)
        if not path:
            await logger.log("PROC", f"all_go_home: '{name}' no path {start_cell} -> {home_cell}")
            return (name, False)

        ctrl = WaypointController(path, cfg)
        max_steps = max(1, int(cfg.max_time_s / cfg.dt))

        for _ in range(max_steps):
            x, y, th = ukf.x
            v, omega = ctrl.compute(x, y, th)

            driver.set_velocity(v, omega)            # real command (direct velocity stream)
            loc.apply_command(name, v, omega, cfg.dt)  # advance simulated truth (no-op for real ArUco)

            meas = await loc.get_pose(name)          # noisy pose now / real camera later
            ukf.predict((v, omega), cfg.dt)
            ukf.update(meas)                         # update() ignores None (dropped frame)

            if ctrl.reached(ukf.x[0], ukf.x[1]):
                driver.set_velocity(0.0, 0.0)
                await logger.log("PROC", f"all_go_home: '{name}' reached home")
                return (name, True)

            await asyncio.sleep(cfg.dt)

        driver.set_velocity(0.0, 0.0)
        await logger.log("PROC", f"all_go_home: '{name}' timed out before home")
        return (name, False)


class SyncTest(AbstractProcedure):
    """
    Group synchronization self-test for the whole active fleet.

    Drives every reachable robot through an identical sequence at the same time
    and verifies that each one is streaming live telemetry. Useful as a one-shot
    "is the lab healthy?" check before a session.

    Sequence (each phase is submitted to ALL devices at once and awaited, so the
    fleet stays in lockstep):
        1. beep for _BEEP_S seconds
        2. spin in place LEFT  for _SPIN_S seconds
        3. spin in place RIGHT for _SPIN_S seconds
        4. verify telemetry is flowing from every device

    Telemetry liveness: the driver keeps the latest sensor snapshot in
    get_telemetry() independently of any client subscription. We sample it,
    wait a short window, and sample again — a fresh snapshot object means new
    messages are arriving. If any device is silent, the procedure raises, so the
    caller receives a procedure_error naming the dead robots.

    Possible extensions: assert pwm_left/right != 0 during the spin phases to
    confirm the motion command actually reached the firmware, or compare encoder
    deltas to confirm the wheels turned.
    """

    name = "sync_test"

    _PRIORITY:    int   = 10    # above the default 5, so the test isn't starved
    _BEEP_S:      float = 1.0
    _SPIN_S:      float = 3.0
    _SPIN_SPEED:  float = 1.0   # rad/s, in-place rotation
    _TELE_WINDOW: float = 1.5   # sampling window for the telemetry liveness check

    async def run(self, manager: "RemoteLabManager", client_id: str, args: dict):
        logger = Logger.get()

        all_names = [d.name for d in manager.get_devices()]
        if not all_names:
            await logger.log("PROC", "sync_test: no active devices — nothing to test")
            return

        # Acquire every device. Shared devices always succeed; an exclusive device
        # held by another client is skipped (we test whatever we can reach).
        acquired = []
        for name in all_names:
            if manager.acquire_device(name, client_id):
                acquired.append(name)
            else:
                await logger.log("PROC", f"sync_test: skipping '{name}' (busy/owned)")
        if not acquired:
            await logger.log("PROC", "sync_test: no devices available right now")
            return

        try:
            await logger.log("PROC", f"sync_test: running on {acquired}")

            # 1. Beep — the whole group at once.
            await manager.submit_command_wait(
                client_id, acquired, "beep", self._PRIORITY,
                {"duration": self._BEEP_S},
            )

            # 2. Spin LEFT in place.
            await manager.submit_command_wait(
                client_id, acquired, "move", self._PRIORITY,
                {"speed_lin": 0.0, "speed_ang": self._SPIN_SPEED, "duration": self._SPIN_S},
            )

            # 3. Spin RIGHT in place.
            await manager.submit_command_wait(
                client_id, acquired, "move", self._PRIORITY,
                {"speed_lin": 0.0, "speed_ang": -self._SPIN_SPEED, "duration": self._SPIN_S},
            )

            # 4. Telemetry liveness check.
            alive, dead = await self._verify_telemetry(manager, acquired)
            for name in alive:
                await logger.log("PROC", f"sync_test: telemetry OK   '{name}'")
            for name in dead:
                await logger.log("PROC", f"sync_test: telemetry DEAD '{name}'")
            if dead:
                raise RuntimeError(f"No telemetry from: {dead}")

            await logger.log("PROC", f"sync_test: PASSED for {acquired}")

        finally:
            for name in acquired:
                manager.release_device(name, client_id)

    async def _verify_telemetry(self, manager, names):
        """Return (alive, dead): a device is alive if a fresh snapshot arrives within the window."""
        def snapshot(name):
            drv = manager.get_driver(name)
            return drv.get_telemetry() if drv else None

        first = {name: snapshot(name) for name in names}
        await asyncio.sleep(self._TELE_WINDOW)

        alive, dead = [], []
        for name in names:
            second = snapshot(name)
            # A new snapshot OBJECT (driver replaces it per message) proves live flow.
            if second is not None and first[name] is not second:
                alive.append(name)
            else:
                dead.append(name)
        return alive, dead


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

    def cancel_all(self, except_proc_client_id: Optional[str] = None) -> None:
        """
        Cancel every running procedure (used by the emergency stop_all).

        `except_proc_client_id` is the proc-client-id of the caller so stop_all does
        not cancel itself: a running procedure's proc-client-id is f"proc:{id[:8]}".
        """
        for proc_id, task in list(self._running.items()):
            if except_proc_client_id is not None and f"proc:{proc_id[:8]}" == except_proc_client_id:
                continue
            task.cancel()
