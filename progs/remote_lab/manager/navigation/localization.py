"""
Localization providers: the source of global robot poses (x, y, theta) for the
AllGoHome procedure.

The procedure is written against the abstract LocalizationProvider, so the pose
source can be swapped without touching the navigation logic.

Currently active: SimulatedLocalizationProvider — advances an internal "true" pose
per robot from the commanded velocities and returns it with Gaussian noise (and
occasional dropped frames), exactly like the camera model in the AllGoHome
simulator. This lets the whole procedure run and be validated end-to-end before the
cameras exist.

╔══════════════════════════════════════════════════════════════════════════════╗
║  ArucoLocalizationProvider реализован (потолочные камеры + ArUco + гомография). ║
║  Чтобы переключиться на реальные камеры: provider="aruco" в nav_config.json и   ║
║  заполнить секцию "aruco" (камеры + точки калибровки). Логику навигации менять  ║
║  НЕ нужно — меняется только источник поз. Захват — см. navigation/aruco.py.    ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import math
import random
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Set, Tuple

Pose = Tuple[float, float, float]  # (x, y, theta) in metres, radians


def _normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class LocalizationProvider(ABC):
    """Abstract source of global robot poses."""

    @abstractmethod
    async def get_pose(self, robot_name: str) -> Optional[Pose]:
        """Latest measured pose, or None if the robot is not currently observed."""

    def register(self, robot_name: str, start_pose: Pose) -> None:
        """Optional init hook (used by the simulated provider to seed true state)."""

    def apply_command(self, robot_name: str, v: float, omega: float, dt: float) -> None:
        """
        Feed back the commanded velocity.

        No-op for a real provider (the cameras observe actual motion). The simulated
        provider uses it to advance its internal ground truth.
        """

    def get_dynamic_obstacles(self) -> Set[Tuple[int, int]]:
        """
        Grid cells currently occupied by camera-detected obstacles (ArUco cubes).

        Merged with the static config obstacles by the procedure at planning time.
        Empty for sources that cannot see obstacles (e.g. the simulated provider).
        """
        return set()

    def describe(self) -> str:
        return self.__class__.__name__


class SimulatedLocalizationProvider(LocalizationProvider):
    """
    Placeholder pose source for development without cameras.

    Keeps a "true" pose per robot, advances it by the unicycle model using the
    commanded (v, omega), and returns a noisy measurement — mirroring the simulator
    camera model (Gaussian noise + MEAS_DROP_PROB dropped frames).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._truth: Dict[str, List[float]] = {}

    def register(self, robot_name: str, start_pose: Pose) -> None:
        self._truth[robot_name] = [float(start_pose[0]), float(start_pose[1]), float(start_pose[2])]

    def apply_command(self, robot_name: str, v: float, omega: float, dt: float) -> None:
        st = self._truth.get(robot_name)
        if st is None:
            return
        x, y, th = st
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = _normalize_angle(th + omega * dt)
        self._truth[robot_name] = [x, y, th]

    async def get_pose(self, robot_name: str) -> Optional[Pose]:
        st = self._truth.get(robot_name)
        if st is None:
            return None
        if random.random() < self.cfg.drop_prob:
            return None  # dropped frame -> filter skips its update step
        x, y, th = st
        return (
            x + random.gauss(0.0, self.cfg.meas_pos_std),
            y + random.gauss(0.0, self.cfg.meas_pos_std),
            _normalize_angle(th + random.gauss(0.0, self.cfg.meas_theta_std)),
        )

    def describe(self) -> str:
        return "SimulatedLocalizationProvider (ПЛЕЙСХОЛДЕР — заменить на ArUco)"


class ArucoLocalizationProvider(LocalizationProvider):
    """
    Real pose source: ceiling camera(s) + ArUco markers, mapped to arena metres by
    a per-camera homography (planar scene, no intrinsic calibration required).

    Construction starts a background capture/detection engine (one thread per
    camera) that keeps a per-robot pose cache up to date; get_pose() just reads it.
    The heavy lifting (OpenCV, threads, homography) lives in navigation/aruco.py,
    imported lazily here so the simulated provider works without OpenCV installed.

    Designed to be a long-lived singleton (see build_localization_provider): the
    cameras open once and the engine runs for the whole server lifetime, regardless
    of how many times AllGoHome is invoked. apply_command stays a no-op — the
    cameras observe the robots' actual motion.

    Configuration (nav_config.json):
      - aruco.dictionary / aruco.max_pose_age_s / aruco.cameras[] (source + the
        pixel<->metre calibration points);
      - per robot: marker_id (which tag) and marker_yaw_offset (mounting rotation).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        # marker_id -> robot_name, and robot_name -> mounting yaw offset (radians).
        self._marker_to_robot = {
            r.marker_id: name for name, r in cfg.robots.items() if r.marker_id is not None
        }
        yaw_by_robot = {name: r.marker_yaw_offset for name, r in cfg.robots.items()}

        try:
            from .aruco import ArucoEngine  # lazy: OpenCV is only needed for the real provider
        except ImportError as e:
            raise RuntimeError(
                "ArUco provider requires opencv-contrib-python (cv2.aruco): "
                "pip install opencv-contrib-python"
            ) from e

        self._engine = ArucoEngine(cfg.aruco, self._marker_to_robot, yaw_by_robot)
        self._engine.start()

    async def get_pose(self, robot_name: str) -> Optional[Pose]:
        # Reading the cache is non-blocking; the blocking camera I/O runs in the
        # engine's own threads, so this never stalls the event loop.
        return self._engine.get(robot_name)

    def get_dynamic_obstacles(self) -> Set[Tuple[int, int]]:
        """Map every freshly-seen obstacle cube to the grid cell it sits in."""
        cell = self.cfg.cell_size_m
        cells: Set[Tuple[int, int]] = set()
        for x, y in self._engine.get_obstacle_points():
            cx = min(max(int(x // cell), 0), self.cfg.grid_w - 1)
            cy = min(max(int(y // cell), 0), self.cfg.grid_h - 1)
            cells.add((cx, cy))
        return cells

    def close(self) -> None:
        """Stop the capture threads and release the cameras (call on server shutdown)."""
        self._engine.stop()

    def describe(self) -> str:
        return (
            f"ArucoLocalizationProvider ({len(self.cfg.aruco.cameras)} camera(s), "
            f"{self.cfg.aruco.dictionary})"
        )


# Long-lived singleton: the ArUco engine opens the cameras once and runs for the
# whole server. build_localization_provider() is called on every AllGoHome run, so
# it must return the SAME instance rather than re-opening the cameras each time.
_aruco_singleton: Optional[ArucoLocalizationProvider] = None


def build_localization_provider(cfg) -> LocalizationProvider:
    """
    Factory: pick the provider by cfg.provider ('simulated' | 'aruco').

    The simulated provider is stateful per run and created fresh each time. The
    ArUco provider is a process-wide singleton (cameras stay open between runs);
    camera/calibration changes in nav_config.json take effect on server restart.
    """
    if cfg.provider == "aruco":
        global _aruco_singleton
        if _aruco_singleton is None:
            _aruco_singleton = ArucoLocalizationProvider(cfg)
        return _aruco_singleton
    return SimulatedLocalizationProvider(cfg)


def shutdown_localization() -> None:
    """
    Stop the ArUco singleton's capture threads and release its cameras.

    Idempotent and safe to call when no ArUco provider was ever built. Intended to
    be invoked from RemoteLabManager.shutdown() so USB cameras are released cleanly
    on a normal server stop.
    """
    global _aruco_singleton
    if _aruco_singleton is not None:
        _aruco_singleton.close()
        _aruco_singleton = None
