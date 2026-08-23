"""
Localization providers: the source of global robot pose MEASUREMENTS (x, y, theta)
for the AllGoHome procedure.

The procedure is written against the abstract LocalizationProvider, so the pose
source can be swapped without touching the navigation logic.

FUSION model: get_measurements(robot) returns a LIST of (pose, R) — one entry per
camera that currently sees the robot, each with that camera's measurement-noise
covariance R. The procedure folds them all into the UKF (sequential updates), so
more views give a better estimate. A single measurement is just a list of length 1.

Providers:
  - SimulatedLocalizationProvider — no cameras. Advances an internal "true" pose
    per robot from the commanded velocities and returns N independent noisy
    measurements (N = number of configured cameras, default 2), so the fusion path
    can be validated end-to-end before the cameras exist.
  - ArucoLocalizationProvider — real ceiling cameras + ArUco, fused. Switch with
    "provider": "aruco" in nav_config.json. Capture/homography live in
    navigation/aruco.py (imported lazily so OpenCV is only needed for the real one).
"""

import math
import random
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import numpy as np

Pose = Tuple[float, float, float]                 # (x, y, theta) in metres, radians
Measurement = Tuple[Pose, np.ndarray]             # (pose, R 3x3)


def _normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class LocalizationProvider(ABC):
    """Abstract source of global robot pose measurements (fused downstream)."""

    @abstractmethod
    async def get_measurements(self, robot_name: str) -> List[Measurement]:
        """
        All fresh pose measurements for a robot, each as (pose, R).

        One entry per camera that currently sees the robot; empty if unseen. The
        caller fuses them in its filter.
        """

    async def get_pose(self, robot_name: str) -> Optional[Pose]:
        """Convenience: the first available measurement's pose, or None. (For init.)"""
        m = await self.get_measurements(robot_name)
        return m[0][0] if m else None

    def register(self, robot_name: str, start_pose: Pose) -> None:
        """Optional init hook (used by the simulated provider to seed true state)."""

    def apply_command(self, robot_name: str, v: float, omega: float, dt: float) -> None:
        """
        Feed back the commanded velocity.

        No-op for a real provider (the cameras observe actual motion). The simulated
        provider uses it to advance its internal ground truth.
        """

    def get_dynamic_obstacles(self):
        """
        Grid cells currently occupied by camera-detected obstacles (ArUco cubes).

        Merged (and inflated) with the static config obstacles by the procedure at
        planning time. Empty for sources that cannot see obstacles (e.g. simulated).
        """
        return set()

    def describe(self) -> str:
        return self.__class__.__name__


class SimulatedLocalizationProvider(LocalizationProvider):
    """
    Placeholder pose source for development without cameras.

    Keeps a "true" pose per robot, advances it by the unicycle model using the
    commanded (v, omega), and returns N independent noisy measurements per tick
    (N = number of configured cameras, default 2) — mirroring N cameras that each
    observe the same robot, so the fusion path is exercised exactly as in reality.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._truth: Dict[str, List[float]] = {}
        self._n_cams = max(1, len(cfg.aruco.cameras))
        self._R = np.diag([cfg.r_pos, cfg.r_pos, cfg.r_theta]) ** 2

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

    async def get_measurements(self, robot_name: str) -> List[Measurement]:
        st = self._truth.get(robot_name)
        if st is None:
            return []
        x, y, th = st
        out: List[Measurement] = []
        for _ in range(self._n_cams):
            if random.random() < self.cfg.drop_prob:
                continue  # this "camera" dropped the frame
            pose = (
                x + random.gauss(0.0, self.cfg.meas_pos_std),
                y + random.gauss(0.0, self.cfg.meas_pos_std),
                _normalize_angle(th + random.gauss(0.0, self.cfg.meas_theta_std)),
            )
            out.append((pose, self._R))
        return out

    def describe(self) -> str:
        return f"SimulatedLocalizationProvider (ПЛЕЙСХОЛДЕР, {self._n_cams} вирт. камеры — заменить на ArUco)"


class ArucoLocalizationProvider(LocalizationProvider):
    """
    Real pose source: multiple ceiling cameras + ArUco, FUSED. Every camera sees the
    whole arena and shares the 4 corner anchors; each camera's pose is one fused
    measurement.

    Construction starts a background capture/detection engine (one thread per
    camera) that keeps per-(robot, camera) measurements up to date;
    get_measurements() just reads them. The heavy lifting (OpenCV, threads,
    homography) lives in navigation/aruco.py, imported lazily here so the simulated
    provider works without OpenCV installed.

    Long-lived singleton (see build_localization_provider): cameras open once and
    the engine runs for the whole server lifetime. apply_command stays a no-op —
    the cameras observe the robots' actual motion.

    Configuration (nav_config.json):
      - arena.corner_markers -> the shared anchors; aruco.cameras[] (source + optional
        r_pos/r_theta fusion weight); aruco.dictionary / max_pose_age_s / obstacle_*;
      - per robot: marker_id (which tag) and marker_yaw_offset (mounting rotation).
    """

    def __init__(self, cfg):
        self.cfg = cfg
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

    async def get_measurements(self, robot_name: str) -> List[Measurement]:
        # Reading the caches is non-blocking; the blocking camera I/O runs in the
        # engine's own threads, so this never stalls the event loop.
        return self._engine.get_measurements(robot_name)

    def get_dynamic_obstacles(self):
        """Map every freshly-seen obstacle cube to the grid cell it sits in."""
        cell = self.cfg.cell_size_m
        cells = set()
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
            f"ArucoLocalizationProvider (fusion of {len(self.cfg.aruco.cameras)} camera(s), "
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
