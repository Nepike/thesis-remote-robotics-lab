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
║  TODO (когда установят камеры + ArUco): реализовать ArucoLocalizationProvider  ║
║  и переключить provider в nav_config.json на "aruco". Логику навигации менять  ║
║  НЕ нужно — только источник поз. См. класс ArucoLocalizationProvider ниже.     ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import math
import random
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

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
    Real pose source: two ceiling cameras + ArUco markers (НЕ РЕАЛИЗОВАНО).

    TODO (завтра, после установки камер):
      1. Открыть камеры (cv2.VideoCapture) по индексам/URL из cfg.
      2. На каждый кадр: cv2.aruco.detectMarkers со словарём из cfg.
      3. cv2.aruco.estimatePoseSingleMarkers (нужны cameraMatrix/distCoeffs из
         калибровки) → поза маркера в системе камеры.
      4. Перевести в МИРОВУЮ систему координат полигона (extrinsic-калибровка
         камеры: положение/ориентация камеры над ареной), при двух камерах —
         сшить/выбрать лучшую видимость.
      5. Сопоставить marker_id → имя робота (cfg.robots[name].marker_id).
      6. Кэшировать последнюю позу каждого робота; get_pose() возвращает её
         (или None, если маркер сейчас не виден).
    Метод apply_command остаётся no-op (истинное движение наблюдают камеры).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        # marker_id -> robot_name (готовая привязка для шага 5)
        self._marker_to_robot = {
            r.marker_id: name for name, r in cfg.robots.items() if r.marker_id is not None
        }
        raise NotImplementedError(
            "ArucoLocalizationProvider ещё не реализован. Установите камеры и "
            "реализуйте захват/детекцию (см. TODO в классе), затем используйте его."
        )

    async def get_pose(self, robot_name: str) -> Optional[Pose]:  # pragma: no cover
        raise NotImplementedError


def build_localization_provider(cfg) -> LocalizationProvider:
    """Factory: pick the provider by cfg.provider ('simulated' | 'aruco')."""
    if cfg.provider == "aruco":
        return ArucoLocalizationProvider(cfg)
    return SimulatedLocalizationProvider(cfg)
