"""
Navigation subsystem for the AllGoHome group procedure.

ROS-independent on purpose: the algorithms (A*, UKF, waypoint controller) and the
localization providers do not import rospy, so they can be unit-tested in isolation
and reused by any procedure. The procedure layer (Procedures.AllGoHome) wires them
to real devices via the manager.

Ported from the AllGoHome simulator (github.com/Nepike/allgohome).
"""

from .algorithms import UKF, WaypointController, a_star, normalize_angle, world_to_cell
from .localization import (
    LocalizationProvider,
    SimulatedLocalizationProvider,
    ArucoLocalizationProvider,
    build_localization_provider,
)
from .nav_config import NavConfig, RobotNav, load_nav_config

__all__ = [
    "UKF",
    "WaypointController",
    "a_star",
    "normalize_angle",
    "world_to_cell",
    "LocalizationProvider",
    "SimulatedLocalizationProvider",
    "ArucoLocalizationProvider",
    "build_localization_provider",
    "NavConfig",
    "RobotNav",
    "load_nav_config",
]
