"""
Configuration for the AllGoHome navigation: arena map, per-robot homes/markers,
controller and filter parameters.

Stored in manager/nav_config.json (gitignored — deployment-specific). If the file
is missing it is created from a template using the default device names, so the
procedure is runnable out of the box with the simulated pose provider.

Units: cells for the grid; metres and radians for the world; m/s and rad/s for
speeds. One cell == cell_size_m metres.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

_CONFIG_PATH = Path(__file__).parent.parent / "nav_config.json"  # manager/nav_config.json

_TEMPLATE = {
    "_note": "AllGoHome map & params. Cells for the grid; m/s & rad/s for speeds. "
             "provider: 'simulated' (no camera) or 'aruco' (real cameras). "
             "Pre-filled with the Test-1 example: 1 robot (marker 1), 4 anchors "
             "(10-13), 3 obstacle cubes (20-22), 1 camera, arena 1.0x0.6 m. Adjust "
             "the grid size and the MEASURED anchor metres to your arena, then set "
             "provider='aruco' for a real run.",
    "provider": "simulated",
    "cell_size_m": 0.1,
    "grid_w": 10,
    "grid_h": 6,
    "dt": 0.1,
    "max_linear_speed": 0.25,
    "max_angular_speed": 1.5,
    "angular_speed_k": 1.3,
    "waypoint_threshold": 0.45,
    "angle_threshold": 0.25,
    "max_time_s": 120,
    "noise": {
        "pos_std": 0.01,
        "theta_std": 0.03,
        "drop_prob": 0.04,
        "q_diag": [0.02, 0.02, 0.05],
        "r_pos": 0.02,
        "r_theta": 0.05,
    },
    "obstacles": [],
    "aruco": {
        "_note": "Real-camera pose source (provider='aruco'). Per camera, 'anchors' maps "
                 "ArUco marker_id -> world (x, y) metres of that marker's CENTRE; >=4 "
                 "anchors visible in a frame rebuild the homography that frame, so the "
                 "camera may be moved freely as long as the anchors stay put and visible. "
                 "Mount anchor markers at the SAME height as the robot markers to avoid "
                 "parallax. 'obstacle_markers' lists ids treated as obstacles and mapped "
                 "to grid cells live. Keep id ranges distinct: robots 1-9, anchors 10-19, "
                 "obstacles 20-49.",
        "dictionary": "DICT_5X5_50",
        "max_pose_age_s": 0.4,
        "obstacle_max_age_s": 3.0,
        "obstacle_markers": [20, 21, 22],
        "cameras": [
            {
                "source": 0,
                "anchors": {
                    "10": [0.05, 0.05],
                    "11": [0.95, 0.05],
                    "12": [0.95, 0.55],
                    "13": [0.05, 0.55]
                }
            }
        ],
    },
    "robots": {
        "Test-device-1": {"home": [1, 1], "start": [8, 4, 0.0], "marker_id": 1, "marker_yaw_offset": 0.0}
    },
}


@dataclass
class RobotNav:
    home: Tuple[int, int]                 # target cell
    start: Tuple[float, float, float]     # start cell + heading (cx, cy, theta_rad); simulated provider only
    marker_id: Optional[int] = None       # ArUco marker id (real provider)
    marker_yaw_offset: float = 0.0        # radians: marker mounting rotation vs robot forward (real provider)


@dataclass
class CameraConfig:
    source: object                              # cv2.VideoCapture source: device index (int) or stream URL (str)
    anchors: Dict[int, Tuple[float, float]]     # marker_id -> world (x, y) metres of the marker centre (homography)


@dataclass
class ArucoConfig:
    dictionary: str = "DICT_5X5_50"
    max_pose_age_s: float = 0.4               # a robot pose older than this -> get_pose() returns None
    obstacle_max_age_s: float = 3.0           # a detected obstacle cube is remembered this long after last seen
    obstacle_markers: List[int] = field(default_factory=list)   # ids treated as dynamic obstacles
    cameras: List[CameraConfig] = field(default_factory=list)


@dataclass
class NavConfig:
    provider: str
    cell_size_m: float
    grid_w: int
    grid_h: int
    dt: float
    max_linear_speed: float
    max_angular_speed: float
    angular_speed_k: float
    waypoint_threshold: float
    angle_threshold: float
    max_time_s: float
    # measurement / process noise
    meas_pos_std: float
    meas_theta_std: float
    drop_prob: float
    q_diag: List[float]
    r_pos: float
    r_theta: float
    # map
    obstacles: List[Tuple[int, int]]
    robots: Dict[str, RobotNav]
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    blocked: Set[Tuple[int, int]] = field(default_factory=set)

    def __post_init__(self):
        self.blocked = self._inflate(self.obstacles, self.grid_w, self.grid_h)

    @staticmethod
    def _inflate(obstacles, grid_w, grid_h) -> Set[Tuple[int, int]]:
        """Obstacles plus a 1-cell inflation buffer (8-neighbourhood)."""
        blocked: Set[Tuple[int, int]] = set()
        for ox, oy in obstacles:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    x, y = ox + dx, oy + dy
                    if 0 <= x < grid_w and 0 <= y < grid_h:
                        blocked.add((x, y))
        return blocked


def _parse(raw: dict) -> NavConfig:
    noise = raw.get("noise", {})
    robots = {
        name: RobotNav(
            home=tuple(r["home"]),
            start=tuple(r.get("start", [0.0, 0.0, 0.0])),
            marker_id=r.get("marker_id"),
            marker_yaw_offset=float(r.get("marker_yaw_offset", 0.0)),
        )
        for name, r in raw.get("robots", {}).items()
    }
    araw = raw.get("aruco", {})
    aruco = ArucoConfig(
        dictionary=araw.get("dictionary", "DICT_5X5_50"),
        max_pose_age_s=float(araw.get("max_pose_age_s", 0.4)),
        obstacle_max_age_s=float(araw.get("obstacle_max_age_s", 3.0)),
        obstacle_markers=[int(m) for m in araw.get("obstacle_markers", [])],
        cameras=[
            CameraConfig(
                source=c["source"],
                anchors={int(k): tuple(v) for k, v in c.get("anchors", {}).items()},
            )
            for c in araw.get("cameras", [])
        ],
    )
    return NavConfig(
        provider=raw.get("provider", "simulated"),
        cell_size_m=float(raw["cell_size_m"]),
        grid_w=int(raw["grid_w"]),
        grid_h=int(raw["grid_h"]),
        dt=float(raw["dt"]),
        max_linear_speed=float(raw["max_linear_speed"]),
        max_angular_speed=float(raw["max_angular_speed"]),
        angular_speed_k=float(raw["angular_speed_k"]),
        waypoint_threshold=float(raw["waypoint_threshold"]),
        angle_threshold=float(raw["angle_threshold"]),
        max_time_s=float(raw.get("max_time_s", 120)),
        meas_pos_std=float(noise.get("pos_std", 0.12)),
        meas_theta_std=float(noise.get("theta_std", 0.05)),
        drop_prob=float(noise.get("drop_prob", 0.04)),
        q_diag=list(noise.get("q_diag", [0.08, 0.08, 0.03])),
        r_pos=float(noise.get("r_pos", 0.12)),
        r_theta=float(noise.get("r_theta", 0.1)),
        obstacles=[tuple(o) for o in raw.get("obstacles", [])],
        robots=robots,
        aruco=aruco,
    )


def load_nav_config(path: Path = _CONFIG_PATH) -> Optional[NavConfig]:
    """
    Load the navigation config, creating a template on first run.

    Returns a NavConfig, or None if the file is present but unreadable/invalid
    (the caller should then log and abort the procedure).
    """
    if not path.exists():
        path.write_text(json.dumps(_TEMPLATE, indent=2) + "\n", encoding="utf-8")
        print(f"[nav] Created template config: {path}")
        return _parse(_TEMPLATE)

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
        return _parse(raw)
    except (json.JSONDecodeError, OSError, KeyError, ValueError, TypeError) as e:
        print(f"[nav] Cannot load {path.name}: {e}")
        return None
