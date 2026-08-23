"""
Configuration for the AllGoHome navigation: arena map, per-robot homes/markers,
controller and filter parameters, and the multi-camera ArUco fusion setup.

Stored in manager/navigation/nav_config.json (gitignored — deployment-specific). If
the file is missing it is created from a template using the default device names, so
the procedure is runnable out of the box with the simulated pose provider.

Camera model = FUSION: every camera sees the WHOLE arena, so all cameras share the
same 4 corner anchors, and each camera's pose is an independent measurement fused in
the Kalman filter (weighted by per-camera r_pos / r_theta). No tiling / per-camera
anchor subsets.

Units: cells for the grid; metres and radians for the world; m/s and rad/s for
speeds. One cell == cell_size_m metres.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

_CONFIG_PATH = Path(__file__).parent / "nav_config.json"  # manager/navigation/nav_config.json

_TEMPLATE = {
    "_note": "AllGoHome map & params. 'arena' is the SINGLE source of truth: field size, "
             "cell size, and which ArUco id sits in each corner (corner_markers bl/br/tr/tl). "
             "The grid AND the anchor world coordinates both derive from it — measure the "
             "rectangle through the 4 corner-marker centres, name the corners, done. FUSION "
             "setup: every camera sees the WHOLE arena (so all share the same 4 corner anchors) "
             "and their poses are fused in the Kalman filter. m/s & rad/s for speeds. "
             "provider: 'simulated' (no camera) or 'aruco' (real cameras).",
    "provider": "simulated",
    "arena": {
        "width_m": 1.0,
        "height_m": 0.6,
        "cell_size_m": 0.1,
        "corner_markers": {"bl": 10, "br": 11, "tr": 12, "tl": 13}
    },
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
        "_note": "Fusion pose source (provider='aruco'). EVERY camera sees the whole arena and "
                 "shares the 4 corner anchors from arena.corner_markers (>=4 must be visible per "
                 "camera to build its homography). Each camera produces one pose per robot; the "
                 "poses are fused in the UKF, weighted by per-camera r_pos/r_theta (default = "
                 "noise.r_pos/r_theta; a sharper/closer camera -> smaller R -> more trust). "
                 "Mount the corner markers at the SAME height as the robot markers (no parallax). "
                 "obstacle_markers are mapped to grid cells live. Id ranges: robots 1-9, "
                 "anchors 10-19, obstacles 20-49.",
        "dictionary": "DICT_5X5_50",
        "max_pose_age_s": 0.4,
        "obstacle_max_age_s": 3.0,
        "obstacle_markers": [20, 21, 22],
        "cameras": [
            {"source": 0},
            {"source": 1}
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
    source: object        # cv2.VideoCapture source: device index (int) or stream URL (str)
    r_pos: float          # measurement noise std (metres) for this camera's pose -> fusion weight
    r_theta: float        # measurement noise std (radians) for this camera's heading


@dataclass
class ArucoConfig:
    dictionary: str = "DICT_5X5_50"
    max_pose_age_s: float = 0.4               # a robot pose older than this -> dropped from fusion
    obstacle_max_age_s: float = 3.0           # a detected obstacle cube is remembered this long after last seen
    obstacle_markers: List[int] = field(default_factory=list)   # ids treated as dynamic obstacles
    anchors: Dict[int, Tuple[float, float]] = field(default_factory=dict)   # GLOBAL corner anchors (all cameras share)
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
        self.blocked = self.inflate(self.obstacles)

    def inflate(self, cells) -> Set[Tuple[int, int]]:
        """
        Expand each cell by a 1-cell buffer (8-neighbourhood), clamped to the grid.

        Used for the static config obstacles (here) and for the live camera-detected
        obstacle cubes (in the procedure), so both get the same clearance.
        """
        out: Set[Tuple[int, int]] = set()
        for ox, oy in cells:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    x, y = ox + dx, oy + dy
                    if 0 <= x < self.grid_w and 0 <= y < self.grid_h:
                        out.add((x, y))
        return out


_CORNERS = {"bl": (0.0, 0.0), "br": (1.0, 0.0), "tr": (1.0, 1.0), "tl": (0.0, 1.0)}


def _corner_anchors(corner_markers: dict, width_m: float, height_m: float) -> Dict[int, Tuple[float, float]]:
    """Expand corner_markers {'bl':10,...} -> {10:(0,0), 11:(W,0), 12:(W,H), 13:(0,H)}."""
    out: Dict[int, Tuple[float, float]] = {}
    for corner, mid in corner_markers.items():
        key = str(corner).lower()
        if key not in _CORNERS:
            raise ValueError(f"corner '{corner}' must be one of bl/br/tr/tl")
        ux, uy = _CORNERS[key]
        out[int(mid)] = (ux * width_m, uy * height_m)
    return out


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

    # 'arena' is the single source of truth: grid size + the corner anchors (shared
    # by all cameras, since every camera sees the whole arena in the fusion setup).
    arena = raw["arena"]
    cell = float(arena.get("cell_size_m", 0.1))
    width_m = float(arena["width_m"])
    height_m = float(arena["height_m"])
    grid_w = max(1, int(round(width_m / cell)))
    grid_h = max(1, int(round(height_m / cell)))
    anchors = _corner_anchors(arena.get("corner_markers", {}), width_m, height_m)

    g_r_pos = float(noise.get("r_pos", 0.12))
    g_r_theta = float(noise.get("r_theta", 0.1))
    araw = raw.get("aruco", {})
    cameras = [
        CameraConfig(
            source=c["source"],
            r_pos=float(c.get("r_pos", g_r_pos)),       # default to the global measurement noise
            r_theta=float(c.get("r_theta", g_r_theta)),
        )
        for c in araw.get("cameras", [])
    ]
    aruco = ArucoConfig(
        dictionary=araw.get("dictionary", "DICT_5X5_50"),
        max_pose_age_s=float(araw.get("max_pose_age_s", 0.4)),
        obstacle_max_age_s=float(araw.get("obstacle_max_age_s", 3.0)),
        obstacle_markers=[int(m) for m in araw.get("obstacle_markers", [])],
        anchors=anchors,
        cameras=cameras,
    )
    return NavConfig(
        provider=raw.get("provider", "simulated"),
        cell_size_m=cell,
        grid_w=grid_w,
        grid_h=grid_h,
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
        r_pos=g_r_pos,
        r_theta=g_r_theta,
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
