"""
ArUco capture + homography engine behind ArucoLocalizationProvider.

Kept separate from localization.py so that importing the navigation package (and
running the simulated provider) never pulls in OpenCV: cv2 is imported only here,
and this module is only imported when the real ArUco provider is constructed.

Coordinate calibration is fiducial-based and self-correcting: a set of ANCHOR
markers sit at known world (x, y) positions (nav_config.json -> aruco.cameras[].
anchors). Every frame, if >=4 anchors are visible, their detected pixel centres +
known metres rebuild the camera homography (pixels -> arena metres) for that frame
— so the camera can be bumped/moved without re-calibration, as long as the anchors
stay put and visible. If fewer than 4 are visible, the last good homography is
reused. Mount anchor markers at the SAME height as the robot markers so the planar
homography has no parallax error.

Pipeline (per camera, in its own background thread because VideoCapture.read() is
blocking):
    grab frame -> detectMarkers -> rebuild homography from visible anchors
    -> for every non-anchor marker: map its corners through H to arena metres
       (centre = mean of corners, heading = atan2 of the marker's +x edge in world)
    -> robot markers update the pose cache; obstacle markers update the obstacle cache.

get() returns a robot pose only if fresher than max_pose_age_s (else None, matching
the LocalizationProvider contract). With two cameras the larger-area (more head-on)
detection wins while fresh. get_obstacle_points() returns the world positions of
obstacle cubes seen within obstacle_max_age_s (static cubes are remembered a few
seconds so a momentary occlusion does not drop them).
"""

import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import cv2

Pose = Tuple[float, float, float]


def _normalize_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _build_detector(dict_name: str) -> Callable:
    """
    Return a detect(gray) -> (corners, ids, rejected) callable for the named
    predefined dictionary, bridging the cv2.aruco API change in OpenCV 4.7.
    """
    aruco = cv2.aruco
    try:
        dict_id = getattr(aruco, dict_name)
    except AttributeError as e:
        raise ValueError(f"Unknown ArUco dictionary '{dict_name}' (cv2.aruco has no such constant)") from e

    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
    else:                                              # very old API
        dictionary = aruco.Dictionary_get(dict_id)

    if hasattr(aruco, "ArucoDetector"):               # OpenCV >= 4.7
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)

    params = aruco.DetectorParameters_create()        # OpenCV <= 4.6
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def _detect_markers(detect: Callable, frame) -> Dict[int, np.ndarray]:
    """Detect every marker in `frame`. Returns {marker_id: corners (4, 2) pixels}."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detect(gray)
    out: Dict[int, np.ndarray] = {}
    if ids is None:
        return out
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        out[int(marker_id)] = marker_corners.reshape(4, 2)   # TL, TR, BR, BL
    return out


def _homography_from_anchors(
    markers: Dict[int, np.ndarray],
    anchors_world: Dict[int, Tuple[float, float]],
) -> Optional[np.ndarray]:
    """
    Fit a pixels -> arena-metres homography from the currently visible anchors.

    Uses each visible anchor's pixel CENTRE against its known world (x, y). Returns
    None if fewer than 4 anchors are visible (caller should reuse the last good H).
    """
    img_pts, wld_pts = [], []
    for marker_id, world_xy in anchors_world.items():
        corners = markers.get(marker_id)
        if corners is None:
            continue
        img_pts.append(corners.mean(axis=0))      # pixel centre of the anchor
        wld_pts.append([world_xy[0], world_xy[1]])
    if len(img_pts) < 4:
        return None
    H, _ = cv2.findHomography(np.asarray(img_pts, dtype=np.float64),
                              np.asarray(wld_pts, dtype=np.float64))
    return H


def _map_points(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Map an (N, 2) array of pixel points through H to (N, 2) world metres."""
    src = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(src, H).reshape(-1, 2)


def _marker_pose(H: np.ndarray, corners: np.ndarray) -> Pose:
    """
    Map one marker's pixel corners through H to a world pose (x, y, theta).

    Heading WITHOUT any per-robot yaw offset: theta is the direction of the marker's
    +x edge in WORLD coords (averaged over both edges that run along it, because a
    direction is not linear under perspective).
    """
    world = _map_points(H, corners)                  # (4, 2) metres
    cx = float(world[:, 0].mean())
    cy = float(world[:, 1].mean())
    d = (world[1] - world[0]) + (world[2] - world[3])
    theta = math.atan2(float(d[1]), float(d[0]))
    return cx, cy, theta


class _Camera:
    """One physical camera: its capture source, its anchors, and its latest homography."""

    def __init__(self, source, anchors: Dict[int, Tuple[float, float]], detect: Callable):
        self.source = source
        self.anchors = dict(anchors)
        self._detect = detect
        self.H: Optional[np.ndarray] = None          # last good homography (None until 4 anchors seen)

    def process(self, frame) -> Tuple[Dict[int, np.ndarray], Optional[np.ndarray]]:
        """Detect markers and refresh the homography from visible anchors."""
        markers = _detect_markers(self._detect, frame)
        H = _homography_from_anchors(markers, self.anchors)
        if H is not None:
            self.H = H
        return markers, self.H


class ArucoEngine:
    """
    Background ArUco localization: one thread per camera feeding shared caches.

    Constructed once and kept alive for the whole server (the provider is a
    singleton), so cameras open exactly once. Call start() after construction and
    stop() on shutdown.

    Two caches are maintained:
      - robot poses (marker_id -> robot), read by get();
      - obstacle cube positions (ids in obstacle_markers), read by
        get_obstacle_points() and turned into grid cells by the provider.
    """

    def __init__(self, aruco_cfg, marker_to_robot: Dict[int, str], yaw_by_robot: Dict[str, float]):
        self._marker_to_robot = dict(marker_to_robot)
        self._yaw = dict(yaw_by_robot)
        self._obstacle_markers = set(aruco_cfg.obstacle_markers)
        self._max_age = float(aruco_cfg.max_pose_age_s)
        self._obstacle_max_age = float(aruco_cfg.obstacle_max_age_s)

        detect = _build_detector(aruco_cfg.dictionary)
        self._cameras = [_Camera(c.source, c.anchors, detect) for c in aruco_cfg.cameras]

        # robot_name -> (pose, timestamp, pixel_area)
        self._cache: Dict[str, Tuple[Pose, float, float]] = {}
        # obstacle marker_id -> (world (x, y), timestamp)
        self._obstacles: Dict[int, Tuple[Tuple[float, float], float]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    def start(self) -> None:
        if self._threads:
            return
        if not self._cameras:
            print("[aruco] no cameras configured in nav_config.json — get_pose() will always return None")
        for cam in self._cameras:
            if len(cam.anchors) < 4:
                print(f"[aruco] camera {cam.source!r} has {len(cam.anchors)} anchors (<4) — "
                      f"it cannot build a homography and will report nothing")
            t = threading.Thread(target=self._run_camera, args=(cam,), daemon=True, name=f"aruco-cam-{cam.source}")
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

    def get(self, robot_name: str) -> Optional[Pose]:
        """Latest fresh pose for a robot, or None if not observed within max_pose_age_s."""
        with self._lock:
            entry = self._cache.get(robot_name)
        if entry is None:
            return None
        pose, ts, _area = entry
        if time.time() - ts > self._max_age:
            return None
        return pose

    def get_obstacle_points(self) -> List[Tuple[float, float]]:
        """World (x, y) of every obstacle cube seen within obstacle_max_age_s."""
        now = time.time()
        with self._lock:
            return [xy for xy, ts in self._obstacles.values() if now - ts <= self._obstacle_max_age]

    def _update(self, robot: str, pose: Pose, area: float, now: float) -> None:
        with self._lock:
            prev = self._cache.get(robot)
            # Within the freshness window keep the larger-area (more head-on)
            # detection; once the previous one is stale, any new detection replaces it.
            if prev is not None:
                _p, p_ts, p_area = prev
                if (now - p_ts) <= self._max_age and p_area > area:
                    return
            self._cache[robot] = (pose, now, area)

    def _update_obstacle(self, marker_id: int, xy: Tuple[float, float], now: float) -> None:
        with self._lock:
            self._obstacles[marker_id] = (xy, now)

    def _run_camera(self, cam: _Camera) -> None:
        cap = cv2.VideoCapture(cam.source)
        if not cap.isOpened():
            print(f"[aruco] cannot open camera source {cam.source!r}")
            return
        print(f"[aruco] camera {cam.source!r} opened")
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                markers, H = cam.process(frame)
                if H is None:
                    continue   # not enough anchors seen yet — no metric frame available
                now = time.time()
                for marker_id, corners in markers.items():
                    if marker_id in cam.anchors:
                        continue
                    if marker_id in self._obstacle_markers:
                        ox, oy, _ = _marker_pose(H, corners)
                        self._update_obstacle(marker_id, (ox, oy), now)
                        continue
                    robot = self._marker_to_robot.get(marker_id)
                    if robot is None:
                        continue
                    x, y, theta = _marker_pose(H, corners)
                    pose = (x, y, _normalize_angle(theta + self._yaw.get(robot, 0.0)))
                    area = float(cv2.contourArea(corners.astype(np.float32)))
                    self._update(robot, pose, area, now)
        finally:
            cap.release()
            print(f"[aruco] camera {cam.source!r} released")
