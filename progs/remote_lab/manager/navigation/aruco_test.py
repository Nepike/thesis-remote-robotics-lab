#!/usr/bin/env python3
"""
ArUco localization test/debug tool for the remote_lab navigation (FUSION model).

Runs the SAME detection + homography math as the production ArucoLocalizationProvider
(it imports the helpers from navigation/aruco.py) AND the SAME UKF fusion as AllGoHome
(navigation/algorithms.py), so what you see here is exactly what the procedure gets.

Every configured camera sees the whole arena and shares the 4 corner anchors
(nav_config.json -> arena.corner_markers). Each camera produces one pose per frame;
they are FUSED in the UKF (predict + one update per camera, weighted by each
camera's R). The map shows each camera's RAW pose AND the fused estimate, so you can
see the fused point sit between the cameras and be steadier than either.

Run it ON THE SERVER (where the USB webcams are plugged in), with the server NOT
running (only one process can open a camera). Requires opencv-contrib-python; the
visual mode also needs matplotlib. Works with 1 camera too (degenerates to a single
measurement + UKF smoothing); plug in the second and it fuses automatically.

MODES
  Visual (default; needs a display or ssh -X):  python navigation/aruco_test.py
      One panel per camera + an arena map: each camera's raw robot pose in its own
      colour, the fused UKF estimate in black, home (green star), obstacles.
  Headless (console only, plain SSH):           python navigation/aruco_test.py --headless

Flags:
   --robot NAME     which robot to track (default: first robot with a marker_id)
   --camera N       use ONLY this device index (ignore aruco.cameras[]); single-camera check
   --width / --height   capture size px (lower = less lag / fixes V4L2 timeout)
   --config PATH    path to nav_config.json
   --list-cameras   probe device indices 0..5 and exit
"""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

_MANAGER = Path(__file__).resolve().parent.parent      # manager/ (this file lives in manager/navigation/)
sys.path.insert(0, str(_MANAGER))

import numpy as np  # noqa: E402
import cv2  # noqa: E402

from navigation.nav_config import load_nav_config  # noqa: E402
from navigation.algorithms import UKF  # noqa: E402  (same fusion filter as AllGoHome)
from navigation.aruco import (  # noqa: E402  (reuse the production math)
    _build_detector,
    _detect_markers,
    _homography_from_anchors,
    _marker_pose,
    _normalize_angle,
)

_CAM_COLORS_BGR = [(0, 0, 255), (0, 200, 0), (255, 120, 0), (200, 0, 200)]   # per-camera (cam0 red, cam1 green, ...)


def _pick_target_robot(cfg, name):
    robots = cfg.robots
    if name:
        if name not in robots:
            sys.exit(f"Robot '{name}' not in nav_config.json (have: {list(robots)})")
        r = robots[name]
        if r.marker_id is None:
            sys.exit(f"Robot '{name}' has no marker_id in nav_config.json")
        return name, r.marker_id, r.marker_yaw_offset
    for n, r in robots.items():
        if r.marker_id is not None:
            return n, r.marker_id, r.marker_yaw_offset
    sys.exit("No robot has a marker_id in nav_config.json — set one first.")


def _cam_R(cfg, idx):
    """Measurement covariance for camera `idx` (its fusion weight)."""
    cams = cfg.aruco.cameras
    if 0 <= idx < len(cams):
        c = cams[idx]
        return np.diag([c.r_pos, c.r_pos, c.r_theta]) ** 2
    return np.diag([cfg.r_pos, cfg.r_pos, cfg.r_theta]) ** 2


class _FrameGrabber:
    """
    Background camera reader that always holds only the LATEST frame.

    OpenCV's VideoCapture buffers frames internally; if the consumer (matplotlib
    redraw) is slower than the camera, read() keeps returning ever-staler frames and
    lag piles up. This thread reads in a tight loop and keeps just the newest frame.
    Raises RuntimeError on failure so the caller can skip a missing camera.
    """

    def __init__(self, source, width=None, height=None):
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open source {source!r}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        frame = None
        for _ in range(5):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                break
        if frame is None:
            self._cap.release()
            raise RuntimeError(f"source {source!r} opened but returned NO frames (V4L2 select() timeout; "
                               f"wrong /dev/videoN or passthrough off — try --list-cameras / --width 640)")

        self._frame = frame
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"grabber-{source}")
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            with self._lock:
                self._frame = frame

    def read(self):
        with self._lock:
            return True, self._frame.copy()

    def release(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._cap.release()


def _list_cameras(max_index=5):
    print(f"Probing camera indices 0..{max_index} (a dead node can block ~10s)...")
    found = []
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            print(f"  index {i}: not opened")
            cap.release()
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            print(f"  index {i}: OK — streams {w}x{h}   <-- usable")
            found.append(i)
        else:
            print(f"  index {i}: opens but returns NO frames (not a capture node)")
        cap.release()
    print(f"Usable indices: {found}" if found else "No streaming camera found.")


def _open_cameras(cfg, args):
    """Open every configured camera (or the --camera override). Returns [(idx, source, grabber)]."""
    if args.camera is not None:
        sources = [(0, args.camera)]
    else:
        sources = [(i, c.source) for i, c in enumerate(cfg.aruco.cameras)]
    if not sources:
        sys.exit("nav_config.json has no aruco.cameras[] — add cameras first.")

    grabbers = []
    for idx, src in sources:
        try:
            g = _FrameGrabber(src, args.width, args.height)
            grabbers.append((idx, src, g))
            print(f"[test] camera {idx} (source {src!r}) opened")
        except RuntimeError as e:
            print(f"[test] camera {idx} (source {src!r}) FAILED: {e}")
    if not grabbers:
        sys.exit("No camera could be opened. Check --list-cameras / VirtualBox webcam passthrough.")
    if len(grabbers) == 1:
        print("[test] only ONE camera available — fusion degenerates to single measurement + UKF smoothing.")
    return grabbers


def _world_cell(x, y, cfg):
    cx = min(max(int(x // cfg.cell_size_m), 0), cfg.grid_w - 1)
    cy = min(max(int(y // cfg.cell_size_m), 0), cfg.grid_h - 1)
    return cx, cy


def _classify(cfg, anchors, marker_id, target_marker):
    if marker_id in anchors:
        return "anchor"
    if marker_id in set(cfg.aruco.obstacle_markers):
        return "obstacle"
    if marker_id == target_marker:
        return "robot"
    return "other"


class _Fuser:
    """Wraps the AllGoHome UKF: init on first measurement, then predict + N updates per frame."""

    def __init__(self, cfg):
        self._Q = np.diag(cfg.q_diag) ** 2
        self._ukf = None
        self._t = None

    def step(self, measurements):
        """measurements: list of (pose, R). Returns the fused pose or None."""
        now = time.time()
        if not measurements and self._ukf is None:
            return None
        if self._ukf is None:
            self._ukf = UKF(np.array(measurements[0][0], dtype=float), np.eye(3) * 0.3, self._Q, measurements[0][1])
            self._t = now
            return tuple(self._ukf.x)
        dt = max(now - self._t, 1e-3)
        self._t = now
        self._ukf.predict((0.0, 0.0), dt)            # no odometry in the test; updates do the work
        for pose, R in measurements:
            self._ukf.update(pose, R)
        return tuple(self._ukf.x)


def _process(cfg, grabbers, detect, anchors, marker_id, yaw, draw_frames):
    """
    One round over all cameras. Returns:
      per_cam_pose: {cam_idx: (x,y,th)}   raw pose from each camera (if robot visible)
      measurements: [(pose, R), ...]      for the fuser
      obstacle_cells: set of cells
      frames: {cam_idx: annotated_frame} if draw_frames else {}
    """
    per_cam_pose, measurements, obstacle_cells, frames = {}, [], set(), {}
    obstacle_ids = set(cfg.aruco.obstacle_markers)
    for idx, src, g in grabbers:
        ok, frame = g.read()
        if not ok:
            continue
        markers = _detect_markers(detect, frame)
        H = _homography_from_anchors(markers, anchors)

        if draw_frames:
            for mid, corners in markers.items():
                kind = _classify(cfg, anchors, mid, marker_id)
                color = {"anchor": (255, 100, 0), "robot": (0, 0, 255),
                         "obstacle": (0, 140, 255), "other": (160, 160, 160)}[kind]
                c = corners.astype(int)
                cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, color, 2)
                cv2.putText(frame, str(mid), tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            frames[idx] = frame

        if H is None:
            continue
        if marker_id in markers:
            x, y, th = _marker_pose(H, markers[marker_id])
            pose = (x, y, _normalize_angle(th + yaw))
            per_cam_pose[idx] = pose
            measurements.append((pose, _cam_R(cfg, idx)))
        for mid in obstacle_ids:
            if mid in markers:
                ox, oy, _ = _marker_pose(H, markers[mid])
                obstacle_cells.add(_world_cell(ox, oy, cfg))
    return per_cam_pose, measurements, obstacle_cells, frames


def run_headless(cfg, args, robot, marker_id, yaw):
    grabbers = _open_cameras(cfg, args)
    detect = _build_detector(cfg.aruco.dictionary)
    anchors = cfg.aruco.anchors
    fuser = _Fuser(cfg)
    print(f"[test] tracking '{robot}' (marker {marker_id}); anchors {sorted(anchors)}. Ctrl+C to stop.")
    try:
        while True:
            per_cam, meas, obs, _ = _process(cfg, grabbers, detect, anchors, marker_id, yaw, draw_frames=False)
            fused = fuser.step(meas)
            parts = []
            for idx, _src, _g in grabbers:
                if idx in per_cam:
                    x, y, th = per_cam[idx]
                    parts.append(f"cam{idx}: x={x:.3f} y={y:.3f} th={math.degrees(th):6.1f}")
                else:
                    parts.append(f"cam{idx}: --")
            if fused is not None:
                fx, fy, fth = fused
                fstr = f"FUSED x={fx:.3f} y={fy:.3f} th={math.degrees(fth):6.1f} cell={_world_cell(fx, fy, cfg)}"
            else:
                fstr = "FUSED --"
            print(f"  [{len(meas)} meas] " + "  |  ".join(parts) + "   ||   " + fstr + f"   obstacles={sorted(obs)}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    finally:
        for _i, _s, g in grabbers:
            g.release()


def run_visual(cfg, args, robot, marker_id, yaw):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    grabbers = _open_cameras(cfg, args)
    detect = _build_detector(cfg.aruco.dictionary)
    anchors = cfg.aruco.anchors
    fuser = _Fuser(cfg)

    xmax = cfg.grid_w * cfg.cell_size_m
    ymax = cfg.grid_h * cfg.cell_size_m
    home = cfg.robots[robot].home

    n = len(grabbers)
    plt.ion()
    fig, axes = plt.subplots(1, n + 1, figsize=(5 * (n + 1), 5.5))
    if n + 1 == 1:
        axes = [axes]
    cam_axes = axes[:n]
    ax_map = axes[n]
    fig.suptitle(f"ArUco fusion test — '{robot}' (marker {marker_id}), {n} camera(s)")

    def cam_color_mpl(idx):
        b, gg, r = _CAM_COLORS_BGR[idx % len(_CAM_COLORS_BGR)]
        return (r / 255.0, gg / 255.0, b / 255.0)

    def draw_arrow(ax, x, y, th, color, lw):
        L = 0.6 * cfg.cell_size_m
        ax.arrow(x, y, L * math.cos(th), L * math.sin(th),
                 head_width=0.18 * cfg.cell_size_m, color=color, lw=lw, length_includes_head=True)

    def draw_map(per_cam, fused, obs_cells):
        ax_map.clear()
        ax_map.set_xlim(0, xmax)
        ax_map.set_ylim(0, ymax)
        ax_map.set_aspect("equal")
        ax_map.set_xlabel("x, m")
        ax_map.set_ylabel("y, m")
        for i in range(cfg.grid_w + 1):
            ax_map.axvline(i * cfg.cell_size_m, color="0.92", lw=0.5)
        for j in range(cfg.grid_h + 1):
            ax_map.axhline(j * cfg.cell_size_m, color="0.92", lw=0.5)
        for ox, oy in cfg.obstacles:
            ax_map.add_patch(Rectangle((ox * cfg.cell_size_m, oy * cfg.cell_size_m),
                                       cfg.cell_size_m, cfg.cell_size_m, color="0.6"))
        for ox, oy in obs_cells:
            ax_map.add_patch(Rectangle((ox * cfg.cell_size_m, oy * cfg.cell_size_m),
                                       cfg.cell_size_m, cfg.cell_size_m, color="orange", alpha=0.7))
        for aid, (wx, wy) in anchors.items():
            ax_map.plot(wx, wy, "x", color="0.4", markersize=8)
        ax_map.plot((home[0] + 0.5) * cfg.cell_size_m, (home[1] + 0.5) * cfg.cell_size_m,
                    marker="*", color="green", markersize=16)
        # raw per-camera poses (each its own colour)
        for idx, (x, y, th) in per_cam.items():
            col = cam_color_mpl(idx)
            ax_map.plot(x, y, "o", color=col, markersize=7, alpha=0.7, label=f"cam{idx}")
            draw_arrow(ax_map, x, y, th, col, 1.5)
        # fused estimate (black, bold)
        if fused is not None:
            fx, fy, fth = fused
            ax_map.plot(fx, fy, "o", color="black", markersize=9, label="fused (UKF)")
            draw_arrow(ax_map, fx, fy, fth, "black", 2.5)
            ax_map.set_title(f"fused x={fx:.3f} y={fy:.3f} th={math.degrees(fth):.1f}deg "
                             f"cell={_world_cell(fx, fy, cfg)}  [{len(per_cam)} cam]")
        else:
            ax_map.set_title("robot not localized")
        # Only draw a legend when something labelled is on the axes, otherwise
        # matplotlib spams "No artists with labels found" every frame.
        if ax_map.get_legend_handles_labels()[1]:
            ax_map.legend(loc="upper right", fontsize=8)

    print("[test] close the window to quit.")
    while plt.fignum_exists(fig.number):
        per_cam, meas, obs, frames = _process(cfg, grabbers, detect, anchors, marker_id, yaw, draw_frames=True)
        fused = fuser.step(meas)
        for ax_i, (idx, src, _g) in zip(cam_axes, grabbers):
            ax_i.clear()
            if idx in frames:
                ax_i.imshow(cv2.cvtColor(frames[idx], cv2.COLOR_BGR2RGB))
            ax_i.axis("off")
            seen = "robot SEEN" if idx in per_cam else "robot not visible"
            ax_i.set_title(f"camera {idx} (src {src}) — {seen}")
        draw_map(per_cam, fused, obs)
        plt.pause(0.03)

    for _i, _s, g in grabbers:
        g.release()


def main():
    ap = argparse.ArgumentParser(description="ArUco fusion localization test for remote_lab")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent / "nav_config.json"))
    ap.add_argument("--robot", default=None, help="robot name to track")
    ap.add_argument("--camera", type=int, default=None, help="use ONLY this device index (single-camera check)")
    ap.add_argument("--width", type=int, default=None, help="capture width px")
    ap.add_argument("--height", type=int, default=None, help="capture height px")
    ap.add_argument("--headless", action="store_true", help="console-only output (no GUI)")
    ap.add_argument("--list-cameras", action="store_true", help="probe device indices 0..5 and exit")
    args = ap.parse_args()

    if args.list_cameras:
        _list_cameras()
        return

    cfg = load_nav_config(Path(args.config))
    if cfg is None:
        sys.exit(f"Could not load {args.config} (see error above).")

    robot, marker_id, yaw = _pick_target_robot(cfg, args.robot)
    if args.headless:
        run_headless(cfg, args, robot, marker_id, yaw)
    else:
        run_visual(cfg, args, robot, marker_id, yaw)


if __name__ == "__main__":
    main()
