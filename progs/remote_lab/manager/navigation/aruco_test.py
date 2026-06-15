#!/usr/bin/env python3
"""
ArUco localization test/debug tool for the remote_lab navigation system.

Runs the SAME detection + homography math as the production ArucoLocalizationProvider
(it imports the helpers from navigation/aruco.py), so what you see here is exactly
what AllGoHome would get. Calibration is fiducial-based: >=4 ANCHOR markers at known
world coordinates (nav_config.json -> aruco.cameras[].anchors) rebuild the homography
every frame — no manual pixel picking.

Run it ON THE SERVER (where the USB webcam is plugged in), with the server NOT
running (only one process can open the camera). Requires opencv-contrib-python;
the visual mode also needs matplotlib.

────────────────────────────────────────────────────────────────────────────
MODES
────────────────────────────────────────────────────────────────────────────
Visual (default) — needs a display (or ssh -X):
    python navigation/aruco_test.py
  Left: camera with anchors (blue), the tracked robot (red), obstacle cubes
  (orange). Right: arena grid with the robot, home, static + detected obstacles.

Headless — console only, works over plain SSH:
    python navigation/aruco_test.py --headless

Flags:
   --robot NAME     which robot to track (default: first robot with a marker_id)
   --camera N       override the capture source (device index), ignoring the config
   --cam-index K    which entry of aruco.cameras[] to use (default 0)
   --config PATH    path to nav_config.json
"""

import argparse
import math
import sys
import time
from pathlib import Path

_MANAGER = Path(__file__).resolve().parent.parent      # manager/ (this file lives in manager/navigation/)
sys.path.insert(0, str(_MANAGER))

import cv2  # noqa: E402

from navigation.nav_config import load_nav_config  # noqa: E402
from navigation.aruco import (  # noqa: E402  (reuse the production math)
    _build_detector,
    _detect_markers,
    _homography_from_anchors,
    _marker_pose,
    _normalize_angle,
)


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


def _camera_cfg(cfg, args):
    cams = cfg.aruco.cameras
    if not cams or not (0 <= args.cam_index < len(cams)):
        sys.exit("nav_config.json has no aruco.cameras[] — add a camera with anchors first.")
    return cams[args.cam_index]


def _open_capture(source):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        sys.exit(f"Cannot open camera source {source!r}. Check the device index "
                 f"(--camera 0/1/2 ...) and that the server is not already using it.")
    # An opened-but-silent node (V4L2 'select() timeout') usually means the wrong
    # /dev/videoN — many webcams expose several and only one streams. Fail fast with
    # guidance instead of looping on empty reads forever.
    for _ in range(3):
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap
    cap.release()
    sys.exit(
        f"Camera source {source!r} opened but returned NO frames (V4L2 select() timeout).\n"
        f"  - This /dev/videoN is probably not the streaming node, or webcam passthrough is off.\n"
        f"  - Find the streaming index:  python navigation/aruco_test.py --list-cameras\n"
        f"  - Or override the config:    --camera 1   (then 2, 3 ...)\n"
        f"  - System check:              v4l2-ctl --list-devices   (sudo apt install v4l-utils)\n"
        f"  - VirtualBox: attach the cam via Devices > Webcams, or a USB filter + Extension Pack."
    )


def _list_cameras(max_index=5):
    """Probe /dev/video0..max_index and report which indices actually stream frames."""
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
            print(f"  index {i}: OK — streams {w}x{h}   <-- usable (set camera source to {i})")
            found.append(i)
        else:
            print(f"  index {i}: opens but returns NO frames (not a capture node)")
        cap.release()
    if not found:
        print("No streaming camera found. In VirtualBox check webcam passthrough "
              "(Devices > Webcams) / USB filter + Extension Pack; on the host the cam "
              "must be free (not used by another app).")
    else:
        print(f"Use one of: {found}. Put it in nav_config.json -> aruco.cameras[].source "
              f"(or pass --camera <n>).")


def _world_cell(x, y, cfg):
    cx = min(max(int(x // cfg.cell_size_m), 0), cfg.grid_w - 1)
    cy = min(max(int(y // cfg.cell_size_m), 0), cfg.grid_h - 1)
    return cx, cy


def _classify(cfg, camcfg, marker_id, target_marker):
    if marker_id in camcfg.anchors:
        return "anchor"
    if marker_id in set(cfg.aruco.obstacle_markers):
        return "obstacle"
    if marker_id == target_marker:
        return "robot"
    return "other"


def run_headless(cfg, args, robot, marker_id, yaw):
    camcfg = _camera_cfg(cfg, args)
    source = args.camera if args.camera is not None else camcfg.source
    cap = _open_capture(source)
    detect = _build_detector(cfg.aruco.dictionary)
    print(f"[test] tracking '{robot}' (marker {marker_id}); anchors needed: 4 of "
          f"{sorted(camcfg.anchors)}. Ctrl+C to stop.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.03)
                continue
            markers = _detect_markers(detect, frame)
            H = _homography_from_anchors(markers, camcfg.anchors)
            seen_anchors = [a for a in camcfg.anchors if a in markers]
            if H is None:
                print(f"  homography: only {len(seen_anchors)}/4 anchors visible "
                      f"({seen_anchors}) — cannot localize")
                time.sleep(0.4)
                continue
            # robot
            if marker_id in markers:
                x, y, th = _marker_pose(H, markers[marker_id])
                th = _normalize_angle(th + yaw)
                gx, gy = _world_cell(x, y, cfg)
                robot_str = f"x={x:6.3f} y={y:6.3f} th={math.degrees(th):7.2f}deg cell=({gx},{gy})"
            else:
                robot_str = "(robot marker not visible)"
            # obstacles
            obs_cells = []
            for mid in cfg.aruco.obstacle_markers:
                if mid in markers:
                    ox, oy, _ = _marker_pose(H, markers[mid])
                    obs_cells.append(_world_cell(ox, oy, cfg))
            print(f"  anchors={len(seen_anchors)}/4  robot: {robot_str}  obstacles={obs_cells}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


def run_visual(cfg, args, robot, marker_id, yaw):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    camcfg = _camera_cfg(cfg, args)
    source = args.camera if args.camera is not None else camcfg.source
    cap = _open_capture(source)
    detect = _build_detector(cfg.aruco.dictionary)

    xmax = cfg.grid_w * cfg.cell_size_m
    ymax = cfg.grid_h * cfg.cell_size_m
    home = cfg.robots[robot].home
    colors = {"anchor": (255, 100, 0), "robot": (0, 0, 255),
              "obstacle": (0, 140, 255), "other": (160, 160, 160)}  # BGR

    plt.ion()
    fig, (ax_cam, ax_grid) = plt.subplots(1, 2, figsize=(13, 6))
    fig.suptitle(f"ArUco test — tracking '{robot}' (marker {marker_id})")

    def draw_grid(pose, obs_cells, n_anchors):
        ax_grid.clear()
        ax_grid.set_xlim(0, xmax)
        ax_grid.set_ylim(0, ymax)
        ax_grid.set_aspect("equal")
        ax_grid.set_xlabel("x, m")
        ax_grid.set_ylabel("y, m")
        for i in range(cfg.grid_w + 1):
            ax_grid.axvline(i * cfg.cell_size_m, color="0.9", lw=0.5)
        for j in range(cfg.grid_h + 1):
            ax_grid.axhline(j * cfg.cell_size_m, color="0.9", lw=0.5)
        for ox, oy in cfg.obstacles:                      # static obstacles (config)
            ax_grid.add_patch(Rectangle((ox * cfg.cell_size_m, oy * cfg.cell_size_m),
                                        cfg.cell_size_m, cfg.cell_size_m, color="0.6"))
        for ox, oy in obs_cells:                          # camera-detected cubes
            ax_grid.add_patch(Rectangle((ox * cfg.cell_size_m, oy * cfg.cell_size_m),
                                        cfg.cell_size_m, cfg.cell_size_m, color="orange", alpha=0.7))
        for ax_id, (wx, wy) in camcfg.anchors.items():    # anchor world positions
            ax_grid.plot(wx, wy, "x", color="blue", markersize=8)
        hx = (home[0] + 0.5) * cfg.cell_size_m
        hy = (home[1] + 0.5) * cfg.cell_size_m
        ax_grid.plot(hx, hy, marker="*", color="green", markersize=16)
        if pose is not None:
            x, y, th = pose
            ax_grid.plot(x, y, "o", color="red", markersize=8)
            L = 0.6 * cfg.cell_size_m
            ax_grid.arrow(x, y, L * math.cos(th), L * math.sin(th),
                          head_width=0.15 * cfg.cell_size_m, color="red", length_includes_head=True)
            gx, gy = _world_cell(x, y, cfg)
            ax_grid.set_title(f"x={x:.3f} y={y:.3f} th={math.degrees(th):.1f}deg cell=({gx},{gy})")
        else:
            ax_grid.set_title(f"robot not localized (anchors {n_anchors}/4)")

    print("[test] close the window to quit.")
    while plt.fignum_exists(fig.number):
        ok, frame = cap.read()
        if not ok:
            plt.pause(0.03)
            continue
        markers = _detect_markers(detect, frame)
        H = _homography_from_anchors(markers, camcfg.anchors)
        n_anchors = sum(1 for a in camcfg.anchors if a in markers)

        pose, obs_cells = None, []
        for mid, corners in markers.items():
            kind = _classify(cfg, camcfg, mid, marker_id)
            c = corners.astype(int)
            cv2.polylines(frame, [c.reshape(-1, 1, 2)], True, colors[kind], 2)
            cv2.putText(frame, str(mid), tuple(c[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[kind], 2)
            if H is not None and kind == "robot":
                x, y, th = _marker_pose(H, corners)
                pose = (x, y, _normalize_angle(th + yaw))
            elif H is not None and kind == "obstacle":
                ox, oy, _ = _marker_pose(H, corners)
                obs_cells.append(_world_cell(ox, oy, cfg))

        ax_cam.clear()
        ax_cam.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        ax_cam.axis("off")
        ax_cam.set_title("camera — blue=anchor red=robot orange=obstacle")
        draw_grid(pose, obs_cells, n_anchors)
        plt.pause(0.03)

    cap.release()


def main():
    ap = argparse.ArgumentParser(description="ArUco localization test for remote_lab")
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent / "nav_config.json"))
    ap.add_argument("--robot", default=None, help="robot name to track")
    ap.add_argument("--camera", type=int, default=None, help="override capture device index")
    ap.add_argument("--cam-index", type=int, default=0, help="which aruco.cameras[] entry to use")
    ap.add_argument("--headless", action="store_true", help="console-only output (no GUI)")
    ap.add_argument("--list-cameras", action="store_true",
                    help="probe device indices 0..5 for a streaming camera and exit")
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
