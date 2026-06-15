#!/usr/bin/env python3
"""
Generate printable ArUco marker PNGs for the remote_lab robots.

The dictionary MUST match nav_config.json -> aruco.dictionary (default DICT_5X5_50),
and each --id MUST match a robot's marker_id in that config. Print the PNGs at
100% (no "fit to page" scaling), keep the WHITE BORDER around the black square
(it's a required quiet zone), glue each to a rigid flat card, and mount it
horizontally on top of the matching robot.

Examples (run from the manager/ directory):
    python navigation/make_markers.py --id 1 --id 10 --id 11 --id 12 --id 13 --id 20 --id 21 --id 22
    python navigation/make_markers.py --id 1 --dict DICT_5X5_50 --px 800 --out markers
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def make(dict_name, marker_id, px, out_dir, border_px):
    aruco = cv2.aruco
    dict_id = getattr(aruco, dict_name)
    dictionary = (aruco.getPredefinedDictionary(dict_id)
                  if hasattr(aruco, "getPredefinedDictionary")
                  else aruco.Dictionary_get(dict_id))

    # OpenCV >=4.7 renamed drawMarker -> generateImageMarker
    gen = getattr(aruco, "generateImageMarker", None) or aruco.drawMarker
    img = gen(dictionary, marker_id, px)

    # Add a white quiet-zone border (detection needs it).
    canvas = np.full((px + 2 * border_px, px + 2 * border_px), 255, dtype=np.uint8)
    canvas[border_px:border_px + px, border_px:border_px + px] = img

    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{dict_name}_id{marker_id}.png"
    cv2.imwrite(str(path), canvas)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description="Generate ArUco markers")
    ap.add_argument("--dict", default="DICT_5X5_50", help="cv2.aruco dictionary name")
    ap.add_argument("--id", type=int, action="append", required=True, dest="ids",
                    help="marker id (repeat for several)")
    ap.add_argument("--px", type=int, default=800, help="marker size in pixels (print bigger = detected farther)")
    ap.add_argument("--border", type=int, default=80, help="white border in pixels")
    ap.add_argument("--out", default="markers", help="output directory")
    args = ap.parse_args()

    out_dir = Path(args.out)
    for mid in args.ids:
        make(args.dict, mid, args.px, out_dir, args.border)


if __name__ == "__main__":
    main()
