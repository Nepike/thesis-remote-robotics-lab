#!/usr/bin/env python3
"""
Generate printable ArUco markers laid out on A4 sheets for the remote_lab robots.

Markers are tiled onto A4 pages with margins, a cutting gap between them, a thin
cut outline and an id label under each one — so a whole batch prints on as few
sheets as needed and is easy to cut out. Physical size is set in millimetres, so
print at 100% / "actual size" and the markers come out exactly that big.

Output (into --out, default navigation/markers/ next to this script):
  - one PNG per A4 page (markers_page01.png, ...);
  - markers.pdf — a single print-ready A4 PDF with correct sizing (needs Pillow;
    skipped with a note if it is not installed).

The dictionary MUST match nav_config.json -> aruco.dictionary (default DICT_5X5_50),
and each id MUST match its role there (robot marker_id / anchors / obstacle_markers).
Keep the WHITE BORDER around each marker (it is a required quiet zone).

Examples (run from the manager/ directory):
    # robot 1, anchors 10-13, obstacles 20-22 — all on one A4 at 50 mm:
    python navigation/make_markers.py --id 1 --id 10 --id 11 --id 12 --id 13 --id 20 --id 21 --id 22

    # 6 markers per page, 70 mm each, bigger cutting gap:
    python navigation/make_markers.py --id 1 --id 2 --id 3 --id 4 --id 5 --id 6 \
        --marker-mm 70 --per-page 6 --gap-mm 12
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

A4_W_MM, A4_H_MM = 210.0, 297.0


def _mm(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def _make_marker(dict_name: str, marker_id: int, side_px: int) -> np.ndarray:
    """Render one ArUco marker (no border) as a square grayscale image."""
    aruco = cv2.aruco
    try:
        dict_id = getattr(aruco, dict_name)
    except AttributeError as e:
        raise SystemExit(f"Unknown ArUco dictionary '{dict_name}'") from e
    dictionary = (aruco.getPredefinedDictionary(dict_id)
                  if hasattr(aruco, "getPredefinedDictionary")
                  else aruco.Dictionary_get(dict_id))
    gen = getattr(aruco, "generateImageMarker", None) or aruco.drawMarker
    return gen(dictionary, marker_id, side_px)


def _layout(dpi, marker_mm, border_mm, gap_mm, margin_mm, label_mm):
    """Compute pixel geometry and how many marker tiles fit on one A4 page."""
    page_w, page_h = _mm(A4_W_MM, dpi), _mm(A4_H_MM, dpi)
    marker_px = _mm(marker_mm, dpi)
    border_px = _mm(border_mm, dpi)
    gap = _mm(gap_mm, dpi)
    margin = _mm(margin_mm, dpi)
    label_h = _mm(label_mm, dpi)

    tile = marker_px + 2 * border_px              # white tile = marker + quiet zone
    cell_w = tile + gap
    cell_h = tile + label_h + gap
    usable_w = page_w - 2 * margin
    usable_h = page_h - 2 * margin

    cols = int((usable_w + gap) // cell_w)
    rows = int((usable_h + gap) // cell_h)
    if cols < 1 or rows < 1:
        raise SystemExit(
            f"A {marker_mm} mm marker (+{border_mm} mm border) does not fit on A4 with "
            f"these margins/gaps. Reduce --marker-mm/--margin-mm/--gap-mm."
        )
    return dict(page_w=page_w, page_h=page_h, marker_px=marker_px, border_px=border_px,
                gap=gap, margin=margin, label_h=label_h, tile=tile,
                cols=cols, rows=rows, fit=cols * rows)


def _render_page(ids, dict_name, geo, dpi):
    """Draw one A4 page (grayscale) holding the markers in `ids`, centred."""
    page = np.full((geo["page_h"], geo["page_w"]), 255, dtype=np.uint8)
    tile, gap, label_h = geo["tile"], geo["gap"], geo["label_h"]
    cols = min(geo["cols"], len(ids))
    rows_full = max(1, math.ceil(len(ids) / geo["cols"]))

    grid_w = cols * tile + (cols - 1) * gap
    grid_h = rows_full * (tile + label_h) + (rows_full - 1) * gap
    x0 = (geo["page_w"] - grid_w) // 2
    y0 = (geo["page_h"] - grid_h) // 2

    font = cv2.FONT_HERSHEY_SIMPLEX
    fscale = dpi / 300.0 * 0.5
    fthick = max(1, dpi // 300)
    short = dict_name.replace("DICT_", "")

    for idx, mid in enumerate(ids):
        r, c = divmod(idx, cols)
        tx = x0 + c * (tile + gap)
        ty = y0 + r * (tile + label_h + gap)

        marker = _make_marker(dict_name, mid, geo["marker_px"])
        b = geo["border_px"]
        page[ty + b:ty + b + geo["marker_px"], tx + b:tx + b + geo["marker_px"]] = marker

        # cut outline around the white tile + id label centred below it
        cv2.rectangle(page, (tx, ty), (tx + tile, ty + tile), 170, 1)
        text = f"{short}  id {mid}"
        (tw, th), _ = cv2.getTextSize(text, font, fscale, fthick)
        lx = tx + (tile - tw) // 2
        ly = ty + tile + (label_h + th) // 2
        cv2.putText(page, text, (lx, ly), font, fscale, 0, fthick, cv2.LINE_AA)
    return page


def main():
    ap = argparse.ArgumentParser(description="Generate A4 sheets of ArUco markers")
    ap.add_argument("--id", type=int, action="append", required=True, dest="ids",
                    help="marker id (repeat for several)")
    ap.add_argument("--dict", default="DICT_5X5_50", help="cv2.aruco dictionary name")
    ap.add_argument("--marker-mm", type=float, default=50.0, help="black-square size, mm")
    ap.add_argument("--border-mm", type=float, default=5.0, help="white quiet-zone border, mm")
    ap.add_argument("--gap-mm", type=float, default=8.0, help="gap between markers for cutting, mm")
    ap.add_argument("--margin-mm", type=float, default=10.0, help="page margin, mm")
    ap.add_argument("--label-mm", type=float, default=6.0, help="height of the id label strip, mm")
    ap.add_argument("--per-page", type=int, default=0, help="markers per page (0 = as many as fit)")
    ap.add_argument("--dpi", type=int, default=300, help="print resolution")
    ap.add_argument("--out", default=None,
                    help="output directory (default: navigation/markers/ next to this script)")
    args = ap.parse_args()

    geo = _layout(args.dpi, args.marker_mm, args.border_mm, args.gap_mm, args.margin_mm, args.label_mm)
    per_page = geo["fit"] if args.per_page <= 0 else min(args.per_page, geo["fit"])
    if 0 < args.per_page and args.per_page > geo["fit"]:
        print(f"[markers] only {geo['fit']} markers fit per A4 at {args.marker_mm} mm "
              f"({geo['cols']}x{geo['rows']}); using {geo['fit']} per page.")

    # Default next to this script (navigation/markers/) so the folder is created in
    # the navigation package regardless of the current working directory.
    out_dir = Path(args.out) if args.out else Path(__file__).resolve().parent / "markers"
    out_dir.mkdir(parents=True, exist_ok=True)

    pages = []
    n_pages = math.ceil(len(args.ids) / per_page)
    for p in range(n_pages):
        chunk = args.ids[p * per_page:(p + 1) * per_page]
        page = _render_page(chunk, args.dict, geo, args.dpi)
        pages.append(page)
        path = out_dir / f"markers_page{p + 1:02d}.png"
        cv2.imwrite(str(path), page)
        print(f"wrote {path}  ({len(chunk)} markers)")

    print(f"[markers] {len(args.ids)} markers, {per_page}/page, {n_pages} A4 page(s), "
          f"{args.marker_mm} mm each — print at 100% / actual size.")

    # Best-effort print-ready PDF (exact A4 sizing) if Pillow is available.
    try:
        from PIL import Image
    except ImportError:
        print("[markers] Pillow not installed -> PDF skipped. `pip install pillow` for a "
              "single print-ready PDF; otherwise print the PNG pages at 100%.")
        return
    pdf_path = out_dir / "markers.pdf"
    imgs = [Image.fromarray(p) for p in pages]
    imgs[0].save(pdf_path, "PDF", resolution=float(args.dpi), save_all=True, append_images=imgs[1:])
    print(f"wrote {pdf_path}  (print at 100% / actual size)")


if __name__ == "__main__":
    main()
