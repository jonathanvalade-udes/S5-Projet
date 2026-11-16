#!/usr/bin/env python3
"""Texture du circuit Godot pour Webots : ruban noir sur sol blanc, a partir de worlds/godot_course.json.

Le ruban de Godot fait 20 mm ; le vrai ruban isolant fait 18 mm (PRD S2, guide de calibration). Par defaut on erode la
surface du ruban de 1 mm (un point reste noir si tout le disque de 1 mm autour de lui est sur le ruban) : meme trace, et les
raccords internes entre morceaux du maillage n'ont aucun effet. Le maillage Godot a un trou de ~20 x 9 mm a la jonction du T
d'arrivee (aussi present dans Godot) : la barre du T est completee par un rectangle plein, comme un vrai ruban. Anti-crenelage par sur-echantillonnage (3 x 3 points par pixel) : les
bords sont gris, comme la tache d'un capteur qui chevauche le bord du ruban.

    python3 webots/tools/generate_course_texture.py                    # worlds/textures/godot_course.png
    python3 webots/tools/generate_course_texture.py --tape-width 0.020 # largeur Godot d'origine

La texture couvre un sol carre de --floor-size metres centre en (--center-x, --center-y) : ces valeurs doivent etre celles du
RectangleArena de worlds/picar_s_course.wbt (floorSize et translation). Ligne 0 de l'image = cote +y du sol.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from generate_track_texture import write_rgb_png

WEBOTS = Path(__file__).resolve().parent.parent
GODOT_TAPE_M = 0.020
CELL = 0.01   # grille spatiale (m)


def build_grid(items, bbox_of):
    grid = defaultdict(list)
    for item in items:
        x0, y0, x1, y1 = bbox_of(item)
        for gx in range(int(math.floor(x0 / CELL)), int(math.floor(x1 / CELL)) + 1):
            for gy in range(int(math.floor(y0 / CELL)), int(math.floor(y1 / CELL)) + 1):
                grid[(gx, gy)].append(item)
    return grid


def tri_bbox(t):
    xs, ys = [p[0] for p in t], [p[1] for p in t]
    return min(xs), min(ys), max(xs), max(ys)


def inside(px, py, t):
    (ax, ay), (bx, by), (cx, cy) = t
    d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
    d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
    return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))


def generate(out: Path, size_px: int, floor_size: float, cx: float, cy: float, tape_width: float) -> Path:
    data = json.loads((WEBOTS / "worlds" / "godot_course.json").read_text())
    tris = list(data["triangles"])
    # barre du T d'arrivee : rectangle plein sur l'emprise de la barre (bouche le trou du maillage a la jonction)
    bar = [p for t in tris for p in t if p[0] > 1.05 and 0.15 < p[1] < 0.25]
    if bar:
        x0, x1 = min(p[0] for p in bar), max(p[0] for p in bar)
        y0, y1 = min(p[1] for p in bar), max(p[1] for p in bar)
        tris += [[[x0, y0], [x1, y0], [x1, y1]], [[x0, y0], [x1, y1], [x0, y1]]]
    shrink = max(0.0, (GODOT_TAPE_M - tape_width) / 2.0)
    tri_grid = build_grid(tris, tri_bbox)
    px_m = floor_size / size_px
    x_left, y_top = cx - floor_size / 2.0, cy + floor_size / 2.0
    ring = [(shrink * math.cos(k * math.pi / 8), shrink * math.sin(k * math.pi / 8)) for k in range(16)]

    def on_tape(x: float, y: float) -> bool:
        cell = (int(math.floor(x / CELL)), int(math.floor(y / CELL)))
        return any(inside(x, y, t) for t in tri_grid.get(cell, ()))

    def dark(x: float, y: float) -> bool:
        # erosion : le point et tout le cercle de rayon `shrink` autour de lui sont sur le ruban
        return on_tape(x, y) and (shrink <= 0.0 or all(on_tape(x + dx, y + dy) for dx, dy in ring))

    rgb = bytearray(b"\xff" * (size_px * size_px * 3))
    pixels = set()
    for t in tris:   # seuls les pixels sous le ruban sont calcules
        x0, y0, x1, y1 = tri_bbox(t)
        for ix in range(int((x0 - x_left) / px_m) - 1, int((x1 - x_left) / px_m) + 2):
            for iy in range(int((y_top - y1) / px_m) - 1, int((y_top - y0) / px_m) + 2):
                if 0 <= ix < size_px and 0 <= iy < size_px:
                    pixels.add((ix, iy))
    sub = (1 / 6, 3 / 6, 5 / 6)
    for ix, iy in pixels:
        n = sum(dark(x_left + (ix + u) * px_m, y_top - (iy + v) * px_m) for u in sub for v in sub)
        if n:
            g = int(round(255 * (1.0 - n / 9.0)))
            k = (iy * size_px + ix) * 3
            rgb[k:k + 3] = bytes((g, g, g))
    write_rgb_png(out, size_px, size_px, bytes(rgb))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output", type=Path, default=WEBOTS / "worlds" / "textures" / "godot_course.png")
    ap.add_argument("--size", type=int, default=4096, help="pixels par cote (4096 sur 3,4 m = 0,83 mm/pixel)")
    ap.add_argument("--floor-size", type=float, default=3.4)
    ap.add_argument("--center-x", type=float, default=0.15)
    ap.add_argument("--center-y", type=float, default=0.23)
    ap.add_argument("--tape-width", type=float, default=0.018, help="largeur du ruban (m) ; Godot : 0.020")
    a = ap.parse_args()
    out = generate(a.output, a.size, a.floor_size, a.center_x, a.center_y, a.tape_width)
    print(f"Texture du circuit : {out} ({a.size} px, ruban {a.tape_width * 1000:.0f} mm)")


if __name__ == "__main__":
    main()
