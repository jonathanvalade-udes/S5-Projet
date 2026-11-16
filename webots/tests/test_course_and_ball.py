"""Circuit de Godot, support de bille et suivi de la bille physique (sans Webots)."""

import json
import math
import re
import struct
import sys
import unittest
import zlib
from pathlib import Path

WEBOTS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WEBOTS / "controllers" / "picar_s_controller"))
sys.path.insert(0, str(WEBOTS / "tools"))

from ball_tracker import BALL_RADIUS, CUP_RIM_RADIUS, CUP_X, CUP_Z, REST_Z, ball_in_cup_frame, is_dropped  # noqa: E402

IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1]
COURSE = WEBOTS / "worlds" / "picar_s_course.wbt"
PROTO = WEBOTS / "protos" / "PiCarS.proto"


def read_png_gray(path: Path):
    data = path.read_bytes()
    pos, idat, width = 8, b"", 0
    while pos < len(data):
        length = struct.unpack(">I", data[pos:pos + 4])[0]
        kind, chunk = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
        if kind == b"IHDR":
            width = struct.unpack(">I", chunk[:4])[0]
        elif kind == b"IDAT":
            idat += chunk
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = 1 + width * 3
    return width, lambda ix, iy: raw[iy * stride + 1 + ix * 3]


class TestBallTracker(unittest.TestCase):
    def test_rest_position_is_origin(self):
        ball = [CUP_X, 0.0, CUP_Z + REST_Z]
        self.assertEqual(tuple(round(v, 9) for v in ball_in_cup_frame([0, 0, 0], IDENTITY, ball)), (0.0, 0.0, 0.0))

    def test_rotation_of_car_is_removed(self):
        # voiture tournee de 90 deg (cap +y) : la coupelle est en (0, CUP_X) dans le monde
        rot90 = [0, -1, 0, 1, 0, 0, 0, 0, 1]
        rel = ball_in_cup_frame([1.0, 2.0, 0.0], rot90, [1.0, 2.0 + CUP_X + 0.005, CUP_Z + REST_Z])
        self.assertAlmostEqual(rel[0], 0.005, places=9)   # 5 mm vers l'avant de la voiture
        self.assertAlmostEqual(rel[1], 0.0, places=9)

    def test_dropped_beyond_rim_or_below(self):
        self.assertFalse(is_dropped((0.015, 0.0, 0.0)))
        self.assertTrue(is_dropped((CUP_RIM_RADIUS + 0.003, 0.0, 0.0)))
        self.assertTrue(is_dropped((0.0, 0.0, -0.006)))

    def test_ball_dimensions_match_godot(self):
        self.assertEqual(BALL_RADIUS, 0.010)   # main.tscn : CSGSphere radius 0.4 x scale 0.025


class TestCupPlate(unittest.TestCase):
    def test_mesh_is_godot_bowl(self):
        """Plaquette.obj : 50 x 50 mm, fond de coupelle a 2.5 mm, rebord a 4.0 mm a r = 20 mm (bol, pas cone)."""
        verts = [tuple(map(float, l.split()[1:])) for l in (WEBOTS / "cad" / "Plaquette.obj").read_text().splitlines()
                 if l.startswith("v ")]
        self.assertEqual(len(verts), 1530)
        self.assertAlmostEqual(max(v[0] for v in verts) - min(v[0] for v in verts), 0.05, places=4)
        center = min((v for v in verts if math.hypot(v[0], v[1]) < 1e-4), key=lambda v: v[2])
        self.assertAlmostEqual(center[2], 0.0025, places=5)
        mid = [v[2] for v in verts if abs(math.hypot(v[0], v[1]) - 0.0106) < 3e-4]
        # bol spherique de ~133 mm : z(10.6 mm) = 2.5 + 10.6^2 / 266 = 2.92 mm (un cone donnerait 3.30 mm)
        self.assertAlmostEqual(min(mid), 0.00294, places=4)

    def test_proto_has_physical_cup_plate(self):
        text = PROTO.read_text()
        block = text[text.index("DEF CUP_PLATE Solid"):]
        self.assertIn("translation 0.048 0 0.0735", block)
        self.assertIn('contactMaterial "pla"', block)
        self.assertIn("boundingObject USE CUP_MESH", block)


class TestCourseWorld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = COURSE.read_text()

    def test_ball_glass_20mm_10g(self):
        block = self.text[self.text.index("DEF BALL Solid"):]
        self.assertIn("radius 0.01", block)
        self.assertIn("mass 0.01", block)
        self.assertIn('contactMaterial "glass"', block)

    def test_robot_is_supervisor_for_ball(self):
        self.assertRegex(self.text, r"PiCarS \{[^}]*supervisor TRUE")

    def test_two_obstacles_from_godot(self):
        self.assertEqual(len(re.findall(r"DEF OBSTACLE_\d Solid", self.text)), 2)

    def test_glass_pla_contact(self):
        self.assertIn('material1 "glass"', self.text)
        self.assertIn('material2 "pla"', self.text)

    def test_course_json_and_finish_bar(self):
        tris = json.loads((WEBOTS / "worlds" / "godot_course.json").read_text())["triangles"]
        self.assertEqual(len(tris), 954)
        bar = [p for t in tris for p in t if p[0] > 1.1 and p[1] < 0.3]
        self.assertGreater(max(p[0] for p in bar) - min(p[0] for p in bar), 0.2)   # barre du T de 220 mm

    def test_texture_tape_18mm_and_start_on_tape(self):
        size, pixel = read_png_gray(WEBOTS / "worlds" / "textures" / "godot_course.png")
        px = 3.4 / size
        to_px = lambda x, y: (int((x - (0.15 - 1.7)) / px), int(((0.23 + 1.7) - y) / px))
        # ruban de 18 mm sur la ligne de depart (y = -1.0), a x = -0.5
        ix, _ = to_px(-0.5, 0.0)
        dark = [iy for iy in range(size) if pixel(ix, iy) < 128 and abs(((0.23 + 1.7) - iy * px) + 1.0) < 0.05]
        self.assertAlmostEqual(len(dark) * px * 1000, 18.0, delta=1.7)
        # barre de capteurs au depart (essieu arriere a x = -1.06, capteurs 0.17 devant) : sur le ruban
        self.assertLess(pixel(*to_px(-1.06 + 0.17, -1.0)), 64)
        # arrivee : la barre du T est noire
        self.assertLess(pixel(*to_px(1.21, 0.203)), 64)

    def test_generator_small(self):
        from generate_course_texture import generate
        out = WEBOTS / "worlds" / "textures" / "_test_course.png"
        try:
            generate(out, 256, 3.4, 0.15, 0.23, 0.018)
            size, _ = read_png_gray(out)
            self.assertEqual(size, 256)
        finally:
            out.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
