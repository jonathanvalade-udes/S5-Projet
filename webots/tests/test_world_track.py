import os
import re
import struct
import subprocess
import sys
import unittest
import zlib
from pathlib import Path

WEBOTS_DIR = Path(__file__).resolve().parent.parent
WORLD_FILE = WEBOTS_DIR / "worlds" / "picar_s_track.wbt"
TEXTURE_FILE = WEBOTS_DIR / "worlds" / "textures" / "line_track.png"
GEN_SCRIPT = WEBOTS_DIR / "tools" / "generate_track_texture.py"


def read_png_dimensions(file_path: Path):
    """Parses PNG width and height from the IHDR chunk without external dependencies."""
    with open(file_path, "rb") as f:
        sig = f.read(8)
        if sig != b"\x89PNG\r\n\x1a\n":
            raise ValueError(f"Invalid PNG signature in {file_path}")
        length, chunk_type = struct.unpack(">I4s", f.read(8))
        if chunk_type != b"IHDR":
            raise ValueError(f"Expected IHDR chunk, got {chunk_type}")
        width, height = struct.unpack(">II", f.read(8))
        return width, height


class TestWorldTrack(unittest.TestCase):
    """Test suite validating the Webots simulation world and track texture."""

    def test_world_file_exists(self):
        """picar_s_track.wbt must exist and be non-empty."""
        self.assertTrue(WORLD_FILE.is_file(), f"World file does not exist at {WORLD_FILE}")
        size = WORLD_FILE.stat().st_size
        self.assertGreater(size, 200, f"World file is too small: {size} bytes")

    def test_vrml_header(self):
        """Header must declare #VRML_SIM with utf8 and a valid Webots version."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        with open(WORLD_FILE, "r", encoding="utf-8") as f:
            first_line = f.readline().strip()
        self.assertTrue(first_line.startswith("#VRML_SIM"), f"Header must start with #VRML_SIM: {first_line}")
        self.assertIn("utf8", first_line.lower())

    def test_world_info_node(self):
        """WorldInfo node must specify basicTimeStep 20 (50 Hz)."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        content = WORLD_FILE.read_text(encoding="utf-8")
        self.assertIn("WorldInfo {", content)
        match = re.search(r"basicTimeStep\s+(\d+)", content)
        self.assertIsNotNone(match, "basicTimeStep not defined in WorldInfo")
        # Physique fine (4 ms) ; le controleur garde 20 ms (PicarProtocol.CONTROL_STEP_MS), multiple du pas physique
        self.assertEqual(20 % int(match.group(1)), 0, "basicTimeStep doit diviser 20 ms")
        self.assertLessEqual(int(match.group(1)), 5, "pas physique trop grossier pour des roues de 25 g")
        self.assertIn('material2 "picar_tire"', content)
        self.assertRegex(content, r"bounce 0\b")

    def test_viewpoint_and_lighting_nodes(self):
        """Viewpoint, DirectionalLight, and TexturedBackground nodes must be present."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        content = WORLD_FILE.read_text(encoding="utf-8")
        self.assertIn("Viewpoint {", content, "Missing Viewpoint node")
        self.assertIn("DirectionalLight {", content, "Missing DirectionalLight node")
        self.assertIn("TexturedBackground {", content, "Missing TexturedBackground node")

    def test_floor_or_arena_with_texture(self):
        """Floor or RectangleArena node must reference line_track.png via ImageTexture."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        content = WORLD_FILE.read_text(encoding="utf-8")
        has_arena = "RectangleArena {" in content
        has_floor = "Floor {" in content
        self.assertTrue(has_arena or has_floor, "World must contain RectangleArena or Floor node")
        self.assertIn("ImageTexture {", content, "Floor/Arena must contain ImageTexture node")
        self.assertIn("line_track.png", content, "ImageTexture url must reference line_track.png")

    def test_picar_s_instance(self):
        """PiCarS robot instance must be present in the world."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        content = WORLD_FILE.read_text(encoding="utf-8")
        self.assertIn("PiCarS {", content, "PiCarS robot instance not present in world")

    def test_world_syntax_balanced_braces(self):
        """World file must have balanced braces and brackets."""
        if not WORLD_FILE.is_file():
            self.skipTest("World file does not exist")
        content = WORLD_FILE.read_text(encoding="utf-8")
        lines = [line.split("#")[0] for line in content.splitlines()]
        code = "\n".join(lines)
        brace_count = 0
        bracket_count = 0
        for char in code:
            if char == "{":
                brace_count += 1
            elif char == "}":
                brace_count -= 1
                self.assertGreaterEqual(brace_count, 0, "Unmatched closing brace '}'")
            elif char == "[":
                bracket_count += 1
            elif char == "]":
                bracket_count -= 1
                self.assertGreaterEqual(bracket_count, 0, "Unmatched closing bracket ']'")
        self.assertEqual(brace_count, 0, "Mismatched braces count in world file")
        self.assertEqual(bracket_count, 0, "Mismatched brackets count in world file")

    def test_texture_file_exists_and_valid_png(self):
        """line_track.png must exist, be a valid PNG, and have dimensions >= 512x512."""
        self.assertTrue(TEXTURE_FILE.is_file(), f"Texture file does not exist at {TEXTURE_FILE}")
        self.assertGreater(TEXTURE_FILE.stat().st_size, 1024, "Texture file too small")
        w, h = read_png_dimensions(TEXTURE_FILE)
        self.assertGreaterEqual(w, 512, f"Texture width {w} < 512")
        self.assertGreaterEqual(h, 512, f"Texture height {h} < 512")

    def test_tape_width_is_18mm(self):
        """Ruban isolant de 18 mm (PRD S2) : mesure sur la texture, au milieu de la ligne droite du bas."""
        import zlib
        data = TEXTURE_FILE.read_bytes()
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
        col = width // 2
        dark = sum(1 for row in range(width) if raw[row * stride + 1 + col * 3] < 128)
        tape_mm = dark / 2 * 3000.0 / width   # deux droites traversees par la colonne centrale ; sol de 3 m
        self.assertAlmostEqual(tape_mm, 18.0, delta=1.6)

    def test_texture_generator_script(self):
        """generate_track_texture.py must exist and successfully generate valid texture."""
        self.assertTrue(GEN_SCRIPT.is_file(), f"Generator script does not exist at {GEN_SCRIPT}")
        # Run generator in a subprocess with --output to a temporary path
        test_out = WEBOTS_DIR / "worlds" / "textures" / "_test_line_track.png"
        try:
            res = subprocess.run(
                [sys.executable, str(GEN_SCRIPT), "--output", str(test_out), "--size", "256"],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(res.returncode, 0, f"Generator script failed: {res.stderr}")
            self.assertTrue(test_out.is_file(), "Generated file does not exist")
            w, h = read_png_dimensions(test_out)
            self.assertEqual((w, h), (256, 256))
        finally:
            if test_out.is_file():
                test_out.unlink()


if __name__ == "__main__":
    unittest.main()
