import os
import unittest
from pathlib import Path

EXPECTED_FILES = [
    "Chassis.dae",
    "WheelFrontLeft.dae",
    "WheelFrontRight.dae",
    "WheelRearLeft.dae",
    "WheelRearRight.dae",
]

CAD_DIR = Path(__file__).resolve().parent.parent / "cad"


class TestCadAssets(unittest.TestCase):
    def test_cad_directory_exists(self):
        self.assertTrue(CAD_DIR.is_dir(), f"CAD directory does not exist: {CAD_DIR}")

    def test_all_cad_files_exist_and_valid(self):
        for filename in EXPECTED_FILES:
            with self.subTest(file=filename):
                file_path = CAD_DIR / filename
                self.assertTrue(file_path.is_file(), f"Asset {filename} is missing at {file_path}")
                size = file_path.stat().st_size
                self.assertGreater(size, 1024, f"Asset {filename} is too small: {size} bytes (expected > 1024)")
                with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                    content = f.read(2048)
                self.assertIn("<COLLADA", content, f"Asset {filename} does not contain <COLLADA XML tag")


if __name__ == "__main__":
    unittest.main()
