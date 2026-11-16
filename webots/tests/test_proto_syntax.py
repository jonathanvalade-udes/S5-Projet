import os
import re
import unittest
from pathlib import Path

PROTO_FILE = Path(__file__).resolve().parent.parent / "protos" / "PiCarS.proto"
CAD_DIR = Path(__file__).resolve().parent.parent / "cad"


class TestProtoSyntax(unittest.TestCase):
    """Validation test suite for PiCarS.proto syntax, structure, sensors, and kinematics."""

    @classmethod
    def setUpClass(cls):
        cls.proto_path = PROTO_FILE
        if cls.proto_path.is_file():
            with open(cls.proto_path, "r", encoding="utf-8") as f:
                cls.content = f.read()
        else:
            cls.content = ""

    def test_proto_file_exists(self):
        """PiCarS.proto must exist and be non-empty."""
        self.assertTrue(self.proto_path.is_file(), f"PROTO file does not exist at {self.proto_path}")
        self.assertGreater(len(self.content), 100, "PiCarS.proto is empty or too short")

    def test_vrml_header(self):
        """Header must declare #VRML_SIM with Webots R2023 or compatible version and utf8."""
        first_line = self.content.splitlines()[0].strip() if self.content else ""
        self.assertTrue(
            first_line.startswith("#VRML_SIM"),
            f"First line must start with #VRML_SIM, got: {first_line}"
        )
        self.assertIn("utf8", first_line.lower())
        self.assertTrue(
            re.search(r"R202[0-9][a-z]?", first_line),
            f"Header must contain a valid Webots release tag (e.g. R2023b), got: {first_line}"
        )

    def test_proto_declaration_and_fields(self):
        """PROTO must declare standard fields and default controller picar_s_controller."""
        self.assertIn("PROTO PiCarS [", self.content)
        self.assertIsNotNone(
            re.search(r'controller\s+"picar_s_controller"', self.content),
            "Default controller 'picar_s_controller' not declared in PROTO interface"
        )
        self.assertIn("translation", self.content)
        self.assertIn("rotation", self.content)
        self.assertIn("name", self.content)

    def test_bracket_and_brace_balance(self):
        """All curly braces and square brackets in PiCarS.proto must be balanced."""
        # Strip comments
        lines = [line.split("#")[0] for line in self.content.splitlines()]
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

        self.assertEqual(brace_count, 0, f"Unbalanced curly braces: count = {brace_count}")
        self.assertEqual(bracket_count, 0, f"Unbalanced square brackets: count = {bracket_count}")

    def test_cad_references(self):
        """All 5 Collada CAD assets must be referenced and exist on disk."""
        cad_files = [
            "Chassis.dae",
            "WheelFrontLeft.dae",
            "WheelFrontRight.dae",
            "WheelRearLeft.dae",
            "WheelRearRight.dae",
        ]
        for cad in cad_files:
            with self.subTest(cad=cad):
                self.assertIn(cad, self.content, f"Reference to {cad} missing in PiCarS.proto")
                actual_file = CAD_DIR / cad
                self.assertTrue(actual_file.is_file(), f"CAD file {cad} not found on disk at {actual_file}")

    def test_ir_sensors_configuration(self):
        """Must define IR_0 to IR_4 DistanceSensors of type infra-red with valid lookupTable."""
        for i in range(5):
            name = f"IR_{i}"
            with self.subTest(sensor=name):
                self.assertIn(f"DEF {name} DistanceSensor", self.content, f"Node DEF {name} DistanceSensor missing")
                pattern = rf"DEF {name} DistanceSensor\s*\{{(?:[^{{}}]*|\{{[^{{}}]*\}})*\}}"
                match = re.search(pattern, self.content)
                self.assertIsNotNone(match, f"Could not parse sensor block for {name}")
                block = match.group(0)
                self.assertIn('type "infra-red"', block, f"{name} must have type 'infra-red'")
                self.assertIn(f'name "{name}"', block, f"{name} must have name '{name}'")
                self.assertIn("950", block, f"{name} lookupTable must end at 950 (= black tape, DEFAULT_IR_MAX)")
                self.assertIn("100", block, f"{name} lookupTable must contain floor reading 100")

    def test_sonar_sensor_configuration(self):
        """Must define DEF SONAR DistanceSensor of type sonar."""
        self.assertTrue(
            "DEF SONAR DistanceSensor" in self.content or "DEF sonar DistanceSensor" in self.content,
            "DEF SONAR DistanceSensor node missing"
        )
        pattern = r"DEF (?:SONAR|sonar) DistanceSensor\s*\{(?:[^{}]*|\{[^{}]*\})*\}"
        match = re.search(pattern, self.content)
        self.assertIsNotNone(match, "Could not parse SONAR sensor block")
        block = match.group(0)
        self.assertIn('type "sonar"', block, "SONAR must have type 'sonar'")
        self.assertTrue(
            'name "sonar"' in block or 'name "SONAR"' in block,
            "SONAR must have name 'sonar' or 'SONAR'"
        )

    def test_hinge_joints_and_motors(self):
        """Ackermann : un pivot par roue avant ; differentiel : un moteur par roue arriere (comme les 2 moteurs TT reels)."""
        for name in ("left_steer", "right_steer", "rear_left_motor", "rear_right_motor"):
            with self.subTest(motor=name):
                self.assertIn(f'name "{name}"', self.content)
        self.assertNotIn("drive_motor", self.content, "essieu arriere rigide : les roues riperaient en virage")
        self.assertEqual(self.content.count("HingeJoint {"), 6, "2 pivots + 2 roues avant libres + 2 roues arriere")

    def test_wheel_geometry_matches_cad_and_fsm(self):
        """Empattement 0.14 m (= WHEELBASE_M du FSM), voie 0.13 m, rayon 33.5 mm (CAD : diametre 67 mm)."""
        self.assertIn("anchor 0.14 0.050 0.012", self.content)   # trous de pivot du CAD (y = ±50 mm)
        self.assertIn("anchor 0.14 -0.050 0.012", self.content)
        self.assertIn("anchor 0 0.065 0.012", self.content)
        self.assertIn("anchor 0 -0.065 0.012", self.content)
        self.assertEqual(self.content.count("radius 0.0335"), 4)

    def test_physics_and_mass(self):
        """Masse totale ~0.6 kg (kit ~0.5 kg + Pi + 2 x 18650), plus lourde a l'arriere."""
        masses = [float(m) for m in re.findall(r"\bmass\s+([0-9.]+)", self.content)]
        total = sum(masses)
        self.assertGreaterEqual(total, 0.55, f"Masse totale {total}")
        self.assertLessEqual(total, 0.75, f"Masse totale {total}")
        com = re.search(r"centerOfMass \[ ([0-9.]+) 0 ([0-9.]+) \]\n    \}\n  \}\n\}", self.content)
        self.assertIsNotNone(com, "centre de masse du chassis manquant")
        self.assertLess(float(com.group(1)), 0.07, "centre de masse vers l'essieu arriere (batteries, moteurs)")

    def test_tires_have_contact_material(self):
        self.assertEqual(self.content.count('contactMaterial "picar_tire"'), 4)

    def test_ir_sensors_below_chassis_and_parametric(self):
        """Capteurs sous le bas du chassis (z < 0) sinon les rayons infra-rouges voient le robot ; pas et tache parametriques."""
        for field in ("irPitch", "irSpotDiameter", "irNoise"):
            self.assertIn(f"field SFFloat    {field}", self.content)
        self.assertEqual(self.content.count("-0.0115\n        rotation 0 1 0 1.57079632679"), 5)   # 1 cm du sol
        self.assertEqual(self.content.count("numberOfRays 16"), 5)

    def test_bounding_objects(self):
        """Chassis : Box ; roues : Cylinder tournes autour de X (axe des roues selon Y)."""
        self.assertIn("Box", self.content)
        self.assertEqual(self.content.count("boundingObject Cylinder"), 4)
        self.assertEqual(self.content.count("rotation 1 0 0 -1.57079632679"), 4)

if __name__ == "__main__":
    unittest.main()
