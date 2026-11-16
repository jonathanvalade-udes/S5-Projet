"""Exporte depuis les fichiers Blender de Godot ce dont Webots a besoin (a lancer avec Blender, sans interface) :

    blender -b godot/assets/plaquette_blender_scale.blend --python webots/tools/export_godot_assets.py -- plaquette
    blender -b godot/assets/parcours.blend               --python webots/tools/export_godot_assets.py -- parcours

- plaquette : support de la bille (50 x 50 x 4 mm, coupelle en bol) -> webots/cad/Plaquette.obj (metres, z vers le haut)
- parcours  : faces du dessus du ruban du circuit -> webots/worlds/godot_course.json (triangles 2D, metres)

Reperes : Blender (z vers le haut) = Webots ENU. Godot (y vers le haut) importe le .blend avec x_godot = x_blender,
z_godot = -y_blender : le circuit Webots est donc exactement celui de Godot, vu du dessus.
"""

import json
import sys
from pathlib import Path

import bpy  # type: ignore

WEBOTS = Path(__file__).resolve().parent.parent


def export_plaquette() -> None:
    obj = bpy.data.objects["PiCarFrame3_V2b-col"]
    mesh = obj.data
    mesh.calc_loop_triangles()
    out = WEBOTS / "cad" / "Plaquette.obj"
    with out.open("w") as fh:
        fh.write("# Support de bille PiCar (godot/assets/plaquette_blender_scale.blend), metres, z vers le haut\n")
        fh.write("o Plaquette\n")
        for v in mesh.vertices:
            co = obj.matrix_world @ v.co
            fh.write(f"v {co.x:.6f} {co.y:.6f} {co.z:.6f}\n")
        for tri in mesh.loop_triangles:
            a, b, c = (i + 1 for i in tri.vertices)
            fh.write(f"f {a} {b} {c}\n")
    print(f"EXPORT {out} : {len(mesh.vertices)} sommets, {len(mesh.loop_triangles)} triangles")


def export_parcours() -> None:
    obj = bpy.data.objects["Parcours Picar-col"]
    mesh = obj.data
    mesh.calc_loop_triangles()
    top_z = max((obj.matrix_world @ v.co).z for v in mesh.vertices)
    tris = []
    for tri in mesh.loop_triangles:
        pts = [obj.matrix_world @ mesh.vertices[i].co for i in tri.vertices]
        if all(abs(p.z - top_z) < 1e-4 for p in pts):   # face du dessus du ruban
            tris.append([[round(p.x, 6), round(p.y, 6)] for p in pts])
    out = WEBOTS / "worlds" / "godot_course.json"
    out.write_text(json.dumps({
        "source": "godot/assets/parcours.blend (objet 'Parcours Picar-col')",
        "units": "m",
        "tape_width_godot_m": 0.020,
        "triangles": tris,
    }))
    print(f"EXPORT {out} : {len(tris)} triangles")


if __name__ == "__main__":
    what = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else ""
    {"plaquette": export_plaquette, "parcours": export_parcours}[what]()
