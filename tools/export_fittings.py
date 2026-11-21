"""Export the edited Fittings collection without generating or placing parts.

Run with Blender, from the repository root:
  blender --background --factory-startup assets/pole_fittings.blend --python tools/export_fittings.py
Use -- --output path/to/file.glb for a separate verification export.
"""

import argparse
import sys
from pathlib import Path

import bpy


def export_fittings(output):
    collection = bpy.data.collections.get("Fittings")
    if collection is None or not collection.all_objects:
        raise ValueError("Fittings collection is missing or empty")
    objects = set(collection.all_objects)
    roots = [obj for obj in objects if obj.parent not in objects]
    if not roots or any(obj.type != "EMPTY" or not obj.name.endswith("_ROOT") for obj in roots):
        raise ValueError("Each pattern must have one top-level *_ROOT Empty")
    bpy.ops.object.select_all(action="DESELECT")
    for obj in objects:
        obj.select_set(True)
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(output),
        export_format="GLB",
        use_selection=True,
        export_yup=True,
        export_extras=True,
        export_apply=False,
        export_cameras=False,
        export_lights=False,
    )


if __name__ == "__main__":
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=repo / "web/src/assets/pole_fittings.glb")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    export_fittings(args.output)
