"""Create a primitive Japanese distribution pole asset set.

Run with Blender:

  blender --background --python tools/create_japan_distribution_pole_primitive.py -- ^
    --out-dir assets/generated/japan_distribution_pole_primitive_v0

The default output is an editable .blend. Use --export-glb only when a quick
preview/export artifact is useful. Dimensions are intentionally approximate:
this is a starting mesh for later hand editing, not a legal or manufacturer-
accurate standard.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import bpy


POLE = {
    "total_length_m": 12.0,
    "visible_height_m": 10.0,
    "top_diameter_m": 0.190,
    "taper_ratio": 75.0,
    "render_sides": 16,
}

BANDS = {
    "ground_wire_z_m": 10.2,
    "hv_z_m": 9.2,
    "lv_z_m": 7.4,
    "equipment_z_min_m": 5.8,
    "equipment_z_max_m": 7.3,
    "communication_z_min_m": 4.8,
    "communication_z_max_m": 5.8,
}

HV_X = [-0.75, 0.0, 0.75]
LV_X = [-0.45, 0.0, 0.45]
COMM_X = [-0.55, -0.18, 0.18, 0.55]


def bottom_diameter() -> float:
    return POLE["top_diameter_m"] + POLE["total_length_m"] / POLE["taper_ratio"]


def ground_diameter() -> float:
    return POLE["top_diameter_m"] + POLE["visible_height_m"] / POLE["taper_ratio"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default="assets/generated/japan_distribution_pole_primitive_v0",
        help="directory for .blend and .glb outputs",
    )
    parser.add_argument("--name", default="japan_distribution_pole_primitive_v0")
    parser.add_argument("--export-glb", action="store_true")
    script_args = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    return parser.parse_args(script_args)


def clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)


def material(name: str, color: tuple[float, float, float, float]) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    return mat


def assign(obj: bpy.types.Object, mat: bpy.types.Material) -> bpy.types.Object:
    obj.data.materials.append(mat)
    return obj


def add_cube(
    name: str,
    size: tuple[float, float, float],
    location: tuple[float, float, float],
    mat: bpy.types.Material,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return assign(obj, mat)


def add_cylinder(
    name: str,
    radius: float,
    depth: float,
    location: tuple[float, float, float],
    mat: bpy.types.Material,
    vertices: int = 24,
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=location,
        rotation=rotation,
    )
    obj = bpy.context.object
    obj.name = name
    return assign(obj, mat)


def add_marker(
    name: str,
    location: tuple[float, float, float],
    role: str,
    size: float = 0.18,
) -> bpy.types.Object:
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=location)
    obj = bpy.context.object
    obj.name = name
    obj.empty_display_size = size
    obj["wire_role"] = role
    obj["wire_coordinate_space"] = "pole_local_z_up"
    obj["z_m"] = location[2]
    return obj


def add_insulator_stack(
    name: str,
    x: float,
    y: float,
    base_z: float,
    radius: float,
    height: float,
    discs: int,
    mat: bpy.types.Material,
) -> None:
    stem = add_cylinder(
        f"{name}_stem",
        radius * 0.38,
        height,
        (x, y, base_z + height * 0.5),
        mat,
        vertices=16,
    )
    stem["wire_role"] = "insulator_stem"
    spacing = height / max(1, discs)
    for index in range(discs):
        z = base_z + spacing * (index + 0.5)
        disc = add_cylinder(
            f"{name}_disc_{index + 1}",
            radius,
            spacing * 0.32,
            (x, y, z),
            mat,
            vertices=20,
        )
        disc["wire_role"] = "insulator_disc"
    add_marker(f"socket_{name}_top", (x, y, base_z + height), "line_socket", 0.10)


def build_scene(asset_name: str) -> None:
    clear_scene()

    mats = {
        "concrete": material("mat_concrete_warm_gray", (0.55, 0.54, 0.50, 1.0)),
        "dark_metal": material("mat_dark_galvanized_metal", (0.12, 0.13, 0.13, 1.0)),
        "ceramic": material("mat_ceramic_off_white", (0.86, 0.84, 0.76, 1.0)),
        "transformer": material("mat_transformer_muted_green", (0.18, 0.29, 0.23, 1.0)),
        "rubber": material("mat_cable_black", (0.02, 0.02, 0.018, 1.0)),
        "marker": material("mat_marker_blue", (0.1, 0.45, 0.9, 1.0)),
    }

    root = bpy.data.collections.new(asset_name)
    bpy.context.scene.collection.children.link(root)

    buried = POLE["total_length_m"] - POLE["visible_height_m"]
    pole_center_z = (POLE["visible_height_m"] - buried) * 0.5
    bpy.ops.mesh.primitive_cone_add(
        vertices=int(POLE["render_sides"]),
        radius1=bottom_diameter() * 0.5,
        radius2=POLE["top_diameter_m"] * 0.5,
        depth=POLE["total_length_m"],
        location=(0.0, 0.0, pole_center_z),
    )
    pole = bpy.context.object
    pole.name = "pole_body_tapered_12m_visible10m"
    pole["wire_role"] = "pole_body"
    pole["total_length_m"] = POLE["total_length_m"]
    pole["visible_height_m"] = POLE["visible_height_m"]
    pole["buried_length_m"] = buried
    pole["top_diameter_m"] = POLE["top_diameter_m"]
    pole["ground_diameter_m"] = ground_diameter()
    pole["bottom_diameter_m"] = bottom_diameter()
    pole["taper_ratio"] = POLE["taper_ratio"]
    assign(pole, mats["concrete"])

    hv_z = BANDS["hv_z_m"]
    lv_z = BANDS["lv_z_m"]
    add_cube("crossarm_hv_1p7m", (1.7, 0.08, 0.08), (0.0, 0.0, hv_z), mats["dark_metal"])[
        "wire_role"
    ] = "crossarm_hv"
    add_cube("crossarm_lv_1p2m", (1.2, 0.06, 0.06), (0.0, 0.0, lv_z), mats["dark_metal"])[
        "wire_role"
    ] = "crossarm_lv"

    for index, x in enumerate(HV_X, start=1):
        add_insulator_stack(
            f"hv_phase_{index}",
            x,
            0.0,
            hv_z + 0.04,
            radius=0.06,
            height=0.18,
            discs=3,
            mat=mats["ceramic"],
        )
        add_marker(f"band_hv_phase_{index}", (x, 0.0, hv_z), "hv_attachment", 0.12)

    for index, x in enumerate(LV_X, start=1):
        add_cylinder(
            f"lv_spool_{index}",
            0.04,
            0.10,
            (x, 0.0, lv_z + 0.06),
            mats["ceramic"],
            vertices=16,
            rotation=(math.pi / 2, 0.0, 0.0),
        )["wire_role"] = "lv_spool_insulator"
        add_marker(f"band_lv_phase_{index}", (x, 0.0, lv_z), "lv_attachment", 0.12)

    transformer = add_cylinder(
        "transformer_cylinder_primitive",
        0.275,
        0.85,
        (0.0, -0.45, 6.2),
        mats["transformer"],
        vertices=32,
    )
    transformer["wire_role"] = "transformer"
    add_cube("transformer_mount_band", (0.72, 0.04, 0.08), (0.0, -0.18, 6.2), mats["dark_metal"])[
        "wire_role"
    ] = "equipment_band"

    add_cylinder(
        "cutout_tilted_bar_primitive",
        0.04,
        0.55,
        (0.38, -0.28, 7.8),
        mats["ceramic"],
        vertices=16,
        rotation=(0.45, 0.0, 0.0),
    )["wire_role"] = "cutout"
    add_insulator_stack(
        "arrester",
        -0.38,
        -0.28,
        7.72,
        radius=0.035,
        height=0.35,
        discs=4,
        mat=mats["ceramic"],
    )

    comm_z = (BANDS["communication_z_min_m"] + BANDS["communication_z_max_m"]) * 0.5
    for index, x in enumerate(COMM_X, start=1):
        add_cube(
            f"communication_clamp_{index}",
            (0.08, 0.05, 0.06),
            (x, -0.13, comm_z),
            mats["dark_metal"],
        )["wire_role"] = "communication_clamp"
        add_marker(f"band_comm_{index}", (x, -0.17, comm_z), "communication_attachment", 0.10)

    add_marker("ground_plane_origin", (0.0, 0.0, 0.0), "ground_reference", 0.25)
    add_marker("pole_top", (0.0, 0.0, POLE["visible_height_m"]), "pole_top", 0.20)
    add_marker("band_ground_wire", (0.0, 0.0, BANDS["ground_wire_z_m"]), "ground_wire", 0.14)
    add_marker("band_equipment_min", (0.0, -0.24, BANDS["equipment_z_min_m"]), "equipment_band_min", 0.12)
    add_marker("band_equipment_max", (0.0, -0.24, BANDS["equipment_z_max_m"]), "equipment_band_max", 0.12)
    add_marker("band_communication_min", (0.0, -0.24, BANDS["communication_z_min_m"]), "communication_band_min", 0.12)
    add_marker("band_communication_max", (0.0, -0.24, BANDS["communication_z_max_m"]), "communication_band_max", 0.12)

    for obj in bpy.context.scene.objects:
        if obj.name not in root.objects:
            try:
                root.objects.link(obj)
            except RuntimeError:
                pass
        obj["asset_family"] = asset_name

    bpy.context.scene.unit_settings.system = "METRIC"
    bpy.context.scene.unit_settings.scale_length = 1.0
    bpy.context.scene["wire_asset_name"] = asset_name
    bpy.context.scene["wire_coordinate_space"] = "pole_local_z_up"
    bpy.context.scene["wire_attachment_basis"] = "ground_or_pole_local_height_not_mesh_bottom"


def export_outputs(out_dir: Path, asset_name: str, export_glb: bool) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out_dir / f"{asset_name}.blend"))
    if export_glb:
        bpy.ops.export_scene.gltf(
            filepath=str(out_dir / f"{asset_name}.glb"),
            export_format="GLB",
            export_extras=True,
            export_yup=False,
        )


def main() -> None:
    args = parse_args()
    build_scene(args.name)
    export_outputs(Path(args.out_dir), args.name, args.export_glb)


if __name__ == "__main__":
    main()
