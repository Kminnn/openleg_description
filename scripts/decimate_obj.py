#!/usr/bin/env python3
# Decimate every mesh in an OBJ file with Blender.
# Run with Blender, not the system Python:
# blender --background --python scripts/decimate_obj.py -- input.obj output.obj --ratio 0.05

import argparse
import os
import sys

import bpy


def parse_arguments():
    try:
        separator = sys.argv.index("--")
    except ValueError:
        separator = len(sys.argv)

    parser = argparse.ArgumentParser(
        description="Decimate every mesh object in an OBJ and export a visual OBJ."
    )
    parser.add_argument("input_obj", help="Source OBJ file; it is never modified.")
    parser.add_argument("output_obj", help="Path for the decimated visual OBJ.")
    parser.add_argument(
        "--ratio",
        type=float,
        default=0.05,
        help="Fraction of faces to retain per mesh, from 0.01 to 1.0 (default: 0.05).",
    )
    args = parser.parse_args(sys.argv[separator + 1 :])
    if not 0.01 <= args.ratio <= 1.0:
        parser.error("--ratio must be between 0.01 and 1.0")
    return args


def import_obj(filepath):
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=filepath)
    else:
        bpy.ops.import_scene.obj(filepath=filepath)


def export_obj(filepath):
    if hasattr(bpy.ops.wm, "obj_export"):
        bpy.ops.wm.obj_export(filepath=filepath, export_materials=True)
    else:
        bpy.ops.export_scene.obj(filepath=filepath, use_materials=True)


def main():
    args = parse_arguments()
    input_obj = os.path.abspath(args.input_obj)
    output_obj = os.path.abspath(args.output_obj)

    if not os.path.isfile(input_obj):
        raise SystemExit(f"Input OBJ does not exist: {input_obj}")
    if input_obj == output_obj:
        raise SystemExit("Output OBJ must be different from the input OBJ.")

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    import_obj(input_obj)

    meshes = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
    if not meshes:
        raise SystemExit(f"No mesh objects found in: {input_obj}")

    original_faces = sum(len(obj.data.polygons) for obj in meshes)
    for obj in meshes:
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        modifier = obj.modifiers.new(name="RViz_Decimate", type="DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = args.ratio
        bpy.ops.object.modifier_apply(modifier=modifier.name)

    output_dir = os.path.dirname(output_obj)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    export_obj(output_obj)

    final_faces = sum(len(obj.data.polygons) for obj in meshes)
    print(
        f"Decimated {len(meshes)} mesh object(s): "
        f"{original_faces:,} -> {final_faces:,} faces ({args.ratio:.0%} requested ratio)."
    )
    print(f"Wrote visual OBJ: {output_obj}")


if __name__ == "__main__":
    main()
