#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import pathlib
import unreal
import spear
import spear.unreal
import trimesh
import open3d as o3d
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", default=r"F:\intel\interiorsim\pipeline")
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()

pipeline_config = json.load(open(r"F:\intel\interiorsim\pipeline\config.json", mode='r'))

default_values = {
    'convex_decomposition_strategy': "coacd",
    "body_type": "static"
}


def get_config_value(actor_name, key):
    actor_name_short = actor_name.split("/")[-1]
    actor_type = actor_name.split("/")[1]
    actor_type = actor_type[actor_type.find("_") + 1:]
    if actor_name_short in pipeline_config['actors']:
        actor_config = pipeline_config['actors'][actor_name_short]
        if key in actor_config:
            return actor_config[key]
    if actor_type in pipeline_config['semantics']:
        actor_config = pipeline_config['semantics'][actor_type]
        if key in actor_config:
            return actor_config[key]
    return default_values[key]


def process_scene():
    spear.log("Processing scene: ", editor_world_name)

    actors = spear.unreal.find_actors()
    actors = [actor for actor in actors if actor.get_editor_property("root_component") is not None]

    for actor in actors:
        # TODO
        actor_stable_name = spear.unreal.get_stable_actor_name(actor)
        convex_decomposition_strategy = get_config_value(actor_stable_name, "convex_decomposition_strategy")
        if convex_decomposition_strategy == "unreal":
            spear.log("    Processing actor: ", actor_stable_name)
            generate_unreal_collision(actor)
    spear.log("Done.")


def generate_unreal_collision(actor):
    static_mesh_components = spear.unreal.find_components(actor, unreal.StaticMeshComponent)
    static_meshes = [static_mesh_component.get_editor_property("static_mesh") for static_mesh_component in static_mesh_components]
    static_meshes = [static_mesh for static_mesh in static_meshes if static_mesh is not None]
    static_mesh_asset_paths = [pathlib.PurePosixPath(static_mesh.get_path_name()) for static_mesh in static_meshes]
    static_mesh_asset_paths = [path for path in static_mesh_asset_paths if path.parts[:4] == ("/", "Game", "Scenes", editor_world_name)]

    for static_mesh_asset_path in static_mesh_asset_paths:
        spear.log("        Exporting asset: ", static_mesh_asset_path)

        # Export OBJ file. Note that Unreal swaps the y and z coordinates during export, so that the exported
        # mesh achieves visual parity with most third-party viewers, despite the unusual handedness conventions
        # in Unreal. However in our setting, we would prefer numerical parity over visual parity, so we reload
        # the mesh as soon as we're finished exporting from Unreal, and we swap the y and z coordinates back to
        # what they were originally.

        fbx_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".fbx"
        obj_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".obj"
        raw_fbx_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "collision", fbx_path_suffix))
        os.makedirs(os.path.dirname(raw_fbx_path), exist_ok=True)

        exporter = unreal.StaticMeshExporterFBX()
        option = unreal.FbxExportOption()
        option.set_editor_property("collision", True)
        errors = []
        asset_export_task = unreal.AssetExportTask()
        asset_export_task.set_editor_property("automated", True)
        asset_export_task.set_editor_property("errors", errors)
        asset_export_task.set_editor_property("exporter", exporter)
        asset_export_task.set_editor_property("filename", raw_fbx_path)
        asset_export_task.set_editor_property("ignore_object_list", [])
        asset_export_task.set_editor_property("object", asset_registry.get_asset_by_object_path(str(static_mesh_asset_path)).get_asset())
        asset_export_task.set_editor_property("options", option)
        asset_export_task.set_editor_property("prompt", False)
        asset_export_task.set_editor_property("replace_identical", True)
        asset_export_task.set_editor_property("selected", False)
        asset_export_task.set_editor_property("use_file_archive", False)
        asset_export_task.set_editor_property("write_empty_files", False)

        spear.log("            automated:          ", asset_export_task.get_editor_property("automated"))
        spear.log("            errors:             ", asset_export_task.get_editor_property("errors"))
        spear.log("            exporter:           ", asset_export_task.get_editor_property("exporter"))
        spear.log("            filename:           ", asset_export_task.get_editor_property("filename"))
        spear.log("            ignore_object_list: ", asset_export_task.get_editor_property("ignore_object_list"))
        spear.log("            object:             ", asset_export_task.get_editor_property("object"))
        spear.log("            options:            ", asset_export_task.get_editor_property("options"))
        spear.log("            prompt:             ", asset_export_task.get_editor_property("prompt"))
        spear.log("            replace_identical:  ", asset_export_task.get_editor_property("replace_identical"))
        spear.log("            selected:           ", asset_export_task.get_editor_property("selected"))
        spear.log("            use_file_archive:   ", asset_export_task.get_editor_property("use_file_archive"))
        spear.log("            write_empty_files:  ", asset_export_task.get_editor_property("write_empty_files"))

        unreal.Exporter.run_asset_export_task(asset_export_task)
        if len(errors) > 0:
            spear.log(errors)
            assert False
        fbx_mesh = o3d.io.read_triangle_mesh(raw_fbx_path)
        collision_obj_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "collision", obj_path_suffix))

        triangle_material_ids = np.asarray(fbx_mesh.triangle_material_ids)
        print("len(triangle_material_ids)",static_mesh_asset_path,triangle_material_ids.max())
        triangles_to_remove = triangle_material_ids == 0
        fbx_mesh.remove_triangles_by_mask(triangles_to_remove)
        o3d.io.write_triangle_mesh(collision_obj_path, fbx_mesh)

        # Swap y and z coordinates in the mesh that was exported by Unreal. Note that we also need to swap
        # the winding order of the triangle faces so the meshes still render nicely in MeshLab. We set the
        # visual attribute to a default value to prevent exporting material info.
        mesh = trimesh.load_mesh(collision_obj_path, process=False, validate=False)
        # mesh.vertices = mesh.vertices[:,[0,2,1]]
        mesh.vertices[:, 1] *= -1
        # mesh.faces = mesh.faces[:,[0,2,1]]
        mesh.visual = trimesh.visual.ColorVisuals()

        obj_dir_suffix = os.path.join(*static_mesh_asset_path.parts[4:-1])
        numerical_parity_obj_dir = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "numerical_parity_collision", obj_dir_suffix))
        numerical_parity_obj_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "numerical_parity_collision", obj_path_suffix))
        os.makedirs(numerical_parity_obj_dir, exist_ok=True)
        with open(numerical_parity_obj_path, "w"):
            mesh.export(numerical_parity_obj_path, "obj")


if __name__ == "__main__":
    process_scene()
