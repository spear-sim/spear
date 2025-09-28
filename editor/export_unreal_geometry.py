#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import pathlib
import spear
import spear.utils.editor_utils
import trimesh
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--export_dir", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)

editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()


def process_scene():
    spear.log("Processing scene: ", editor_world_name)

    actors = spear.utils.editor_utils.find_actors()
    actors = [ actor for actor in actors if actor.get_editor_property("relevant_for_level_bounds") ]

    for actor in actors:
        spear.log("    Processing actor: ", spear.utils.editor_utils.get_stable_name_for_actor(actor))
        generate_unreal_geometry(actor)

    spear.log("Done.")

def generate_unreal_geometry(actor):

    static_mesh_components  = spear.utils.editor_utils.get_components(actor=actor, component_class=unreal.StaticMeshComponent)
    static_meshes           = [ static_mesh_component.get_editor_property("static_mesh") for static_mesh_component in static_mesh_components ]
    static_meshes           = [ static_mesh for static_mesh in static_meshes if static_mesh is not None ]
    static_mesh_asset_paths = [ pathlib.PurePosixPath(static_mesh.get_path_name()) for static_mesh in static_meshes ]

    for static_mesh_asset_path in static_mesh_asset_paths:
        spear.log("        Exporting asset: ", static_mesh_asset_path)

        # Export OBJ file. Note that Unreal swaps the y and z coordinates during export, so that the exported
        # mesh achieves visual parity with most third-party viewers, despite the unusual handedness conventions
        # in Unreal. However in our setting, we would prefer numerical parity over visual parity, so we reload
        # the mesh as soon as we're finished exporting from Unreal, and we swap the y and z coordinates back to
        # what they were originally.

        obj_path_suffix = f"{os.path.join(*static_mesh_asset_path.parts[1:])}.obj"
        raw_obj_path = os.path.realpath(os.path.join(args.export_dir, "scenes", editor_world_name, "unreal_geometry", "raw", obj_path_suffix))

        errors = []
        asset_export_task = unreal.AssetExportTask()
        asset_export_task.set_editor_property("automated", True)
        asset_export_task.set_editor_property("errors", errors)
        asset_export_task.set_editor_property("exporter", None)            
        asset_export_task.set_editor_property("filename", raw_obj_path)
        asset_export_task.set_editor_property("ignore_object_list", [])
        asset_export_task.set_editor_property("object", asset_registry.get_asset_by_object_path(str(static_mesh_asset_path)).get_asset())
        asset_export_task.set_editor_property("options", None)
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

        # Swap y and z coordinates in the mesh that was exported by Unreal. Note that we also need to swap
        # the winding order of the triangle faces so the meshes still render nicely in MeshLab. We set the
        # visual attribute to a default value to prevent exporting material info.
        mesh = trimesh.load_mesh(raw_obj_path, process=False, validate=False)
        mesh.vertices = mesh.vertices[:,[0,2,1]]
        mesh.faces = mesh.faces[:,[0,2,1]]
        mesh.visual = trimesh.visual.ColorVisuals()

        obj_dir_suffix = os.path.join(*static_mesh_asset_path.parts[1:-1])
        numerical_parity_obj_dir = os.path.realpath(os.path.join(args.export_dir, "scenes", editor_world_name, "unreal_geometry", "numerical_parity", obj_dir_suffix))
        numerical_parity_obj_path = os.path.realpath(os.path.join(args.export_dir, "scenes", editor_world_name, "unreal_geometry", "numerical_parity", obj_path_suffix))
        os.makedirs(numerical_parity_obj_dir, exist_ok=True)
        with open(numerical_parity_obj_path, "w"):
            mesh.export(numerical_parity_obj_path, "obj")


if __name__ == "__main__":
    process_scene()
