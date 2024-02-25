import argparse
import os
import pathlib
import posixpath
import unreal
import spear
import trimesh


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)


def process_scene():

    editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()
    spear.log("Exporting Unreal scene geometry: ", editor_world_name)

    actors = [ actor for actor in spear.unreal.find_actors() if actor.get_editor_property("root_component") is not None ]

    # For each actor, export all static mesh components
    for actor in actors:

        spear.log("    Processing actor: ", spear.unreal.get_stable_name_actor(actor))

        static_mesh_components  = spear.unreal.find_components(actor, "StaticMeshComponent")
        static_meshes           = [ static_mesh_component.get_editor_property("static_mesh") for static_mesh_component in static_mesh_components ]
        static_meshes           = [ static_mesh for static_mesh in static_meshes if static_mesh is not None ]
        static_mesh_asset_paths = [ pathlib.PurePosixPath(static_mesh.get_path_name()) for static_mesh in static_meshes ]
        static_mesh_asset_paths = [ path for path in static_mesh_asset_paths if path.parts[:4] == ("/", "Game", "Scenes", editor_world_name) ]

        for static_mesh_asset_path in static_mesh_asset_paths:
            spear.log("        Exporting asset: ", static_mesh_asset_path)

            asset = asset_registry.get_asset_by_object_path(str(static_mesh_asset_path))
            obj_path_suffix = posixpath.join(*static_mesh_asset_path.parts[4:]) + ".obj"

            # Export OBJ file. Note that Unreal swaps the y and z coordinates during export, so that the exported
            # mesh achieves visual parity with most third-party viewers, despite the unusual handedness conventions
            # in Unreal. However in our setting, we would prefer numerical parity over visual parity, so we reload
            # the mesh as soon as we're finished exporting from Unreal, and we swap the y and z coordinates back to
            # what they were originally.
            visual_parity_obj_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "visual_parity", obj_path_suffix))

            errors = []
            asset_export_task = unreal.AssetExportTask()
            asset_export_task.set_editor_property("automated", True)
            asset_export_task.set_editor_property("errors", errors)
            asset_export_task.set_editor_property("exporter", None)            
            asset_export_task.set_editor_property("filename", visual_parity_obj_path)
            asset_export_task.set_editor_property("ignore_object_list", [])
            asset_export_task.set_editor_property("object", asset.get_asset())
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
            # the winding order of the triangle faces so the meshes still render nicely in MeshLab.
            obj_dir_suffix = posixpath.join(*static_mesh_asset_path.parts[4:-1])
            numerical_parity_obj_dir = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "numerical_parity", obj_dir_suffix))
            numerical_parity_obj_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", "numerical_parity", obj_path_suffix))

            mesh = trimesh.load_mesh(visual_parity_obj_path, process=False, validate=False)
            mesh.vertices = mesh.vertices[:,[0,2,1]]
            mesh.faces = mesh.faces[:,[0,2,1]]

            os.makedirs(numerical_parity_obj_dir, exist_ok=True)
            with open(numerical_parity_obj_path, "w"):
                mesh.export(numerical_parity_obj_path, "obj")

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
