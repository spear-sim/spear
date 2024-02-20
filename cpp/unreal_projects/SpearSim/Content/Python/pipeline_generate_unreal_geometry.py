import argparse
import os
import pathlib
import posixpath
import unreal
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)


def process_scene():

    editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()

    spear.log("Exporting Unreal scene geometry: ", editor_world_name)

    # Sort and filter out actors with no root component.
    actors = editor_actor_subsystem.get_all_level_actors()
    actors = [ actor for actor in sorted(actors, key=lambda actor: get_debug_string_actor(actor)) if actor.root_component is not None ]

    # For each actor, export all static mesh components
    for actor in actors:

        spear.log("    Processing actor: ", get_debug_string_actor(actor))

        components               = [actor.root_component] + list(actor.root_component.get_children_components(include_all_descendants=False))
        static_mesh_components   = [ c for c in components if c.__class__.__name__ == "StaticMeshComponent" ]
        static_meshes            = [ smc.static_mesh for smc in static_mesh_components if smc.static_mesh is not None ]
        static_mesh_asset_paths  = [ pathlib.PurePosixPath(sm.get_path_name()) for sm in static_meshes ]
        static_mesh_asset_paths  = [ smap for smap in static_mesh_asset_paths if smap.parts[0:4] == ("/", "Game", "Scenes", editor_world_name) ]

        for static_mesh_asset_path in static_mesh_asset_paths:
            spear.log("        Exporting asset: ", static_mesh_asset_path)

            asset = asset_registry.get_asset_by_object_path(str(static_mesh_asset_path))
            export_path_suffix = posixpath.join(*static_mesh_asset_path.parts[4:]) + ".obj"
            export_path = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "unreal_geometry", export_path_suffix))

            errors = []
            asset_export_task = unreal.AssetExportTask()
            asset_export_task.set_editor_property("automated", True)
            asset_export_task.set_editor_property("errors", errors)
            asset_export_task.set_editor_property("exporter", None)            
            asset_export_task.set_editor_property("filename", export_path)
            asset_export_task.set_editor_property("ignore_object_list", [])
            asset_export_task.set_editor_property("object", asset.get_asset())
            asset_export_task.set_editor_property("options", None)
            asset_export_task.set_editor_property("prompt", False)
            asset_export_task.set_editor_property("replace_identical", False)
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
            spear.log()

            unreal.Exporter.run_asset_export_task(asset_export_task)
            if len(errors) > 0:
                spear.log(errors)
                assert False

        spear.log()

    spear.log("Done.")


def get_debug_string_actor(actor):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        return actor.get_actor_label()
    else:
        return str(folder_path) + posixpath.sep + actor.get_actor_label()


def get_debug_string_component(component):
    parent_components = list(component.get_parent_components())[::-1] # reverse to get parent_components in root-to-leaf order
    if len(parent_components) == 0:
        return component.get_name()
    else:
        return ".".join([ parent_component.get_name() for parent_component in parent_components ]) + "." + component.get_name()


if __name__ == "__main__":
    process_scene()
