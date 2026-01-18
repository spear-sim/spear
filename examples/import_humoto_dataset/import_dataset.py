#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import posixpath
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--filesystem-base-dir", required=True)
parser.add_argument("--content-base-dir", required=True)
parser.add_argument("--begin-index", type=int)
parser.add_argument("--num-indices", type=int)
parser.add_argument("--skip-if-exists", action="store_true")
args = parser.parse_args()

asset_editor_subsystem = unreal.get_editor_subsystem(unreal.AssetEditorSubsystem)
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

editor_world = unreal_editor_subsystem.get_editor_world()

replace_chars_map = {
    "-": "_",
    " ": "_",
    ".": "_",
    ",": "_",
    "'": "_"}

content_path_extra_chars = 6                             # the Unreal Editor adds up to this many characters when reporting the length of paths
max_content_path_length = 210 - content_path_extra_chars # Unreal's stated maximum path length is 210, but we need to account for the fact that the Unreal Editor adds a few chars when reporting path lengths
num_content_dirs_after_base_dir = 2                      # assume we will be creating Unreal paths of the form /content_base_dir/sub_dir_1/sub_dir_2/fbx_name.fbx_name
num_content_file_parts = 2


def truncate_from_middle(string, max_length):
    truncate_string = "__"
    if len(string) <= max_length:
        return string
    elif max_length <= len(truncate_string):
        return truncate_string[:max_length]
    else:
        return string[:max_length//2-1] + truncate_string + string[-max_length//2+1:]

def replace_chars(string, replace_chars_map):
    return_value = string
    for k, v in replace_chars_map.items():
        return_value = return_value.replace(k, v)
    return return_value


if __name__ == "__main__":

    # get dirty state because we will need to restore it later
    is_editor_world_dirty = unreal.SpPackage.is_dirty(package=editor_world.get_outer())

    # assume there is a top-level filesystem directory that contains subdirectories that contain FBX files

    filesystem_sub_dirs = [ d for d in sorted(os.listdir(args.filesystem_base_dir)) if d != ".DS_Store" ]

    # allow importing in batches because importing content leaks memory

    begin_index = 0
    if args.begin_index is not None:
        begin_index = args.begin_index

    end_index = len(filesystem_sub_dirs) - 1
    if args.num_indices is not None:
        end_index = min(end_index, begin_index + args.num_indices - 1)

    # import each directory

    spear.log("Processing top-level filesystem directory: ", args.filesystem_base_dir)
    spear.log("Processing filesystem subdirectories starting from: ", filesystem_sub_dirs[begin_index])
    spear.log("Processing filesystem subdirectories ending with: ", filesystem_sub_dirs[end_index])

    for filesystem_sub_dir in filesystem_sub_dirs[begin_index: end_index+1]:

        spear.log("    Processing filesystem subdirectory: ", filesystem_sub_dir)

        # import each FBX file

        fbx_files = [ f for f in sorted(os.listdir(os.path.join(args.filesystem_base_dir, filesystem_sub_dir))) if f.lower().endswith(".fbx") ]
        for fbx_file in fbx_files:

            max_content_dir_length = (max_content_path_length - content_path_extra_chars - len(args.content_base_dir) - 2*(len("FbxScene_") + len(fbx_file) - len(".fbx"))) // num_content_file_parts
            spear.log(f"    Truncating content subdirectories to {max_content_dir_length} chars to stay within Unreal's limits for content paths...")

            fbx_content_subdir = truncate_from_middle(string=replace_chars(string=filesystem_sub_dir, replace_chars_map=replace_chars_map), max_length=max_content_dir_length)
            fbx_content_name = truncate_from_middle(string=replace_chars(string=os.path.splitext(fbx_file)[0], replace_chars_map=replace_chars_map), max_length=max_content_dir_length)
            fbx_content_dir = posixpath.join(args.content_base_dir, fbx_content_subdir, fbx_content_name)
            blueprint_content_path = posixpath.join(fbx_content_dir, "FbxScene_" + os.path.splitext(fbx_file)[0])

            spear.log(f"    Blueprint path ({len(blueprint_content_path)} chars): {blueprint_content_path}")
            assert len(blueprint_content_path) <= max_content_path_length

            if args.skip_if_exists and unreal.EditorAssetLibrary.does_directory_exist(directory_path=fbx_content_dir):
                spear.log("        Directory exists, skipping: ", fbx_content_dir)
                continue

            if unreal.EditorAssetLibrary.does_directory_exist(directory_path=fbx_content_dir):
                spear.log("        Directory exists, removing: ", fbx_content_dir)
                success = unreal.EditorAssetLibrary.delete_directory(directory_path=fbx_content_dir)
                assert success
                success = unreal.EditorAssetLibrary.make_directory(directory_path=fbx_content_dir)
                assert success
            else:
                spear.log("        Directory doesn't exist: ", fbx_content_dir)

            fbx_filesystem_path = os.path.join(args.filesystem_base_dir, filesystem_sub_dir, fbx_file)

            spear.log(f"        Importing {fbx_filesystem_path} to {fbx_content_dir}...")



            fbx_scene_import_options = unreal.FbxSceneImportOptions()
            fbx_scene_import_options.set_editor_property("import_as_dynamic", True)

            fbx_scene_import_factory = unreal.FbxSceneImportFactory()
            fbx_scene_import_factory.set_editor_property("edit_after_new", value=False)
            fbx_scene_import_factory.set_editor_property("scene_import_options", value=fbx_scene_import_options)

            asset_import_task = unreal.AssetImportTask()
            asset_import_task.set_editor_property(name="async_", value=False)
            asset_import_task.set_editor_property(name="automated", value=True)
            asset_import_task.set_editor_property(name="destination_path", value=fbx_content_dir)
            asset_import_task.set_editor_property(name="factory", value=fbx_scene_import_factory)
            asset_import_task.set_editor_property(name="filename", value=fbx_filesystem_path)
            asset_import_task.set_editor_property(name="options", value=fbx_scene_import_options)
            asset_import_task.set_editor_property(name="replace_existing", value=True)
            asset_import_task.set_editor_property(name="replace_existing_settings", value=True)
            asset_import_task.set_editor_property(name="save", value=True)

            asset_tools.import_asset_tasks(import_tasks=[asset_import_task])

            assert unreal.EditorAssetLibrary.does_directory_exist(directory_path=fbx_content_dir)
            asset_paths = unreal.EditorAssetLibrary.list_assets(directory_path=fbx_content_dir)

            spear.log("        Imported assets: ")
            for asset_path in asset_paths:
                spear.log("            ", asset_path)

            # # get the blueprint's CDO
            # blueprint_class = unreal.EditorAssetLibrary.load_blueprint_class(asset_path=blueprint_content_path)
            # assert isinstance(blueprint_class, unreal.BlueprintGeneratedClass)
            # blueprint_default_object = unreal.get_default_object(blueprint_class)
            # assert isinstance(blueprint_default_object, unreal.Actor)

            blueprint_asset = unreal.load_asset(name=blueprint_content_path)
            assert isinstance(blueprint_asset, unreal.Blueprint)

            # close the blueprint editor because it opens automatically during import
            asset_editor_subsystem.close_all_editors_for_asset(blueprint_asset)

            # remove the spawned actor from the current level because it spawns automatically
            for actor in unreal.EditorLevelLibrary.get_all_level_actors():
                if actor.get_class() == blueprint_asset.generated_class():
                    spear.log("        Destroying actor: ", spear.utils.editor_utils.get_stable_name_for_actor(actor))
                    unreal.EditorLevelLibrary.destroy_actor(actor)

            # modify the skeletal mesh components in the imported blueprint
            blueprint_subobject_descs = spear.utils.editor_utils.get_subobject_descs_for_blueprint_asset(blueprint_asset=blueprint_asset)
            assert len(blueprint_subobject_descs) >= 2
            assert isinstance(blueprint_subobject_descs[0]["object"], unreal.Actor) # the 0th entry always refers to the actor itself

            # for each skeletal mesh component...
            for blueprint_subobject_desc in blueprint_subobject_descs:
                if isinstance(blueprint_subobject_desc["object"], unreal.SkeletalMeshComponent):

                    skeletal_mesh_component = blueprint_subobject_desc["object"]
                    skeletal_mesh_component_name = skeletal_mesh_component.get_name().removesuffix("_GEN_VARIABLE")

                    spear.log("        Modifying unreal.SkeletalMeshComponent: ", skeletal_mesh_component_name)

                    # ...find its matching animation sequence...
                    anim_sequence_path_candidates = []
                    for asset_path in asset_paths:

                        asset_data = unreal.EditorAssetLibrary.find_asset_data(asset_path=asset_path)
                        asset_class_name = asset_data.get_editor_property(name="asset_class_path").get_editor_property(name="asset_name")
                        asset_name = asset_data.get_editor_property("asset_name")

                        if asset_class_name == "AnimSequence" and str(asset_name).endswith(skeletal_mesh_component_name + "_Anim"):
                            anim_sequence_dir = str(asset_data.get_editor_property(name="package_path"))
                            anim_sequence_name = str(asset_data.get_editor_property(name="asset_name"))
                            anim_sequence_path = posixpath.join(anim_sequence_dir, f"{anim_sequence_name}.{anim_sequence_name}")
                            anim_sequence_path_candidates.append(anim_sequence_path)

                    if len(anim_sequence_path_candidates) == 1:
                        anim_sequence_path = anim_sequence_path_candidates[0]
                    else:
                        spear.log("        ERROR: couldn't find unique AnimSequence, giving up...")
                        continue

                    # ...and modify the component to refer to the animation sequence
                    anim_sequence = unreal.load_asset(name=anim_sequence_path)
                    skeletal_mesh_component.set_animation_mode(animation_mode=unreal.AnimationMode.ANIMATION_SINGLE_NODE) # use animation asset
                    skeletal_mesh_component.set_editor_property(name="animation_data", value=unreal.SingleAnimationPlayData(anim_to_play=anim_sequence))

            spear.log("        Saving blueprint: ", blueprint_content_path)
            editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

            spear.log("        Saving directory: ", fbx_content_dir)
            editor_asset_subsystem.save_directory(directory_path=fbx_content_dir, only_if_is_dirty=True, recursive=False)

    # restore dirty state
    unreal.SpPackage.set_dirty_flag(package=editor_world.get_outer(), is_dirty=is_editor_world_dirty)

    spear.log("Done.")
