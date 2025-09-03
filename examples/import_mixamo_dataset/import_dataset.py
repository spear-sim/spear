#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import posixpath
import spear
import spear.utils.editor_utils
import unreal


asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)

invalid_chars_map = {
    "-": "_",
    " ": "_",
    ".": "_",
    ",": "_",
    "'": "_"}


def replace_chars(string, replace_chars_map):
    return_value = string
    for k, v in replace_chars_map.items():
        return_value = return_value.replace(k, v)
    return return_value


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--animation_dataset_raw_base_dir", required=True)
    parser.add_argument("--animation_dataset_content_base_dir", required=True)
    parser.add_argument("--animation_dataset_begin_index", type=int)
    parser.add_argument("--animation_dataset_num_indices", type=int)
    parser.add_argument("--skip_if_exists", action="store_true")
    args = parser.parse_args()

    # assume there is a top-level base directory that contains subdirectories that contain FBX files

    animation_dataset_raw_anim_dirs = [ d for d in sorted(os.listdir(args.animation_dataset_raw_base_dir)) if d != ".DS_Store" ]

    # allow importing in batches because importing content leaks memory

    animation_dataset_begin_index = 0
    if args.animation_dataset_begin_index is not None:
        animation_dataset_begin_index = args.animation_dataset_begin_index

    animation_dataset_end_index = len(animation_dataset_raw_anim_dirs) - 1
    if args.animation_dataset_num_indices is not None:
        animation_dataset_end_index = min(animation_dataset_end_index, animation_dataset_begin_index + args.animation_dataset_num_indices - 1)

    # import each directory

    spear.log("Processing directories starting from: ", animation_dataset_raw_anim_dirs[animation_dataset_begin_index])
    spear.log("Processing directories ending with:   ", animation_dataset_raw_anim_dirs[animation_dataset_end_index])

    for animation_dataset_raw_anim_dir in animation_dataset_raw_anim_dirs[animation_dataset_begin_index: animation_dataset_end_index+1]:

        spear.log(f"Processing: ", animation_dataset_raw_anim_dir)

        # import each FBX file

        animation_dataset_raw_anim_files = [ f for f in sorted(os.listdir(os.path.join(args.animation_dataset_raw_base_dir, animation_dataset_raw_anim_dir))) if f.lower().endswith(".fbx") ]
        for animation_dataset_raw_anim_file in animation_dataset_raw_anim_files:

            anim_content_dir = replace_chars(animation_dataset_raw_anim_dir, invalid_chars_map)
            anim_content_name = os.path.splitext(animation_dataset_raw_anim_file)[0].replace("-", "_").replace(" ", "_")
            anim_content_path = posixpath.join(args.animation_dataset_content_base_dir, anim_content_dir, anim_content_name)

            if args.skip_if_exists and unreal.EditorAssetLibrary.does_directory_exist(anim_content_path):
                spear.log("Directory exists, skipping: ", anim_content_path)
                continue

            if unreal.EditorAssetLibrary.does_directory_exist(anim_content_path):
                spear.log("Directory exists, removing: ", anim_content_path)
                success = unreal.EditorAssetLibrary.delete_directory(anim_content_path)
                assert success
                success = unreal.EditorAssetLibrary.make_directory(anim_content_path)
                assert success
            else:
                spear.log("Directory doesn't exist: ", anim_content_path)

            animation_dataset_raw_anim_path = os.path.join(args.animation_dataset_raw_base_dir, animation_dataset_raw_anim_dir, animation_dataset_raw_anim_file)

            spear.log(f"Importing {animation_dataset_raw_anim_path} to {anim_content_path}...")

            asset_import_task = unreal.AssetImportTask()
            asset_import_task.set_editor_property("async_", False)
            asset_import_task.set_editor_property("automated", True)
            asset_import_task.set_editor_property("destination_path", anim_content_path)
            asset_import_task.set_editor_property("filename", animation_dataset_raw_anim_path)
            asset_import_task.set_editor_property("replace_existing", True)
            asset_import_task.set_editor_property("replace_existing_settings", True)
            asset_import_task.set_editor_property("save", True)

            asset_tools.import_asset_tasks([asset_import_task])

            asset_paths = unreal.EditorAssetLibrary.list_assets(anim_content_path)
            assert unreal.EditorAssetLibrary.does_directory_exist(anim_content_path)

            spear.log(f"Imported assets: ")
            for asset_path in asset_paths:
                spear.log("    ", asset_path)

            assert unreal.EditorAssetLibrary.does_directory_exist(anim_content_path)

            # create a blueprint for each AnimSequence asset, and configure it to match the actor that would
            # be created by manually dragging the AnimSequence asset into the editor viewport

            for asset_path in asset_paths:
                asset_data = unreal.EditorAssetLibrary.find_asset_data(asset_path)

                if asset_data.get_editor_property("asset_class_path").get_editor_property("asset_name") == "AnimSequence":

                    anim_sequence_dir = str(asset_data.get_editor_property("package_path"))
                    anim_sequence_name = str(asset_data.get_editor_property("asset_name"))
                    anim_sequence_path = posixpath.join(anim_sequence_dir, f"{anim_sequence_name}.{anim_sequence_name}")
                    spear.log(f"Creating blueprint for AnimSequence: ", anim_sequence_path)

                    blueprint_dir = anim_sequence_dir
                    blueprint_name = f"BP_{anim_sequence_name}"
                    blueprint_path = posixpath.join(blueprint_dir, blueprint_name)

                    # remove existing blueprint
                    if unreal.EditorAssetLibrary.does_asset_exist(blueprint_path):
                        spear.log("Asset exists, removing: ", blueprint_path)
                        success = unreal.EditorAssetLibrary.delete_asset(blueprint_path)
                        assert success

                    # create blueprint
                    spear.log("Creating blueprint: ", blueprint_path)
                    blueprint_asset, blueprint_subobject_descs = spear.utils.editor_utils.create_blueprint(
                        asset_name=blueprint_name,
                        package_path=blueprint_dir,
                        actor_class=unreal.SkeletalMeshActor)

                    actor = blueprint_subobject_descs["actor"]["object"]
                    assert isinstance(actor, unreal.SkeletalMeshActor)

                    skeletal_mesh_component = blueprint_subobject_descs["root_component"]["object"]
                    assert isinstance(skeletal_mesh_component, unreal.SkeletalMeshComponent)

                    skeletal_mesh_path = posixpath.join(anim_sequence_dir, f"{anim_content_name}.{anim_content_name}")
                    skeletal_mesh = unreal.load_asset(skeletal_mesh_path)

                    anim_sequence = unreal.load_asset(anim_sequence_path)

                    skeletal_mesh_component.set_animation_mode(unreal.AnimationMode.ANIMATION_SINGLE_NODE) # use animation asset
                    skeletal_mesh_component.set_skeletal_mesh_asset(skeletal_mesh)
                    skeletal_mesh_component.set_editor_property("animation_data", unreal.SingleAnimationPlayData(anim_sequence))

                    spear.log("Saving blueprint: ", blueprint_path)
                    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log("Done.")
