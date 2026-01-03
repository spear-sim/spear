#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import pathlib
import posixpath
import shutil
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--source-content-path", required=True)
parser.add_argument("--destination-content-path", required=True)
args = parser.parse_args()

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()


def get_object_path_from_asset_path(asset_path):
    asset_dir, asset_name = posixpath.split(asset_path)
    asset_name_tokens = asset_name.split(".")
    if len(asset_name_tokens) == 1:
        return posixpath.join(asset_dir, f"{asset_name_tokens[0]}.{asset_name_tokens[0]}")
    elif len(asset_name_tokens) == 2:
        assert asset_name_tokens[0].lower() == asset_name_tokens[1].lower()
        return asset_path
    else:
        assert False


if __name__ == "__main__":

    # Strip the trailing "/"" from our input paths. Calling does_asset_exist(...) with a path that ends in
    # "/" will cause a non-fatal error. The editor won't terminate, but it will return a non-zero error code
    # when exiting, which can cause problems for outer scripts.
    source_content_path = args.source_content_path.rstrip("/")
    destination_content_path = args.destination_content_path.rstrip("/")

    # Validate arguments.
    assert len(pathlib.PurePosixPath(source_content_path).parts) >= 2
    assert pathlib.PurePosixPath(source_content_path).parts[1] != "Engine"
    assert pathlib.PurePosixPath(destination_content_path).parts[1] != "Engine"
    assert not unreal.EditorAssetLibrary.does_asset_exist(destination_content_path)
    assert not unreal.EditorAssetLibrary.does_directory_exist(destination_content_path)

    # If the source content is an asset, then rename and remove the redirector if it was created during the
    # renaming operation.
    if unreal.EditorAssetLibrary.does_asset_exist(source_content_path):
        spear.log(f"Renaming asset: {source_content_path} -> {destination_content_path}")
        unreal.EditorAssetLibrary.rename_asset(source_content_path, destination_content_path)

        if unreal.EditorAssetLibrary.does_asset_exist(source_content_path):
            asset_data = asset_registry.get_asset_by_object_path(get_object_path_from_asset_path(asset_path=source_content_path))
            assert asset_data.is_redirector()
            spear.log("Deleting redirector: ", source_content_path)
            unreal.EditorAssetLibrary.delete_asset(source_content_path)

        assert not unreal.EditorAssetLibrary.does_asset_exist(source_content_path)
        assert unreal.EditorAssetLibrary.does_asset_exist(destination_content_path)

    # Otherwise, if the source content is directory, then rename and remove the empty source directory if it
    # is still there after the renaming operation.
    elif unreal.EditorAssetLibrary.does_directory_exist(source_content_path):
        spear.log(f"Renaming directory: {source_content_path} -> {destination_content_path}")
        unreal.EditorAssetLibrary.rename_directory(source_content_path, destination_content_path)

        if unreal.EditorAssetLibrary.does_directory_exist(source_content_path):
            assert not unreal.EditorAssetLibrary.does_directory_have_assets(source_content_path)
            filesystem_path = spear.utils.editor_utils.get_filesystem_path_from_content_path(content_path=source_content_path)
            spear.log("Directory was not deleted, deleting via the filesystem: ", filesystem_path)
            shutil.rmtree(filesystem_path, ignore_errors=True)
            assert not os.path.exists(filesystem_path)

    else:
        assert False

    spear.log("Done.")
