#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import pathlib
import shutil
import spear
import spear.pipeline_utils
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--destination_content_path", required=True)
    parser.add_argument("--source_content_path", required=True)
    parser.add_argument("--source_unreal_project_dir", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--destination_unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--temp_dir", default="tmp")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)

    #
    # Our strategy for importing external content from a source project into a destination project,
    # potentially into a different location in the Content Browser, is to perform a renaming step in a
    # minimal temporary Unreal project. The reason for doing it this way is as follows. Performing bulk
    # renaming operations only seem to work reliably if the assets being renamed are not currently loaded.
    # So, bulk asset renaming will not work in the case where source assets are loaded by a source project's
    # project's default map, and we therefore do our renaming in a minimal temporary project.
    #
    # However, an important consequence of this design decision is that both the source and destination
    # content paths need to be valid in our minimal project. This prevents us from using content paths that
    # refer to plugin folders, even if the source path is valid in the source project and the destination
    # path is valid in the destination project, since these content paths will not be valid in our minimal
    # temporary project. Therefore, only "/Game" content paths are supported in this script.
    #

    assert pathlib.PurePosixPath(args.source_content_path).parts[1] == "Game"
    assert pathlib.PurePosixPath(args.destination_content_path).parts[1] == "Game"

    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor-Cmd.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
    else:
        assert False

    #
    # projects
    #

    reference_unreal_project_dir = os.path.realpath(os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "DefaultProject")))
    reference_unreal_project_dir_name = os.path.split(reference_unreal_project_dir)[1]
    reference_uprojects = glob.glob(os.path.realpath(os.path.join(reference_unreal_project_dir, "*.uproject")))
    assert len(reference_uprojects) == 1
    reference_uproject = reference_uprojects[0]
    reference_uproject_name = os.path.splitext(os.path.split(reference_uproject)[1])[0]

    source_unreal_project_dir = os.path.realpath(args.source_unreal_project_dir)

    temp_unreal_project_dir = os.path.realpath(os.path.join(args.temp_dir, reference_unreal_project_dir_name))
    temp_uproject = os.path.realpath(os.path.join(temp_unreal_project_dir, reference_uproject_name + ".uproject"))

    destination_unreal_project_dir = os.path.realpath(args.destination_unreal_project_dir)

    #
    # filesystem paths
    #

    source_unreal_project_source_content_filesystem_path = spear.pipeline_utils.get_filesystem_path_from_content_path(
        content_path=args.source_content_path,
        unreal_project_dir=source_unreal_project_dir,
        unreal_engine_dir=args.unreal_engine_dir)

    temp_unreal_project_source_content_filesystem_path = spear.pipeline_utils.get_filesystem_path_from_content_path(
        content_path=args.source_content_path,
        unreal_project_dir=temp_unreal_project_dir,
        unreal_engine_dir=args.unreal_engine_dir)

    temp_unreal_project_destination_content_filesystem_path = spear.pipeline_utils.get_filesystem_path_from_content_path(
        content_path=args.destination_content_path,
        unreal_project_dir=temp_unreal_project_dir,
        unreal_engine_dir=args.unreal_engine_dir)

    destination_unreal_project_destination_content_filesystem_path = spear.pipeline_utils.get_filesystem_path_from_content_path(
        content_path=args.destination_content_path,
        unreal_project_dir=destination_unreal_project_dir,
        unreal_engine_dir=args.unreal_engine_dir)

    #
    # copy reference project to temp project
    #

    if os.path.exists(temp_unreal_project_dir):
        spear.log(f"Directory exists, removing: {temp_unreal_project_dir}")
        shutil.rmtree(temp_unreal_project_dir, ignore_errors=True)

    # If symlinks is true, symbolic links in the source tree are represented as symbolic links in the new
    # tree and the metadata of the original links will be copied as far as the platform allows; if false or
    # omitted, the contents and metadata of the linked files are copied to the new tree.
    spear.log(f"Copying directory: {reference_unreal_project_dir} -> {temp_unreal_project_dir}")
    shutil.copytree(reference_unreal_project_dir, temp_unreal_project_dir, symlinks=False, ignore_dangling_symlinks=False)

    #
    # copy source project content to temp project
    #

    assert os.path.exists(source_unreal_project_source_content_filesystem_path)
    assert not os.path.exists(temp_unreal_project_source_content_filesystem_path)

    #
    # Set symlink parameters to follow all symlinks in the source, such that any symlinks in the source are
    # replaced with a complete deep copy in the destination, but won't invalidate any existing symlinks in
    # the destination.
    #

    # If follow_symlinks is false and src is a symbolic link, a new symbolic link will be created instead of
    # copying the file src points to.
    if os.path.isfile(source_unreal_project_source_content_filesystem_path):
        spear.log(f"Copying file: {source_unreal_project_source_content_filesystem_path} -> {temp_unreal_project_source_content_filesystem_path}")
        shutil.copyfile(source_unreal_project_source_content_filesystem_path, temp_unreal_project_source_content_filesystem_path, follow_symlinks=True)

    # If symlinks is true, symbolic links in the source tree are represented as symbolic links in the new
    # tree and the metadata of the original links will be copied as far as the platform allows; if false or
    # omitted, the contents and metadata of the linked files are copied to the new tree.
    elif os.path.isdir(source_unreal_project_source_content_filesystem_path):
        spear.log(f"Copying directory: {source_unreal_project_source_content_filesystem_path} -> {temp_unreal_project_source_content_filesystem_path}")
        shutil.copytree(source_unreal_project_source_content_filesystem_path, temp_unreal_project_source_content_filesystem_path, symlinks=False, ignore_dangling_symlinks=False)

    else:
        assert False

    assert os.path.exists(temp_unreal_project_source_content_filesystem_path)

    #
    # rename content in temp project
    #

    # We could use run_editor_script.py here, but calling python scripts from other python scripts
    # isn't any more convenient than calling the editor directly.

    script = os.path.realpath(os.path.join(os.path.dirname(__file__), "editor_asset_library_utils_rename_content.py"))
    script_args = \
        "--source_content_path " + args.source_content_path + " " + \
        "--destination_content_path " + args.destination_content_path

    # need shell=True to correctly handle the quotes in cmd
    cmd = unreal_editor_bin + " " + temp_uproject + ' -run=pythonscript -script="' + script + " " + script_args + '"'
    spear.log(f"Executing: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

    assert not os.path.exists(temp_unreal_project_source_content_filesystem_path)
    assert os.path.exists(temp_unreal_project_destination_content_filesystem_path)

    #
    # copy temp project content to destination project
    #

    # If follow_symlinks is false and src is a symbolic link, a new symbolic link will be created instead of
    # copying the file src points to.
    if os.path.isfile(temp_unreal_project_destination_content_filesystem_path):
        if os.path.exists(destination_unreal_project_destination_content_filesystem_path):
            spear.log(f"File exists, removing: {destination_unreal_project_destination_content_filesystem_path}")
            os.remove(destination_unreal_project_destination_content_filesystem_path)
        spear.log(f"Copying file: {temp_unreal_project_destination_content_filesystem_path} -> {destination_unreal_project_destination_content_filesystem_path}")
        shutil.copyfile(temp_unreal_project_destination_content_filesystem_path, destination_unreal_project_destination_content_filesystem_path, follow_symlinks=True)

    # If symlinks is true, symbolic links in the source tree are represented as symbolic links in the new
    # tree and the metadata of the original links will be copied as far as the platform allows; if false or
    # omitted, the contents and metadata of the linked files are copied to the new tree.
    elif os.path.isdir(temp_unreal_project_destination_content_filesystem_path):
        if os.path.exists(destination_unreal_project_destination_content_filesystem_path):
            spear.log(f"Directory exists, removing: {destination_unreal_project_destination_content_filesystem_path}")
            shutil.rmtree(destination_unreal_project_destination_content_filesystem_path, ignore_errors=True)
        spear.log(f"Copying directory: {temp_unreal_project_destination_content_filesystem_path} -> {destination_unreal_project_destination_content_filesystem_path}")
        shutil.copytree(temp_unreal_project_destination_content_filesystem_path, destination_unreal_project_destination_content_filesystem_path, symlinks=False, ignore_dangling_symlinks=False)

    else:
        assert False

    spear.log("Done.")
