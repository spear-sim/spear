#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import ntpath
import os
import posixpath
import spear
import subprocess
import sys

def cook(unreal_editor_bin, uproject, platform):
    # see https://docs.unrealengine.com/5.2/en-US/SharingAndReleasing/Deployment/Cooking for more information on these parameters
    cmd = [
        unreal_editor_bin,
        uproject,
        "-run=Cook",
        "-targetplatform=" + platform,
        "-unattended",                           # don't require any user input
        "-iterate",                              # only cook content that needs to be updated
        "-fileopenlog",                          # generate a log of which files are opened in which order
        "-ddc=InstalledDerivedDataBackendGraph", # use the default cache location for installed (i.e., not source) builds of the engine
        "-unversioned",                          # save all of the cooked packages without versions
        "-stdout",                               # ensure log output is written to the terminal 
        "-fullstdoutlogoutput",                  # ensure log output is written to the terminal
        "-nologtimes"                            # don't print timestamps next to log messages twice
    ]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

def create_pak(unreal_pak_bin, pak_file, txt_file, platform):
    # construct command to generate the final pak file
    cmd = [
        unreal_pak_bin,
        pak_file,
        "-create=" + txt_file,
        "-platform=" + platform,
        "-multiprocess",
        "-compressed"]

    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    assert os.path.exists(pak_file)
    spear.log(f"Successfully built {pak_file}")


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--skip_create_symlinks", action="store_true")
    parser.add_argument("--perforce_content_dir")
    parser.add_argument("--scene_ids")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)
    if args.skip_create_symlinks:
        assert args.scene_ids is not None
    else:
        assert os.path.exists(args.perforce_content_dir)

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    assert os.path.exists(unreal_project_dir)

    uproject                   = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))
    unreal_project_content_dir = os.path.realpath(os.path.join(unreal_project_dir, "Content"))
    output_dir                 = os.path.realpath(args.output_dir)

    if sys.platform == "win32":
        platform          = "Windows"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe"))
    elif sys.platform == "darwin":
        platform          = "Mac"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak"))
    elif sys.platform == "linux":
        platform          = "Linux"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak"))
    else:
        assert False

    assert os.path.exists(unreal_editor_bin)
    assert os.path.exists(unreal_pak_bin)

    # once we know the platform, set our cooked dir
    unreal_project_cooked_dir       = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", platform))
    unreal_project_cooked_dir_posix = unreal_project_cooked_dir.replace(ntpath.sep, posixpath.sep)

    # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
    # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
    unreal_project_content_scenes_dir = os.path.join(unreal_project_content_dir, "Scenes")

    # We use different strategies for setting scene_ids, depending on if we're creating symlinks or not.
    # If we're not creating symlinks, then the user must specify args.scene_ids. If we are creating
    # symlinks, then we get a list of candidate scene_ids from Perforce and optionally filter.
    if args.skip_create_symlinks:
        scene_ids = [args.scene_ids]
    else:
        perforce_content_scenes_dir = os.path.realpath(os.path.join(args.perforce_content_dir, "Scenes"))
        assert os.path.exists(perforce_content_scenes_dir)

        ignore_names = [".DS_Store"]
        scene_ids = [ os.path.basename(x) for x in os.listdir(perforce_content_scenes_dir) if x not in ignore_names ]
        assert len(scene_ids) > 0
        if args.scene_ids is not None:
            scene_ids = [ s for s in scene_ids if fnmatch.fnmatch(s, args.scene_ids) ]
        assert len(scene_ids) > 0

    # Create a symlink to common top-level directories
    if not args.skip_create_symlinks:
        # We do not want to use os.path.realpath(...) for the values in this dictionary, because that will resolve
        # to the directory inside the user's Perforce workspace. Instead, we want this path to refer to the symlinked
        # version inside the user's Unreal project directory.
        perforce_dirs_to_unreal_dirs = {
            os.path.realpath(os.path.join(args.perforce_content_dir, "Megascans")) : os.path.join(unreal_project_content_dir, "Megascans"),
            os.path.realpath(os.path.join(args.perforce_content_dir, "MSPresets")) : os.path.join(unreal_project_content_dir, "MSPresets")
        }

        for perforce_dir, unreal_project_dir in perforce_dirs_to_unreal_dirs.items():
            assert os.path.exists(perforce_dir)

            if spear.path_exists(unreal_project_dir):
                spear.log(f"File or directory or symlink exists, removing: {unreal_project_dir}")
                spear.remove_path(unreal_project_dir)

            spear.log(f"Creating symlink: {unreal_project_dir} -> {perforce_dir}")
            os.symlink(perforce_dir, unreal_project_dir)

    # build and package common content's pak file
    common_txt_file = os.path.realpath(os.path.join(output_dir, "common-" + args.version_tag + "-" + platform + ".txt"))
    common_pak_file = os.path.realpath(os.path.join(output_dir, "common-" + args.version_tag + "-" + platform + ".pak"))

    common_dirs = [
        os.path.realpath(os.path.join(unreal_project_cooked_dir, "Engine", "Content")),
        os.path.realpath(os.path.join(unreal_project_cooked_dir, "Engine", "Plugins")),
        os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Megascans")),
        os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "MSPresets"))]

    # cook contents with no scene-specific information
    cook(unreal_editor_bin, uproject, platform)

    # once cooking is complete, package the cooked contents into a pak file
    with open(common_txt_file, mode="w") as f:
        for common_dir in common_dirs:
            for content_file in glob.glob(os.path.realpath(os.path.join(common_dir, "**", "*.*")), recursive=True):
                content_file_posix = content_file.replace(ntpath.sep, posixpath.sep)
                assert content_file_posix.startswith(unreal_project_cooked_dir_posix)
                mount_file_posix = posixpath.join("..", "..", "..", content_file_posix.replace(unreal_project_cooked_dir_posix + posixpath.sep, ""))
                f.write(f'"{content_file_posix}" "{mount_file_posix}" "" \n')

    create_pak(unreal_pak_bin, common_pak_file, common_txt_file, platform)

    for scene_id in scene_ids:

        scene_dir = os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Scenes", scene_id))

        txt_file = os.path.realpath(os.path.join(output_dir, scene_id + "-" + args.version_tag + "-" + platform + ".txt"))
        pak_file = os.path.realpath(os.path.join(output_dir, scene_id + "-" + args.version_tag + "-" + platform + ".pak"))

        # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
        # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
        unreal_project_content_scene_dir = os.path.join(unreal_project_content_dir, "Scenes", scene_id)

        if not args.skip_create_symlinks:
            perforce_content_scene_dir = os.path.realpath(os.path.join(perforce_content_scenes_dir, scene_id))

            if spear.path_exists(unreal_project_content_scene_dir):
                spear.log(f"File or directory or symlink exists, removing: {unreal_project_content_scene_dir}")
                spear.remove_path(unreal_project_content_scene_dir)

            spear.log(f"Creating symlink: {unreal_project_content_scene_dir} -> {perforce_content_scene_dir}")
            os.symlink(perforce_content_scene_dir, unreal_project_content_scene_dir)

        # Now that we have created a symlink, our Unreal project should contain exactly four scenes: apartment_0000, debug_0000, debug_0001 and scene_id
        ignore_names = [".DS_Store"]
        unreal_project_scenes = { os.path.basename(x) for x in os.listdir(unreal_project_content_scenes_dir) if x not in ignore_names }
        assert unreal_project_scenes == {"apartment_0000", "debug_0000", "debug_0001", scene_id}

        # cook with the updated scene contents
        cook(unreal_editor_bin, uproject, platform)

        # create the output_dir
        os.makedirs(output_dir, exist_ok=True)

        with open(txt_file, mode="w") as f:
            for content_file in glob.glob(os.path.realpath(os.path.join(scene_dir, "**", "*.*")), recursive=True):
                content_file_posix = content_file.replace(ntpath.sep, posixpath.sep)
                assert content_file_posix.startswith(unreal_project_cooked_dir_posix)
                mount_file_posix = posixpath.join("..", "..", "..", content_file_posix.replace(unreal_project_cooked_dir_posix + posixpath.sep, ""))
                f.write(f'"{content_file_posix}" "{mount_file_posix}" "" \n')

        # create paks
        create_pak(unreal_pak_bin, pak_file, txt_file, platform)

        if not args.skip_create_symlinks:
            spear.log(f"Removing symlink: {unreal_project_content_scene_dir}")
            spear.remove_path(unreal_project_content_scene_dir)

    if not args.skip_create_symlinks:
        for perforce_dir, unreal_project_dir in perforce_dirs_to_unreal_dirs.items():
            spear.log(f"File or directory or symlink exists, removing: {unreal_project_dir}")
            spear.remove_path(unreal_project_dir)

    spear.log("Done.")
