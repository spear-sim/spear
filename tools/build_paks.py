#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import os
import posixpath
import spear
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)    
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

    unreal_project_dir         = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
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
    unreal_project_cooked_dir = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", platform + "NoEditor"))

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

    # We do not want to use os.path.realpath(...) for the values in this dictionary, because that will resolve
    # to the directory inside the user's Perforce workspace. Instead, we want this path to refer to the symlinked
    # version inside the user's Unreal project directory.
    perforce_dirs_to_unreal_dirs = {
        os.path.realpath(os.path.join(args.perforce_content_dir, "Megascans")) : os.path.join(unreal_project_content_dir, "Megascans"),
        os.path.realpath(os.path.join(args.perforce_content_dir, "MSPresets")) : os.path.join(unreal_project_content_dir, "MSPresets"),
        os.path.realpath(os.path.join(args.perforce_content_dir, "Shared")) : os.path.join(unreal_project_content_dir, "Shared")
    }

    # Create a symlink to common top-level directories
    if not args.skip_create_symlinks:
        for perforce_dir, unreal_project_dir in perforce_dirs_to_unreal_dirs.items():
            assert os.path.exists(perforce_dir)

            if spear.path_exists(unreal_project_dir):
                spear.log(f"File or directory or symlink exists, removing: {unreal_project_dir}")
                spear.remove_path(unreal_project_dir)

            spear.log(f"Creating symlink: {unreal_project_dir} -> {perforce_dir}")
            os.symlink(perforce_dir, unreal_project_dir)

    for scene_id in scene_ids:

        pak_dirs = [
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "Engine", "Content")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Megascans")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "MSPresets")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Shared")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Scenes", scene_id)),
        ]

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

        # Now that we have created a symlink, our Unreal project should contain exactly two scenes: debug_0000 and scene_id
        ignore_names = [".DS_Store"]
        unreal_project_scenes = { os.path.basename(x) for x in os.listdir(unreal_project_content_scenes_dir) if x not in ignore_names }
        assert unreal_project_scenes == {"debug_0000", scene_id}

        # see https://docs.unrealengine.com/5.2/en-US/SharingAndReleasing/Deployment/Cooking for more information on these parameters
        cmd = [
            unreal_editor_bin,
            uproject,
            "-run=Cook",
            "-targetplatform=" + platform + "NoEditor",
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

        # create the output_dir
        os.makedirs(output_dir, exist_ok=True)

        for i, pak_dir in enumerate(pak_dirs):
            with open(txt_file, mode="w" if i==0 else "a") as f:
                for content_file in glob.glob(os.path.realpath(os.path.join(pak_dir, "**", "*.*")), recursive=True):
                    assert content_file.startswith(unreal_project_cooked_dir)
                    content_file = content_file.replace('\\', "/")
                    mount_file = posixpath.join("..", "..", ".." + content_file.split(platform + "NoEditor")[1])
                    f.write(f'"{content_file}" "{mount_file}" "" \n')

        # construct command to generate the final pak file
        cmd = [
            unreal_pak_bin,
            pak_file,
            "-create=" + txt_file,
            "-platform=" + platform,
            "-multiprocess",
            "-compress"
        ]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        assert os.path.exists(pak_file)
        spear.log(f"Successfully built {pak_file}")

        if not args.skip_create_symlinks:
            spear.log(f"Removing symlink: {unreal_project_content_scene_dir}")
            spear.remove_path(unreal_project_content_scene_dir)

    if not args.skip_create_symlinks:
        for perforce_dir, unreal_project_dir in perforce_dirs_to_unreal_dirs.items():
            spear.log(f"File or directory or symlink exists, removing: {unreal_project_dir}")
            spear.remove_path(unreal_project_dir)

    spear.log("Done.")
