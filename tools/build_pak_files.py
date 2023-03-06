#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import os
import posixpath
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--contents_dir", required=True)
    parser.add_argument("--platforms", nargs="*", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--scene_names")
    args = parser.parse_args()
    
    assert os.path.exists(args.unreal_engine_dir)
    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UE4Editor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UE4Editor.app", "Contents", "MacOS", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak"))

    unreal_project_dir         = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
    uproject                   = os.path.join(unreal_project_dir, "SpearSim.uproject")
    unreal_project_content_dir = os.path.join(unreal_project_dir, "Content")

    assert os.path.exists(args.contents_dir)
    shared_dir = os.path.realpath(os.path.join(args.contents_dir, "Shared"))
    scenes_dir = os.path.realpath(os.path.join(args.contents_dir, "Scenes"))

    assert os.path.exists(shared_dir)
    unreal_project_content_shared_dir = os.path.join(unreal_project_content_dir, "Shared")
    if not os.path.exists(unreal_project_content_shared_dir):
        print(f"[SPEAR | build_pak_files.py] Creating symlink: {unreal_project_content_shared_dir} -> {shared_dir}")
        os.symlink(shared_dir, unreal_project_content_shared_dir)

    assert os.path.exists(scenes_dir)
    scene_names = [ os.path.basename(x) for x in glob.glob(os.path.join(scenes_dir, "*")) ]
    assert len(scene_names) > 0

    if args.scene_names is not None:
        scene_content_dirs = [ os.path.realpath(os.path.join(scenes_dir, x)) for x in scene_names if fnmatch.fnmatch(x, args.scene_names) ]
    else:
        scene_content_dirs = [ os.path.realpath(os.path.join(scenes_dir, x)) for x in scene_names ]
    assert len(scene_content_dirs) > 0

    for scene_content_dir in scene_content_dirs:

        scene_name = os.path.basename(scene_content_dir)
        unreal_project_content_scene_dir = os.path.join(unreal_project_content_dir, "Scenes", scene_name)

        # remove existing scene dir
        if os.path.exists(unreal_project_content_scene_dir):
            print(f"[SPEAR | build_pak_files.py] File or directory or symlink exists, removing: {unreal_project_content_scene_dir}")
            os.unlink(unreal_project_content_scene_dir)

        # create symlink
        print(f"[SPEAR | build_pak_files.py] Creating symlink: {unreal_project_content_scene_dir} -> {scene_content_dir}")
        os.symlink(scene_content_dir, unreal_project_content_scene_dir)
        
        for platform in args.platforms:

            # construct command to cook the unreal project
            # refer https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Deployment/Cooking/ for more information on the parameters
            cmd = [
                unreal_editor_bin,
                uproject,
                "-run=Cook",
                "-TargetPlatform=" + platform + "NoEditor",
                "-fileopenlog",
                "-ddc=InstalledDerivedDataBackendGraph",
                "-unversioned",
                "-stdout",
                "-FullStdOutLogOutput",
                "-CrashForUAT",
                "-unattended",
                "-NoLogTimes",
                "-UTF8Output"
            ]
            print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)

            platform_dir = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor"))
            content_dirs_for_pak = [
                os.path.join(platform_dir, "Engine", "Content", "Animation"),
                os.path.join(platform_dir, "Engine", "Content", "BasicShapes"),
                os.path.join(platform_dir, "Engine", "Content", "EngineResources"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Math"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Texturing"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Utility"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "MaterialLayerFunctions"),
                os.path.join(platform_dir, "SpearSim", "Content")
            ]

            # create the output_dir
            os.makedirs(args.output_dir, exist_ok=True)

            pak_file_prefix = f"{scene_name}_{platform}"

            # construct path to the output pak file
            pak_file = os.path.join(args.output_dir, pak_file_prefix + ".pak")

            # text file used to generate the final pak file
            txt_file = os.path.join(args.output_dir, pak_file_prefix + ".txt")

            for i, dir in enumerate(content_dirs_for_pak):
                with open(txt_file, mode="w" if i==0 else "a") as f:
                    for content_file in glob.glob(os.path.join(dir, "**", "*.*"), recursive=True):
                        assert content_file.startswith(platform_dir)
                        content_file = content_file.replace('\\', "/")
                        mount_file = posixpath.join("..", "..", f"..{content_file.split(platform + 'NoEditor')[1]}")
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
            print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)

            assert os.path.exists(pak_file)
            print(f"[SPEAR | build_pak_files.py] Successfully built {pak_file} for {platform} platform.")

            print(f"[SPEAR | build_pak_files.py] Removing symlink: {unreal_project_content_scene_dir}")
            os.unlink(unreal_project_content_scene_dir)

    print("[SPEAR | build_pak_files.py] Done.")
