#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import posixpath
import spear
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--contents_dir", required=True)
    parser.add_argument("--platforms", nargs="*", required=True)
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()
    
    assert os.path.exists(args.unreal_engine_dir)
    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Win64", "UE4Editor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Win64", "UnrealPak.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Mac", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Mac", "UnrealPak"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Linux", "UE4Editor.sh"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Binaries", "Linux", "UnrealPak.sh"))

    assert os.path.exists(args.contents_dir)
    content_dirs = [ os.path.realpath(os.path.join(args.contents_dir, x)) for x in os.listdir(args.contents_dir) ]
    assert len(content_dirs) > 0

    unreal_project_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "cpp", "unreal_projects", "SpearSim"))
    uproject           = os.path.join(unreal_project_dir, "SpearSim.uproject")

    for content_dir in content_dirs:

        # remove existing Content dir
        unreal_project_content_dir = os.path.join(unreal_project_dir, "Content")
        if spear.path_exists(unreal_project_content_dir):
            print(f"[SPEAR | build_pak_files.py] File or directory or symlink exists, removing: {unreal_project_content_dir}")
            spear.remove_path(unreal_project_content_dir)

        # create symlink
        print(f"[SPEAR | build_pak_files.py] Creating symlink: {unreal_project_content_dir} -> {content_dir}")
        os.symlink(content_dir, unreal_project_content_dir)

        for platform in args.platforms:

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
            cmd_result = subprocess.run(cmd, check=True)

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

            pak_file_prefix = f"Map_{os.path.basename(content_dir)}_{platform}"

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

            # command to generate the final pak file
            cmd = [
                unreal_pak_bin,
                pak_file,
                "-create=" + txt_file,
                "-platform=" + platform,
                "-multiprocess",
                "-compress"
            ]
            print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
            cmd_result = subprocess.run(cmd, check=True)

            assert os.path.exists(pak_file)
            print(f"[SPEAR | build_pak_files.py] Successfully built {pak_file} for {platform} platform.")

    print("[SPEAR | build_pak_files.py] Done.")
