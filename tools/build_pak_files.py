#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import posixpath
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_binaries_dir", required=True)
    parser.add_argument("--scene_id", required=True)
    parser.add_argument("--platforms", nargs="*", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--unreal_project_dir")
    args = parser.parse_args()
    
    assert os.path.exists(args.unreal_binaries_dir)
    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Win64", "UE4Editor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Win64", "UnrealPak.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Mac", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Mac", "UnrealPak"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Linux", "UE4Editor.sh"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Linux", "UnrealPak.sh"))
    
    if args.unreal_project_dir:
        unreal_project_dirs = [os.path.realpath(args.unreal_project_dir)]
    else:
        unreal_projects_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "cpp", "unreal_projects"))
        unreal_project_dirs = [ os.path.join(unreal_projects_dir, project) for project in os.listdir(unreal_projects_dir) ]
    
    for unreal_project_dir in unreal_project_dirs:

        assert os.path.exists(unreal_project_dir)
        _, project = os.path.split(unreal_project_dir)
        uproject   = os.path.realpath(os.path.join(unreal_project_dir, project + ".uproject"))

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
            content_dirs = [
                os.path.join(platform_dir, "Engine", "Content", "Animation"),
                os.path.join(platform_dir, "Engine", "Content", "BasicShapes"),
                os.path.join(platform_dir, "Engine", "Content", "EngineResources"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Math"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Texturing"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Utility"),
                os.path.join(platform_dir, "Engine", "Content", "Functions", "MaterialLayerFunctions"),
                os.path.join(platform_dir, project,  "Content")
            ]

            # create the output_dir
            os.makedirs(args.output_dir, exist_ok=True)
            
            # construct path to the output pak file
            pak_file = os.path.join(args.output_dir, f"Map_{args.scene_id}_{platform}.pak")

            # text file used to generate the final pak file
            txt_path = os.path.join(args.output_dir, f"Map_{args.scene_id}_{platform}.txt")

            for i, content_dir in enumerate(content_dirs):
                with open(txt_path, mode="w" if i==0 else "a") as f:
                    for content_file in glob.glob(os.path.join(content_dir, "**", "*.*"), recursive=True):
                        assert content_file.startswith(platform_dir)
                        content_file = content_file.replace('\\', "/")
                        mount_file = posixpath.join("..", "..", f"..{content_file.split(platform + 'NoEditor')[1]}")
                        f.write(f'"{content_file}" "{mount_file}" "" \n')

            # command to generate the final pak file
            cmd = [
                unreal_pak_bin,
                pak_file,
                "-create=" + txt_path,
                "-platform=" + platform,
                "-multiprocess",
                "-compress"
            ]
            print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
            cmd_result = subprocess.run(cmd, check=True)

            assert os.path.exists(pak_file)
            print(f"[SPEAR | build_pak_files.py] Successfully built {pak_file} for {platform} platform.")

    print("[SPEAR | build_pak_files.py] Done.")
