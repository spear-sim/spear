#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_binaries_dir", required=True)
    parser.add_argument("--unreal_project_dir", required=True)
    parser.add_argument("--scene_id", required=True)
    parser.add_argument("--platforms", nargs="*", required=True, help="Supported platform values are Windows, Linux, and Mac")
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)
    
    assert os.path.exists(args.unreal_binaries_dir)
    unreal_editor_exe = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Win64", "UE4Editor.exe"))
    unreal_pak_exe    = os.path.realpath(os.path.join(args.unreal_binaries_dir, "Win64", "UnrealPak.exe"))
    
    assert os.path.exists(args.unreal_project_dir)
    _, project = os.path.split(args.unreal_project_dir)
    uproject   = os.path.realpath(os.path.join(args.unreal_project_dir, project + ".uproject"))
    
    for platform in args.platforms:
        cmd = [ 
            unreal_editor_exe,
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
        cmd_result = subprocess.run(cmd, stdout=sys.stdout.buffer ,stderr=sys.stdout.buffer)
        assert cmd_result.returncode == 0

        content_dirs = [
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", project,  "Content")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "Animation")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "BasicShapes")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Texturing")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Math")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "Functions", "MaterialLayerFunctions")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "Functions", "Engine_MaterialFunctions02", "Utility")),
            os.path.realpath(os.path.join(args.unreal_project_dir, "Saved", "Cooked", f"{platform}NoEditor", "Engine", "Content", "EngineResources"))
        ]

        # construct output pak file path
        output_pak = os.path.join(args.output_dir, f"Map_{args.scene_id}_{platform}.pak")

        # temporary text file used by unreal_pak command to generate final pak file
        pak_txt_path = os.path.join(args.output_dir, f"pak_data_{platform}.txt")

        if os.path.exists(pak_txt_path):
            os.remove(pak_txt_path)

        for dir in content_dirs:
            with open(pak_txt_path, mode='a') as fout:
                for fpath in glob.glob(dir + "\**\*.*", recursive=True):
                    fpath = fpath.replace('\\', "/")
                    mount_path = fpath.split(f"{platform}NoEditor")[1]
                    fout.write(f'"{fpath}" "../../..{mount_path}" "" \n')

        # unreal pak command to generate the final pak file
        cmd = [
            unreal_pak_exe,
            output_pak,
            "-create=" + pak_txt_path,
            "-platform=" + platform,
            "-multiprocess",
            "-compress"
        ]

        print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

        if os.path.exists(output_pak):
            print(f"[SPEAR | build_pak_files.py] Successfully built {output_pak}.")
        else:
            print(f"[SPEAR | build_pak_files.py] Could not build {output_pak} for {platform} platform.")

        # remove the temporary text file
        os.remove(pak_txt_path)
