#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import ntpath
import os
import pandas as pd
import posixpath
import shutil
import spear
import spear.tool_utils
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--include_assets_file", required=True)
    parser.add_argument("--pak_file", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--cook_dirs_file")
    parser.add_argument("--cook_maps_file")
    parser.add_argument("--exclude_assets_file")
    parser.add_argument("--skip_cook_default_maps", action="store_true")
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)
    assert os.path.exists(args.include_assets_file)

    if args.cook_dirs_file is not None:
        assert os.path.exists(args.cook_dirs_file)

    if args.exclude_assets_file is not None:
        assert os.path.exists(args.exclude_assets_file)

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

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    unreal_project_cooked_dir = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", platform))
    unreal_project_cooked_dir_posix = unreal_project_cooked_dir.replace(ntpath.sep, posixpath.sep)

    shutil.rmtree(unreal_project_cooked_dir, ignore_errors=True)

    # assemble dirs to cook

    cook_dirs = []
    if args.cook_dirs_file is not None:
        cook_dirs.extend(pd.read_csv(args.cook_dirs_file)["cook_dirs"].tolist())

    cook_dir_args = [ f'-cookdir="{os.path.join(args.unreal_project_dir, cook_dir)}"' for cook_dir in cook_dirs ]

    # assemble maps to cook

    cook_maps = []
    if not args.skip_cook_default_maps:
        cook_maps.extend(spear.tool_utils.get_default_maps_to_cook())
    if args.cook_maps_file is not None:
        cook_maps.extend(pd.read_csv(args.cook_maps_file)["cook_maps"].tolist())

    if len(cook_maps) == 0:
        cook_maps_arg = []
    else:
        cook_maps_arg = [f"-map={'+'.join(cook_maps)}"]

    # cook project, see https://docs.unrealengine.com/5.2/en-US/SharingAndReleasing/Deployment/Cooking for more details
    cmd = [
        unreal_editor_bin,
        uproject,
        "-run=Cook",
        f"-targetplatform={platform}",
        "-unattended",                           # don't require any user input
        "-fileopenlog",                          # generate a log of which files are opened in which order
        "-ddc=InstalledDerivedDataBackendGraph", # use the default cache location for installed (i.e., not source) builds of the engine
        "-unversioned",                          # save all of the cooked packages without versions
        "-stdout",                               # ensure log output is written to the terminal 
        "-fullstdoutlogoutput",                  # ensure log output is written to the terminal, -nologtimes means don't print timestamps next to log messages twice
        "-nologtimes"] + \
        cook_dir_args + \
        cook_maps_arg
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)

    # create manifest file
    pak_file = os.path.realpath(os.path.join(args.pak_file))
    pak_dir = os.path.split(pak_file)[0]
    os.makedirs(pak_dir, exist_ok=True)

    include_assets = pd.read_csv(args.include_assets_file)["include_assets"]

    exclude_assets = []
    if args.exclude_assets_file is not None:
        exclude_assets = pd.read_csv(args.exclude_assets_file)["exclude_assets"].tolist() # need tolist() so we can test for membership using "in" keyword below

    # create manifest file in output dir
    manifest_file = f"{os.path.splitext(pak_file)[0]}.txt"
    with open(manifest_file, "w") as f:
        for include_asset in include_assets:
            include_asset_path = os.path.realpath(os.path.join(unreal_project_cooked_dir, include_asset))
            for asset_path in sorted(glob.glob(include_asset_path, recursive=True)):
                asset_path_posix = asset_path.replace(ntpath.sep, posixpath.sep)
                assert asset_path.startswith(unreal_project_cooked_dir_posix)
                asset_path_posix_suffix = asset_path_posix.removeprefix(unreal_project_cooked_dir_posix + posixpath.sep)
                if asset_path_posix_suffix not in exclude_assets:
                    mount_asset_path_posix = posixpath.join("..", "..", "..", asset_path_posix_suffix)
                    f.write(f'"{asset_path_posix}" "{mount_asset_path_posix}" "" \n')

    # build pak file
    cmd = [unreal_pak_bin, pak_file, f'-create="{manifest_file}"', f"-platform={platform}", "-multiprocess", "-compressed"]
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)

    assert os.path.exists(pak_file)
    spear.log("Successfully built: ", pak_file)

    spear.log("Done.")
