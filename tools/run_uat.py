#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import shutil
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--build-target")
parser.add_argument("--unreal-project-dir")
parser.add_argument("--clean-archive-dir", action="store_true")
parser.add_argument("--skip-cook-default-maps", action="store_true")
parser.add_argument("--cook-dirs", nargs="*")
parser.add_argument("--cook-maps", nargs="*")
parser.add_argument("--build-config", default="Development")
args, unknown_args = parser.parse_known_args() # get unknown args to pass to RunUAT

assert os.path.exists(args.unreal_engine_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    if sys.platform == "win32":
        run_uat_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        target_platform = "Win64"

        cxx_compiler = "cl"
        cxx_compiler_path = shutil.which(cxx_compiler)
        if cxx_compiler_path is None:
            spear.log("ERROR: Can't find the Visual Studio command-line tools. All SPEAR build steps must run in a terminal where the Visual Studio command-line tools are visible. Giving up...")
            assert False
        if cxx_compiler_path.lower().endswith("hostx86\\x86\\cl.exe") or cxx_compiler_path.lower().endswith("hostx86\\x64\\cl.exe"):
            spear.log("ERROR: 32-bit terminal detected. All SPEAR build steps must run in a 64-bit terminal. Giving up...")
            spear.log("ERROR: Compiler path:", cxx_compiler_path)
            assert False

    elif sys.platform == "darwin":
        run_uat_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        target_platform = "Mac"
    elif sys.platform == "linux":
        run_uat_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.sh"))
        target_platform = "Linux"
    else:
        assert False

    # The Unreal Build Tool expects "~/.config/" to be owned by the user, so it can create and write to "~/.config/Unreal Engine/"
    # without requiring admin privileges. This check might seem esoteric, but we have seen cases where "~/.config/"
    # is owned by root in some corporate environments, so we choose to check it here as a courtesy to new
    # users. We don't know if we need a similar check on Windows.
    if sys.platform in ["darwin", "linux"]:
        config_dir = os.path.expanduser(os.path.join("~", ".config"))
        if os.path.exists(config_dir):
            import pwd # not available on Windows
            current_user = pwd.getpwuid(os.getuid()).pw_name
            config_dir_owner = pwd.getpwuid(os.stat(config_dir).st_uid).pw_name
            if current_user != config_dir_owner:
                spear.log(f"ERROR: The Unreal Build Tool expects {current_user} to be the owner of {config_dir}, but the current owner is {config_dir_owner}. To update, run the following command:")
                spear.log(f"    sudo chown {current_user} {config_dir}")
                assert False

    if args.unreal_project_dir is None:
        unreal_project_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
    else:
        unreal_project_dir = os.path.realpath(args.unreal_project_dir)

    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    uproject_name = os.path.splitext(os.path.split(uproject)[1])[0]
    archive_dir = os.path.realpath(os.path.join(unreal_project_dir, f"Standalone-{args.build_config}"))

    if args.build_target is None:
        build_target = uproject_name
    else:
        build_target = args.build_target

    if args.clean_archive_dir and os.path.exists(archive_dir):
        spear.log("Archive directory exists, removing: ", archive_dir)
        shutil.rmtree(archive_dir, ignore_errors=True)

    # assemble dirs to cook

    cook_dirs = []
    if args.cook_dirs is not None:
        cook_dirs = args.cook_dirs

    cook_dir_args = [ f'-cookdir="{os.path.join(unreal_project_dir, cook_dir)}"' for cook_dir in cook_dirs ]

    # assemble maps to cook

    cook_maps = []
    if args.unreal_project_dir is None and not args.skip_cook_default_maps:
        cook_maps.extend(spear.utils.tool_utils.get_default_maps_to_cook())
    if args.cook_maps is not None:
        cook_maps.extend(args.cook_maps)

    if len(cook_maps) == 0:
        cook_maps_arg = []
    else:
        cook_maps_arg = [f"-map={'+'.join(cook_maps)}"]

    # build project
    cmd = [
        run_uat_script,
        "BuildCookRun",
        f'-project="{uproject}"',
        f"-target={build_target}",
        f"-targetplatform={target_platform}",
        f"-clientconfig={args.build_config}",
        f'-archivedirectory="{archive_dir}"'] + \
        unknown_args + \
        cook_dir_args + \
        cook_maps_arg
    spear.log("Executing: ", " ".join(cmd))
    subprocess.run(cmd, check=True)

    spear.log("Done.")
