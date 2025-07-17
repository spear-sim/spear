#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--skip_cook_default_maps", action="store_true")
    parser.add_argument("--cook_dirs", nargs="*")
    parser.add_argument("--cook_maps", nargs="*")
    parser.add_argument("--build_config", default="Development")
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    args, unknown_args = parser.parse_known_args() # get remaining args to pass to RunUAT

    assert os.path.exists(args.unreal_engine_dir)
    assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]

    if sys.platform == "win32":
        run_uat_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "RunUAT.bat"))
        target_platform = "Win64"
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

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    uproject_name = os.path.splitext(os.path.split(uproject)[1])[0]
    archive_dir = os.path.realpath(os.path.join(unreal_project_dir, f"Standalone-{args.build_config}"))

    # assemble dirs to cook

    cook_dirs = []
    if args.cook_dirs is not None:
        cook_dirs = args.cook_dirs

    cook_dir_args = [ f'-cookdir="{os.path.join(unreal_project_dir, cook_dir)}"' for cook_dir in cook_dirs ]

    # assemble maps to cook

    cook_maps = []
    if not args.skip_cook_default_maps:
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
        f"-target={uproject_name}",
        f"-targetplatform={target_platform}",
        f"-clientconfig={args.build_config}",
        f'-archivedirectory="{archive_dir}"'] + \
        unknown_args + \
        cook_dir_args + \
        cook_maps_arg
    spear.log("Executing: ", ' '.join(cmd))
    subprocess.run(cmd, check=True)

    spear.log("Done.")
