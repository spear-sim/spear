#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--build-target")
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
parser.add_argument("--clean-archive-dir", action="store_true")
parser.add_argument("--skip-cook-default-maps", action="store_true")
parser.add_argument("--cook-dirs", nargs="*")
parser.add_argument("--cook-maps", nargs="*")
parser.add_argument("--build-config", default="Development")
args, unknown_args = parser.parse_known_args() # get unknown args to pass to RunUAT

assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.unreal_project_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    spear.tools.validate_build_environment()

    target_platform = spear.tools.get_target_platform()

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    spear_sim_unreal_project_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))

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
    cook_dirs = args.cook_dirs if args.cook_dirs is not None else []
    cook_dir_args = [ f"-cookdir={spear.tools.normalize_unreal_cmd_path(path=os.path.join(unreal_project_dir, cook_dir))}" for cook_dir in cook_dirs ]

    # assemble maps to cook; only cook default maps if we're building the default SpearSim project and the caller hasn't opted out
    cook_maps = []
    if unreal_project_dir == spear_sim_unreal_project_dir and not args.skip_cook_default_maps:
        cook_maps.extend(spear.tools.get_default_maps_to_cook())
    if args.cook_maps is not None:
        cook_maps.extend(args.cook_maps)
    cook_maps_arg = [] if len(cook_maps) == 0 else [f"-map={'+'.join(cook_maps)}"]

    cook_args = cook_dir_args + cook_maps_arg

    # build project
    run_uat_args = [
        "BuildCookRun",
        f"-project={spear.tools.normalize_unreal_cmd_path(path=uproject)}",
        f"-target={build_target}",
        f"-targetplatform={target_platform}",
        f"-clientconfig={args.build_config}",
        f"-archivedirectory={spear.tools.normalize_unreal_cmd_path(path=archive_dir)}"] + \
        unknown_args + \
        cook_args

    spear.tools.run_uat(unreal_engine_dir=args.unreal_engine_dir, args=run_uat_args)

    spear.log("Done.")
