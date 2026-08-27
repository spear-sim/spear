#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--build-target")
parser.add_argument("--unreal-project-dir")
parser.add_argument("--build-config", default="Development")
args, unknown_args = parser.parse_known_args() # get unknown args to pass to RunUAT

assert os.path.exists(args.unreal_engine_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    spear.tools.validate_build_environment()

    target_platform = spear.tools.get_target_platform()

    if args.unreal_project_dir is None:
        unreal_project_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
    else:
        unreal_project_dir = os.path.realpath(args.unreal_project_dir)

    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]
    uproject_name = os.path.splitext(os.path.split(uproject)[1])[0]

    if args.build_target is None:
        build_target = uproject_name
    else:
        build_target = args.build_target

    # build project
    build_args = [
        build_target,
        target_platform,
        args.build_config,
        uproject] + \
        unknown_args

    spear.tools.run_build(unreal_engine_dir=args.unreal_engine_dir, args=build_args)

    spear.log("Done.")
