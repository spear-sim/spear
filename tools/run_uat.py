#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--build_config", required=True)
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

    project = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "SpearSim.uproject"))
    archive_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Standalone-" + args.build_config))

    cmd = [
        run_uat_script,
        "BuildCookRun",
        "-project=" + project,
        "-target=SpearSim",
        "-targetPlatform=" + target_platform,
        "-clientConfig=" + args.build_config,
        "-archivedirectory=" + archive_dir] + unknown_args # append remaining args to pass to RunUAT
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Done.")
