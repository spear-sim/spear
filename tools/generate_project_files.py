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
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)

    if sys.platform == "win32":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Windows", "GenerateProjectFiles.bat"))
    elif sys.platform == "darwin":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Mac", "GenerateProjectFiles.sh"))
    elif sys.platform == "linux":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Linux", "GenerateProjectFiles.sh"))
    else:
        assert False

    project = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "SpearSim.uproject"))

    cmd = [generate_project_files_script, "-project=" + project]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Done.")
