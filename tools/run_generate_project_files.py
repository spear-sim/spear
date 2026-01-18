#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
args = parser.parse_args()

assert os.path.exists(args.unreal_engine_dir)


if __name__ == "__main__":

    if sys.platform == "win32":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Build.bat"))
        generate_project_files_args = "-projectfiles"
    elif sys.platform == "darwin":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Mac", "GenerateProjectFiles.sh"))
        generate_project_files_args = ""
    elif sys.platform == "linux":
        generate_project_files_script = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Build", "BatchFiles", "Linux", "GenerateProjectFiles.sh"))
        generate_project_files_args = ""
    else:
        assert False

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]

    cmd = f'{generate_project_files_script} -project="{uproject}" {generate_project_files_args}'
    spear.log("Executing: ", cmd)
    subprocess.run(cmd, shell=True, check=True)

    spear.log("Done.")
