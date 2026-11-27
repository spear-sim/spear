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
parser.add_argument("--unreal-project-dir", default=spear.path.realpath(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
args = parser.parse_args()

assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.unreal_project_dir)


if __name__ == "__main__":

    spear.tools.validate_build_environment()

    unreal_project_dir = spear.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(spear.path.realpath(unreal_project_dir, "*.uproject"))
    assert len(uprojects) == 1
    uproject = uprojects[0]

    generate_project_files_args = [
        f"-project={spear.tools.normalize_unreal_cmd_path(path=uproject)}"]

    spear.tools.run_generate_project_files(unreal_engine_dir=args.unreal_engine_dir, args=generate_project_files_args)

    spear.log("Done.")
