#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)
    assert os.path.exists(args.unreal_project_dir)

    unreal_engine_starter_content_dir = os.path.realpath(os.path.join(args.unreal_engine_dir, "Samples", "StarterContent", "Content", "StarterContent"))
    project_starter_content_dir = os.path.join(args.unreal_project_dir, "Content", "StarterContent") # don't want os.path.realpath here in case it is a symlink

    if spear.path_exists(project_starter_content_dir):
        spear.log(f"File or directory or symlink exists, removing: {project_starter_content_dir}")
        spear.remove_path(project_starter_content_dir)

    spear.log(f"Copying: {unreal_engine_starter_content_dir} -> {project_starter_content_dir}")
    shutil.copytree(unreal_engine_starter_content_dir, project_starter_content_dir)

    spear.log("Done.")
