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

    content_dirs = {
        os.path.realpath(os.path.join(args.unreal_engine_dir, "Samples", "StarterContent", "Content", "StarterContent")): \
            os.path.realpath(os.path.join(args.unreal_project_dir, "Content", "StarterContent")),
        os.path.realpath(os.path.join(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "Characters", "Content", "Mannequins")): \
            os.path.realpath(os.path.join(args.unreal_project_dir, "Content", "Characters", "Mannequins"))}

    for unreal_engine_dir, project_dir in content_dirs.items():

        if os.path.exists(project_dir):
            spear.log(f"Directory exists, removing: {project_dir}")
            shutil.rmtree(project_dir)

        spear.log(f"Copying: {unreal_engine_dir} -> {project_dir}")
        shutil.copytree(unreal_engine_dir, project_dir)

    spear.log("Done.")
