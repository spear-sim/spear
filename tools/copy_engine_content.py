#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--unreal-project-dir", default=spear.path.realpath(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
args = parser.parse_args()

assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.unreal_project_dir)


if __name__ == "__main__":

    content_dirs = {
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "__ExternalActors__", "ThirdPerson"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "__ExternalActors__", "ThirdPerson"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "__ExternalObjects__", "ThirdPerson"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "__ExternalObjects__", "ThirdPerson"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "Characters", "Content", "Mannequins"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "Characters", "Mannequins"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "LevelPrototyping", "Content"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "LevelPrototyping"),
        spear.path.realpath(args.unreal_engine_dir, "Samples", "StarterContent", "Content", "StarterContent"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "StarterContent"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "ThirdPerson"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "ThirdPerson"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "Standard", "Vehicles", "Content"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "Vehicles"),
        spear.path.realpath(args.unreal_engine_dir, "Templates", "TP_VehicleAdvBP", "Content", "VehicleTemplate"): \
            spear.path.realpath(args.unreal_project_dir, "Content", "VehicleTemplate")}

    for unreal_engine_dir, project_dir in content_dirs.items():

        if os.path.exists(project_dir):
            spear.log("Directory exists, removing: ", project_dir)
            shutil.rmtree(project_dir)

        spear.log(f"Copying: {unreal_engine_dir} -> {project_dir}")
        shutil.copytree(unreal_engine_dir, project_dir)

    spear.log("Done.")
