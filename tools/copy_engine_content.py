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
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
args = parser.parse_args()

assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.unreal_project_dir)


if __name__ == "__main__":

    def realpath(*args):
        return os.path.realpath(os.path.join(*args))

    content_dirs = {
        realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "Characters", "Content"):       realpath(args.unreal_project_dir, "Content", "Characters"),
        realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "Input", "Content"):            realpath(args.unreal_project_dir, "Content", "Input"),
        realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "High", "LevelPrototyping", "Content"): realpath(args.unreal_project_dir, "Content", "LevelPrototyping"),
        realpath(args.unreal_engine_dir, "Templates", "TemplateResources", "Standard", "Vehicles", "Content"):     realpath(args.unreal_project_dir, "Content", "Vehicles"),

        realpath(args.unreal_engine_dir, "Templates", "TP_FirstPersonBP", "Content", "FirstPerson"):               realpath(args.unreal_project_dir, "Content", "FirstPerson"),
        realpath(args.unreal_engine_dir, "Templates", "TP_FirstPersonBP", "Content", "__ExternalActors__"):        realpath(args.unreal_project_dir, "Content", "__ExternalActors__"),
        realpath(args.unreal_engine_dir, "Templates", "TP_FirstPersonBP", "Content", "__ExternalObjects__"):       realpath(args.unreal_project_dir, "Content", "__ExternalObjects__"),

        realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "ThirdPerson"):               realpath(args.unreal_project_dir, "Content", "ThirdPerson"),
        realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "__ExternalActors__"):        realpath(args.unreal_project_dir, "Content", "__ExternalActors__"),
        realpath(args.unreal_engine_dir, "Templates", "TP_ThirdPersonBP", "Content", "__ExternalObjects__"):       realpath(args.unreal_project_dir, "Content", "__ExternalObjects__"),

        # realpath(args.unreal_engine_dir, "Templates", "TP_UEIntro_BP", "Content", "DemoTemplate"):                 realpath(args.unreal_project_dir, "Content", "DemoTemplate"), /Game/DemoTemplate/_Core/Lvl_IntroRoom doesn't work in standalone builds

        realpath(args.unreal_engine_dir, "Templates", "TP_VehicleAdvBP", "Content", "VehicleTemplate"):            realpath(args.unreal_project_dir, "Content", "VehicleTemplate"),
        realpath(args.unreal_engine_dir, "Templates", "TP_VehicleAdvBP", "Content", "__ExternalActors__"):         realpath(args.unreal_project_dir, "Content", "__ExternalActors__"),
        realpath(args.unreal_engine_dir, "Templates", "TP_VehicleAdvBP", "Content", "__ExternalObjects__"):        realpath(args.unreal_project_dir, "Content", "__ExternalObjects__")}

    for unreal_engine_dir, project_dir in content_dirs.items():
        if os.path.exists(project_dir):
            spear.log("Directory exists, removing: ", project_dir)
            shutil.rmtree(project_dir)

    for unreal_engine_dir, project_dir in content_dirs.items():
        spear.log(f"Copying: {unreal_engine_dir} -> {project_dir}")
        shutil.copytree(unreal_engine_dir, project_dir, dirs_exist_ok=True)

    spear.log("Done.")
