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
parser.add_argument("--config-file")
parser.add_argument("--graphics-adaptor")
parser.add_argument("--map")
parser.add_argument("--vk-icd-filenames")
parser.add_argument("--editor-launch-mode", default="editor")
parser.add_argument("--temp-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to editor

assert os.path.exists(args.unreal_engine_dir)


if __name__ == "__main__":

    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
    else:
        assert False

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]

    if args.config_file is not None:
        user_config_files = [os.path.realpath(os.path.join(os.path.dirname(__file__), args.config_file))]
    else:
        user_config_files = []

    # load config
    config = spear.get_config(user_config_files=user_config_files)

    # modify config params
    config.defrost()

    # modify config params
    config.defrost()

    config.SPEAR.LAUNCH_MODE = "editor"
    config.SPEAR.INSTANCE.EDITOR_EXECUTABLE = unreal_editor_bin
    config.SPEAR.INSTANCE.EDITOR_UPROJECT = uproject
    config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE = args.editor_launch_mode

    if args.graphics_adaptor is not None:
        config.SPEAR.INSTANCE.COMMAND_LINE_ARGS.graphics_adaptor = args.graphics_adaptor

    if args.map is not None:
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_EDITOR_STARTUP_MAP = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EDITOR_STARTUP_MAP = args.map

    if args.vk_icd_filenames is not None:
        config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES = args.vk_icd_filenames

    config.freeze()

    # configure system based on config settings
    spear.configure_system(config=config)

    # launch the executable
    instance = spear.Instance(config=config)
    while instance.is_running():
        time.sleep(1.0)
    instance.close()

    spear.log("Done.")
