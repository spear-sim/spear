#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import sys
import time


parser = argparse.ArgumentParser()
parser.add_argument("--executable")
parser.add_argument("--graphics-adaptor")
parser.add_argument("--map")
parser.add_argument("--vk-icd-filenames")
parser.add_argument("--skip-override-benchmarking", action="store_true")
parser.add_argument("--skip-override-game-paused", action="store_true")
parser.add_argument("--config-files", nargs="*")
parser.add_argument("--pak-files", nargs="*")
args = parser.parse_args()


if __name__ == "__main__":

    if args.executable is not None:
        executable = args.executable
    else:
        if sys.platform == "win32":
            executable = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Standalone-Development", "Windows", "SpearSim.exe"))
        elif sys.platform == "darwin":
            executable = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Standalone-Development", "Mac", "SpearSim.app"))
        elif sys.platform == "linux":
            executable = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Standalone-Development", "Linux", "SpearSim.sh"))
        else:
            assert False

    if args.config_files is not None:
        user_config_files = [ os.path.realpath(os.path.join(os.path.dirname(__file__), config_file)) for config_file in args.config_files ]
    else:
        user_config_files = []

    # load config
    config = spear.get_config(user_config_files=user_config_files)

    # modify config params

    config.defrost()

    config.SPEAR.LAUNCH_MODE = "game"
    config.SPEAR.INSTANCE.GAME_EXECUTABLE = executable

    if not args.skip_override_benchmarking:
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_BENCHMARKING = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.BENCHMARKING = False

    if not args.skip_override_game_paused:
        config.SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.OVERIDE_GAME_PAUSED = True
        config.SP_SERVICES.INITIALIZE_GAME_WORLD_SERVICE.GAME_PAUSED = False

    if args.graphics_adaptor is not None:
        config.SPEAR.INSTANCE.COMMAND_LINE_ARGS.graphics_adaptor = args.graphics_adaptor

    if args.map is not None:
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_GAME_DEFAULT_MAP = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.GAME_DEFAULT_MAP = args.map

    if args.pak_files is not None:
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.MOUNT_PAK_FILES = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.PAK_FILES = args.pak_files

    if args.vk_icd_filenames is not None:
        config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES = args.vk_icd_filenames

    config.freeze()

    # configure system based on config settings
    spear.configure_system(config=config)

    # launch the executable
    instance = spear.Instance(config=config)
    while instance.is_running():
        time.sleep(1.0)
    instance.close(force=True)

    spear.log("Done.")
