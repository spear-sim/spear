#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import time


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--paks_dir")
    parser.add_argument("--version_tag")
    parser.add_argument("--scene_id")
    parser.add_argument("--vk_icd_filenames")
    parser.add_argument("--graphics_adaptor")
    args = parser.parse_args()

    assert os.path.exists(args.executable)

    # load config
    config = spear.get_config(user_config_files=[])

    # modify config params
    config.defrost()
    config.SPEAR.LAUNCH_MODE = "standalone"
    config.SPEAR.STANDALONE_EXECUTABLE = args.executable
    if args.paks_dir is not None:
        config.SPEAR.PAKS_DIR = args.paks_dir
    if args.version_tag is not None:
        config.SPEAR.PAKS_VERSION_TAG = args.version_tag
    if args.scene_id is not None:
        config.SP_SERVICES.GAME_DEFAULT_MAP = "/Game/Scenes/" + args.scene_id + "/Maps/" + args.scene_id
    if args.vk_icd_filenames is not None:
        config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES = args.vk_icd_filenames
    if args.graphics_adaptor is not None:
        config.SPEAR.INSTANCE.COMMAND_LINE_ARGS.graphics_adaptor = args.graphics_adaptor
    config.freeze()

    # configure system based on config settings
    spear.configure_system(config)

    # launch the executable
    instance = spear.Instance(config)
    while instance.is_running():
        time.sleep(1.0)
    instance.close()

    spear.log("Done.")
