#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--paks_dir")
    parser.add_argument("--scene_id")
    parser.add_argument("--map_id")
    parser.add_argument("--vk_icd_filenames")
    parser.add_argument("--gpu_id")
    args = parser.parse_args()

    assert os.path.exists(args.executable)

    # load config
    config = spear.get_config(user_config_files=[])

    # modify config params
    config.defrost()
    config.SPEAR.INSTANCE.LAUNCH_MODE = "standalone"
    config.SPEAR.INSTANCE.STANDALONE = args.executable
    if args.scene_id is not None:
        config.SP_ENGINE.LEGACY_SERVICE.SCENE_ID = args.scene_id
    if args.map_id is not None:
        config.SP_ENGINE.LEGACY_SERVICE.MAP_ID = args.map_id
    if args.paks_dir is not None:
        config.SPEAR.INSTANCE.PAKS_DIR = args.paks_dir
    if args.vk_icd_filenames is not None:
        config.SPEAR.VK_ICD_FILENAMES = args.vk_icd_filenames
    if args.gpu_id is not None:
        config.SPEAR.INSTANCE.COMMAND_LINE_ARGS.graphics_adaptor = args.gpu_id
    config.freeze()

    # configure system based on config settings
    spear.configure_system(config)

    # launch the executable
    spear.Instance(config)

    spear.log("Done.")
