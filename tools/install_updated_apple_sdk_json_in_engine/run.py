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
args = parser.parse_args()

assert os.path.exists(args.unreal_engine_dir)


if __name__ == "__main__":

    apple_sdk_json_file = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Config", "Apple", "Apple_SDK.json"))
    apple_sdk_json_bak_file = apple_sdk_json_file + ".bak"
    new_apple_sdk_json_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "Apple_SDK.json"))

    assert os.path.exists(apple_sdk_json_file)

    if not os.path.exists(apple_sdk_json_bak_file):
        spear.log("Creating backup: ", apple_sdk_json_bak_file)
        shutil.copy(apple_sdk_json_file, apple_sdk_json_bak_file)
    else:
        spear.log("Backup already exists, skipping: ", apple_sdk_json_bak_file)

    spear.log("Installing updated Apple_SDK.json: ", apple_sdk_json_file)
    shutil.copy(new_apple_sdk_json_file, apple_sdk_json_file)

    spear.log("Done.")
