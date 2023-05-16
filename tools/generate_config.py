#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--user_config_files", nargs="*")
    parser.add_argument("--output_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp", "config.yaml")))
    args = parser.parse_args()

    if args.user_config_files:
        user_config_files = args.user_config_files
    else:
        user_config_files = []

    # create a single CfgNode that contains data from all config files
    config = spear.get_config(user_config_files=user_config_files)

    # dump config params into a new yaml file
    output_dir = os.path.realpath(os.path.dirname(args.output_file))
    os.makedirs(output_dir, exist_ok=True)

    with open(args.output_file, "w") as output:
        config.dump(stream=output, default_flow_style=False)

    spear.log("Generated config file: " + args.output_file)
    spear.log("Done.")
