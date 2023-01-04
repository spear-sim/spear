#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_project_dir")
    parser.add_argument("--user_config_files", nargs="*")
    args = parser.parse_args()

    tools_dir           = os.path.dirname(os.path.realpath(__file__))
    unreal_projects_dir = os.path.realpath(os.path.join(tools_dir, "..", "cpp", "unreal_projects"))

    if args.unreal_project_dir:
        unreal_project_dirs = [ os.path.realpath(args.unreal_project_dir) ]
    else:
        unreal_project_dirs = [ os.path.join(unreal_projects_dir, project) for project in os.listdir(unreal_projects_dir) ]

    if args.user_config_files:
        user_config_files = args.user_config_files
    else:
        user_config_files = []

    # create a single CfgNode that contains data from all config files
    config_node = spear.get_config(user_config_files=user_config_files)

    # for each project...
    for unreal_project_dir in unreal_project_dirs:

        assert os.path.exists(unreal_project_dir)

        # ...if the project is a valid project (i.e., project dir has a uproject file)
        _, project = os.path.split(unreal_project_dir)
        uproject = os.path.join(unreal_project_dir, project + ".uproject")
        if os.path.exists(uproject):

            print(f"[SPEAR | generate_config.py] Found uproject: {uproject}")

            # dump config params into a new yaml file
            output_temp_dir = os.path.realpath(os.path.join(unreal_project_dir, "Temp"))
            os.makedirs(output_temp_dir, exist_ok=True)

            output_config_file = os.path.join(output_temp_dir, "config.yaml")
            with open(output_config_file, "w") as output:
                config_node.dump(stream=output, default_flow_style=False)

            print("[SPEAR | generate_config.py] Generated config file: " + output_config_file)

        print()

    print("[SPEAR | generate_config.py] Done.")
