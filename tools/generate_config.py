import argparse
import os
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--user_config_files", nargs="*")
    parser.add_argument("--unreal_project_dir", required=True)
    args = parser.parse_args()

    # create a list of config files to be used
    if args.user_config_files:
        user_config_files = args.user_config_files
    else:
        user_config_files = []
    
    # create a single CfgNode that contains data from all config files
    config_node = spear.get_config(user_config_files=user_config_files)

    # dump updated config params into a new yaml file
    output_temp_dir = os.path.realpath(os.path.join(args.unreal_project_dir, "Temp"))
    if not os.path.exists(output_temp_dir):
        os.makedirs(output_temp_dir)

    output_config_file = os.path.join(output_temp_dir, "config.yaml")
    with open(output_config_file, "w") as output:
        config_node.dump(stream=output, default_flow_style=False)

    print("Config file generated successfully: " + output_config_file)
    print()

    print("Done.")
