import argparse
import os

from interiorsim import config

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("--user_config_files", nargs="*")
    parser.add_argument("--output_unreal_project_dir", required=True)

    args = parser.parse_args()

    # create a list of config files to be used
    config_files = []

    # add config file from input if valid
    if args.user_config_files:
        for file in args.user_config_files:
            config_files.append(file)
    
    # create a single CfgNode that contains data from all config files
    config_node = config.get_config(config_files)

    # Dump updated config params into a new yaml file
    output_temp_dir = os.path.join(args.output_unreal_project_dir, "Temp")
    if not os.path.exists(output_temp_dir):
        os.makedirs(output_temp_dir)

    output_config_file = os.path.join(output_temp_dir, "config.yaml")
    with open(output_config_file, "w") as output:
        config_node.dump(stream=output, default_flow_style=False)

    print("Config file generated successfully: " + output_config_file)
