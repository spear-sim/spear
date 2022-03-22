import argparse
import os

from interiorsim import InteriorSimEnv
from interiorsim.constants import PACKAGE_DEFAULT_CONFIG_FILE

if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("--config_files", nargs="*", help="List that contains locations to config files. If this argument is skipped, only default_config.yaml from unrealai package will be used to generate an output config file.")
    parser.add_argument("--output_unreal_project_dir", required=True, help="Location of the Unreal project directory where you want the generated config file written to.")

    args = parser.parse_args()

    # create a list of config files to be used
    config_files = []

    # add config file from input if valid
    if args.config_files:
        for file in args.config_files:
            config_files.append(file)
    
    # create a single CfgNode that contains data from all config files
    config_node = InteriorSimEnv.get_config(config_files)

    # Dump updated config params into a new yaml file
    output_temp_dir = os.path.join(args.output_unreal_project_dir, "Temp")
    if not os.path.exists(output_temp_dir):
        os.makedirs(output_temp_dir)

    output_config_file = os.path.join(output_temp_dir, "config.yaml")
    try:
        with open(output_config_file, "w") as output:
            config_node.dump(stream=output, default_flow_style=False)
    except EnvironmentError as e:
        raise Exception(e)
