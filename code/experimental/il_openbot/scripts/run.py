#############################################################################

# Quick and dirty script to issue random commands to an OpenBot agent and get
# egocentric visual obsevations.

#############################################################################

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
#
#
# python run.py -i 10 -r 3 -s "Data" -m "235114801" "235114775"
# python run.py -i 10 -r 3 -s "Infer" -m "235114801" "235114775"
# python run.py -i 10 -r 3 -s "Debug" -m "235114801" "235114775"

import argparse
import os
from behavior_policy import BehaviorPolicy

from interiorsim.config import get_config

DIR_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")


def get_args():
    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--create_video", action="store_true",
                        help="create a video out of the observations.")
    parser.add_argument("-i", "--iterations", type=int,
                        help="number of iterations through the environment",
                        default=1000)
    parser.add_argument("-r", "--runs", type=int,
                        default=5,
                        help="number of distinct runs in the considered environment")
    parser.add_argument("-s", "--setup", type=str,
                        default="Data",
                        help="Data (for data collection), Infer (for ANN inference)")
    parser.add_argument("-p", "--policy", type=str,
                        help="1_env, 5_envs,  25_envs, 50_envs or real_envs",
                        required=False)
    parser.add_argument("-m", "--map", nargs="+", default=["237081739"],
                        help="Array of map references. A number of s distinct "
                        + "runs will be executed in each map. This argument "
                        + "overwrites the MAP_ID argument and allows "
                        + "collecting data in multiple environments programatically",
                        required=False)
    parser.add_argument("-c", "--connect", type=int,
                        default=2,
                        help="id of the connection port")
    parser.add_argument("-d", "--debug", type=bool,
                        default=False,
                        help="Running in debug mode, where images can be seen")

    args = parser.parse_args()
    return args


def get_config_setup(connect):

    # List of config files to be used
    config_files = []

    # Add default config files first and then user config files
    config_files.append(os.path.join(DIR_PATH, "scripts/user_config.yaml"))

    # Load configs
    config = get_config(config_files)
    config.defrost()
    config.SIMULATION_CONTROLLER.PORT = 30000 + connect
    config.freeze()
    return config


def get_map_names(maps, config):

    if maps == [""]:  # Use the default MAP_ID argument from parameter file
        if config.INTERIORSIM.MAP_ID == "":
            map_names = ["simpleMap"]
        elif config.INTERIORSIM.MAP_ID[0:15] == "/Game/Maps/Map_":
            map_names = [config.INTERIORSIM.MAP_ID[15:]]
        else:
            map_names = ["errorMap"]
    else:  # Overwrite the map value
        map_names = maps
    return map_names


if __name__ == "__main__":

    args = get_args()

    config = get_config_setup(args.connect)

    # Arguments
    map_names = get_map_names(args.map, config)
    learning_mode = args.setup
    runs = args.runs
    policy_path = "/Users/yuankai/code/github/interiorsim/code/experimental/il_openbot/models/TestSession_1_pilot_net_lr0.0001_bz512_bn/checkpoints/best-val.tflite"

    # Policy object
    policy = BehaviorPolicy(config=config,
                            dataset_path=DIR_PATH,
                            debug_mode=args.debug,
                            num_iter=args.iterations,
                            map_names=map_names,
                            run_mode=args.setup,
                            runs=args.runs,
                            policy_path=policy_path)

    policy.perform_run_in_all_maps()
