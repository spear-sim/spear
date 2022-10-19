# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
from random import random
import cv2
import ntpath
import numpy as np
import os
import pandas as pd
import shutil
import sys

from interiorsim import Env
from interiorsim.config import get_config


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--pak_file", type=str, required=True)
    parser.add_argument("--executable_content_dir", type=str, required=True)
    parser.add_argument("--num_images", type=int, default=10)
    parser.add_argument("--poses_file", type=str, required=True)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    # reset seed
    np.random.seed(0)

    # copy pak to the executable dir as this is required for launching the appropriate pak file
    pak_file_name = ntpath.basename(args.pak_file)
    if not os.path.exists(f"{args.executable_content_dir}/Paks/{pak_file_name}"):
        shutil.copy(args.pak_file, f"{args.executable_content_dir}/Paks")

    # create Env object
    env = Env(config)

    # reset the simulation
    _ = env.reset()

    # get random positions based on number of images requested
    _, _, _, step_info = env.step({"set_pose":[0,0,0,0,0,0], "set_num_random_points": [args.num_images]})
    random_positions = step_info["agent_controller_step_info"]["random_points"]

    # genearte random arrays for orientation
    random_pitch_values = np.random.uniform(low=0.0, high=0.0, size=random_positions.shape[0])
    random_yaw_values   = np.random.uniform(low=-180.0, high=180.0, size=random_positions.shape[0])
    random_roll_values  = np.random.uniform(low=0.0, high=0.0, size=random_positions.shape[0])

    # store poses in a csv file
    df_objects = pd.DataFrame({"map_id"    : f"{ntpath.basename(config.INTERIORSIM.MAP_ID)}",
                               "pos_x_cms" : random_positions[:, 0],
                               "pos_y_cms" : random_positions[:, 1],
                               "pos_z_cms" : random_positions[:, 2] + config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,
                               "pitch_degs": random_pitch_values,
                               "yaw_degs"  : random_yaw_values,
                               "roll_degs" : random_roll_values})
    df_objects.to_csv(args.poses_file, index=False)

    # close the current scene
    env.close()
    
    # remove copied pak file from exectuable dir's Content folder as it is no longer required
    os.remove(f"{args.executable_content_dir}/Paks/{pak_file_name}")
    