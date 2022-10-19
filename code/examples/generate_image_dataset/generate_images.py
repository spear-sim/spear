# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
from operator import index
import cv2
import ntpath
import os
import pandas as pd
import shutil

from interiorsim import Env
from interiorsim.config import get_config


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--pak_file", type=str, required=True)
    parser.add_argument("--executable_content_dir", type=str, required=True)
    parser.add_argument("--poses_file", type=str, required=True)
    parser.add_argument("--output_dir", "-o", type=str, required=True)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    # copy pak to the executable dir as this is required for launching the appropriate pak file
    pak_file_name = ntpath.basename(args.pak_file)
    if not os.path.exists(f"{args.executable_content_dir}/Paks/{pak_file_name}"):
        shutil.copy(args.pak_file, f"{args.executable_content_dir}/Paks")

    # create dir for storing images
    for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
        if not os.path.exists(os.path.join(args.output_dir, f"{render_pass}")):
            os.makedirs(os.path.join(args.output_dir, f"{render_pass}"))

    # read poses for this scene
    df = pd.read_csv(args.poses_file)

    # create Env object
    env = Env(config)

    # reset the simulation
    _ = env.reset()

    # iterate over these positions can capture images at each position
    for record in df.to_records():

        # set the pose and obtain corresponding images
        obs, _, _, _ = env.step({"set_pose": [record["pos_x_cms"], record["pos_y_cms"], record["pos_z_cms"], record["pitch_degs"], record["yaw_degs"], record["roll_degs"]], "set_num_random_points": [1]})

        # view image
        # cv2.imshow(f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)

        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
            output_path = os.path.join(args.output_dir, f"{render_pass}")
            assert os.path.exists(output_path)
            return_status = cv2.imwrite(output_path +f"/{record['index']}.png", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            assert return_status == True

    #cv2.destroyAllWindows()

    # close the current scene
    env.close()

    # remove copied pak file from exectuable dir's Content folder as it is no longer required
    os.remove(f"{args.executable_content_dir}/Paks/{pak_file_name}")
    