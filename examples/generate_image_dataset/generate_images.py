# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import sys

from interiorsim import Env
from interiorsim.config import get_config


if sys.platform == "linux":
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", type=str, required=True)
    parser.add_argument("--output_dir", "-o", type=str, required=True)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    # read data from csv
    df = pd.read_csv(args.poses_file)

    # iterate over all scenes in poses_file
    for scene in df["map_id"].unique():

        print(f"processing scene {scene}")

        # change config based on current scene
        config.defrost()
        # scene type is a numpy int, yaml config doesn't recognise it, so we need to explicitly cast it, otherwise we get an Invalid type error
        config.SIMULATION_CONTROLLER.LEVEL_ID = str(scene)
        config.freeze()

        # create dir for storing images
        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
            if not os.path.exists(os.path.join(args.output_dir, f"{scene}/{render_pass}")):
                os.makedirs(os.path.join(args.output_dir, f"{scene}/{render_pass}"))

        # create Env object
        env = Env(config)

        # reset the simulation
        _ = env.reset()

        # iterate over all poses to capture images
        for pose in df.loc[df["map_id"] == scene].to_records():

            # set the pose and obtain corresponding images
            obs, _, _, _ = env.step(action={"set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32), "set_num_random_points": np.array([0], np.uint32)})
            
            # view image
            # cv2.imshow(f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.waitKey(0)
            
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                output_path = os.path.join(args.output_dir, f"{scene}/{render_pass}")
                assert os.path.exists(output_path)
                return_status = cv2.imwrite(output_path +f"/{pose['index']}.png", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                assert return_status == True

        #cv2.destroyAllWindows()

        # close the current scene
        env.close()
    