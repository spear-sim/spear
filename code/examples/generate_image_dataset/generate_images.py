# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import cv2
import ntpath
import numpy as np
import os
import shutil
import sys

from interiorsim import Env
from interiorsim.config import get_config


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--pak_file", type=str, required=True)
    parser.add_argument("--executable_content_dir", type=str, required=True)
    parser.add_argument("--num_images_per_scene", type=int, default=10)
    parser.add_argument("--output_dir", "-o", type=str, required=True)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    # random generator
    random_number_generator = np.random.default_rng(config.IMAGE_DATASET_EXAMPLE.SEED)

    # copy pak to the executable dir as this is required for launching the appropriate pak file
    pak_file_name = ntpath.basename(args.pak_file)
    if not os.path.exists(f"{args.executable_content_dir}/Paks/{pak_file_name}"):
        shutil.copy(args.pak_file, f"{args.executable_content_dir}/Paks")

    # create dir for storing images
    for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
        if not os.path.exists(os.path.join(args.output_dir, f"{render_pass}")):
            os.makedirs(os.path.join(args.output_dir, f"{render_pass}"))

    # create a csv writer for writing poses
    if sys.platform == "win32":
        pose_output_file = open(os.path.join(args.output_dir, "poses.txt"), "w", newline='')
    else:
        pose_output_file = open(os.path.join(args.output_dir, "poses.txt"), "w")
    pose_csv_writer = csv.writer(pose_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    pose_csv_writer.writerow(["pos_x_cm", "pos_y_cm", "pos_z_cm", "roll_deg", "pitch_deg", "yaw_deg"])

    # create Env object
    env = Env(config)

    # reset the simulation
    _ = env.reset()

    # get random positions based on number of images requested
    _, _, _, step_info = env.step({"set_pose":[0,0,config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,0,0,0], "set_num_random_points": [args.num_images_per_scene]})
    random_positions = step_info["agent_controller_step_info"]["random_points"]

    # iterate over these positions can capture images at each position
    for idx, position in enumerate(random_positions):
        # create a pose variable
        pose = [float(position[0]),
                float(position[1]),
                config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,
                random_number_generator.uniform(low=config.IMAGE_DATASET_EXAMPLE.PITCH_LOW_DEG, high=config.IMAGE_DATASET_EXAMPLE.PITCH_HIGH_DEG),
                random_number_generator.uniform(low=config.IMAGE_DATASET_EXAMPLE.YAW_LOW_DEG,   high=config.IMAGE_DATASET_EXAMPLE.YAW_HIGH_DEG),
                random_number_generator.uniform(low=config.IMAGE_DATASET_EXAMPLE.ROLL_LOW_DEG,  high=config.IMAGE_DATASET_EXAMPLE.ROLL_HIGH_DEG)]

        # store the pose
        pose_csv_writer.writerow([pose[0], pose[1], pose[2], pose[5], pose[3], pose[4]])

        # set the pose and obtain corresponding images
        obs, _, _, _ = env.step({"set_pose": pose, "set_num_random_points": [1]})

        # view any image
        # cv2.imshow(f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)

        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
            output_path = os.path.join(args.output_dir, f"{render_pass}")
            assert os.path.exists(output_path) == True
            return_status = cv2.imwrite(output_path +f"/{idx}.png", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]])
            assert return_status == True

    #cv2.destroyAllWindows()

    # close the current scene
    env.close()
    
    # close pose output file
    pose_output_file.close()

    # remove copied pak file from exectuable dir's Content folder as it is no longer required
    os.remove(f"{args.executable_content_dir}/Paks/{pak_file_name}")