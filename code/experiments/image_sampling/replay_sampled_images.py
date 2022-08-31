# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import cv2
import numpy as np
import os
import time
import shutil
import sys

from interiorsim import Env
from interiorsim.config import get_config


if sys.platform == "linux":    
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"


def read_recorded_data(args, scene):
    pose_data_path = os.path.join(args.input_dir, f"scene_{scene}/poses.txt")
    image_data_path = os.path.join(args.input_dir, f"scene_{scene}/rgb")

    with open(pose_data_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        poses = {}
        images = {}
        for index, row in enumerate(list(csv_reader)[1:]):
            poses[index] = [float(i) for i in row[:]]
            images[index] = cv2.imread(os.path.join(image_data_path, f"{index}.png"))
            
    return poses, images

    
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_path", type=str, default="/media/rachithp/Extreme SSD/interiorsim_all_scene_paks")
    parser.add_argument("--executable_dir", type=str, default="/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development")
    parser.add_argument("--input_dir", type=str, required=True)
    args = parser.parse_args()

    # read scenes list
    scene_list_path = os.path.join(args.input_dir, "scenes.txt")
    with open(scene_list_path, 'r') as f:
        csv_reader = csv.reader(f, delimiter=',')
        scenes = next(csv_reader)
        scenes = scenes[:-1]

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    for scene in scenes[:]:
        
        print()
        print(f"running through scene {scene}...")

        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set first scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to disk
        if not os.path.exists(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # check if data path for storing images, exists
        if not os.path.exists(os.path.join(args.input_dir, f"scene_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}")):
            os.makedirs(os.path.join(args.input_dir, f"scene_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}"))

        assert config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.ACTION_MODE == "replay_sampled_images"

        poses, images = read_recorded_data(args, scene)

        # create Env object
        env = Env(config)

        # reset the simulation
        _ = env.reset()

        # iterate over recorded poses
        for index, data in poses.items():

            obs, _, _, _ = env.step({"set_position_xyz_centimeters": [data[0], data[1], data[2]], "set_orientation_pyr_degrees": [data[4], data[5], data[3]]}) # set_orientation_pyr_degrees: [pitch, yaw, roll]

            assert obs["pose"].all() == np.array(data).all()

            # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of rgb
            # cv2.imshow(f"recorded image", images[index]) # OpenCV expects BGR instead of rgb
            # cv2.waitKey(0)

            # write data
            output_path = os.path.join(args.input_dir, f"scene_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}")
            assert os.path.exists(output_path) == True

            return_status = cv2.imwrite(output_path +f"/{index}.png", obs["visual_observation"][:,:,[2,1,0]])
            assert return_status == True

        env.close()
        time.sleep(5)
        os.remove(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak")     
        cv2.destroyAllWindows()
