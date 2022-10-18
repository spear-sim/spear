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
    pose_data_path = os.path.join(args.input_dir, f"Map_{scene}/poses.txt")
    # image_data_path = os.path.join(args.input_dir, f"Map_{scene}/seg")

    with open(pose_data_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        poses = {}
        # images = {}
        for index, row in enumerate(list(csv_reader)[1:]):
            poses[index] = [float(i) for i in row[:]]
            # images[index] = cv2.imread(os.path.join(image_data_path, f"{index}.png"))
    
    return poses#, images

    
if __name__ == "__main__":

    description = "This script expects a certain directory structure for your input_dir. It should be as follows:\n"\
                    "input_dir/\n"\
                    "|-- scenes.txt/\n"\
                    "|-- Map_2355*/\n"\
                    "|  |--poses.txt\n"\
                    "|  |--rgb or seg\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- Map_2342*/\n"\
                    "|  |--poses.txt\n"\
                    "|  |--rgb or seg\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- .../\n"\
                    "|  |--poses.txt\n"\
                    "|  |--rgb or seg\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "After you run this script, expect a output_dir directory like this:\n"\
                    "output_dir/\n"\
                    "|-- Map_2355*/\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- Map_2342*/\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- .../\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"
                    
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--scenes_path", type=str, default="/media/rachithp/Extreme SSD/interiorsim_all_scene_paks")
    parser.add_argument("--executable_dir", type=str, default="/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development")
    parser.add_argument("--input_dir", type=str, required=True)
    parser.add_argument("--output_dir", type=str, required=True)
    args = parser.parse_args()

    # # read scenes list
    # scene_list_path = os.path.join(args.input_dir, "scenes.txt")
    # with open(scene_list_path, 'r') as f:
        # csv_reader = csv.reader(f, delimiter=',')
        # scenes = next(csv_reader)
        # scenes = scenes[:-1]
    
    # choose scenes from input dir
    scenes_on_disk = [foldr for foldr in os.listdir(args.input_dir) if os.path.isdir(os.path.join(args.input_dir, foldr))]
    scenes = []
    for scene in scenes_on_disk:
        split_string_list = scene.split('_')
        scenes.append(split_string_list[1])

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    NUM_IMAGES_PER_FRAME = 6

    for scene in scenes:

        if scene != "237101160":
            continue

        print()
        print(f"running through scene {scene}...")
        
        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set first scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to disk
        # if not os.path.exists(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}/paks/Windows/{scene}/{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # check if data path for storing images, exists
        if not os.path.exists(os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}")):
            os.makedirs(os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}"))

        poses = read_recorded_data(args, scene)

        # create Env object
        env = Env(config)

        # reset the simulation
        _ = env.reset()

        start_time = time.time()

        # iterate over recorded poses
        for index, data in poses.items():
            if "final_color" in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                env.customSetActionTick({"set_pose": [data[0], data[1], data[2], data[4], data[5], data[3]], "set_num_random_points": [1]}) # set_pose: [x, y, z, pitch, yaw, roll]
                for j in range(0, NUM_IMAGES_PER_FRAME - 2):
                    env.customEmptyTick()
                obs, _, _, _ = env.customGetObservationTick()
            elif "segmentation" in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                obs, _, _, _ = env.step({"set_pose": [data[0], data[1], data[2], data[4], data[5], data[3]], "set_num_random_points": [1]}) # set_pose: [x, y, z, pitch, yaw, roll]
            else:
                assert False, "render pass mode in config file is not supported. Supported types are 'final_color' and 'segmentation'."

            # cv2.imshow(f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.imshow(f"recorded image", images[index]) # OpenCV expects BGR instead of rgb
            # cv2.waitKey(0)

            output_path = os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}")
            assert os.path.exists(output_path) == True

            # write data
            return_status = cv2.imwrite(output_path +f"/{index}.png", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]}"][:,:,[2,1,0]])
            assert return_status == True
        
        stop_time = time.time()
        env.close()

        print(f"elapsed time for scene {scene}", stop_time-start_time)
        time.sleep(5)
        os.remove(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak")  
        cv2.destroyAllWindows()
