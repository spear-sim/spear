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


if __name__ == "__main__":

    description = "This script expects a certain directory structure for the scenes_path argument. scenes_path folder should be as follows:\n"\
                    "scenes_path/\n"\
                    "|-- 2355*.pak\n"\
                    "|-- 2382*.pak\n"\
                    "|-- ...\n"\
                    "\n"\
                    "After you run this script, expect a output_dir directory like this:\n"\
                    "output_dir/\n"\
                    "|-- scenes.txt/\n"\
                    "|-- bad_scenes.txt/\n"\
                    "|-- skipped_scenes.txt/\n"\
                    "|-- Map_2355*/\n"\
                    "|  |--poses.txt\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- Map_2342*/\n"\
                    "|  |--poses.txt\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"\
                    "|-- .../\n"\
                    "|  |--poses.txt\n"\
                    "|  |--IMAGE_TYPE\n"\
                    "|  |  |-- 0.png\n"\
                    "|  |  |-- 1.png\n"\
                    "|  |  |-- ...\n"\
                    "|  |  |-- x.png\n"
                    
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--scenes_path", type=str, default="/media/rachithp/Extreme SSD/interiorsim_all_scene_paks", help="input path to where all .pak files are stored.")
    parser.add_argument("--executable_dir", type=str, default="/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development", help="this should point to the directory that contains the UE executable. Eg. <path_to_executable_dir>/WinNoEditor/RobotProject.exe can be your path to executable, so your input should be just the outer dir path.")
    parser.add_argument("--output_dir", "-o", type=str, required=True)
    parser.add_argument("-num_images", type=int, default=54000)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    assert config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.ACTION_MODE == "sample_images"
    
    # random generator
    rng = np.random.default_rng(config.IMAGE_SAMPLING_EXPERIMENT.SEED)
    
    # check if path exists
    if not os.path.exists(os.path.join(args.output_dir)):
        os.makedirs(os.path.join(args.output_dir))

    scenes_sampled = open(os.path.join(args.output_dir, "scenes.txt"), "w")
    bad_scenes = open(os.path.join(args.output_dir, "bad_scenes.txt"), "w")
    skipped_scenes = open(os.path.join(args.output_dir, "skipped_scenes.txt"), "w")

    # choose scenes from input dir
    scenes_on_disk = os.listdir(args.scenes_path)
    chosen_scenes = []
    for scene in scenes_on_disk:
        split_string_list = scene.split('_')
        if len(split_string_list) > 1 and split_string_list[1] == f"{PLATFORM}.pak":
            chosen_scenes.append(split_string_list[0])

    # number of images per scene based on # of scenes.
    # while debugging just few images from scenes, you can change this number to 10 or 20 or something small
    NUM_IMAGES_PER_SCENE = int(args.num_images / len(chosen_scenes)) + 1

    for scene in chosen_scenes:

        print(f"processing scene {scene}")

        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to the executable dir as this is required for launching the appropriate pak file
        if not os.path.exists(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # check if data path for storing images exists
        if not os.path.exists(os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}")):
            os.makedirs(os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}"))
            print(f"collecting images for scene {scene}")
        else:
            images = os.listdir(os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}"))
            print(f"scene {scene}, num_images = {len(images)}")
            # if len(images) == NUM_IMAGES_PER_SCENE:
                # print(f"scene - {scene} has {len(images)} images already, so skipping.")
                # skipped_scenes.write(scene)
                # skipped_scenes.write("\n")
                # continue
        
        # write headers
        scenes_sampled.write(scene)
        scenes_sampled.write(",")
        pose_output_file = open(os.path.join(args.output_dir, "Map_{}/poses.txt".format(scene)), "w")
        pose_csv_writer = csv.writer(pose_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        pose_csv_writer.writerow(["pos_x_cm", "pos_y_cm", "pos_z_cm", "roll_deg", "pitch_deg", "yaw_deg"])

        # create Env object
        try:
            env = Env(config)
        except:
            bad_scenes.write(scene)
            bad_scenes.write("\n")
            continue

        # reset the simulation
        _ = env.reset()

        # for NUM_IMAGES_PER_SCENE capture images and pose data
        for i in range(0, NUM_IMAGES_PER_SCENE):
            obs, _, _, _ = env.step({"set_random_orientation_pyr_deg":  [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_HIGH_DEG),
                                                                        rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.YAW_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.YAW_HIGH_DEG),
                                                                        rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_HIGH_DEG)],
                                     "set_random_agent_height_cms"   :  [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.AGENT_HEIGHT_LOW_CMS, high=config.IMAGE_SAMPLING_EXPERIMENT.AGENT_HEIGHT_HIGH_CMS)]})

            # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.waitKey(0)
            
            # write data
            output_path = os.path.join(args.output_dir, f"Map_{scene}/{config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.IMAGE_TYPE}")

            assert os.path.exists(output_path) == True
            return_status = cv2.imwrite(output_path +f"/{i}.png", obs["visual_observation"][:,:,[2,1,0]])
            assert return_status == True

            pose_csv_writer.writerow([obs["pose"][0], obs["pose"][1], obs["pose"][2], obs["pose"][3], obs["pose"][4], obs["pose"][5]])        
        
        # close the current environment after collecting required number of images
        env.close()
        time.sleep(10)
        pose_output_file.close()
        os.remove(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak")        

    scenes_sampled.close()
    bad_scenes.close()
    cv2.destroyAllWindows()
