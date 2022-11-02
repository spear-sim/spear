# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import time
import shutil
import sys
import time

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
    parser.add_argument("--scenes_path", type=str, required=True)
    parser.add_argument("--executable_dir", type=str, required=True)
    parser.add_argument("--poses_file", type=str, required=True)
    parser.add_argument("--num_poses_per_scene", type=int, default=181)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    # random generator
    rng = np.random.default_rng(config.IMAGE_SAMPLING_EXPERIMENT.SEED)

    # make a note of scenes that throw errors when tried to launch via Env(config)
    bad_scenes = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "bad_scenes.txt"), "w")

    # choose scenes from input dir
    # scenes_on_disk = os.listdir(args.scenes_path)
    # scenes = []
    # for scene in scenes_on_disk:
    #     split_string_list = scene.split('_')
    #     if len(split_string_list) > 1 and split_string_list[1] == f"{PLATFORM}.pak":
    #         scenes.append(split_string_list[0])

    # make a list of scenes already completed
    # completed_scenes = [fp for fp in os.listdir(args.output_dir) if os.path.isdir(os.path.join(args.output_dir, fp))]

    # get all scenes
    scenes = os.listdir(args.scenes_path)

    for idx, scene in enumerate(scenes):

        # skip completed scenes
        # if f"Map_{scene}" in completed_scenes:
            # continue

        print(f"processing scene {scene}")

        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to the executable dir as this is required for launching the appropriate pak file
        # if not os.path.exists(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}/paks/Windows/{scene}/{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # create Env object
        try:
            env = Env(config)
        except:
            bad_scenes.write(scene)
            bad_scenes.write("\n")
            continue

        # reset the simulation
        _ = env.reset()

        # only for debug purposes
        if config.IMAGE_SAMPLING_EXPERIMENT.EXPORT_NAV_DATA_DEBUG_POSES:
            _, _, _, step_info = env.step({"set_pose": np.array([0,0,config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,0,0,0], dtype=np.float32), "set_num_random_points": np.array([config.IMAGE_SAMPLING_EXPERIMENT.DEBUG_POSES_NUM], dtype=np.uint32)})
            random_positions = step_info["agent_controller_step_info"]["random_points"]

            df = pd.DataFrame({ "pos_x_cms" : random_positions[:, 0],
                                "pos_y_cms" : random_positions[:, 1],
                                "pos_z_cms" : random_positions[:, 2] })
            df.to_csv(os.path.join(config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.EXPORT_NAV_DATA_OBJ_DIR, f"Map_{scene}/poses_for_debug.csv"), mode='w', index=False)

        
        _, _, _, step_info = env.step({"set_pose": np.array([0,0,config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,0,0,0], dtype=np.float32), "set_num_random_points": np.array([args.num_poses_per_scene], dtype=np.uint32)})
        random_positions = step_info["agent_controller_step_info"]["random_points"]

        # genearte random arrays for orientation
        random_pitch_values = rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_HIGH_DEG,   size=random_positions.shape[0])
        random_yaw_values   = rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.YAW_LOW_DEG,   high=config.IMAGE_SAMPLING_EXPERIMENT.YAW_HIGH_DEG,     size=random_positions.shape[0])
        random_roll_values  = rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_LOW_DEG,  high=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_HIGH_DEG,    size=random_positions.shape[0])

        df = pd.DataFrame({ "map_id"    : f"{scene}",
                            "pos_x_cms" : random_positions[:, 0],
                            "pos_y_cms" : random_positions[:, 1],
                            "pos_z_cms" : random_positions[:, 2] + config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.NAVMESH.AGENT_HEIGHT,
                            "pitch_degs": random_pitch_values,
                            "yaw_degs"  : random_yaw_values,
                            "roll_degs" : random_roll_values    })
        df.to_csv(args.poses_file, mode='a', index=False, header=idx==0)

        # close the current environment after collecting required number of images
        env.close()
        time.sleep(5)
        # os.remove(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak")

    bad_scenes.close()
    cv2.destroyAllWindows()
