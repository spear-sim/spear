# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import pandas as pd
import spear
import sys


if sys.platform == "linux":
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", default="poses.csv")
    parser.add_argument("--num_poses_per_scene", default=10)
    parser.add_argument("--scene_id", type=str)
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # reset seed
    np.random.seed(0)

    # remove existing poses file
    if os.path.exists(args.poses_file):
        os.remove(args.poses_file)

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # iterate through all scenes
    for i, scene_id in enumerate(scene_ids):

        print("Processing scene: " + scene_id)

        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + "." + "Map_" + scene_id
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id
        config.freeze()

        # create Env object
        env = spear.Env(config)

        # reset the simulation
        _ = env.reset()

        # get random positions based on number of poses requested
        _, _, _, step_info = env.step(
            action={
                "set_pose": np.array([0,0,0,0,0,0], dtype=np.float32),
                "set_num_random_points": np.array([args.num_poses_per_scene], dtype=np.uint32)})

        random_positions = step_info["agent_step_info"]["random_points"]

        # generate random pitch, yaw, roll values
        random_pitch_values = np.random.uniform(low=0.0, high=0.0, size=random_positions.shape[0])
        random_yaw_values = np.random.uniform(low=-180.0, high=180.0, size=random_positions.shape[0])
        random_roll_values = np.random.uniform(low=0.0, high=0.0, size=random_positions.shape[0])

        # store poses in a csv file
        df = pd.DataFrame({"scene_id"  : f"{scene_id}",
                           "pos_x_cms" : random_positions[:, 0],
                           "pos_y_cms" : random_positions[:, 1],
                           "pos_z_cms" : random_positions[:, 2] + config.SIMULATION_CONTROLLER.CAMERA_AGENT.NAVMESH.AGENT_HEIGHT,
                           "pitch_degs": random_pitch_values,
                           "yaw_degs"  : random_yaw_values,
                           "roll_degs" : random_roll_values})
        df.to_csv(args.poses_file, mode="a", index=False, header=i==0)

        # close the current scene
        env.close()
