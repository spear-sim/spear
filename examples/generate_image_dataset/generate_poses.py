#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "poses.csv")))
    parser.add_argument("--num_poses_per_scene", type=int, default=10)
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scenes.csv"))
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file)["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # iterate over all scenes
    for i, scene_id in enumerate(scene_ids):

        spear.log("Processing scene: " + scene_id)

        # create Env object
        env = spear.Env(config)

        # reset the simulation
        _ = env.reset()

        # get a few random points
        points = env.get_random_points(args.num_poses_per_scene)

        # generate random pitch, yaw, roll values
        pitch_values = np.random.uniform(low=0.0, high=0.0, size=args.num_poses_per_scene)
        yaw_values   = np.random.uniform(low=-180.0, high=180.0, size=args.num_poses_per_scene)
        roll_values  = np.random.uniform(low=0.0, high=0.0, size=args.num_poses_per_scene)

        # store poses in a csv file
        df = pd.DataFrame({"scene_id"  : scene_id,
                           "location_x_cms" : points[:,0],
                           "location_y_cms" : points[:,1],
                           "location_z_cms" : points[:,2] + config.SIMULATION_CONTROLLER.NAVMESH.AGENT_HEIGHT,
                           "rotation_pitch_degs": pitch_values,
                           "rotation_yaw_degs"  : yaw_values,
                           "rotation_roll_degs" : roll_values})
        df.to_csv(args.poses_file, mode="w" if i==0 else "a", index=False, header=i==0)

        plt.scatter(points[:,0], points[:,1], s=1.0)
        plt.gca().set_aspect("equal")
        plt.gca().invert_yaxis() # invert the y-axis so the plot matches a top-down view of the scene in Unreal
        plt.show()

        # close the current scene
        env.close()

    spear.log("Done.")
