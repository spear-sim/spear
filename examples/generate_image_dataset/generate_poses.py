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

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys
sys.path.append(common_dir)
import instance_utils
import navmesh


CAMERA_LOCATION_Z_OFFSET = 200.0


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "poses.csv")))
    parser.add_argument("--num_poses_per_scene", type=int, default=10)
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scenes.csv"))
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file)["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # create dataframe
    df_columns = ["scene_id", "location_x", "location_y", "location_z", "rotation_pitch", "rotation_yaw", "rotation_roll"]
    df = pd.DataFrame(columns=df_columns)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    config.defrost()
    config.SP_SERVICES.LEGACY_SERVICE.AGENT = "NullAgent"
    config.freeze()

    spear.configure_system(config)
    instance = spear.Instance(config)
    navmesh = navmesh.NavMesh(instance)

    # iterate over all scenes
    for scene_id in scene_ids:

        spear.log("Processing scene: " + scene_id)

        instance_utils.open_level(instance, scene_id)

        # get a few random points
        points = navmesh.get_random_points(args.num_poses_per_scene)

        # generate random pitch, yaw, roll values
        pitch_values = np.random.uniform(low=0.0, high=0.0, size=args.num_poses_per_scene)
        yaw_values   = np.random.uniform(low=-180.0, high=180.0, size=args.num_poses_per_scene)
        roll_values  = np.random.uniform(low=0.0, high=0.0, size=args.num_poses_per_scene)

        # store poses in a csv file
        df_ = pd.DataFrame(
            columns=df_columns,
            data={
                "scene_id":       scene_id,
                "location_x":     points[:,0],
                "location_y":     points[:,1],
                "location_z":     points[:,2] + CAMERA_LOCATION_Z_OFFSET,
                "rotation_pitch": pitch_values,
                "rotation_yaw":   yaw_values,
                "rotation_roll":  roll_values})

        df = pd.concat([df, df_])

        plt.scatter(points[:,0], points[:,1], s=1.0)
        plt.gca().set_aspect("equal")
        plt.gca().invert_yaxis() # invert the y-axis so the plot matches a top-down view of the scene in Unreal
        plt.show()

    # close the unreal instance
    instance.close()

    # write to a csv file
    df.to_csv(args.poses_file, index=False)

    spear.log("Done.")
