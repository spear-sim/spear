#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import pandas as pd
import shutil
import spear
import sys


num_camera_poses_per_scene = 50
camera_pose_location_offset = np.array([[0.0, 0.0, 180.0]])

scene_desc = \
{
    "name": "apartment_0000",
}


if __name__ == "__main__":

    # create output dir
    camera_poses_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "camera_poses"))
    if os.path.exists(camera_poses_dir):
        spear.log("Directory exists, removing: ", camera_poses_dir)
        shutil.rmtree(camera_poses_dir, ignore_errors=True)
    os.makedirs(camera_poses_dir, exist_ok=True)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    spear.log("Processing scene: ", scene_desc["name"])

    with instance.begin_frame():

        # get navigation system and navigation data
        navigation_system_v1_static_class = game.unreal_service.get_static_class(class_name="UNavigationSystemV1")
        get_navigation_system_func = game.unreal_service.find_function_by_name(uclass=navigation_system_v1_static_class, function_name="GetNavigationSystem")
        navigation_system_v1_default_object = game.unreal_service.get_default_object(uclass=navigation_system_v1_static_class, create_if_needed=False)
        return_values = game.unreal_service.call_function(uobject=navigation_system_v1_default_object, ufunction=get_navigation_system_func)
        navigation_system = spear.to_handle(string=return_values["ReturnValue"])
        navigation_data = game.navigation_service.get_nav_data_for_agent_name(navigation_system=navigation_system, agent_name="Default")

        # sample random points
        points = game.navigation_service.get_random_points(navigation_data=navigation_data, num_points=num_camera_poses_per_scene)

        camera_locations = points + camera_pose_location_offset

        # generate random pitch, yaw, roll values
        pitch_values = np.random.uniform(low=0.0, high=0.0, size=num_camera_poses_per_scene)
        yaw_values   = np.random.uniform(low=-180.0, high=180.0, size=num_camera_poses_per_scene)
        roll_values  = np.random.uniform(low=0.0, high=0.0, size=num_camera_poses_per_scene)

        # store poses in a csv file
        df = pd.DataFrame(
            columns=["location_x", "location_y", "location_z", "rotation_pitch", "rotation_yaw", "rotation_roll"],
            data={
                "location_x":     camera_locations[:,0],
                "location_y":     camera_locations[:,1],
                "location_z":     camera_locations[:,2],
                "rotation_pitch": pitch_values,
                "rotation_yaw":   yaw_values,
                "rotation_roll":  roll_values})

        # write to a csv file
        camera_poses_file = os.path.realpath(os.path.join(camera_poses_dir, f"{scene_desc['name']}.csv"))
        df.to_csv(camera_poses_file, index=False)

    with instance.end_frame():
        pass

    instance.close()

    spear.log("Done.")
