#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import spear
import sys


R_camera_from_world = None
M_hypersim_camera_from_unreal_camera = np.array([[  0, 1, 0],
                                                 [  0, 0, 1],
                                                 [ -1, 0, 0]], dtype=np.float32)

scene_desc = \
{
    "name": "apartment_0000"
}

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "sp_camera_normal",
        "long_name": "DefaultSceneRoot.world_normal_",
        "visualize_func": lambda data : np.clip((1.0 + (data[:,:,[0,1,2]] @ R_camera_from_world.T @ M_hypersim_camera_from_unreal_camera.T))/2.0, 0.0, 1.0)
    },
    {
        "name": "sp_depth_meters",
        "long_name": "DefaultSceneRoot.sp_depth_meters_",
        "visualize_func": lambda data : np.clip((data[:,:,0] - np.min(data[:,:,0])) / np.minimum((np.max(data[:,:,0]) - np.min(data[:,:,0])), 7.5), 0.0, 1.0) # normalize to max depth of 7.5 meters
    }
]


if __name__ == "__main__":

    spear.log("Processing scene: ", scene_desc["name"])

    # create output dir
    for component_desc in component_descs:
        component_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", scene_desc["name"], component_desc["name"]))
        if os.path.exists(component_dir):
            spear.log("Directory exists, removing: ", component_dir)
            shutil.rmtree(component_dir, ignore_errors=True)
        os.makedirs(component_dir, exist_ok=True)

    # get camera poses
    camera_poses_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "camera_poses"))
    camera_poses_file = os.path.realpath(os.path.join(camera_poses_dir, f"{scene_desc['name']}.csv"))
    df = pd.read_csv(camera_poses_file, comment="#")

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # spawn camera sensor
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # initialize components
        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(uclass="USceneComponent", actor=bp_camera_sensor, component_name=component_desc["long_name"])
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and can fix occasional render-to-texture initialization issues on macOS
    for i in range(1):
        instance.flush()

    for camera_pose in df.to_records():

        with instance.begin_frame():

            # set camera pose
            bp_camera_sensor.K2_SetActorLocation(NewLocation={"X": camera_pose["location_x"], "Y": camera_pose["location_y"], "Z": camera_pose["location_z"]})
            bp_camera_sensor.K2_SetActorRotation(NewRotation={"Pitch": camera_pose["rotation_pitch"], "Yaw": camera_pose["rotation_yaw"], "Roll": camera_pose["rotation_roll"]})

            # set rotation matrix
            R_world_from_camera = spear.to_numpy_matrix_from_rotator(rotator={"Pitch": camera_pose["rotation_pitch"], "Yaw": camera_pose["rotation_yaw"], "Roll": camera_pose["rotation_roll"]}, as_matrix=True)
            R_camera_from_world = R_world_from_camera.T.A

        #
        # let temporal anti-aliasing etc accumulate additional information across multiple frames
        #

        # with instance.end_frame():
        #     pass

        # for i in range(1):
        #     instance.flush()

        # with instance.begin_frame():
        #     pass

        with instance.end_frame():

            # get rendered frame
            for component_desc in component_descs:
                data_bundle = component_desc["component"].read_pixels()
                data = data_bundle["arrays"]["data"]

                component_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", scene_desc["name"], component_desc["name"]))
                image_file = os.path.realpath(os.path.join(component_dir, f"{camera_pose['index']:04d}.png"))
                image = component_desc["visualize_func"](data=data_bundle["arrays"]["data"])
                plt.imsave(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    # instance.close()

    spear.log("Done.")
