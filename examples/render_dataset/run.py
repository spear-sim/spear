#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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


scene_desc = \
{
    "name": "apartment_0000",
}

component_descs = \
[
    {
        "name": "final_tone_curve_hdr_component",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "visualize_func": lambda data : data[:,:,[2,1,0]]
    },
    {
        "name": "normal",
        "long_name": "DefaultSceneRoot.normal_",
        "visualize_func": lambda data : np.clip(((data + 1.0) / 2.0)[:,:,0:3], 0.0, 1.0)
    },
    {
        "name": "depth",
        "long_name": "DefaultSceneRoot.depth_",
        "visualize_func": lambda data : data[:,:,0]
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
    df = pd.read_csv(camera_poses_file)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # find functions
        actor_static_class = game.unreal_service.get_static_class(class_name="AActor")
        set_actor_location_func = game.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorLocation")
        set_actor_rotation_func = game.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorRotation")

        sp_scene_capture_component_2d_static_class = game.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_camera_sensor_uclass)

        # initialize components and get handles to their shared memory
        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_camera_sensor_actor, component_name=component_desc["long_name"])
            game.unreal_service.call_function(uobject=component_desc["component"], ufunction=initialize_func)
            component_desc["shared_memory_handles"] = instance.sp_func_service.create_shared_memory_handles_for_object(uobject=component_desc["component"])

    with instance.end_frame():
        pass

    for camera_pose in df.to_records():

        with instance.begin_frame():

            # set camera pose
            game.unreal_service.call_function(
                uobject=bp_camera_sensor_actor,
                ufunction=set_actor_location_func,
                args={"NewLocation": {"X": camera_pose["location_x"], "Y": camera_pose["location_y"], "Z": camera_pose["location_z"]}})

            game.unreal_service.call_function(
                uobject=bp_camera_sensor_actor,
                ufunction=set_actor_rotation_func,
                args={"NewRotation": {"Pitch": camera_pose["rotation_pitch"], "Yaw": camera_pose["rotation_yaw"], "Roll": camera_pose["rotation_roll"]}})

        #
        # let temporal anti-aliasing etc accumulate additional information across multiple frames
        #

        # with instance.end_frame():
        #     pass

        # for i in range(1):
        #     with instance.begin_frame():
        #         pass
        #     with instance.end_frame():
        #         pass

        # with instance.begin_frame():
        #     pass

        with instance.end_frame():

            # get rendered frame
            for component_desc in component_descs:
                return_values = instance.sp_func_service.call_function(
                    uobject=component_desc["component"],
                    function_name="read_pixels",
                    uobject_shared_memory_handles=component_desc["shared_memory_handles"])
                data = return_values["arrays"]["data"]

                # spear.log("component: ", component_desc["name"])
                # spear.log("shape:     ", return_values["arrays"]["data"].shape)
                # spear.log("dtype:     ", return_values["arrays"]["data"].dtype)
                # spear.log("min:       ", np.min(return_values["arrays"]["data"]))
                # spear.log("max:       ", np.max(return_values["arrays"]["data"]))

                component_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", scene_desc["name"], component_desc["name"]))
                image_file = os.path.realpath(os.path.join(component_dir, "%04d.png"%camera_pose["index"]))
                image = component_desc["visualize_func"](return_values["arrays"]["data"])
                plt.imsave(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            instance.sp_func_service.destroy_shared_memory_handles_for_object(shared_memory_handles=component_desc["shared_memory_handles"])
            game.unreal_service.call_function(uobject=component_desc["component"], ufunction=terminate_func)
        game.unreal_service.destroy_actor(actor=bp_camera_sensor_actor)

    instance.close()

    spear.log("Done.")
