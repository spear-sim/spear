#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--skip-save-images", action="store_true")
parser.add_argument("--skip-read-pixels", action="store_true")
args = parser.parse_args()

num_frames = 25
component_names = []
components = {}
component_data = {}

visualize_func = lambda data : data[:,:,[2,1,0]] # BGRA to RGB

# save an image for each component using the component's visualizer function
def save_images(images_dir, frame_index):
    assert not args.skip_save_images
    for component_name in component_names:
        data = component_data[component_name]
        image_file = os.path.realpath(os.path.join(images_dir, component_name, f"{frame_index:04d}.png"))
        image = visualize_func(data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)
    game = instance.get_game()

    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # get player controller
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)

        # spawn camera sensor and get its components
        bp_multi_view_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_MultiViewCameraSensor_13.BP_MultiViewCameraSensor_13_C")
        bp_multi_view_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_multi_view_camera_sensor_uclass)
        components = game.unreal_service.get_components_by_class_as_dict(actor=bp_multi_view_camera_sensor, uclass="USpSceneCaptureComponent2D")
        component_names = sorted(components.keys())

        # create output dirs
        if not args.skip_read_pixels and not args.skip_save_images:
            images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
            if os.path.exists(images_dir):
                spear.log("Directory exists, removing: ", images_dir)
                shutil.rmtree(images_dir, ignore_errors=True)
            os.makedirs(images_dir, exist_ok=True)
            for component_name in component_names:
                os.makedirs(os.path.realpath(os.path.join(images_dir, component_name)), exist_ok=True)

        # get player camera manager
        player_camera_manager = player_controller.PlayerCameraManager.get()

        # configure components to match the viewport
        viewport_size_x = 1280
        viewport_size_y = 720

        component_names = sorted(components.keys())

        for component_name in component_names:
            component = components[component_name]
            component.Width = viewport_size_x
            component.Height = viewport_size_y

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_name in component_names:
            component = components[component_name]
            component.Initialize()
            component.initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    #
    # execute warm-up frames to give Unreal's default auto-exposure settings a chance to settle down, and to
    # give the drone camera actor a chance to rotate
    #

    for _ in range(65):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # set camera pose and configure components to match the viewport
    with instance.begin_frame():
        view_target_pov = player_camera_manager.ViewTarget.POV.get()

        bp_multi_view_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_multi_view_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y
        fov = view_target_pov["fOV"]*math.pi/180.0 # this adjustment is necessary to compute an FOV value that matches the game viewport
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        # make the FOV bigger than the game viewport
        fov_adjusted_degrees = 1.35*fov_adjusted_degrees

        for component_name in component_names:
            component = components[component_name]
            component.FOVAngle = fov_adjusted_degrees

    with instance.end_frame():
        pass

    #
    # initialize frame counter
    #

    frame_index = 0

    #
    # begin capturing frames
    #

    for i in range(num_frames):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)

        with instance.end_frame():
            # read pixels from camera sensor
            if not args.skip_read_pixels:
                for component_name in component_names:
                    component = components[component_name]
                    data_bundle = component.read_pixels()
                    component_data[component_name] = data_bundle["arrays"]["data"]

            gameplay_statics.SetGamePaused(bPaused=True)

        if not args.skip_read_pixels and not args.skip_save_images:
            save_images(images_dir=images_dir, frame_index=frame_index)
            frame_index = frame_index + 1

    #
    # unpause now that we're finished capturing frames
    #

    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
    with instance.end_frame():
        pass

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_name in component_names:
            component = components[component_name]
            component.terminate_sp_funcs()
            component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_multi_view_camera_sensor)

    spear.log("Done.")
