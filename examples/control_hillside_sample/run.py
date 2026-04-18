#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--skip-save-images", action="store_true")
parser.add_argument("--skip-read-pixels", action="store_true")
args = parser.parse_args()

num_frames = 100

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    }
]

# save an image for each component using the component's visualizer function
def save_images(images_dir, frame_index):
    assert not args.skip_save_images
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, component_desc["name"], f"{frame_index:04d}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)


if __name__ == "__main__":

    # create output dirs
    if not args.skip_read_pixels and not args.skip_save_images:
        images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
        if os.path.exists(images_dir):
            spear.log("Directory exists, removing: ", images_dir)
            shutil.rmtree(images_dir, ignore_errors=True)
        os.makedirs(images_dir, exist_ok=True)
        for component_desc in component_descs:
            os.makedirs(os.path.realpath(os.path.join(images_dir, component_desc["name"])), exist_ok=True)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    #
    # get gameplay statics
    #

    with instance.begin_frame():
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
    with instance.end_frame():
        pass

    #
    # load sublevels
    #

    with instance.begin_frame():
        gameplay_statics.LoadStreamLevel(LevelName="LV_Lighting_Default", bMakeVisibleAfterLoad=True, bShouldBlockOnLoad=True)
    with instance.end_frame():
        pass

    with instance.begin_frame():
        gameplay_statics.LoadStreamLevel(LevelName="LV_S_Vehicles", bMakeVisibleAfterLoad=True, bShouldBlockOnLoad=True)
    with instance.end_frame():
        pass

    #
    # set up scene and camera
    #

    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # get player controller
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)

        # get and configure pawn
        pawn = player_controller.K2_GetPawn()
        pawn.K2_SetActorLocation(NewLocation={"X": 1020.0, "Y": -9100.0, "Z": 1720.0})
        player_controller.SetControlRotation(NewRotation={"Roll": 0.0, "Pitch": 8.0, "Yaw": 15.0})

        # spawn camera sensor
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # get camera components

        final_tone_curve_hdr_component = None

        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=component_desc["long_name"], uclass="USpSceneCaptureComponent2D")

            # get final_tone_curve_hdr_component
            if component_desc["name"] == "final_tone_curve_hdr":
                final_tone_curve_hdr_component = component_desc["component"]

        assert final_tone_curve_hdr_component is not None

        # configure components to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        components = [ desc["component"] for desc in component_descs ]
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_desc=viewport_desc, widths=1280, heights=720)

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    with instance.begin_frame():
        viewport_desc = game.rendering_service.get_current_viewport_desc(only_get_pose=True)
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_desc=viewport_desc, only_align_pose=True)
    with instance.end_frame():
        pass

    #
    # execute warm-up frames to give Unreal's default auto-exposure settings a chance to settle down
    #

    instance.step(num_frames=90)

    #
    # render image
    #

    #
    # initialize frame counter
    #

    frame_index = 0

    #
    # render images
    #

    for i in range(num_frames):

        with instance.begin_frame():
            pass
        with instance.end_frame():
            # read pixels from camera sensor
            if not args.skip_read_pixels:
                for component_desc in component_descs:
                    data_bundle = component_desc["component"].read_pixels()
                    component_desc["data"] = data_bundle["arrays"]["data"]

        if not args.skip_read_pixels and not args.skip_save_images:
            save_images(images_dir=images_dir, frame_index=frame_index)
            frame_index = frame_index + 1

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    spear.log("Done.")
