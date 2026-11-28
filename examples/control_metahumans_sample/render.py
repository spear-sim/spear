#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--skip-save-images", action="store_true")
parser.add_argument("--skip-read-pixels", action="store_true")
args = parser.parse_args()

num_frames = 25


if __name__ == "__main__":

    # create instance
    instance = spear.Instance()
    game = instance.get_game()

    with instance.begin_frame():

        # spawn camera sensor and get its components
        bp_multi_view_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensorMultiView_13.BP_CameraSensorMultiView_13_C")
        bp_multi_view_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_multi_view_camera_sensor_uclass)
        components = game.unreal_service.get_components_by_class_as_dict(actor=bp_multi_view_camera_sensor, uclass="USpSceneCaptureComponent2D")
        component_descs = []
        for component_name in sorted(components.keys()):
            component_descs.append({
                "name": component_name,
                "component": components[component_name],
                "visualize_func": lambda data : data[:,:,[0,1,2]]}) # native BGR (drop alpha)

        # create output dirs
        if not args.skip_read_pixels and not args.skip_save_images:
            images_dir = spear.path.realpath(os.path.dirname(__file__), "images")
            if os.path.exists(images_dir):
                spear.log("Directory exists, removing: ", images_dir)
                shutil.rmtree(images_dir, ignore_errors=True)
            os.makedirs(images_dir, exist_ok=True)
            for component_desc in component_descs:
                os.makedirs(spear.path.realpath(images_dir, component_desc["name"]), exist_ok=True)

        # configure component resolutions (we align the components with the viewport below)
        for component_desc in component_descs:
            component_desc["component"].Width = 1280
            component_desc["component"].Height = 720

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    #
    # execute warm-up frames to give Unreal's default auto-exposure settings a chance to settle down
    #

    for _ in range(65):
        with instance.begin_frame():
            game.cdo.UGameplayStatics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            game.cdo.UGameplayStatics.SetGamePaused(bPaused=True)

    # set camera pose and configure components to match the viewport
    with instance.begin_frame():
        viewport_desc = game.viewport_service.get_current_viewport_desc()
        components = [ desc["component"] for desc in component_descs ]
        game.viewport_service.align_camera_with_viewport(camera_sensor=bp_multi_view_camera_sensor, camera_components=components, viewport_desc=viewport_desc, widths=1280, heights=720)

        # make the FOV bigger than the game viewport
        for component_desc in component_descs:
            component_desc["component"].FOVAngle = 1.35*component_desc["component"].FOVAngle.get()

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
            game.cdo.UGameplayStatics.SetGamePaused(bPaused=False)

        with instance.end_frame():
            # read pixels from camera sensor
            if not args.skip_read_pixels:
                for component_desc in component_descs:
                    data_bundle = component_desc["component"].read_pixels()
                    component_desc["data"] = data_bundle["arrays"]["data"]

            game.cdo.UGameplayStatics.SetGamePaused(bPaused=True)

        if not args.skip_read_pixels and not args.skip_save_images:
            spear.image.save_images(component_descs=component_descs, images_dir=images_dir, frame_index=frame_index)
            frame_index = frame_index + 1

    #
    # unpause now that we're finished capturing frames
    #

    with instance.begin_frame():
        game.cdo.UGameplayStatics.SetGamePaused(bPaused=False)
    with instance.end_frame():
        pass

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_multi_view_camera_sensor)

    spear.log("Done.")
