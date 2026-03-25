#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import math
import matplotlib.pyplot as plt
import os
import spear
import unreal # not needed but useful to demonstrate that we're running inside the editor


parser = argparse.ArgumentParser()
parser.add_argument("--teaser", action="store_true")
args = parser.parse_args()

file = __file__


@spear.editor.script
def script():

    instance = spear.Instance()
    game = yield from instance.get_game_in_editor_script()

    # initialize actors and components
    with instance.begin_frame():
        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        
        engine = game.engine_globals_service.get_engine()
        game_viewport_client = engine.GameViewport.get()

        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
        player_camera_manager = player_controller.PlayerCameraManager.get()
        view_target_pov = player_camera_manager.ViewTarget.POV.get()

        post_process_volume_settings = None
        post_process_volumes = game.unreal_service.find_actors_by_class(uclass="APostProcessVolume")
        if len(post_process_volumes) == 1:
            post_process_volume = post_process_volumes[0]
            spear.log("Found unique post-process volume.")
            post_process_volume_settings = post_process_volume.Settings.get()

        # GetViewportSize(...) modifies arguments in-place, so we need as_dict=True so all arguments get returned
        sp_game_viewport = game.get_unreal_object(uclass="USpGameViewportClient")
        return_values = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)

        if args.teaser:
            viewport_size_x = 1920
            viewport_size_y = 1080
        else:
            viewport_size_x = return_values["ViewportSize"]["x"]
            viewport_size_y = return_values["ViewportSize"]["y"]

        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y
        fov = view_target_pov["fOV"]*math.pi/180.0
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"]) # this adjustment is necessary to compute an FOV value that matches the game viewport
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

        final_tone_curve_hdr_component.Width = viewport_size_x
        final_tone_curve_hdr_component.Height = viewport_size_y
        final_tone_curve_hdr_component.FOVAngle = fov_adjusted_degrees
        # if post_process_volume_settings is not None:
        #     final_tone_curve_hdr_component.PostProcessSettings = post_process_volume_settings

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()
    yield

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames
    yield

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame can fix occasional render-to-texture initialization issues on macOS
    for i in range(1):
        yield from instance.flush_in_editor_script()

    # get rendered frame
    with instance.begin_frame():
        pass
    yield

    with instance.end_frame():
        data_bundle = final_tone_curve_hdr_component.read_pixels()
    yield

    # show rendered frame using spear.editor.imshow() since cv2 and matplotlib cause problems in the editor
    spear.editor.imshow(name="final_tone_curve_hdr", image=data_bundle["arrays"]["data"])

    # save image
    image = data_bundle["arrays"]["data"][:,:,[2,1,0]]
    image_file = os.path.realpath(os.path.join(os.path.dirname(file), "image.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    yield

    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)
    yield

    spear.log("Done.")


if __name__ == "__main__":
    script()
