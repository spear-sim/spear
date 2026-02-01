#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import math
import os
import spear
import time


parser = argparse.ArgumentParser()
parser.add_argument("--benchmark", action="store_true")
parser.add_argument("--no-shared-memory", action="store_true")
args = parser.parse_args()


if __name__ == "__main__":

    shared_memory = not args.no_shared_memory

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

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
            spear.log("Found unique post-process volume: ", post_process_volume)
            post_process_volume_settings = post_process_volume.Settings.get()

        # GetViewportSize(...) modifies arguments in-place, so we need as_dict=True so all arguments get returned
        sp_game_viewport = game.get_unreal_object(uclass="USpGameViewportClient")
        return_values = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)

        if args.benchmark:
            viewport_size_x = 1920
            viewport_size_y = 1080
        else:
            viewport_size_x = return_values["ViewportSize"]["x"]
            viewport_size_y = return_values["ViewportSize"]["y"]

        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y
        fov = view_target_pov["fOV"]*math.pi/180.0 # this adjustment is necessary to compute an FOV value that matches the game viewport
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

        final_tone_curve_hdr_component.Width = viewport_size_x
        final_tone_curve_hdr_component.Height = viewport_size_y
        final_tone_curve_hdr_component.FOVAngle = fov_adjusted_degrees
        if post_process_volume_settings is not None:
            final_tone_curve_hdr_component.PostProcessSettings = post_process_volume_settings

        if not shared_memory:
            final_tone_curve_hdr_component.bUseSharedMemory = False

        final_tone_curve_hdr_component.Initialize()

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and can fix occasional render-to-texture initialization issues on macOS
    for i in range(1):
        instance.flush()

    # get rendered frame using call_async API
    with instance.begin_frame():
        future = final_tone_curve_hdr_component.call_async.read_pixels()
    with instance.end_frame():
        data_bundle = future.get()

    # show debug data now that we're outside of instance.end_frame()
    spear.log('data_bundle["arrays"]["data"].flags["ALIGNED"]: ', data_bundle["arrays"]["data"].flags["ALIGNED"])

    # show rendered frame now that we're outside of with instance.end_frame()
    if not args.benchmark:
        cv2.imshow("final_tone_curve_hdr", data_bundle["arrays"]["data"])
        cv2.waitKey(0)

    # optional benchmarking
    if args.benchmark:

        # instance._client.get_timeout()
        num_steps = 100000
        start_time_seconds = time.time()
        for i in range(num_steps):
            instance._client.get_timeout()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average time for instance._client.get_timeout(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.engine_globals_service.get_current_process_id()
        num_steps = 1000
        start_time_seconds = time.time()
        for i in range(num_steps):
            instance.engine_globals_service.get_current_process_id()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average time for instance.engine_globals_service.get_current_process_id(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # empty with instance.begin_frame() / with instance.end_frame()
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                pass
            with instance.end_frame():
                pass
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for empty with instance.begin_frame() / with instance.end_frame(): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # game.unreal_service.get_default_object(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                pass
            with instance.end_frame():
                return_value = game.unreal_service.get_default_object(uclass="AActor")
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for game.unreal_service.get_default_object(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # game.unreal_service.call_async.get_default_object(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                future = game.unreal_service.call_async.get_default_object(uclass="AActor")
            with instance.end_frame():
                return_value = future.get()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for game.unreal_service.call_async.get_default_object(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_function(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                pass
            with instance.end_frame():
                data_bundle = final_tone_curve_hdr_component.read_pixels()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_function(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_async.call_function(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                future = final_tone_curve_hdr_component.call_async.read_pixels()
            with instance.end_frame():
                data_bundle = future.get()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_async.call_function(...): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

        # instance.sp_func_service.call_async.call_function(...)
        num_steps = 100
        start_time_seconds = time.time()
        for i in range(num_steps):
            with instance.begin_frame():
                future = final_tone_curve_hdr_component.call_async.read_pixels()
            with instance.end_frame(single_step=True):
                data_bundle = future.get()
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(f"Average frame time for instance.sp_func_service.call_async.call_function(...) (single-step): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} fps)")

    input("Press any key to continue...")

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    if args.benchmark:
        instance.close()

    spear.log("Done.")
