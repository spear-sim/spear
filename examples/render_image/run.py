#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import math
import os
import spear
import time


if __name__ == "__main__":

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

        gameplay_statics_static_class = game.unreal_service.get_static_class(class_name="UGameplayStatics")
        get_player_controller_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="GetPlayerController")

        sp_game_viewport_client_static_class = game.unreal_service.get_static_class(class_name="USpGameViewportClient")
        get_viewport_size_func = game.unreal_service.find_function_by_name(uclass=sp_game_viewport_client_static_class, function_name="GetViewportSize")

        sp_scene_capture_component_2d_static_class = game.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # get default objects
        gameplay_statics_default_object = game.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)
        sp_game_viewport_client_default_object = game.unreal_service.get_default_object(uclass=sp_game_viewport_client_static_class, create_if_needed=False)

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_camera_sensor_actor, component_name="DefaultSceneRoot.final_tone_curve_hdr_")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)

        post_process_volume = game.unreal_service.find_actor_by_type(class_name="APostProcessVolume")
        return_values = game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func, args={"PlayerIndex": 0})
        player_controller = spear.to_handle(string=return_values["ReturnValue"])
        player_camera_manager_desc = game.unreal_service.find_property_by_name_on_object(uobject=player_controller, property_name="PlayerCameraManager")
        player_camera_manager_string = game.unreal_service.get_property_value(property_desc=player_camera_manager_desc)
        player_camera_manager = spear.to_handle(string=player_camera_manager_string)

        engine = instance.engine_service.get_engine()
        game_viewport_client_property_desc = game.unreal_service.find_property_by_name_on_object(uobject=engine, property_name="GameViewport")
        game_viewport_client_string = game.unreal_service.get_property_value(property_desc=game_viewport_client_property_desc)
        game_viewport_client = spear.to_handle(string=game_viewport_client_string)
        return_values = game.unreal_service.call_function(uobject=sp_game_viewport_client_default_object, ufunction=get_viewport_size_func, args={"GameViewportClient": game_viewport_client})
        viewport_size_x = return_values["ViewportSize"]["x"]
        viewport_size_y = return_values["ViewportSize"]["y"]
        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y

        view_target_pov_desc = game.unreal_service.find_property_by_name_on_object(uobject=player_camera_manager, property_name="ViewTarget.POV")
        view_target_pov = game.unreal_service.get_property_value(property_desc=view_target_pov_desc)

        fov = view_target_pov["fOV"]*math.pi/180.0 # this adjustment is necessary to compute an FOV value that matches the game viewport
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        volume_settings_desc = game.unreal_service.find_property_by_name_on_object(uobject=post_process_volume, property_name="Settings")
        volume_settings = game.unreal_service.get_property_value(property_desc=volume_settings_desc)

        game.unreal_service.call_function(uobject=bp_camera_sensor_actor, ufunction=set_actor_location_func, args={"NewLocation": view_target_pov["location"]})
        game.unreal_service.call_function(uobject=bp_camera_sensor_actor, ufunction=set_actor_rotation_func, args={"NewRotation": view_target_pov["rotation"]})

        width_desc = game.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="Width")
        height_desc = game.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="Height")
        fov_angle_desc = game.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="FOVAngle")
        component_settings_desc = game.unreal_service.find_property_by_name_on_object(uobject=final_tone_curve_hdr_component, property_name="PostProcessSettings")
        game.unreal_service.set_property_value(property_desc=width_desc, property_value=viewport_x)
        game.unreal_service.set_property_value(property_desc=height_desc, property_value=viewport_y)
        game.unreal_service.set_property_value(property_desc=fov_angle_desc, property_value=fov_adjusted_degrees)
        game.unreal_service.set_property_value(property_desc=component_settings_desc, property_value=volume_settings)

        # now that the final_tone_curve_hdr component is fully configured, initialize it and get handles to its shared memory
        game.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=initialize_func)
        final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_object(uobject=final_tone_curve_hdr_component)

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # # let temporal anti-aliasing etc accumulate additional information across multiple frames
    # for i in range(1):
    #     with instance.begin_frame():
    #         pass
    #     with instance.end_frame():
    #         pass

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        return_values = instance.sp_func_service.call_function(
            uobject=final_tone_curve_hdr_component,
            function_name="read_pixels",
            uobject_shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)

    # show debug data now that we're outside of instance.end_frame()
    spear.log('return_values["arrays"]["data"]: ')
    spear.log_no_prefix(return_values["arrays"]["data"])

    # show rendered frame now that we're outside of with instance.end_frame()
    cv2.imshow("final_tone_curve_hdr", return_values["arrays"]["data"])
    cv2.waitKey(0)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        instance.sp_func_service.destroy_shared_memory_handles_for_object(shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
        game.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=terminate_func)
        game.unreal_service.destroy_actor(actor=bp_camera_sensor_actor)

    spear.log("Done.")
