#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear

import cv2

if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    # initialize actors and components
    with instance.begin_frame():

        # find functions
        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        get_actor_location_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_GetActorLocation")
        get_actor_rotation_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_GetActorRotation")
        set_actor_location_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorLocation")
        set_actor_rotation_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorRotation")

        sp_scene_capture_component_2d_static_class = instance.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")
        terminate_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # find actors
        post_process_volume = instance.unreal_service.find_actor_by_type(class_name="APostProcessVolume")
        spectator_pawn = instance.unreal_service.find_actor_by_name(class_name="AActor", actor_name="__SP_DEFAULT_PAWN__SpSpectatorPawn")

        # spawn camera sensor
        bp_camera_sensor_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Camera_Sensor.BP_Camera_Sensor_C")
        bp_camera_sensor = instance.unreal_service.spawn_actor_from_uclass(uclass=bp_camera_sensor_uclass)

        # get component
        final_tone_curve_hdr_component = instance.unreal_service.get_component_by_name(
            class_name="USceneComponent",
            actor=bp_camera_sensor,
            component_name="DefaultSceneRoot.final_tone_curve_hdr")

        # update camera sensor pose to match the spectator pawn
        spectator_pawn_location = instance.unreal_service.call_function(uobject=spectator_pawn, ufunction=get_actor_location_func)["ReturnValue"]
        spectator_pawn_rotation = instance.unreal_service.call_function(uobject=spectator_pawn, ufunction=get_actor_rotation_func)["ReturnValue"]
        instance.unreal_service.call_function(uobject=bp_camera_sensor, ufunction=set_actor_location_func, args={"NewLocation": spectator_pawn_location})
        instance.unreal_service.call_function(uobject=bp_camera_sensor, ufunction=set_actor_rotation_func, args={"NewRotation": spectator_pawn_rotation})

        # call Initialize() on component
        instance.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=initialize_func)

        # get handles to the component's shared memory
        final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_uobject(uobject=final_tone_curve_hdr_component)

        # update post-process settings on the component to match the post-process volume in the scene
        post_process_volume_settings_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=post_process_volume, property_name="Settings")
        final_tone_curve_hdr_settings_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=final_tone_curve_hdr_component, property_name="PostProcessSettings")

        post_process_volume_settings = instance.unreal_service.get_property_value(property_desc=post_process_volume_settings_desc)
        instance.unreal_service.set_property_value(property_desc=final_tone_curve_hdr_settings_desc, property_value=post_process_volume_settings)

    with instance.end_frame():
        pass # we could get rendered data here, but it will look better if we let temporal anti-aliasing accumulate additional information across frames

    # # let temporal anti-aliasing accumulate additional information across frames
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
            final_tone_curve_hdr_component,
            "read_pixels",
            uobject_shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
        cv2.imshow("final_tone_curve_hdr", return_values["arrays"]["data"])
        cv2.waitKey(0)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        instance.sp_func_service.destroy_shared_memory_handles_for_uobject(shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
        instance.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=terminate_func)
        instance.unreal_service.destroy_actor(actor=bp_camera_sensor)

    spear.log("Done.")
