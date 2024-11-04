#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import math
import os
import spear


if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    # initialize actors and components
    with instance.begin_frame():

        # find functions
        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        set_actor_location_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorLocation")
        set_actor_rotation_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="K2_SetActorRotation")

        sp_scene_capture_component_2d_static_class = instance.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # find actors
        post_process_volume = instance.unreal_service.find_actor_by_type(class_name="APostProcessVolume")

        player_controller = instance.world_service.get_first_player_controller()
        player_camera_manager_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=player_controller, property_name="PlayerCameraManager")
        player_camera_manager = spear.to_handle(instance.unreal_service.get_property_value(property_desc=player_camera_manager_desc))

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Camera_Sensor.BP_Camera_Sensor_C")
        bp_camera_sensor = instance.unreal_service.spawn_actor_from_uclass(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = instance.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr")

        # update camera sensor to match the player camera manager and the post-process volume in the scene
        viewport_size = instance.engine_service.get_viewport_size()
        viewport_x = viewport_size[0]
        viewport_y = viewport_size[1]
        aspect_ratio = viewport_x/viewport_y

        view_target_pov_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=player_camera_manager, property_name="ViewTarget.POV")
        view_target_pov = instance.unreal_service.get_property_value(property_desc=view_target_pov_desc)

        # this logic is necessary to determine the final FOV value that matches the game viewport
        fov = view_target_pov["fOV"]*math.pi/180.0
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi
        fov_value = fov_adjusted_degrees

        instance.unreal_service.call_function(uobject=bp_camera_sensor, ufunction=set_actor_location_func, args={"NewLocation": view_target_pov["location"]})
        instance.unreal_service.call_function(uobject=bp_camera_sensor, ufunction=set_actor_rotation_func, args={"NewRotation": view_target_pov["rotation"]})
        fov_angle_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=final_tone_curve_hdr_component, property_name="FOVAngle")
        instance.unreal_service.set_property_value(property_desc=fov_angle_desc, property_value=fov_value)

        width_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=final_tone_curve_hdr_component, property_name="Width")
        height_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=final_tone_curve_hdr_component, property_name="Height")
        instance.unreal_service.set_property_value(property_desc=width_desc, property_value=viewport_x)
        instance.unreal_service.set_property_value(property_desc=height_desc, property_value=viewport_y)

        volume_settings_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=post_process_volume, property_name="Settings")
        volume_settings = instance.unreal_service.get_property_value(property_desc=volume_settings_desc)
        component_settings_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=final_tone_curve_hdr_component, property_name="PostProcessSettings")
        instance.unreal_service.set_property_value(property_desc=component_settings_desc, property_value=volume_settings)

        # call Initialize() on component
        instance.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=initialize_func)

        # get handles to the component's shared memory
        final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_uobject(uobject=final_tone_curve_hdr_component)

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing accumulate additional information across frames

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

    # show rendered frame now that we're outside of with instance.end_frame()
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
