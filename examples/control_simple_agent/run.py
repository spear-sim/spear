#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os
import spear


num_steps = 150


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    # initialize actors and components
    with instance.begin_frame():

        # find functions
        gameplay_statics_static_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="SetGamePaused")

        scene_component_static_class = instance.unreal_service.get_static_class(class_name="USceneComponent")
        add_relative_rotation_func = instance.unreal_service.find_function_by_name(uclass=scene_component_static_class, function_name="K2_AddRelativeRotation")
        get_component_rotation_func = instance.unreal_service.find_function_by_name(uclass=scene_component_static_class, function_name="K2_GetComponentRotation")

        primitive_component_static_class = instance.unreal_service.get_static_class(class_name="UPrimitiveComponent")
        add_force_func = instance.unreal_service.find_function_by_name(uclass=primitive_component_static_class, function_name="AddForce")

        sp_scene_capture_component_2d_static_class = instance.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = instance.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # get UGameplayStatics default object
        gameplay_statics = instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class)

        # spawn object and get components
        bp_sphere_agent_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Sphere_Agent.BP_Sphere_Agent_C")
        bp_sphere_agent = instance.unreal_service.spawn_actor_from_uclass(uclass=bp_sphere_agent_uclass, location={"X": -10.0, "Y": 280.0, "Z": 150.0})
        root_component = instance.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_sphere_agent, component_name="DefaultSceneRoot")        
        final_tone_curve_hdr_component = instance.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_sphere_agent, component_name="DefaultSceneRoot.final_tone_curve_hdr")

        # get sphere component by path because it gets relocated in the component hierarchy implicitly by Unreal
        sphere_component = instance.unreal_service.get_component_by_path(class_name="USceneComponent", actor=bp_sphere_agent, component_path="DefaultSceneRoot.sphere_")

        # initialize final_tone_curve_hdr_component and get handles to its shared memory
        instance.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=initialize_func)
        final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_uobject(uobject=final_tone_curve_hdr_component)

        # show FPS
        instance.unreal_service.execute_console_command("stat fps")

    with instance.end_frame():
        pass

    # take a few steps
    for i in range(num_steps):

        # apply action
        with instance.begin_frame():

            # unpause
            instance.unreal_service.call_function(uobject=gameplay_statics, ufunction=set_game_paused_func, args={"bPaused": False})

            # add rotation
            instance.unreal_service.call_function(uobject=root_component, ufunction=add_relative_rotation_func, args={"DeltaRotation": {"Pitch": 0.0, "Yaw": 1.0, "Roll": 0.0}})

            # add force
            return_values = instance.unreal_service.call_function(uobject=root_component, ufunction=get_component_rotation_func)
            M_world_from_component = spear.to_matrix_from_rotator(return_values["ReturnValue"])
            force_component = np.matrix([1000.0, 0.0, 0.0]).T
            force_world = M_world_from_component*force_component
            instance.unreal_service.call_function(uobject=sphere_component, ufunction=add_force_func, args={"Force": spear.to_vector_from_array(force_world)})

        # get observation
        with instance.end_frame():

            # read pixels
            return_values = instance.sp_func_service.call_function(
                uobject=final_tone_curve_hdr_component,
                function_name="read_pixels",
                uobject_shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)

            # pause
            instance.unreal_service.call_function(uobject=gameplay_statics, ufunction=set_game_paused_func, args={"bPaused": True})

        # show image in an OpenCV window
        cv2.imshow("final_tone_curve_hdr", return_values["arrays"]["data"])
        cv2.waitKey(0)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        instance.sp_func_service.destroy_shared_memory_handles_for_uobject(shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
        instance.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=terminate_func)
        instance.unreal_service.destroy_actor(actor=bp_sphere_agent)

    # close OpenCV window
    cv2.destroyAllWindows()

    spear.log("Done.")
