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
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # find functions
        gameplay_statics_static_class = game.unreal_service.get_static_class(class_name="UGameplayStatics")
        set_game_paused_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="SetGamePaused")

        scene_component_static_class = game.unreal_service.get_static_class(class_name="USceneComponent")
        add_relative_rotation_func = game.unreal_service.find_function_by_name(uclass=scene_component_static_class, function_name="K2_AddRelativeRotation")
        get_component_rotation_func = game.unreal_service.find_function_by_name(uclass=scene_component_static_class, function_name="K2_GetComponentRotation")

        primitive_component_static_class = game.unreal_service.get_static_class(class_name="UPrimitiveComponent")
        add_force_func = game.unreal_service.find_function_by_name(uclass=primitive_component_static_class, function_name="AddForce")

        sp_scene_capture_component_2d_static_class = game.unreal_service.get_static_class(class_name="USpSceneCaptureComponent2D")
        initialize_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Initialize")
        terminate_func = game.unreal_service.find_function_by_name(uclass=sp_scene_capture_component_2d_static_class, function_name="Terminate")

        # get UGameplayStatics default object
        gameplay_statics = game.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        # spawn object and get components
        bp_sphere_agent_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Sphere_Agent.BP_Sphere_Agent_C")
        bp_sphere_agent_actor = game.unreal_service.spawn_actor_from_class(uclass=bp_sphere_agent_uclass, location={"X": -10.0, "Y": 280.0, "Z": 150.0})
        root_component = game.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_sphere_agent_actor, component_name="DefaultSceneRoot")        
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(class_name="USceneComponent", actor=bp_sphere_agent_actor, component_name="DefaultSceneRoot.final_tone_curve_hdr_")

        #
        # We get the sphere component "by path" (instead of "by name") because this particular component gets
        # automatically relocated in the component hierarchy by Unreal at some point during BeginPlay().
        # Unreal performs this relocation operation because the component is physics-enabled. Searching by
        # path will look for the component at the path "DefaultSceneRoot.sphere_" (its original path before
        # getting relocated) and the path "sphere_" (its path after getting relocated).
        #
        # At this point in the code, we are certain that the relocation operation has already happened, so we
        # could just search by name using the name "sphere_". But this would be confusing. And we have a
        # dedicated C++ function for searching by path, which is helpful, e.g., inside BeginPlay() functions,
        # since in this case it isn't clear if a relocation operation has happened yet. We choose to use this
        # functionality here, so we can search for the component using it's original path, as a matter of
        # readability.
        #

        sphere_component = game.unreal_service.get_component_by_path(class_name="USceneComponent", actor=bp_sphere_agent_actor, component_path="DefaultSceneRoot.sphere_")

        # initialize final_tone_curve_hdr_component and get handles to its shared memory
        game.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=initialize_func)
        final_tone_curve_hdr_component_shared_memory_handles = instance.sp_func_service.create_shared_memory_handles_for_object(uobject=final_tone_curve_hdr_component)

        # show FPS
        game.unreal_service.execute_console_command("stat fps")

    with instance.end_frame():
        pass

    # take a few steps
    for i in range(num_steps):

        # apply action
        with instance.begin_frame():

            # unpause
            game.unreal_service.call_function(uobject=gameplay_statics, ufunction=set_game_paused_func, args={"bPaused": False})

            # add rotation
            game.unreal_service.call_function(uobject=root_component, ufunction=add_relative_rotation_func, args={"DeltaRotation": {"Pitch": 0.0, "Yaw": 1.0, "Roll": 0.0}})

            # add force
            return_values = game.unreal_service.call_function(uobject=root_component, ufunction=get_component_rotation_func)
            M_world_from_component = spear.to_matrix_from_rotator(rotator=return_values["ReturnValue"])
            force_component = np.matrix([1000.0, 0.0, 0.0]).T
            force_world = M_world_from_component*force_component
            game.unreal_service.call_function(uobject=sphere_component, ufunction=add_force_func, args={"Force": spear.to_vector_from_array(array=force_world)})

        # get observation
        with instance.end_frame():

            # read pixels
            return_values = instance.sp_func_service.call_function(
                uobject=final_tone_curve_hdr_component,
                function_name="read_pixels",
                uobject_shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)

            # pause
            game.unreal_service.call_function(uobject=gameplay_statics, ufunction=set_game_paused_func, args={"bPaused": True})

        # show image in an OpenCV window
        cv2.imshow("final_tone_curve_hdr", return_values["arrays"]["data"])
        cv2.waitKey(0)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        instance.sp_func_service.destroy_shared_memory_handles_for_object(shared_memory_handles=final_tone_curve_hdr_component_shared_memory_handles)
        game.unreal_service.call_function(uobject=final_tone_curve_hdr_component, ufunction=terminate_func)
        game.unreal_service.destroy_actor(actor=bp_sphere_agent_actor)

    # close OpenCV window
    cv2.destroyAllWindows()

    spear.log("Done.")
