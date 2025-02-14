#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear

if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():

        # find functions
        gameplay_statics_static_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="GetPlayerController")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="SetGamePaused")

        player_controller_static_class = instance.unreal_service.get_static_class(class_name="APlayerController")
        get_pawn_func = instance.unreal_service.find_function_by_name(uclass=player_controller_static_class, function_name="K2_GetPawn")
        possess_func = instance.unreal_service.find_function_by_name(uclass=player_controller_static_class, function_name="Possess")

        actor_static_class = instance.unreal_service.get_static_class(class_name="AActor")
        get_actor_transform_func = instance.unreal_service.find_function_by_name(uclass=actor_static_class, function_name="GetTransform")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        return_values = instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func, args={"PlayerIndex": 0})
        player_controller = spear.to_handle(return_values["ReturnValue"])

        # find current possessed by first player controller
        return_values = instance.unreal_service.call_function(uobject=player_controller, ufunction=get_pawn_func)
        character = spear.to_handle(return_values["ReturnValue"])

        return_values = instance.unreal_service.call_function(uobject=character, ufunction=get_actor_transform_func)
        character_transform = return_values["ReturnValue"]

        translation = character_transform['translation']
        print("character_transform", character_transform)
        translation['z'] += 100
        rotation = {}
        # spawn a new car with

        bp_drone_uclass = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Gameplay/Character/BP_CitySampleHoverDrone.BP_CitySampleHoverDrone_C")

        drone = instance.unreal_service.spawn_actor_from_class(
            uclass=bp_drone_uclass,
            location=character_transform['translation'],
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})

        # If we spawned a new car, then we would need to possess it as follows:
        instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.to_ptr(drone)})

    with instance.end_frame():
        pass

    # move up
    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            instance.enhanced_input_service.inject_input_for_actor(
                actor=drone,
                input_action_name="IA_HD_ChangeAltitude",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # move forward
    for _ in range(100):
        # set throttle to 1.0 and steering to -1.0 (values will persist across frames until reset)
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            instance.enhanced_input_service.inject_input_for_actor(
                actor=drone,
                input_action_name="IA_HD_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 1.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # unpause now that we're finished controlling the drone
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    with instance.end_frame():
        pass

    spear.log("Done.")
