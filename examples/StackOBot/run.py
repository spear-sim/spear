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

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        return_values = instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func, args={"PlayerIndex": 0})
        player_controller = spear.to_handle(return_values["ReturnValue"])

        # find character possessed by first player controller
        return_values = instance.unreal_service.call_function(uobject=player_controller, ufunction=get_pawn_func)
        character = spear.to_handle(return_values["ReturnValue"])


    with instance.end_frame():
        pass

    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            instance.enhanced_input_service.inject_input_for_actor(
                actor=character,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
            instance.enhanced_input_service.inject_input_for_actor(
                actor=character,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 0.0, "Y": -1.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
            instance.enhanced_input_service.inject_input_for_actor(
                actor=character,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # unpause now that we're finished controlling the car
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    with instance.end_frame():
        pass

    spear.log("Done.")
