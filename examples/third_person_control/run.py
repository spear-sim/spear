#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import spear
import time

import pandas as pd


def get_action(row):
    names = [name[:-2] for name in row.dtype.names][::3]  # strip .x .y .z from each name, select every third entry
    data = np.array([row[name] for name in row.dtype.names], dtype=np.float64).reshape(-1, 3)  # get data as Nx3 array
    return dict(zip(names, data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # read pre-recorded actions from the actions file
    df = pd.read_csv(args.actions_file)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():
        gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        controller_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Controller")
        agent_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
        character_movement_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.CharacterMovementComponent")

        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, function_name="SetGamePaused")
        get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, function_name="GetPlayerController")
        possess_func = instance.unreal_service.find_function_by_name(uclass=controller_class, function_name="Possess")
        unpossess_func = instance.unreal_service.find_function_by_name(uclass=controller_class, function_name="UnPossess")
        set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_class, function_name="SetMovementMode")

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)
        player_controller = spear.func_utils.to_handle(instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func,
                                                                                             args={"PlayerIndex": 0})["ReturnValue"])

        agent = instance.unreal_service.spawn_actor_from_uclass(
            uclass=agent_class,
            location={"X": 0.0, "Y": 100.0, "Z": 150.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

        agent2 = instance.unreal_service.spawn_actor_from_uclass(
            uclass=agent_class,
            location={"X": 0.0, "Y": 200.0, "Z": 130.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent2", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

        agent3 = instance.unreal_service.spawn_actor_from_uclass(
            uclass=agent_class,
            location={"X": 0.0, "Y": 300.0, "Z": 130.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent3", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        # need player_controller current actor to inject input
        # instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})

        # set bRunPhysicsWithNoController to True to control pawn without controller
        character_movement_component = instance.unreal_service.get_component_by_class(agent, character_movement_component_class)
        instance.unreal_service.set_object_properties_for_uobject(character_movement_component, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

        character_movement_component2 = instance.unreal_service.get_component_by_class(agent2, character_movement_component_class)
        instance.unreal_service.set_object_properties_for_uobject(character_movement_component2, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component2, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

        character_movement_component3 = instance.unreal_service.get_component_by_class(agent3, character_movement_component_class)
        instance.unreal_service.set_object_properties_for_uobject(character_movement_component3, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component3, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

        # instance.enhanced_input_service.init_input_for_actor(agent3)
    with instance.end_frame():
        pass

    for count in range(0, 100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # with instance.begin_frame():
    #     instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    #
    #     instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})
    #     instance.enhanced_input_service.inject_input_for_blueprint_actor(
    #         actor=agent,
    #         input_action_name="IA_Jump",
    #         trigger_event="Started",
    #         input_action_value={},
    #         input_action_instance={},verbose=True)
    #     instance.unreal_service.call_function(uobject=player_controller, ufunction=unpossess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})
    # #
    # # with instance.end_frame():
    # #     instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
    # #
    # # for count in range(0, 100):
    # #     with instance.begin_frame():
    # #         instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    # #
    # #     with instance.end_frame():
    # #         instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
    # #
    # #
    # # with instance.begin_frame():
    # #     instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    #
    #     instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent2)})
    #     instance.enhanced_input_service.inject_input_for_blueprint_actor(
    #         actor=agent2,
    #         input_action_name="IA_Jump",
    #         trigger_event="Started",
    #         input_action_value={},
    #         input_action_instance={},verbose=True)
    #     instance.unreal_service.call_function(uobject=player_controller, ufunction=unpossess_func, args={"InPawn": spear.func_utils.to_ptr(agent2)})
    #
    # with instance.end_frame():
    #     instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # with instance.end_frame():
    #     instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for count in range(0, 100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
            if count == 0:
                instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})
            instance.enhanced_input_service.inject_input_for_actor(
                actor=agent,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 0, "Y": 1, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0,
                                       "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01},
                modifiers=[],
                triggers=[])
            # instance.unreal_service.call_function(uobject=player_controller, ufunction=unpossess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})
            #
            # instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent2)})
            # instance.enhanced_input_service.inject_input_for_blueprint_actor(
            #     actor=agent2,
            #     input_action_name="IA_Move",
            #     trigger_event="Triggered",
            #     input_action_value={"ValueType": "Axis2D", "Value": {"X": 0, "Y": 1, "Z": 0.0}},
            #     input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0,
            #                            "ElapsedProcessedTime": 0.1, "ElapsedTriggeredTime": 0.1},
            #     modifiers=[],
            #     triggers=[])
            # instance.unreal_service.call_function(uobject=player_controller, ufunction=unpossess_func, args={"InPawn": spear.func_utils.to_ptr(agent2)})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # close the instance
    instance.close()

    spear.log("Done.")
