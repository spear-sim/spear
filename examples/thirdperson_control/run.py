#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import sys

import numpy as np
import pandas as pd
import spear

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))

sys.path.append(common_dir)


def get_action(row):
    names = [name[:-2] for name in row.dtype.names][::3]  # strip .x .y .z from each name, select every third entry
    data = np.array([row[name] for name in row.dtype.names], dtype=np.float64).reshape(-1, 3)  # get data as Nx3 array
    return dict(zip(names, data))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    parser.add_argument("--bp_class_reference", default="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
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
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)

        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, function_name="SetGamePaused")

        get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, function_name="GetPlayerController")
        get_player_controller_return_values = instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func,
                                                                                    args={"PlayerIndex": 0})
        player_controller = spear.func_utils.to_handle(get_player_controller_return_values["ReturnValue"])
        controller_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Controller", filename="")
        possess_func = instance.unreal_service.find_function_by_name(uclass=controller_class, function_name="Possess")

        # get functions for control agent
        character_movement_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.CharacterMovementComponent", filename="")
        set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_class, function_name="SetMovementMode")
        # spawn a blueprint actor
        agent_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name=args.bp_class_reference, filename="")
        add_movement_input_func = instance.unreal_service.find_function_by_name(uclass=agent_class, function_name="AddMovementInput")
        jump_func = instance.unreal_service.find_function_by_name(uclass=agent_class, function_name="Jump")

        agent = instance.unreal_service.spawn_actor_from_uclass(
            uclass=agent_class,
            location={"X": 0.0, "Y": 300.0, "Z": 130.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

    with instance.end_frame():
        pass

    with instance.begin_frame():
        # need player_controller current actor to inject input
        # instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.func_utils.to_ptr(agent)})

        # set bRunPhysicsWithNoController to True to control pawn without controller
        character_movement_component = instance.unreal_service.get_component_by_class(agent, character_movement_component_class)
        instance.unreal_service.set_object_properties_for_uobject(character_movement_component, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

    with instance.end_frame():
        pass

    spear.log("Executing sequence of actions as provided in the actions file...")

    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        # instance.unreal_service.call_function(uobject=agent, ufunction=jump_func, args={})
        instance.enhanced_input_service.inject_input_for_blueprint_actor(
            actor=agent,
            input_action_name="IA_Jump",
            trigger_event="Triggered",
            input_action_value={},
            input_action_instance={})

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    frame = 0
    for frame in range(0, 100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
            instance.enhanced_input_service.inject_input_for_blueprint_actor(
                actor=agent,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 0.0, "Y": 1.0, "Z": 0.0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0,
                                       "ElapsedProcessedTime": 0.1, "ElapsedTriggeredTime": 0.1},
                modifiers=[],
                triggers=[]
            )
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for row in df.to_records(index=False):
        action = get_action(row)
        # print("action", action)
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            Jump_value = action['Jump']
            if Jump_value[0] != 0:
                # instance.unreal_service.call_function(uobject=agent, ufunction=jump_func, args={})
                instance.enhanced_input_service.inject_input_for_actor(
                    actor=agent,
                    input_action_name="IA_Jump",
                    trigger_event="Triggered",
                    input_action_value={},
                    input_action_instance={})

            AddMovementInput_value = action['AddMovementInput']
            print("AddMovementInput_value", AddMovementInput_value)
            if np.any(AddMovementInput_value):
                instance.enhanced_input_service.inject_input_for_actor(
                    actor=agent,
                    input_action_name="IA_Move",
                    trigger_event="Triggered",
                    input_action_value={"ValueType": "Axis2D", "Value": {"X": AddMovementInput_value[0], "Y": AddMovementInput_value[1], "Z": 0.0}},
                    input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0,
                                           "ElapsedProcessedTime": 0.1, "ElapsedTriggeredTime": 0.1},
                    modifiers=[],
                    triggers=[])

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

        spear.log("frame ", frame)
        frame += 1

    # close the instance
    instance.close()

    spear.log("Done.")
