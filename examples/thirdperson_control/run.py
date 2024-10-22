#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse

import numpy as np
import os

import pandas as pd
import spear
import time

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

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
    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml")),
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    spear.configure_system(config)
    instance = spear.Instance(config)

    instance.engine_service.begin_tick()

    gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
    gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)

    set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, name="SetGamePaused")

    get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, name="GetPlayerController")
    get_player_controller_return_values = instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func,
                                                                                args={"PlayerIndex": 0})
    player_controller = instance.unreal_service.to_handle(get_player_controller_return_values["ReturnValue"])
    controller_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Controller", filename="")

    enhanced_input_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/EnhancedInput.EnhancedInputComponent", filename="")
    enhanced_input_component = instance.unreal_service.get_component_by_class(player_controller, enhanced_input_component_class)
    print("enhanced_input_component", enhanced_input_component)

    character_movement_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.CharacterMovementComponent", filename="")
    set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_class, name="SetMovementMode")

    # spawn a blueprint actor
    agent_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name=args.bp_class_reference, filename="")
    agent = instance.unreal_service.spawn_actor_from_uclass(
        uclass=agent_uclass,
        location={"X": 0.0, "Y": 300.0, "Z": 130.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
    )

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    instance.engine_service.begin_tick()

    # set bRunPhysicsWithNoController to True
    movement_component = instance.unreal_service.get_component_by_class(agent, character_movement_component_class)
    instance.unreal_service.set_object_properties_for_uobject(movement_component, {"bRunPhysicsWithNoController": True})
    instance.unreal_service.call_function(uobject=movement_component, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

    add_movement_input_func = instance.unreal_service.find_function_by_name(uclass=agent_uclass, name="AddMovementInput")
    jump_func = instance.unreal_service.find_function_by_name(uclass=agent_uclass, name="Jump")

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    spear.log("Executing sequence of actions as provided in the actions file...")

    frame = 0
    start_time_seconds = time.time()
    for row in df.to_records(index=False):
        action = get_action(row)
        # print("action", action)
        instance.engine_service.begin_tick()
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        AddMovementInput_value = action['AddMovementInput']
        AddMovementInputScaleValue = np.linalg.norm(AddMovementInput_value)
        if AddMovementInputScaleValue != 0:
            AddMovementInputWorldDirection = AddMovementInput_value / AddMovementInputScaleValue
            instance.unreal_service.call_function(uobject=agent, ufunction=add_movement_input_func, args={
                "WorldDirection": {"X": AddMovementInputWorldDirection[0], "Y": AddMovementInputWorldDirection[1], "Z": AddMovementInputWorldDirection[2]},
                "ScaleValue": AddMovementInputScaleValue * 10,
            })

        Jump_value = action['Jump']
        if Jump_value[0] != 0:
            instance.unreal_service.call_function(uobject=agent, ufunction=jump_func, args={})

        instance.engine_service.tick()

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
        instance.engine_service.end_tick()

        spear.log("frame ", frame)
        frame += 1

    # close the instance
    instance.close()

    spear.log("Done.")
