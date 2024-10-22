#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse

import numpy as np
import os
import spear
import time

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    parser.add_argument("--bp_class_reference", default="/Game/VehicleTemplate/Blueprints/OffroadCar/OffroadCar_Pawn.OffroadCar_Pawn_C")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

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

    vehicle_movement_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/ChaosVehicles.ChaosVehicleMovementComponent", filename="")

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

    movement_component = instance.unreal_service.get_component_by_class(agent, vehicle_movement_component_class)
    set_throttle_input_func = instance.unreal_service.find_function_by_name(uclass=vehicle_movement_component_class, name="SetThrottleInput")
    set_steering_input_func = instance.unreal_service.find_function_by_name(uclass=vehicle_movement_component_class, name="SetSteeringInput")
    instance.unreal_service.set_object_properties_for_uobject(movement_component, {"bRequiresControllerForInputs": False})

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    spear.log("Executing sequence of actions as provided in the actions file...")

    frame = 0
    start_time_seconds = time.time()
    while frame < 200:
        instance.engine_service.begin_tick()
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        if frame < 300 and frame >= 10:
            result = instance.unreal_service.call_function(uobject=movement_component, ufunction=set_throttle_input_func,
                                                           args={"Throttle": 1.0, })
        if frame < 100 and frame >= 10:
            result = instance.unreal_service.call_function(uobject=movement_component, ufunction=set_steering_input_func,
                                                           args={"Steering": 1.0, })

        instance.engine_service.tick()

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
        instance.engine_service.end_tick()

        frame += 1
        spear.log("frame", frame)

    # close the instance
    instance.close()

    spear.log("Done.")
