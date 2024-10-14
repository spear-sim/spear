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
    parser.add_argument("--bp_class_reference", default="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
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
    possess_func = instance.unreal_service.find_function_by_name(uclass=controller_class, name="Possess")

    # spawn a blueprint actor
    agent_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name=args.bp_class_reference, filename="")
    agent = instance.unreal_service.spawn_actor_from_uclass(
        uclass=agent_uclass,
        location={"X": 0.0, "Y": 300.0, "Z": 130.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        spawn_parameters={"Name": "BP_Actor", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
    )

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    instance.engine_service.begin_tick()

    # TODO PlayerController possess agent, otherwise nothing happens when invoke Jump
    instance.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": instance.unreal_service.to_ptr(agent)})
    jump_func = instance.unreal_service.find_function_by_name(uclass=agent_uclass, name="Jump")

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    spear.log("Executing sequence of actions as provided in the actions file...")

    frame = 0
    start_time_seconds = time.time()
    while frame < 500:
        instance.engine_service.begin_tick()
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        # TODO apply action
        if frame == 100:
            result = instance.unreal_service.call_function(uobject=agent, ufunction=jump_func, args={})
            print("result", result)

        instance.engine_service.tick()

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
        instance.engine_service.end_tick()

        frame += 1
        spear.log("frame", frame)

    # close the instance
    instance.close()

    spear.log("Done.")
