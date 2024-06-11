#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import json
import mmap
import numpy as np
import os
import spear

from examples.common.camera import CameraSensor

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--num_steps", default=10000)
    parser.add_argument("--keyboard", default=True)

    parser.add_argument("--use_force", default=True)
    parser.add_argument("--agent_class", default="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C")

    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

    spear.configure_system(config)
    instance = spear.Instance(config)

    agent_location = np.array([0, 0, 0])
    agent_rotation = np.array([0, 0, 0])

    # spawn agent
    instance.engine_service.begin_tick()

    agent = instance.unreal_service.spawn_actor(
        class_name=args.agent_class,
        location=dict(zip(["X", "Y", "Z"], agent_location.tolist())),
        rotation=dict(zip(["Roll", "Pitch", "Yaw"], agent_rotation.tolist())),
        spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
    )
    spear.log("agent", agent)
    if agent == 0:
        spear.log("Spawn agent failed.")
        instance.close()
        exit()

    # get access to camera sensor
    unreal_camera_sensor = CameraSensor(instance, agent)

    root_component = instance.unreal_service.get_component_by_name("UStaticMeshComponent", agent, "StaticMeshComponent")

    unreal_actor_static_class = instance.unreal_service.get_static_class("AActor")
    unreal_set_actor_location_and_rotation_func = instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")
    unreal_get_actor_location_func = instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class, name="K2_GetActorLocation")
    unreal_get_actor_rotation_func = instance.unreal_service.find_function_by_name(uclass=unreal_actor_static_class, name="K2_GetActorRotation")

    unreal_static_mesh_static_class = instance.unreal_service.get_static_class("UStaticMeshComponent")
    unreal_add_force_func = instance.unreal_service.find_function_by_name(uclass=unreal_static_mesh_static_class, name="AddForce")

    gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
    gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)
    set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, name="SetGamePaused")

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    img = np.zeros([unreal_camera_sensor._width, unreal_camera_sensor._height, 3])
    cv2.imshow('img', img)

    action = np.array([0, 0, 0])
    scale = 100

    for i in range(args.num_steps):
        instance.engine_service.begin_tick()
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        # apply action
        if args.use_force:
            add_force_args = {
                "Force": dict(zip(["X", "Y", "Z"], action.tolist()))
            }
            instance.unreal_service.call_function(root_component, unreal_add_force_func, add_force_args)
            action = np.array([0, 0, 0])
        else:
            update_location = current_location + action
            transform_args = {
                "NewLocation": dict(zip(["X", "Y", "Z"], update_location.tolist())),
                "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], current_rotation.tolist())),
                "bSweep": False,
                "bTeleport": True}
            instance.unreal_service.call_function(agent, unreal_set_actor_location_and_rotation_func, transform_args)

        instance.engine_service.tick()

        # get observation
        current_location = instance.unreal_service.call_function(agent, unreal_get_actor_location_func)['ReturnValue']
        current_location = json.loads(current_location)
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])

        current_rotation = instance.unreal_service.call_function(agent, unreal_get_actor_rotation_func)['ReturnValue']
        current_rotation = json.loads(current_rotation)
        current_rotation = np.array([current_rotation['roll'], current_rotation['yaw'], current_rotation['pitch']])

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

        img = unreal_camera_sensor.get_rgb_img()
        instance.engine_service.end_tick()

        cv2.imshow('img', img)
        k = cv2.waitKey(10)

        # generate action
        if args.keyboard:
            if k == 27:  # Esc key to stop
                break
            elif k == -1:  # normally -1 returned,so don't print it
                pass
            elif k == ord('w'):
                action += np.array([1, 0, 0]) * scale
            elif k == ord('s'):
                action += np.array([-1, 0, 0]) * scale
            elif k == ord('a'):
                action += np.array([0, -1, 0]) * scale
            elif k == ord('d'):
                action += np.array([0, 1, 0]) * scale
            else:
                pass
        else:
            action += np.array([1, 0, 0]) * scale
        print("step", i, current_location, current_rotation)

    instance.close()

    spear.log("Done.")
