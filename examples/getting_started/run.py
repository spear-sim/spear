#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import json

import cv2
import numpy as np
import os
import spear
import time

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--agent_class", default="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C")
    parser.add_argument("--num_steps", default=10000)
    parser.add_argument("--keyboard", default=True)
    parser.add_argument("--use_force", default=True)
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

    spear.configure_system(config)
    instance = spear.Instance(config)

    # spawn agent
    instance.engine_service.begin_tick()

    agent = instance.unreal_service.spawn_actor(
        class_name=args.agent_class,
        location={"X": 0.0, "Y": 0.0, "Z": 0.0},
        rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
    )
    spear.log("agent", agent)
    if agent == 0:
        spear.log("spawn agent failed")
        instance.close()
        exit()

    camera_sensor_component = instance.unreal_service.get_component_by_type("UCameraSensorComponent", agent)
    camera_sensor_component_class = instance.unreal_service.get_class(camera_sensor_component)
    print("camera_sensor_component_class", camera_sensor_component_class)
    unreal_get_observation_func = instance.unreal_service.find_function_by_name(uclass=camera_sensor_component_class, name="getObservation")
    print("unreal_get_observation_func", unreal_get_observation_func)

    root_component = instance.unreal_service.get_component_by_name("UStaticMeshComponent", agent, "StaticMeshComponent")

    unreal_actor_static_class = instance.unreal_service.get_static_class("AActor")
    unreal_set_actor_location_and_rotation_func = instance.unreal_service.find_function_by_name(
        uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")
    unreal_get_actor_location_func = instance.unreal_service.find_function_by_name(
        uclass=unreal_actor_static_class, name="K2_GetActorLocation")
    unreal_get_actor_rotation_func = instance.unreal_service.find_function_by_name(
        uclass=unreal_actor_static_class, name="K2_GetActorRotation")

    unreal_static_mesh_static_class = instance.unreal_service.get_static_class("UStaticMeshComponent")
    unreal_add_force_func = instance.unreal_service.find_function_by_name(
        uclass=unreal_static_mesh_static_class, name="AddForce")

    gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
    gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)
    set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, name="SetGamePaused")

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    dummy_img = np.zeros([512, 512, 3])
    cv2.imshow('img', dummy_img)

    action = np.array([0, 0, 0])
    current_location = np.array([0, 0, 0])
    current_rotation = np.array([0, 0, 0])
    scale = 100

    for i in range(args.num_steps):
        # set updated poses in SPEAR
        spear.log("frame=", i)
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

        current_location = instance.unreal_service.call_function(agent, unreal_get_actor_location_func)['ReturnValue']
        current_location = json.loads(current_location)
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        print("current_location", current_location)

        current_rotation = instance.unreal_service.call_function(agent, unreal_get_actor_rotation_func)['ReturnValue']
        current_rotation = json.loads(current_rotation)
        current_rotation = np.array([current_rotation['roll'], current_rotation['yaw'], current_rotation['pitch']])
        print("current_rotation", current_rotation)

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

        rgb_data = instance.unreal_service.call_function(uobject=camera_sensor_component, ufunction=unreal_get_observation_func)['ReturnValue']
        rgb_data = json.loads(rgb_data)
        img = np.zeros([512, 512, 4])
        i = 0
        for x in range(0, 512):
            for y in range(0, 512):
                val = rgb_data[x * 512 + y]
                img[x, y, :] = np.array([val['b'], val['g'], val['r'], val['a']]) / 255.0
                i += 1
        print("rgb_data", rgb_data)
        instance.engine_service.end_tick()

        # TODO get image obs
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

    instance.close()

    spear.log("Done.")
