#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
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
        location={"X": 0.0, "Y": 0.0, "Z": 100.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent"}
    )
    spear.log("agent", agent)

    unreal_actor_static_class = instance.unreal_service.get_static_class("AActor")
    unreal_set_actor_location_and_rotation_func = instance.unreal_service.find_function_by_name(
        uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    dummy_img = np.zeros([512, 512, 3])
    cv2.imshow('img', dummy_img)

    agent_location = np.array([0, 0, 0])
    speed = 10
    if agent == 0:
        spear.log("spawn agent failed")
        args.num_steps = 0

    for i in range(args.num_steps):
        # set updated poses in SPEAR
        spear.log("frame=", i)
        instance.engine_service.begin_tick()

        # TODO update transform or apply keyboard actions
        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], agent_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], [0, 0, 0])),
            "bSweep": False,
            "bTeleport": True}
        instance.unreal_service.call_function(agent, unreal_set_actor_location_and_rotation_func, transform_args)
        instance.engine_service.tick()
        instance.engine_service.end_tick()

        cv2.imshow('img', dummy_img)
        k = cv2.waitKey(10)

        if args.keyboard:
            if k == 27:  # Esc key to stop
                break
            elif k == -1:  # normally -1 returned,so don't print it
                pass
            elif k == ord('w'):
                agent_location += np.array([1, 0, 0]) * speed
            elif k == ord('s'):
                agent_location += np.array([-1, 0, 0]) * speed
            elif k == ord('a'):
                agent_location += np.array([0, -1, 0]) * speed
            elif k == ord('d'):
                agent_location += np.array([0, 1, 0]) * speed
            else:
                pass
        else:
            agent_location += np.array([1, 0, 0]) * speed


    instance.close()

    spear.log("Done.")
