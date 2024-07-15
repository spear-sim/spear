#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear

from examples.common.agent import SimpleAgent, OpenBotAgent, SimpleForceAgent, HabitatNavAgent, UrdfRobotAgent
from examples.common.camera import CameraSensor

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)


def get_keyboard_action(k, agent_type):
    action = {}
    if agent_type == "simple":
        action = {
            "add_to_location": np.array([0, 0, 0])
        }
        if k == ord('w'):
            action['add_to_location'] = np.array([1, 0, 0]) * 100
        elif k == ord('s'):
            action['add_to_location'] = np.array([-1, 0, 0]) * 100
        elif k == ord('a'):
            action['add_to_location'] = np.array([0, -1, 0]) * 100
        elif k == ord('d'):
            action['add_to_location'] = np.array([0, 1, 0]) * 100

    elif agent_type == "simple_force":
        action = {
            "add_force": np.array([0]),
            "add_torque": np.array([0]),
        }
        if k == ord('w'):
            action['add_force'] = np.array([1])
        elif k == ord('s'):
            action['add_force'] = np.array([-1])
        elif k == ord('a'):
            action['add_torque'] = np.array([-1])
        elif k == ord('d'):
            action['add_torque'] = np.array([1])

    elif agent_type == "habitat":
        action = {
            "move_forward": np.array([0, ]),
            "move_right": np.array([0, ]),
        }
        if k == ord('w'):
            action['move_forward'] = np.array([10, ])
        elif k == ord('s'):
            action['move_forward'] = np.array([-10, ])
        elif k == ord('a'):
            action['move_right'] = np.array([-15, ])
        elif k == ord('d'):
            action['move_right'] = np.array([15, ])
    elif agent_type == "openbot":
        action = {
            "apply_voltage": np.array([0.0, 0.0, ]),
            # "set_brake_torque": np.array([0.0, 0.0, ]),
        }
        if k == ord('w'):
            action['apply_voltage'] = np.array([1.0, 1.0])
        elif k == ord('s'):
            action['apply_voltage'] = np.array([-1.0, -1.0, ])
        elif k == ord('a'):
            action['apply_voltage'] = np.array([-1.0, 1.0, ])
        elif k == ord('d'):
            action['apply_voltage'] = np.array([1.0, -1.0, ])
        elif k == 32:
            action['set_brake_torque'] = np.array([1.0, 1.0, ])
    elif agent_type == "urdf":
        action = {
            "wheel_joint_l": np.array([0.0, ]),
            "wheel_joint_r": np.array([0.0, ]),
        }
        if k == ord('w'):
            action['wheel_joint_l'] = np.array([1.0, ])
            action['wheel_joint_r'] = np.array([1.0, ])
        elif k == ord('s'):
            action['wheel_joint_l'] = np.array([-1.0, ])
            action['wheel_joint_r'] = np.array([-1.0, ])
        elif k == ord('a'):
            action['wheel_joint_l'] = np.array([1.0, ])
            action['wheel_joint_r'] = np.array([-1.0, ])
        elif k == ord('d'):
            action['wheel_joint_l'] = np.array([-1.0, ])
            action['wheel_joint_r'] = np.array([1.0, ])
    else:
        pass
    return action


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true", default=False)
    parser.add_argument("--num_steps", default=10000)
    parser.add_argument("--keyboard", default=True)
    parser.add_argument("--agent", default="urdf")

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

    try:
        instance.engine_service.begin_tick()

        # spawn agent
        agent = None
        if args.agent == "simple":
            agent = SimpleAgent(instance)
        elif args.agent == "simple_force":
            agent = SimpleForceAgent(instance)
        elif args.agent == "habitat":
            agent = HabitatNavAgent(instance)
        elif args.agent == "openbot":
            agent = OpenBotAgent(instance)
        elif args.agent == "urdf":
            agent = UrdfRobotAgent(instance)
        else:
            spear.log("Unknown agent: ", args.agent)
            exit(-1)

        # get access to camera sensor
        unreal_camera_sensor = CameraSensor(instance, agent._agent, render_pass_names=["final_color", "depth", "normal", "segmentation"])

        obs = agent.get_observation()
        action = get_keyboard_action(-1, args.agent)

        gameplay_statics_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_class, create_if_needed=False)
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_class, name="SetGamePaused")

        unreal_actors = instance.unreal_service.find_actors_by_type_as_dict(class_name="AActor")
        print("unreal_actors", unreal_actors)
        unreal_actors_map = {v: k for k, v in unreal_actors.items()}

        unreal_obstacle_actor = instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_Obstacle.BP_Obstacle_C",
            location={"X": 100.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Obstacle", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        for unreal_actor_name, unreal_actor_ptr in unreal_actors.items():
            assert unreal_actor_name == instance.unreal_service.get_stable_name_for_actor(unreal_actor_ptr)
        instance.engine_service.tick()
        instance.engine_service.end_tick()

        img = np.zeros([320, 240, 1])
        if not args.benchmark:
            cv2.imshow('img', img)

        for i in range(args.num_steps):
            instance.engine_service.begin_tick()
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            hit_actors = agent.get_hit_actors()
            if len(hit_actors) > 0:
                hit_actor_names = []
                if len(hit_actors) > 2:
                    print("hit_actors", hit_actors)
                for hit_actor in hit_actors:
                    if hit_actor in unreal_actors_map:
                        hit_actor_name = unreal_actors_map[hit_actor]
                        hit_actor_name_list = hit_actor_name.split("/")
                        if len(hit_actor_name_list) >= 2 and hit_actor_name_list[1] == "02_floor":
                            pass
                        else:
                            hit_actor_names.append(hit_actor_name)
                    else:
                        print("hit ", hit_actor)
                        if hit_actor == unreal_obstacle_actor:
                            hit_actor_names.append("obstacle")
                if len(hit_actor_names) > 0:
                    spear.log("hit!", len(hit_actor_names), hit_actor_names)

            # apply actions
            agent.apply_action(action)

            instance.engine_service.tick()

            # get observation after tick
            obs = agent.get_observation()
            image = unreal_camera_sensor.get_images()

            path_between_agent_and_origin = agent.get_path_between_points(obs["location"], np.array([0.0, 0.0, 0.0]))
            print("path_between_agent_and_origin", path_between_agent_and_origin)

            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
            instance.engine_service.end_tick()

            # generate action
            if not args.benchmark:
                # print("image",image.shape)
                cv2.imshow('img', image)
                k = cv2.waitKey(10)
                if args.keyboard:
                    if k == 27:  # Esc key to stop
                        break
                    if k == ord('r'):  # random reset agent location
                        instance.engine_service.begin_tick()
                        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
                        agent.reset()
                        instance.engine_service.tick()
                        agent.get_observation()
                        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
                        instance.engine_service.end_tick()
                    else:
                        # update action based on keyboard input
                        action = get_keyboard_action(k, args.agent)
            else:
                # apply fixed action
                action = get_keyboard_action(ord('w'), args.agent)
    finally:
        instance.close()

        spear.log("Done.")
