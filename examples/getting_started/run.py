#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear

from examples.common.agent import SimpleAgent, OpenBotAgent, SimpleForceAgent, HabitatNavAgent
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
            "add_force": np.array([0, 0, 0])
        }
        if k == ord('w'):
            action['add_force'] = np.array([1, 0, 0]) * 100
        elif k == ord('s'):
            action['add_force'] = np.array([-1, 0, 0]) * 100
        elif k == ord('a'):
            action['add_force'] = np.array([0, -1, 0]) * 100
        elif k == ord('d'):
            action['add_force'] = np.array([0, 1, 0]) * 100

    elif agent_type == "habitat":
        action = {
            "move_foward": np.array([0, ]),
            "move_left": np.array([0, ]),
            "move_right": np.array([0, ]),
        }
        if k == ord('w'):
            action['move_foward'] = np.array([10, ])
        elif k == ord('s'):
            action['move_foward'] = np.array([-10, ])
        elif k == ord('a'):
            action['move_left'] = np.array([15, ])
        elif k == ord('d'):
            action['move_right'] = np.array([15, ])
    elif agent_type == "openbot":
        action = {
            "set_drive_torque": np.zeros([4, ]),
            "set_brake_torque": np.zeros([4, ]),
        }
        if k == ord('w'):
            action['set_drive_torque'] = np.array([1, 1, 1, 1]) * 0.1
        elif k == ord('s'):
            action['set_drive_torque'] = np.array([-1, -1, -1, -1]) * 0.1
        elif k == ord('a'):
            action['set_drive_torque'] = np.array([-1, 1, -1, 1]) * 0.05
        elif k == ord('d'):
            action['set_drive_torque'] = np.array([1, -1, 1, -1]) * 0.05
        elif k == 32:
            action['set_brake_torque'] = np.array([1, 1, 1, 1]) * 0.1
    else:
        pass
    return action


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--num_steps", default=10000)
    parser.add_argument("--keyboard", default=True)
    parser.add_argument("--agent", default="habitat")

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

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    img = np.zeros([unreal_camera_sensor._width, unreal_camera_sensor._height, 3])
    if not args.benchmark:
        cv2.imshow('img', img)

    for i in range(args.num_steps):
        instance.engine_service.begin_tick()
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        hit_actors = agent.get_hit_actors()
        if len(hit_actors) > 0:
            spear.log("hit!", len(hit_actors), hit_actors)

        # apply actions
        agent.apply_action(action)

        instance.engine_service.tick()

        # get observation after tick
        obs = agent.get_observation()
        image = unreal_camera_sensor.get_images(render_pass_name='final_color')

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})
        instance.engine_service.end_tick()

        # generate action
        if not args.benchmark:
            cv2.imshow('img', image)
            k = cv2.waitKey(10)
            if args.keyboard:
                if k == 27:  # Esc key to stop
                    break
                if k == ord('r'):  # random reset agent location
                    instance.engine_service.begin_tick()
                    agent.reset()
                    instance.engine_service.tick()
                    agent.get_observation()
                    instance.engine_service.end_tick()
                else:
                    # update action based on keyboard input
                    action = get_keyboard_action(k, args.agent)
        else:
            # apply fixed action
            action = get_keyboard_action(ord('w'), args.agent)

    instance.close()

    spear.log("Done.")
