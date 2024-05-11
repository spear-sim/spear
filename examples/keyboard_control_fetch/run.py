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

from examples.common import visualization_utils

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    spear.configure_system(config)
    instance = spear.Instance(config)
    env = spear.Env(instance, config)

    # reset the simulation
    obs = env.reset()

    spear.log("Executing sequence of actions as provided in the actions file...")

    # init action space
    action = {}
    for action_key, value in env.action_space.items():
        action[action_key] = np.array([0, 0, 0], dtype=np.float64)

    index = 0
    start_time_seconds = time.time()
    while True:
        index += 1
        obs, reward, done, info = env.step(action=action)

        obs_render_pass_vis = visualization_utils.get_final_color_image_for_visualization(obs["camera.final_color"])
        cv2.imshow('img', obs["camera.final_color"])
        k = cv2.waitKey(10)
        if k == 27:  # Esc key to stop
            break
        elif k == -1:  # normally -1 returned,so don't print it
            continue
        elif k == ord('w'):
            action['wheel_joint_l.set_angular_velocity_target'] = np.array([1, 0, 0], dtype=np.float64)
            action['wheel_joint_r.set_angular_velocity_target'] = np.array([1, 0, 0], dtype=np.float64)
        elif k == ord('s'):
            action['wheel_joint_l.set_angular_velocity_target'] = np.array([-1, 0, 0], dtype=np.float64)
            action['wheel_joint_r.set_angular_velocity_target'] = np.array([-1, 0, 0], dtype=np.float64)
        elif k == ord('a'):
            action['wheel_joint_l.set_angular_velocity_target'] = np.array([1, 0, 0], dtype=np.float64)
            action['wheel_joint_r.set_angular_velocity_target'] = np.array([-1, 0, 0], dtype=np.float64)
        elif k == ord('d'):
            action['wheel_joint_l.set_angular_velocity_target'] = np.array([-1, 0, 0], dtype=np.float64)
            action['wheel_joint_r.set_angular_velocity_target'] = np.array([1, 0, 0], dtype=np.float64)
        else:
            spear.log("unknown key", k)
        # print("obs", obs)
        # print("reward", reward)
        spear.log("step")

        if done:
            env.reset()
            # print("agent done???")
            # break

    # close the environment
    env.close()

    # close the instance
    instance.close()

    spear.log("Done.")
