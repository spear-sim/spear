#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import argparse
import os
import numpy as np
import pandas as pd

# fetch arm poses come from https://github.com/StanfordVL/iGibson/blob/master/igibson/robots/fetch.py#L100
arm_poses = {
    "init": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    "default": np.array([-1.17079, 1.47079, 0.4, 1.67079, 0.0, 1.57079, 0.0]),
    "vertical": np.array([-0.94121, -0.64134, 1.55186, 1.65672, -0.93218, 1.53416, 2.14474]),
    "diagonal15": np.array([-0.95587, -0.34778, 1.46388, 1.47821, -0.93813, 1.4587, 1.9939]),
    "diagonal30": np.array([-1.06595, -0.22184, 1.53448, 1.46076, -0.84995, 1.36904, 1.90996]),
    "diagonal45": np.array([-1.11479, -0.0685, 1.5696, 1.37304, -0.74273, 1.3983, 1.79618]),
    "horizontal": np.array([-1.43016, 0.20965, 1.86816, 1.77576, -0.27289, 1.31715, 2.01226]),
}


def add_actions(move_forward=0.0, move_right=0.0, gripper_state=50.0, arm_current="init", arm_next="init", arm_alpha=0.0):
    action_map = {}

    # base joints
    action_map["joint.wheel_joint_r"] = [move_forward - move_right]
    action_map["joint.wheel_joint_l"] = [move_forward + move_right]

    # arm joints
    arm_pose = arm_poses[arm_current] * (1.0 - arm_alpha) + arm_poses[arm_next] * arm_alpha
    action_map["joint.arm_joint_0"] = [arm_pose[0]]
    action_map["joint.arm_joint_1"] = [arm_pose[1]]
    action_map["joint.arm_joint_2"] = [arm_pose[2]]
    action_map["joint.arm_joint_3"] = [arm_pose[3]]
    action_map["joint.arm_joint_4"] = [arm_pose[4]]
    action_map["joint.arm_joint_5"] = [arm_pose[5]]
    action_map["joint.arm_joint_6"] = [arm_pose[6]]

    # gripper joints
    action_map["joint.gripper_finger_joint_r"] = [gripper_state]
    action_map["joint.gripper_finger_joint_l"] = [gripper_state]

    # other joints
    action_map["joint.head_pan_joint"] = [0.0]
    action_map["joint.head_tilt_joint"] = [0.0]

    return action_map


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "actions.csv"))
    args = parser.parse_args()

    df = pd.DataFrame()

    # move forward
    for i in range(0, 100):
        df = pd.concat([df, pd.DataFrame(add_actions(move_forward=0.01))])
    # hold target
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(add_actions(gripper_state=-50))])
    # rotate base
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(add_actions(move_right=0.009, gripper_state=-100))])
    # move forward while moving arm
    for i in range(0, 100):
        df = pd.concat(
            [df, pd.DataFrame(add_actions(move_forward=0.01, gripper_state=-100, arm_current="init", arm_next="diagonal45", arm_alpha=i / 100))])
    # release gripper
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(add_actions(gripper_state=50, arm_current="diagonal45"))])
    # move back and fold arm
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(add_actions(move_forward=-0.01, arm_current="diagonal45", arm_next="default", arm_alpha=i / 100))])
    # keep folding arm
    for i in range(30, 100):
        df = pd.concat([df, pd.DataFrame(add_actions(move_forward=0, arm_current="diagonal45", arm_next="default", arm_alpha=i / 100))])
    # stay still
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(add_actions(arm_current="default"))])

    # save to csv
    df.to_csv(args.actions_file, mode="w", index=False, header=True)
