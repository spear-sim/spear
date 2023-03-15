#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import numpy as np
import pandas as pd

wheel_joints = ["wheel_joint_r", "wheel_joint_l"]
arm_joints = ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"]
gripper_joints = ["gripper_finger_joint_r", "gripper_finger_joint_l"]
other_joints = ["head_pan_joint", "head_tilt_joint"]

# fetch arm poses come from https://github.com/StanfordVL/iGibson/blob/master/igibson/robots/fetch.py#L100
arm_poses = {
    "init": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "default": [-1.17079, 1.47079, 0.4, 1.67079, 0.0, 1.57079, 0.0],
    "vertical": [-0.94121, -0.64134, 1.55186, 1.65672, -0.93218, 1.53416, 2.14474],
    "diagonal15": [-0.95587, -0.34778, 1.46388, 1.47821, -0.93813, 1.4587, 1.9939],
    "diagonal30": [-1.06595, -0.22184, 1.53448, 1.46076, -0.84995, 1.36904, 1.90996],
    "diagonal45": [-1.11479, -0.0685, 1.5696, 1.37304, -0.74273, 1.3983, 1.79618],
    "horizontal": [-1.43016, 0.20965, 1.86816, 1.77576, -0.27289, 1.31715, 2.01226],
}


def add_action(action_map, joint, name):
    action_map["joint." + joint] = np.array([name], dtype=np.float32)


def add_actions(actions_file, move_forward=0.0, move_right=0.0, gripper_state=50.0, arm_current="init", arm_next="init",
                arm_alpha=0.0,
                init=False):
    action_map = {}
    # base actions
    add_action(action_map, wheel_joints[0], move_forward - move_right)
    add_action(action_map, wheel_joints[1], move_forward + move_right)

    # arm actions
    for i in range(len(arm_joints)):
        add_action(action_map, arm_joints[i],
                   arm_poses[arm_current][i] * (1 - arm_alpha) + arm_poses[arm_next][i] * arm_alpha)

    # gripper actions
    add_action(action_map, gripper_joints[0], gripper_state)
    add_action(action_map, gripper_joints[1], gripper_state)

    # other joints
    for joint in other_joints:
        add_action(action_map, joint, 0.0)

    # save to csv
    df = pd.DataFrame(action_map)
    df.to_csv(actions_file, mode="w" if init else "a", index=False, header=init)


def fetch_generate_actions(actions_file):
    # init with headers
    add_actions(actions_file, init=True)

    # move forward
    for i in range(0, 100):
        add_actions(actions_file, move_forward=0.01)
    # hold target
    for i in range(0, 30):
        add_actions(actions_file, gripper_state=-50)
    # rotate base
    for i in range(0, 30):
        add_actions(actions_file, move_right=0.009, gripper_state=-100)
    # move forward and move arm
    for i in range(0, 100):
        add_actions(actions_file, move_forward=0.01, gripper_state=-100, arm_current="init", arm_next="diagonal45",
                    arm_alpha=i / 100)

    # release
    for i in range(0, 30):
        add_actions(actions_file, gripper_state=50, arm_current="diagonal45")

    # move back and fold arm
    for i in range(0, 30):
        add_actions(actions_file, move_forward=-0.01, arm_current="diagonal45", arm_next="default", arm_alpha=i / 100)
    for i in range(30, 100):
        add_actions(actions_file, move_forward=0, arm_current="diagonal45", arm_next="default", arm_alpha=i / 100)
    for i in range(0, 30):
        add_actions(actions_file, arm_current="default")


if __name__ == '__main__':
    actions_file = "actions.csv"
    fetch_generate_actions(actions_file)
