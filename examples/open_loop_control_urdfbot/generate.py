#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import numpy as np
import pandas as pd

arm_joints = ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6"]
other_joints = ["head_pan_joint", "head_tilt_joint"]

arm_poses = np.pi / 180 * np.array([
    np.array([0, 0, 0, 0, 0., 0, 0.]),
    np.array([-63.87276204, -3.9247609, 89.93145552, 78.6693971, -42.55529432, 80.11668849, 102.91353325]),
    np.array([67.08168819, 84.27042205, -22.91831181, 95.72957795, 0., 90., 0.]),
    np.array([-81.94213203, 12.01206017, 107.03768346, 101.74355343, -15.63544527, 75.46713599, 115.29400528])
])


def add_action(action_map, joint, name):
    action_map["joint." + joint] = np.array([name], dtype=np.float32)


def add_actions(move_forward=0.0, move_right=0.0, gripper_state=50.0, arm_current=0, arm_next=0, arm_aplha=0.0,
                init=False):
    action_map = {}
    # base actions
    add_action(action_map, "wheel_joint_r", move_forward - move_right)
    add_action(action_map, "wheel_joint_l", move_forward + move_right)

    # arm actions
    for i in range(len(arm_joints)):
        add_action(action_map, arm_joints[i],
                   arm_poses[arm_current][i] * (1 - arm_aplha) + arm_poses[arm_next][i] * arm_aplha)

    # gripper actiosn
    add_action(action_map, "gripper_finger_joint_r", gripper_state)
    add_action(action_map, "gripper_finger_joint_l", gripper_state)

    # other joints
    for joint in other_joints:
        add_action(action_map, joint, 0.0)

    df = pd.DataFrame(action_map)
    df.to_csv(episodes_file, mode="w" if init else "a", index=False, header=init)


def fetch_generate_actions(episodes_file):
    # init with headers
    add_actions(init=True)

    # move forward
    for i in range(0, 100):
        add_actions(move_forward=0.01)
    # hold target
    for i in range(0, 30):
        add_actions(gripper_state=-50)
    # rotate base
    for i in range(0, 30):
        add_actions(move_right=0.009, gripper_state=-100)
    # move forward and move arm
    for i in range(0, 100):
        add_actions(move_forward=0.01, gripper_state=-100, arm_current=0, arm_next=1, arm_aplha=i / 90)
    # release
    for i in range(0, 30):
        add_actions(gripper_state=50, arm_current=1)
    for i in range(0, 30):
        add_actions(move_forward=-0.01, arm_current=1, arm_next=2, arm_aplha=i / 100)
    for i in range(30, 100):
        add_actions(move_forward=0, arm_current=1, arm_next=2, arm_aplha=i / 100)
    for i in range(0, 30):
        add_actions(arm_current=2)


if __name__ == '__main__':
    episodes_file = "actions.csv"
    fetch_generate_actions(episodes_file)
