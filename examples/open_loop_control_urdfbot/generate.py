#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import numpy as np
import pandas as pd

episodes_file = "actions.csv"
keys = "joint.wheel_joint_r, joint.wheel_joint_l, joint.head_pan_joint, joint.head_tilt_joint, joint.arm_joint_0, joint.arm_joint_1, joint.arm_joint_2, joint.arm_joint_3, joint.arm_joint_4, joint.arm_joint_5, joint.arm_joint_6, joint.gripper_finger_joint_r, joint.gripper_finger_joint_l"

joints = ["arm_joint_0", "arm_joint_1", "arm_joint_2", "arm_joint_3",
          "arm_joint_4", "arm_joint_5", "arm_joint_6"]
pose2 = np.pi / 180 * np.array(
    [67.08168819, 84.27042205, -22.91831181, 95.72957795, 0., 90., 0.])
pose5 = np.pi / 180 * np.array(
    [-63.87276204, -3.9247609, 89.93145552, 78.6693971, -42.55529432, 80.11668849, 102.91353325])
pose6 = np.pi / 180 * np.array(
    [-81.94213203, 12.01206017, 107.03768346, 101.74355343, -15.63544527, 75.46713599, 115.29400528])

end0 = {}
end1 = {}
for i in range(len(joints)):
    end0[joints[i]] = pose5[i]
    end1[joints[i]] = pose2[i]


def add_action(base_left, base_right, gripper_state, arm_state, init=False):
    action_map = {}
    for key in keys.split(", "):
        joint_name = key.replace("joint.", "")
        if joint_name == "wheel_joint_r":
            action_map[key] = np.array([base_left], dtype=np.float32)
        elif joint_name == "wheel_joint_l":
            action_map[key] = np.array([base_right], dtype=np.float32)
        elif joint_name == "gripper_finger_joint_r":
            action_map[key] = np.array([gripper_state], dtype=np.float32)
        elif joint_name == "gripper_finger_joint_l":
            action_map[key] = np.array([gripper_state], dtype=np.float32)
        elif joint_name in joints:
            if arm_state <= 1:
                action_map[key] = np.array([arm_state * end0[joint_name]], dtype=np.float32)
            elif arm_state > 1 and arm_state <= 2:
                ratio = arm_state - 1
                start = end0[joint_name]
                end = end1[joint_name]
                val = start * (1 - ratio) + end * ratio
                action_map[key] = np.array([val], dtype=np.float32)
        else:
            action_map[key] = np.array([0.0], dtype=np.float32)

    df = pd.DataFrame(action_map)
    df.to_csv(episodes_file, mode="w" if init else "a", index=False, header=init)


def generate_actions():
    # init with headers
    add_action(0.00, 0.00, 0, 0, init=True)
    # move forward
    for i in range(0, 100):
        add_action(0.01, 0.01, 50, 0)
    # hold target
    for i in range(0, 30):
        add_action(0.00, 0.00, -50, 0)
    # rotate base
    for i in range(0, 30):
        add_action(0.009, -0.009, -50, 0)
    # move forward and move arm
    for i in range(0, 100):
        add_action(0.01, 0.01, -50, i / 100)
    # release
    for i in range(0, 30):
        add_action(0.00, 0.00, 50, 1)
    for i in range(0, 100):
        add_action(-0.01, -0.01, 50, 1 + i / 100)
    for i in range(0, 30):
        add_action(0.00, 00.00, 50, 2)


def validate():
    df = pd.read_csv(episodes_file)
    print("df.shape", df.shape)
    for i in range(0, df.shape[0]):
        action = df.to_records()[i]
        print(action["joint.wheel_joint_l"])
        actions = {}
        print("action.dtype.names",action.dtype.names )
        for field_name in action.dtype.names:
            if field_name.startswith("joint."):
                actions[field_name] = np.array(action[field_name], dtype=np.float32)
        print(actions)
        break


if __name__ == '__main__':
    generate_actions()
    validate()
