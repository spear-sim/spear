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
    "default": np.array([-1.2079, 1.47079, 0.4, 1.67079, 0.0, 1.57079, 0.0]),
    "vertical": np.array([-0.94121, -0.64134, 1.55186, 1.65672, -0.93218, 1.53416, 2.14474]),
    "diagonal15": np.array([-0.95587, -0.34778, 1.46388, 1.47821, -0.93813, 1.4587, 1.9939]),
    "diagonal30": np.array([-1.06595, -0.22184, 1.53448, 1.46076, -0.84995, 1.36904, 1.90996]),
    "diagonal45": np.array([-1.11479, -0.0685, 1.5696, 1.37304, -0.74273, 1.3983, 1.79618]),
    "horizontal": np.array([-1.43016, 0.20965, 1.86816, 1.77576, -0.27289, 1.31715, 2.01226]),
    "horizontal_high": np.array([-0.94121, -0.60, 1.55186, 1.25672, -0.93218, 0.0, -0.2]),
}

def get_action(move_forward=0.0, move_right=0.0, gripper_force=1.0, arm_pose_blend_weights={"init": 1.0}):
    action = {}

    # base joints
    action["joint.wheel_joint_r"] = move_forward + move_right
    action["joint.wheel_joint_l"] = move_forward - move_right

    # arm joints
    arm_pose = np.zeros([7])
    total_weight = 0.0
    for pose_name, weight in arm_pose_blend_weights.items():
        arm_pose += arm_poses[pose_name] * weight
        total_weight += weight
    assert total_weight > 0.0
    arm_pose /= total_weight
    action["joint.arm_joint_0"] = arm_pose[0]
    action["joint.arm_joint_1"] = arm_pose[1]
    action["joint.arm_joint_2"] = arm_pose[2]
    action["joint.arm_joint_3"] = arm_pose[3]
    action["joint.arm_joint_4"] = arm_pose[4]
    action["joint.arm_joint_5"] = arm_pose[5]
    action["joint.arm_joint_6"] = arm_pose[6]

    # gripper joints
    action["joint.gripper_finger_joint_r"] = gripper_force
    action["joint.gripper_finger_joint_l"] = gripper_force

    # other joints
    action["joint.head_pan_joint"] = 0.0
    action["joint.head_tilt_joint"] = 0.0

    return action

def get_actions_for_starter_content_0000():
    df = pd.DataFrame()

    # move forward
    for i in range(0, 100):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=0.01), index=[0])])

    # hold target
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(gripper_force=-1.0), index=[0])])

    # rotate base
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_right=0.0095, gripper_force=-1.0), index=[0])])

    # move forward while moving arm
    for i in range(0, 50):
        df = pd.concat(
            [df,
             pd.DataFrame(get_action(move_forward=0.01, gripper_force=-1.0, arm_pose_blend_weights={"init": (50.0 - i) / 50.0, "vertical": i / 50.0}),
                          index=[0])])

    # move forward
    for i in range(0, 50):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=0.01, gripper_force=-1.0, arm_pose_blend_weights={"vertical": 1.0}), index=[0])])

    # release gripper
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(gripper_force=0, arm_pose_blend_weights={"vertical": 1.0}), index=[0])])

    # move back and fold arm
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=-0.01, arm_pose_blend_weights={"vertical": (100.0 - i) / 100.0, "default": i / 100.0}), index=[0])])

    # keep folding arm
    for i in range(30, 100):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=0, arm_pose_blend_weights={"vertical": (100 - i) / 100.0, "default": i / 100.0}), index=[0])])

    # stay still
    for i in range(0, 30):
        df = pd.concat([df, pd.DataFrame(get_action(arm_pose_blend_weights={"default": 1.0}), index=[0])])

    return df

def get_actions_for_kujiale_0000():
    df = pd.DataFrame()

    # move to target object
    for i in range(0, 100):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=0.01), index=[0])])

    # hold target
    for i in range(0, 10):
        df = pd.concat(
            [df, pd.DataFrame(get_action(gripper_force=-1.0), index=[0])])

    # move backward
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=-0.01, gripper_force=-1.0), index=[0])])

    # rotate base
    for i in range(0, 12):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_right=-0.009, gripper_force=-1.0), index=[0])])

    # move and change arm pose
    for i in range(0, 60):
        df = pd.concat(
            [df, pd.DataFrame(
                get_action(move_forward=0.009, gripper_force=-1.0, arm_pose_blend_weights={"init": (60.0 - i) / 60.0, "horizontal_high": i / 60.0}),
                index=[0])])

    # move to target pose
    for i in range(0, 40):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=0.009, gripper_force=-1.0, arm_pose_blend_weights={"horizontal_high": 1}), index=[0])])

    # keep target still
    for i in range(0, 30):
        df = pd.concat(
            [df, pd.DataFrame(get_action(gripper_force=0, arm_pose_blend_weights={"horizontal_high": 1}), index=[0])])

    # withdraw base
    for i in range(0, 20):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_forward=-0.009, arm_pose_blend_weights={"horizontal_high": 1}), index=[0])])

    # rotate around and fold arm
    for i in range(0, 100):
        df = pd.concat(
            [df, pd.DataFrame(get_action(move_right=-0.002, arm_pose_blend_weights={"horizontal_high": (100.0 - i) / 100.0, "default": i / 100.0}), index=[0])])

    # stay still
    for i in range(0, 20):
        df = pd.concat(
            [df, pd.DataFrame(get_action(arm_pose_blend_weights={"default": 1}), index=[0])])

    return df

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_id", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "kujiale_0000"))
    args = parser.parse_args()

    if args.scene_id == "starter_content_0000":
        df = get_actions_for_starter_content_0000()
    elif args.scene_id == "kujiale_0000":
        df = get_actions_for_kujiale_0000()
    else:
        assert False

    # save to csv
    actions_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "actions." + args.scene_id + ".csv"))
    df.to_csv(actions_file, float_format="%.5f", mode="w", index=False, header=True)

    spear.log("Done.")
