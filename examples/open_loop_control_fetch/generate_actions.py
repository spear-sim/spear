#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import os
import numpy as np
import pandas as pd
import spear

def build_action_dict():
    action = {}
    action["wheel_joint_l.add_to_angular_velocity_target"]      = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["wheel_joint_r.add_to_angular_velocity_target"]      = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_0.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_1.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_2.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_3.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_4.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_5.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["arm_joint_6.add_to_angular_orientation_target"]     = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["gripper_finger_joint_l.add_force"]                  = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["gripper_finger_joint_r.add_force"]                  = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["head_pan_joint.add_to_angular_orientation_target"]  = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
    action["head_tilt_joint.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]

    return action


if __name__ == '__main__':

    df = pd.DataFrame()

    # turn left
    for _ in range(12):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.098, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.098,  0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn right (negates accumulated "turn left" torque values)
    for _ in range(12):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.098, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.098,  0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # lift arm_joint_1
    for _ in range(1):
        action = build_action_dict()
        action["arm_joint_1.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn arm_joint_6
    for _ in range(52):
        action = build_action_dict()
        action["arm_joint_6.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 2.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move forward keeping the gripper open
    for _ in range(18):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.103, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.103, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([100.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move back keeping the gripper open (negates accumulated "move forward" torque values)
    for _ in range(18):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.103, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.103, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([100.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move forward slowly keeping the gripper open
    for _ in range(10):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.02, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.02, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([100.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move back slowly keeping the gripper open (negates accumulated "move forward" torque values)
    for _ in range(10):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.02, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.02, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([100.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # grab object
    for _ in range(10):
        action  = build_action_dict()
        action["gripper_finger_joint_l.add_force"] = [np.array([-500.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-500.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move back keeping the gripper closed
    for _ in range(25):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.1, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.1, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move forward keeping the gripper closed (negates accumulated "move back" torque values)
    for _ in range(25):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.1, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.1, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn right keeping the gripper closed
    for _ in range(28):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.02, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.02,  0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn left keeping the gripper closed (negates accumulated "turn right" torque values)
    for _ in range(28):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.02, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.02,  0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move forward keeping the gripper closed
    for _ in range(30):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.06, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.06,  0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # move back keeping the gripper closed (negates accumulated "move forward" torque values)
    for _ in range(30):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.06, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.06,  0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # lower arm_joint_1, arm_joint_3, raise arm_joint_5
    for _ in range(25):
        action = build_action_dict()
        action["arm_joint_1.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, -1.0], dtype=np.float64)]
        action["arm_joint_3.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, -1.0], dtype=np.float64)]
        action["arm_joint_5.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        action["gripper_finger_joint_l.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        action["gripper_finger_joint_r.add_force"] = [np.array([-1000.0, 0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # lower arm_joint_1, raise arm_joint_3 and drop the object
    for _ in range(20):
        action = build_action_dict()
        action["arm_joint_1.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        action["arm_joint_3.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, -1.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # raise arm_joint_3
    for _ in range(20):
        action = build_action_dict()
        action["arm_joint_3.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn left
    for _ in range(12):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([0.1, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([-0.1,  0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # turn right (negates accumulated "turn left" torque values)
    for _ in range(12):
        action = build_action_dict()
        action["wheel_joint_l.add_to_angular_velocity_target"] = [np.array([-0.1, 0.0, 0.0], dtype=np.float64)]
        action["wheel_joint_r.add_to_angular_velocity_target"] = [np.array([0.1,  0.0, 0.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])
    
    # lower arm_joint_1, arm_joint_3, raise arm_joint_5
    for _ in range(25):
        action = build_action_dict()
        action["arm_joint_3.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        action["arm_joint_5.add_to_angular_orientation_target"] = [np.array([0.0, 0.0, 1.0], dtype=np.float64)]
        df = pd.concat([df, pd.DataFrame(action)])

    # save to csv
    actions_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.pkl"))
    df.to_pickle(actions_file)

    spear.log("Done.")
