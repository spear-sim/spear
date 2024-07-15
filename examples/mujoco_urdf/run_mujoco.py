#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse

import cv2
import mujoco
import mujoco.viewer
import numpy as np
import os
import scipy
import spear
import scipy.spatial


def unreal_rpy_from_mujoco_quaternion(mujoco_quaternion):
    # MuJoCo assumes quaternions are stored in scalar-first (wxyz) order, but scipy.spatial.transform.Rotation assumes scalar-last (xyzw) order
    scipy_quaternion = mujoco_quaternion[[1, 2, 3, 0]]

    # Unreal and scipy.spatial.transform.Rotation have different Euler angle conventions, see python/spear/pipeline.py for details
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_quat(scipy_quaternion).as_euler("xyz")
    unreal_roll = np.rad2deg(-scipy_roll)
    unreal_pitch = np.rad2deg(-scipy_pitch)
    unreal_yaw = np.rad2deg(scipy_yaw)

    return np.array([unreal_roll, unreal_pitch, unreal_yaw])


# see https://github.com/google-deepmind/mujoco/blob/195bd32aa6bd9361245e0832f22651df7c44e81d/src/engine/engine_vis_visualize.c#L2133
def compute_camera_transform(cam):
    ca = np.cos(cam.azimuth / 180.0 * np.pi)
    sa = np.sin(cam.azimuth / 180.0 * np.pi)
    ce = np.cos(cam.elevation / 180.0 * np.pi)
    se = np.sin(cam.elevation / 180.0 * np.pi)
    forward = np.array([ce * ca, ce * sa, se])
    cam_location = np.array(cam.lookat) + forward * (-cam.distance)
    cam_rotation = np.array([0, cam.elevation, cam.azimuth])
    return cam_location.tolist(), cam_rotation.tolist()


"""
fetch_in_apartment_0000
fetch_in_plane
fetch_debug
"""
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf_file", default=r"F:\intel\interiorsim\pipeline\fetch_debug.mjcf")
    args = parser.parse_args()

    # TODO create a mjcf with urdf agent
    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # get MuJoCo bodies
    mj_bodies = {mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if True}

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    # initialize MuJoCo camera (not needed when launching the viewer through the command-line, but needed when using launch_passive)
    mj_viewer.cam.distance = 30.0 * 100.0  # 30 meters * 100 Unreal units per meter
    mj_viewer.cam.azimuth = 90.0
    mj_viewer.cam.elevation = -45.0
    mj_viewer.cam.lookat = np.array([0.0, 0.0, 0.0])

    # initialize MuJoCo viewer options
    mj_viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION

    # update MuJoCo viewer state
    mj_viewer.sync()

    img = np.zeros([240, 320, 3])
    cv2.imshow('img', img)
    while mj_viewer.is_running():

        # perform multiple MuJoCo simulation steps per Unreal frame
        mj_update_steps = 10
        for _ in range(mj_update_steps):
            # print("mj_data.ctrl", mj_data.ctrl)
            mujoco.mj_step(mj_model, mj_data)
        mj_viewer.sync()
        # get updated poses from MuJoCo
        mj_bodies_xpos = {mj_body_name: mj_data.body(mj_body).xpos for mj_body_name, mj_body in mj_bodies.items()}
        mj_bodies_xquat = {mj_body_name: mj_data.body(mj_body).xquat for mj_body_name, mj_body in mj_bodies.items()}

        k = cv2.waitKey(10)
        action_scale = 1e5
        if k == 27:  # Esc key to stop
            break
        elif k == ord('w'):
            mj_data.actuator("wheel_left").ctrl = 1 * action_scale
            mj_data.actuator("wheel_right").ctrl = 1 * action_scale
            pass
        elif k == ord('s'):
            mj_data.actuator("wheel_left").ctrl = -1 * action_scale
            mj_data.actuator("wheel_right").ctrl = -1 * action_scale
        elif k == ord('a'):
            mj_data.actuator("wheel_left").ctrl = -1 * action_scale
            mj_data.actuator("wheel_right").ctrl = 1 * action_scale
        elif k == ord('d'):
            mj_data.actuator("wheel_left").ctrl = 1 * action_scale
            mj_data.actuator("wheel_right").ctrl = -1 * action_scale
        elif k == ord('x'):
            mj_data.actuator("wheel_left").ctrl = 1 * action_scale * 0.5
            mj_data.actuator("wheel_right").ctrl = -1 * action_scale * 0.5
        else:
            pass
        if k>0:
            print("k",k)
        # TODO update urdf mesh actors

    mj_viewer.close()

    spear.log("Done.")
