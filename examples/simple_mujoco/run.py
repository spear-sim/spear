#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mujoco.viewer
import os
import spear

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf_file", default=r"F:\intel\interiorsim\pipeline\apartment_0000\mujoco_scene\main.mjcf")
    args = parser.parse_args()

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
    mj_viewer.sync()

    # set viewer camera paramters to view the kitchen chairs
    mj_viewer.cam.lookat = [0, 300.0, 100]
    mj_viewer.cam.distance = 500.0
    mj_viewer.cam.azimuth = 180
    mj_viewer.cam.elevation = 0

    while mj_viewer.is_running():
        # perform multiple MuJoCo simulation steps per Unreal frame
        mj_update_steps = 30
        for _ in range(mj_update_steps):
            mujoco.mj_step(mj_model, mj_data)
        mj_viewer.sync()

    mj_viewer.close()

    spear.log("Done.")
