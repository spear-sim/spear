#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import scipy
import spear


name_prefix = "Meshes/05_chair"


def unreal_rpy_from_mujoco_quaternion(mujoco_quaternion):

    # MuJoCo assumes quaternions are stored in scalar-first (wxyz) order, but scipy.spatial.transform.Rotation assumes scalar-last (xyzw) order
    scipy_quaternion = mujoco_quaternion[[1,2,3,0]]

    # Unreal and scipy.spatial.transform.Rotation have different Euler angle conventions, see python/spear/pipeline.py for details
    scipy_roll, scipy_pitch, scipy_yaw = scipy.spatial.transform.Rotation.from_quat(scipy_quaternion).as_euler("xyz")
    unreal_roll  = np.rad2deg(-scipy_roll)
    unreal_pitch = np.rad2deg(-scipy_pitch)
    unreal_yaw   = np.rad2deg(scipy_yaw)

    return np.array([unreal_roll, unreal_pitch, unreal_yaw])


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf_file", required=True)
    args = parser.parse_args()

    # create SPEAR instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    spear_instance = spear.Instance(config)

    # get Unreal actors and functions
    spear_instance.engine_service.begin_tick()

    unreal_actors = spear_instance.unreal_service.find_actors_as_dict()
    unreal_actors = { unreal_actor_name: unreal_actor for unreal_actor_name, unreal_actor in unreal_actors.items() if unreal_actor_name.startswith(name_prefix) }

    unreal_actor_static_class = spear_instance.unreal_service.get_static_class("AActor")
    unreal_set_actor_location_and_rotation_func = spear_instance.unreal_service.find_function_by_name(
        uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")

    spear_instance.engine_service.tick()
    spear_instance.engine_service.end_tick()

    # initialize MuJoCo
    mj_model = mujoco.MjModel.from_xml_path(os.path.realpath(args.mjcf_file))
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)

    # get MuJoCo bodies
    mj_bodies = { mj_model.body(mj_body).name: mj_body for mj_body in range(mj_model.nbody) if mj_model.body(mj_body).name.startswith(name_prefix) }

    # launch MuJoCo viewer
    mj_viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

    # initialize MuJoCo camera (not needed when launching the viewer through the command-line, but needed when using launch_passive)
    mj_viewer.cam.distance = 30.0*100.0 # 30 meters * 100 Unreal units per meter
    mj_viewer.cam.azimuth = 90.0
    mj_viewer.cam.elevation = -45.0
    mj_viewer.cam.lookat = np.array([0.0, 0.0, 0.0])

    # initialize MuJoCo viewer options
    mj_viewer.opt.label = mujoco.mjtLabel.mjLABEL_SELECTION

    # update MuJoCo viewer state
    mj_viewer.sync()

    while mj_viewer.is_running():

        # perform multiple MuJoCo simulation steps per Unreal frame
        mj_update_steps = 10
        for _ in range(mj_update_steps):
            mujoco.mj_step(mj_model, mj_data)
        mj_viewer.sync()

        # get updated poses from MuJoCo
        mj_bodies_xpos = { mj_body_name: mj_data.body(mj_body).xpos for mj_body_name, mj_body in mj_bodies.items() }
        mj_bodies_xquat = { mj_body_name: mj_data.body(mj_body).xquat for mj_body_name, mj_body in mj_bodies.items() }

        # set updated poses in SPEAR
        spear_instance.engine_service.begin_tick()

        for unreal_actor_name, unreal_actor in unreal_actors.items():

            # call function for each actor
            args = {
                "NewLocation": dict(zip(["X", "Y", "Z"], mj_bodies_xpos[unreal_actor_name + ":StaticMeshComponent0"])),
                "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], unreal_rpy_from_mujoco_quaternion(mj_bodies_xquat[unreal_actor_name + ":StaticMeshComponent0"]))),
                "bSweep":      False,
                "bTeleport":   True}
            spear_instance.unreal_service.call_function(unreal_actor, unreal_set_actor_location_and_rotation_func, args)

        spear_instance.engine_service.tick()
        spear_instance.engine_service.end_tick()

    mj_viewer.close()
    spear_instance.close()

    spear.log("Done.")
