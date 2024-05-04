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


name_prefix  = "Meshes/05_chair/"


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

    # create spear Instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    spear_instance = spear.Instance(config)

    # get Unreal actors and functions
    spear_instance.engine_service.begin_tick()

    unreal_actors = spear_instance.game_world_service.find_actors_as_map()
    unreal_actors = { unreal_actor_name: unreal_actor for unreal_actor_name, unreal_actor in unreal_actors.items() if unreal_actor_name.startswith(name_prefix) }

    unreal_actor_classes = list(set([ spear_instance.game_world_service.get_class(unreal_actor) for unreal_actor in unreal_actors.values() ]))
    assert len(unreal_actor_classes) == 1
    unreal_actor_class = unreal_actor_classes[0]

    unreal_set_actor_location_and_rotation_func = spear_instance.game_world_service.find_function_by_name(
        uclass=unreal_actor_class, name="K2_SetActorLocationAndRotation", include_super_flag=1)

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
    mj_viewer.sync()

    while mj_viewer.is_running():

        # perform multiple MuJoCo simulation steps per Unreal frame
        mj_update_steps = 30
        for _ in range(mj_update_steps):
            mujoco.mj_step(mj_model, mj_data)
        mj_viewer.sync()

        # get updated poses from MuJoCo
        mj_body_xpos = { mj_body_name: mj_data.body(mj_body).xpos for mj_body_name, mj_body in mj_bodies.items() }
        mj_body_xquat = { mj_body_name: mj_data.body(mj_body).xquat for mj_body_name, mj_body in mj_bodies.items() }

        # set updated poses in SPEAR
        spear_instance.engine_service.begin_tick()

        for unreal_actor_name, unreal_actor in unreal_actors.items():
            unreal_location_dict = zip(["X", "Y", "Z"], mj_body_xpos[unreal_actor_name + ":StaticMeshComponent0"])
            unreal_rotation_dict = zip(["Roll", "Pitch", "Yaw"], unreal_rpy_from_mujoco_quaternion(mj_body_xquat[unreal_actor_name + ":StaticMeshComponent0"]))

            spear_instance.game_world_service.call_function(
                unreal_actor, unreal_set_actor_location_and_rotation_func,
                NewLocation=unreal_location_dict, NewRotation=unreal_rotation_dict, bSweep=False, bTeleport=True)

        spear_instance.engine_service.tick()
        spear_instance.engine_service.end_tick()

    mj_viewer.close()
    spear_instance.close()

    spear.log("Done.")
