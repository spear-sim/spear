#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import spear
from scipy.spatial.transform import Rotation as R


chair_actor_name_prefix = "Meshes/05_chair/"


def muj_2_ue_as_rpy(quat):

    reorder_quat_idx = [1,2,3,0]
    rpy_to_unreal_matrix = np.array(
        [
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]
        ])
    return R.from_quat(quat[reorder_quat_idx]).as_euler('xyz', degrees=True).dot(rpy_to_unreal_matrix)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", required=True, help="path to scene MJCF file")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # configure spear
    spear.configure_system(config)
    instance = spear.Instance(config)

    # get chair actors and function pointer to set location and rotation
    instance.engine_service.begin_tick()
    actors = instance.game_world_service.find_actors_as_map()
    sp_chair_actors = { name:ptr for name,ptr in actors.items() if chair_actor_name_prefix in name }
    set_pose_function_ptr = instance.game_world_service.find_function_by_name(instance.game_world_service.get_class_from_instance(next(iter(sp_chair_actors.values()))), "K2_SetActorLocationAndRotation", 1)
    instance.engine_service.tick()
    instance.engine_service.end_tick()

    # create mujoco objects
    mujoco_model = mujoco.MjModel.from_xml_path(os.path.expanduser(args.mjcf))
    mujoco_data = mujoco.MjData(mujoco_model)

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    # filter out only actors and components that are chairs
    mj_chair_body_ids = [ x for x in range(mujoco_model.nbody) if chair_actor_name_prefix in mujoco_model.body(x).name ]

    # get mujoco's interactive viewer
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    # set viewer camera paramters to view the kitchen chairs
    viewer.cam.lookat = [0, 300.0, 100]
    viewer.cam.distance = 500.0
    viewer.cam.azimuth = 180
    viewer.cam.elevation = 0
    viewer.sync()

    muj_update_steps = 30

    while viewer.is_running():

        for _ in range(muj_update_steps):
            mujoco.mj_step(mujoco_model, mujoco_data)

        viewer.sync()

        # get updated pose from MuJoCo
        xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in mj_chair_body_ids}
        xrpy_dict  = {mujoco_model.body(body_id).name: muj_2_ue_as_rpy(mujoco_data.body(body_id).xquat) for body_id in mj_chair_body_ids}

        # send updated pose to SPEAR
        instance.engine_service.begin_tick()

        call_func_ret = [
            instance.game_world_service.call_function(
                actor,
                set_pose_function_ptr,
                NewLocation="{\"x\": " + str(xpos_dict[name+":StaticMeshComponent0"][0]) + ", \"y\": " + str(xpos_dict[name+":StaticMeshComponent0"][1]) + ", \"z\": " + str(xpos_dict[name+":StaticMeshComponent0"][2]) + "}",
                NewRotation="{\"Pitch\": " + str(xrpy_dict[name+":StaticMeshComponent0"][1]) + ", \"Yaw\": " + str(xrpy_dict[name+":StaticMeshComponent0"][2]) + ", \"Roll\": " + str(xrpy_dict[name+":StaticMeshComponent0"][0]) + "}",
                bSweep="false",
                bTeleport="true")
            for name, actor in sp_chair_actors.items() ]

        instance.engine_service.tick()
        instance.engine_service.end_tick()

    viewer.close()
    instance.close()
    spear.log("Done.")
