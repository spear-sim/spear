#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
# this script should be run with [mjpython](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer), not python.
# mjpython is required for the passive mujoco viewer.

import argparse
import mujoco
import mujoco.viewer
import numpy as np
import os
import spear
import time
from scipy.spatial.transform import Rotation as R


rpy_to_unreal_matrix = np.array(
    [
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
    ])

reorder_quat_idx = [1,2,3,0]

query_actor_name = "Meshes/05_chair"


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--mjcf", required=True, help="path to scene MJCF file")
    parser.add_argument("--run_for", default=30)
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    #
    #   SPEAR setup
    #
    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # configure spear
    spear.configure_system(config)
    instance = spear.Instance(config)

    spear.log("Getting chair actors from the world...")
    instance.engine_service.begin_tick()

    actors = instance.game_world_service.find_actors_as_map()
    sp_chair_actors = { name:ptr for name,ptr in actors.items() if query_actor_name in name }
    spear.log("Chair actors :", sp_chair_actors)

    instance.engine_service.tick()

    spear.log("Getting K2_SetActorLocation function pointers...")
    set_location_function_ptr = instance.game_world_service.find_function_by_name(instance.game_world_service.get_class_from_instance(next(iter(sp_chair_actors.values()))), "K2_SetActorLocation", 1)

    spear.log("Getting K2_SetActorRotation function pointers...")
    set_rotation_function_ptr = instance.game_world_service.find_function_by_name(instance.game_world_service.get_class_from_instance(next(iter(sp_chair_actors.values()))), "K2_SetActorRotation", 1)

    instance.engine_service.end_tick()

    #
    #   MuJoCo setup
    #
    # create mujoco objects
    mujoco_model = mujoco.MjModel.from_xml_path(os.path.expanduser(args.mjcf))
    mujoco_data = mujoco.MjData(mujoco_model)

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    # filter out only actors and components that are chairs
    mj_chair_body_ids = [ x for x in range(mujoco_model.nbody) if query_actor_name in mujoco_model.body(x).name ]

    ##############################################
    ####### mujoco and spear communication #######
    ##############################################
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    # set viewer camera paramters to view the kitchen chairs
    viewer.cam.lookat = [0, 300.0, 100]
    viewer.cam.distance = 500.0
    viewer.cam.azimuth = 180
    viewer.cam.elevation = 0
    viewer.sync()

    start = time.time()
    muj_update_steps = 20

    while viewer.is_running():# and time.time() - start < args.run_for:

        for _ in range(muj_update_steps):
            mujoco.mj_step(mujoco_model, mujoco_data)

        viewer.sync()

        # get updated pose from MuJoCo
        xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in mj_chair_body_ids}
        xquat_dict = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xquat for body_id in mj_chair_body_ids}
        xeul_dict  = {name: R.from_quat(quat[reorder_quat_idx]).as_euler('xyz', degrees=True).dot(rpy_to_unreal_matrix) for name, quat in xquat_dict.items()}

        # send updated pose to SPEAR
        instance.engine_service.begin_tick()

        call_func_ret = [
            instance.game_world_service.call_function(
                actor,
                set_location_function_ptr,
                NewLocation="{\"x\": " + str(xpos_dict[name+":StaticMeshComponent0"][0]) + ", \"y\": " + str(xpos_dict[name+":StaticMeshComponent0"][1]) + ", \"z\": " + str(xpos_dict[name+":StaticMeshComponent0"][2]) + "}",
                bSweep="false",
                bTeleport="true")
            for name, actor in sp_chair_actors.items() ]

        call_func_ret = [
            instance.game_world_service.call_function(
                actor,
                set_rotation_function_ptr,
                NewRotation="{\"Pitch\": " + str(xeul_dict[name+":StaticMeshComponent0"][1]) + ", \"Yaw\": " + str(xeul_dict[name+":StaticMeshComponent0"][2]) + ", \"Roll\": " + str(xeul_dict[name+":StaticMeshComponent0"][0]) + "}",
                bTeleportPhysics="true")
            for name, actor in sp_chair_actors.items() ]

        instance.engine_service.tick()
        instance.engine_service.end_tick()

    viewer.close()

    instance.close()

    spear.log("Done.")
