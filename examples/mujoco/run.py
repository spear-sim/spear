#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
# this script should be run with [mjpython](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer), not python.
# mjpython is required for the passive mujoco viewer.

import argparse
import cv2
import mujoco
import mujoco.viewer
import numpy as np
import os
import spear
import time
# from scipy.spatial.transform import Rotation as R

# declaratives
osp = os.path


NUM_STEPS = 100


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--mjcf", required=True, help="path to scene MJCF file")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(osp.join(osp.dirname(__file__), "user_config.yaml"))])

    # configure spear
    spear.configure_system(config)
    instance = spear.Instance(config)

    spear.log("Getting actors from the world...")
    instance.engine_service.begin_tick()
    actors = instance.game_world_service.find_actors_as_map()
    sp_chair_actors = { name:ptr for name,ptr in actors.items() if "Meshes/05_chair" in name }
    spear.log("Chair actors :", sp_chair_actors)
    instance.engine_service.tick()
    spear.log("Getting components ...")
    components_info = { name:instance.game_world_service.get_components_as_map(ptr) for name,ptr in sp_chair_actors.items()}
    components_filtered = {}
    for actor_name, component_info in components_info.items():
        component_list = [ {component_name:ptr} for component_name, ptr in component_info.items() if "StableName" not in component_name ]
        components_filtered.update({actor_name: component_list})
    spear.log(components_filtered)
    instance.engine_service.end_tick()


    # create mujoco objects
    mujoco_model = mujoco.MjModel.from_xml_path(osp.expanduser(args.mjcf))
    mujoco_data = mujoco.MjData(mujoco_model)

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    spear.log("Number of bodies in this scene are ", mujoco_model.nbody)
    body_ids = [ x for x in range(mujoco_model.nbody) ]
    body_names = [ mujoco_model.body(body_id).name for body_id in range(mujoco_model.nbody) ]
    spear.log("body_names:", body_names)

    # filter out only actors and components that are chairs
    mj_chair_body_ids = [ x for x, y in zip(body_ids, body_names) if "Meshes/05_chair/Kitchen" in y ]
    mj_chair_body_names = [ y for x, y in zip(body_ids, body_names) if "Meshes/05_chair/Kitchen" in y ]
    
    mj_kitchen_chair_joint_ids = [ for x in range(mujoco_model.njnt) if mujoco_model.mujoco_data.joint(x).name ]
    
    for x in range(mujoco_model.njnt):
        print(mujoco_model.body(mujoco_model.joint(x).bodyid[0]).name)

    # get corresponding xpos, xquat
    xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in mj_chair_body_ids}
    xquat_dict = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xquat for body_id in mj_chair_body_ids}

    print("xpos dict.......................")
    print(xpos_dict)
    print("xquat dict......................")
    print(xquat_dict)

    ##############################################
    ####### mujoco and spear communication #######
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    # set viewer camera paramters to view the kitchen chairs
    viewer.cam.lookat = [0, 300.0, 100]
    viewer.cam.distance = 500.0
    viewer.cam.azimuth = 180
    viewer.cam.elevation = 0

    # act_mids = [np.mean(mujoco_model.actuator(i).ctrlrange) for i in range(mujoco_model.nu)]
    # act_mags = [0.5*(mujoco_model.actuator(i).ctrlrange[1]-mujoco_model.actuator(i).ctrlrange[0]) for i in range(mujoco_model.nu)]
    chair_quat_means = [mujoco_data.body(i).xquat for i in mj_chair_body_ids]

    period = 1000

    start = time.time()
    t = 0
    muj_update_steps = 20

    while viewer.is_running() and time.time() - start < 10:
        # send actutations to mujoco
        # mujoco_data.actuator("Cabinet/PhysicsConstraint_door_01_x_revolute_actuator").ctrl[0] = 1 # body/component_name Cabinet.Door_01

        for _ in range(muj_update_steps):
            l = np.sign(np.sin(2*np.pi*t/period))
            # for i in range(mujoco_model.nu):
                # mujoco_data.ctrl[i] = act_mids[i] + l*act_mags[i]
            for i, id in enumerate(mj_chair_body_ids):
                mujoco_data.body(id).xquat = chair_quat_means[i] + l * 0.1
                mujoco_data.joint()

            mujoco.mj_step(mujoco_model, mujoco_data)

            # increament time
            t += 1

        viewer.sync()

        # get updated xpos, xquat
        # xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in mj_chair_body_ids}
        xquat_dict = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xquat for body_id in mj_chair_body_ids}

        # spear.log("mujoco xpos:  ", xpos_dict)
        # spear.log()
        spear.log("mujoco xquat: ", xquat_dict)    
        spear.log()

        # set simulate physics from python in next iteration

        # # spear.log("Getting root component property description ...")
        # # root_component_property_desc = [ instance.game_world_service.find_property_by_name_from_object(x, "RelativeLocation.X") for x in actors ]
        # # spear.log(root_component_property_desc)
        # # spear.log("Getting property value as string ... ")
        # # spear.log([ instance.game_world_service.get_property_value_as_string(x) for x in root_component_property_desc ])
        # spear.log("Get functions ...")
        # function_ptrs = [ instance.game_world_service.find_function_by_name(x, "GetOwner", 0) for x in components_flatten ]
        # spear.log(function_ptrs)
        # spear.log("Call functions ...")
        # spear.log([ instance.game_world_service.call_function(x, y) for x,y in zip(components_flatten, function_ptrs) ])
        # # spear.log("Getting object properties as string ...")
        # # spear.log([ instance.game_world_service.get_object_properties_as_string_from_object(x) for x in actors])
        # instance.engine_service.end_tick()

    viewer.close()

    instance.close()

    spear.log("Done.")