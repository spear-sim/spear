#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import mujoco
import mujoco.viewer
import numpy as np
import os
import spear
import time


NUM_STEPS = 100


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--xml_path", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "apartment_0000", "scene.xml")))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # create mujoco objects
    mujoco_model = mujoco.MjModel.from_xml_path(args.xml_path)
    mujoco_data = mujoco.MjData(mujoco_model)

    mujoco.mj_forward(mujoco_model, mujoco_data)

    body_ids = []
    for jnt_id in range(mujoco_model.njnt):
        joint = mujoco_model.joint(jnt_id)
        body_ids.append(joint.bodyid[0])


    # get all mujoco bodies and corresponding xpos, xquat
    xpos_dict  = {mujoco_model.body(body_id).name.replace('/', '.'): mujoco_data.body(body_id).xpos  for body_id in body_ids}
    xquat_dict = {mujoco_model.body(body_id).name.replace('/', '.'): mujoco_data.body(body_id).xquat for body_id in body_ids}

    # send this info
    print("xpos dict.......................")
    print(xpos_dict)
    print("xquat dict......................")
    print(xquat_dict)

    # create SimulationController object
    simulation_controller = spear.SimulationController(config)

    # create Scene object
    scene = spear.Scene(config, simulation_controller)

    spear.log()
    spear.log("All actor names:")
    actor_names = scene.get_all_actor_names()
    spear.log("printing all actor names\n", actor_names)

    
    
    # data = mujoco.reset() # xpos, xquat

    #### step
    # user sends actions to mujoco
    # mujoco_data.ctrl({actuator_id:ctrl_value}) # cabinet->door->revolute_joint->actuator
    # mujoco.step(mujoco_model, mujoco_data)
    # get_xpos_xquat()
    # scene.set_pose(xpos, xquat)
    # scene.tick()


    simulation_controller.close()
    quit()
    
    static_mesh_components = scene.get_static_mesh_components_for_actors(actor_names)
    spear.log("printing all static mesh components\n", static_mesh_components)

    spear.log()
    spear.log("all actor locations")
    actor_locations = scene.get_all_actor_locations()
    for name, actor_location in actor_locations.items():
        spear.log(name, actor_location)
    
    spear.log()
    spear.log("all actor rotations")
    actor_rotations = scene.get_all_actor_rotations()
    for name, actor_rotation in actor_rotations.items():
        spear.log(name, actor_rotation)

    spear.log()
    value = np.array([1000, 1000, 1000], dtype=np.float64)
    spear.log("setting actor location for ", actor_names[4], "value ", value)
    scene.set_actor_locations({actor_names[4]: value})

    spear.log()
    value = np.array([0, 360, 0], dtype=np.float64)
    spear.log("setting actor rotation for ", actor_names[4], "value ", value)
    scene.set_actor_rotations({actor_names[4]: value})

    spear.log()
    spear.log("getting actor locations for ", actor_names[:10])
    actor_locations = scene.get_actor_locations(actor_names[:10])
    for name, actor_location in zip(actor_names[:10], actor_locations):
        spear.log(name, actor_location)

    spear.log()
    spear.log("getting actor rotations for ", actor_names[:10])
    actor_rotations = scene.get_actor_rotations(actor_names[:10])
    for name, actor_rotation in zip(actor_names[:10], actor_rotations):
        spear.log(name, actor_rotation)

    # close the environment
    simulation_controller.close()

    spear.log("Done.")
