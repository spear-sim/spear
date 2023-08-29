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

def muj_2_ue_position(position):
    position[1]*=-1         # mujoco_y = -ue_y
    return position * 100   # m to cms

def muj_2_ue_quat(quaternion):
    return np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

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

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    # get all body ids that needs to be updated in every step
    # these body ids are the first child of all joints
    # first child is only to ensure that in case of free joints wherein there are 6 of the same children, we need only one
    # and in every other joint case, there will only be 1 child
    body_ids = [ mujoco_model.joint(jnt_id).bodyid[0] for jnt_id in range(mujoco_model.njnt) ]
    # body_names = [ mujoco_model.body(body_id).name.replace('/', '.') for body_id in body_ids ]

    actor_names = []
    actor_ids = []
    component_names = []
    component_ids = []
    for body_id in body_ids:
        body = mujoco_model.body(body_id)
        body_name = body.name.replace('/', '.')
        if body.parentid == 0:  # if parent id world, then body is an actor (in UE terms)
            actor_names.append(body_name)
            actor_ids.append(body_id)
        else:
            if "Kitchen_Sink.Drawer_02" in body_name:
                component_names.append(body_name)
                component_ids.append(body_id)

    # get all mujoco bodies and corresponding xpos, xquat
    xpos_dict  = {mujoco_model.body(body_id).name.replace('/', '.'): mujoco_data.body(body_id).xpos  for body_id in body_ids} # replace '/' by '.' until @Samarth updates the mujoco export pipeline to use '.' instead of '/'
    xquat_dict = {mujoco_model.body(body_id).name.replace('/', '.'): mujoco_data.body(body_id).xquat for body_id in body_ids}

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
    spear_actor_names = scene.get_all_actor_names()
    spear.log("printing all actor names\n", spear_actor_names)

    spear.log()
    spear.log("All scene_component names:")
    scene_component_names = scene.get_all_scene_component_names()
    spear.log("printing all scene_component names\n", scene_component_names)

    spear.log()
    spear.log("All actor body names from mujoco:")
    spear.log(actor_names)

    spear.log()
    spear.log("All component body names from mujoco:")
    spear.log(component_names)

    spear.log()
    spear.log("printing is_using_absolute_location for components")
    absolute_locations = scene.is_using_absolute_location(component_names)
    spear.log(absolute_locations)

    spear.log()
    spear.log("printing is_using_absolute_rotation for components")
    absolute_rotations = scene.is_using_absolute_rotation(component_names)
    spear.log(absolute_rotations)

    spear.log()
    spear.log("printing is_using_absolute_scale for components")
    absolute_scales = scene.is_using_absolute_scale(component_names)
    spear.log(absolute_scales)

    spear.log(len(component_names), len(absolute_locations), len(absolute_rotations), len(absolute_scales))
    scene.set_absolute(component_names, [True for _ in absolute_locations], [True for _ in absolute_rotations], absolute_scales)

    ##############################################
    ####### mujoco and spear communication #######
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    for _ in range(100):
        # send actutations to mujoco
        # mujoco_data.actuator("Cabinet/PhysicsConstraint_door_01_x_revolute_actuator").ctrl[0] = 1 # body/component_name Cabinet.Door_01
        mujoco.mj_step(mujoco_model, mujoco_data)
        viewer.sync()

        # get updated xpos, xquat
        xpos_dict  = {mujoco_model.body(body_id).name.replace('/', '.'): muj_2_ue_position(mujoco_data.body(body_id).xpos)  for body_id in component_ids} # replace '/' by '.' until mujoco export pipeline to use '.' instead of '/'
        # xquat_dict = {mujoco_model.body(body_id).name.replace('/', '.'): muj_2_ue_quat(mujoco_data.body(body_id).xquat) for body_id in component_ids}

        spear.log("mujoco xpos:  ", xpos_dict)
        spear.log()
        # spear.log("mujoco xquat: ", xquat_dict)

        # send these updated poses to SPEAR

        scene.set_component_world_locations(xpos_dict)
        locs = scene.get_component_world_locations(component_names)
        spear.log("UE loc:       ", {x:y for x,y in zip(component_names, locs)})
        spear.log()
        # scene.set_component_world_rotations(xquat_dict)
        # rots = scene.get_component_world_rotations(component_names)
        # spear.log()
        # spear.log("UE rot:   ", {x:y for x,y in zip(component_names, rots)})

    viewer.close()
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
