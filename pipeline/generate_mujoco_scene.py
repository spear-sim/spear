#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import colorsys
import json
import numpy as np
import os
import pathlib
import spear
import spear.pipeline
import xml.etree.ElementTree
import xml.dom.minidom
import spear.pipeline

parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--ignore_actors")
parser.add_argument("--color_mode", default="unique_color_per_geom")
args = parser.parse_args()

assert args.color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_body", "unique_color_per_merge_id", "unique_color_per_geom"]

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

np.random.seed(0)

main_mjcf_str = \
"""
<mujoco model="{0}">

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
    <include file="meshes.mjcf"/>
  </asset>

  <worldbody>
    <geom size="1000 1000 1" type="plane" material="grid"/>
    <light pos="0 0 1000"/>
    <include file="bodies.mjcf"/>
  </worldbody>

</mujoco>
""".format(args.scene_id)


def process_scene():

    collision_geometry_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "collision_geometry"))
    actors_json_file = os.path.realpath(os.path.join(collision_geometry_dir, "actors.json"))
    spear.log("Reading JSON file: " + actors_json_file)
    assert os.path.exists(collision_geometry_dir)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    meshes_element = xml.etree.ElementTree.Element("meshes")
    bodies_element = xml.etree.ElementTree.Element("bodies")

    color = (0.75, 0.75, 0.75)

    actors = { actor_name: actor_kinematic_tree for actor_name, actor_kinematic_tree in actors_json.items() if actor_name not in ignore_actors }
    actor_mujoco_elements = {
        actor_name: add_mujoco_elements(
            actor_name, actor_kinematic_tree, meshes_element, bodies_element, color) for actor_name, actor_kinematic_tree in actors.items() }

    mujoco_scene_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "mujoco_scene"))
    os.makedirs(mujoco_scene_dir, exist_ok=True)

    meshes_mjcf_file = os.path.realpath(os.path.join(mujoco_scene_dir, "meshes.mjcf"))
    spear.log("Writing MJCF file: " + meshes_mjcf_file)
    with open(meshes_mjcf_file, "w") as f:
        f.write(get_element_str(meshes_element))

    bodies_mjcf_file = os.path.realpath(os.path.join(mujoco_scene_dir, "bodies.mjcf"))
    spear.log("Writing MJCF file: " + bodies_mjcf_file)
    with open(bodies_mjcf_file, "w") as f:
        f.write(get_element_str(bodies_element))

    main_mjcf_file = os.path.realpath(os.path.join(mujoco_scene_dir, "main.mjcf"))
    spear.log("Writing MJCF file: " + main_mjcf_file)
    with open(main_mjcf_file, "w") as f:
        f.write(main_mjcf_str)

    spear.log("Done.")


def add_mujoco_elements(actor_name, kinematic_tree, meshes_element, bodies_element, color):
    spear.log("Processing actor: ", actor_name)
    if args.color_mode == "unique_color_per_actor":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
    add_mujoco_elements_for_kinematic_tree_node(
        actor_name=actor_name,
        kinematic_tree_node=kinematic_tree["root_node"],
        meshes_element=meshes_element,
        parent_element=bodies_element,
        transform_world_from_parent_node=spear.pipeline.TRANSFORM_IDENTITY,
        color=color,
        log_prefix_str="    ")


def add_mujoco_elements_for_kinematic_tree_node(
    actor_name, kinematic_tree_node, meshes_element, parent_element, transform_world_from_parent_node, color, log_prefix_str):

    kinematic_tree_node_name = kinematic_tree_node["name"]
    spear.log(log_prefix_str, "Processing kinematic tree node: ", kinematic_tree_node_name)

    transform_parent_node_from_current_node = spear.pipeline.get_transform_from_transform_data(kinematic_tree_node["transform_parent_node_from_current_node"])
    transform_world_from_current_node = spear.pipeline.compose_transforms([transform_world_from_parent_node, transform_parent_node_from_current_node])

    rotation_x_axis = transform_parent_node_from_current_node["rotation"][:,0].A1
    rotation_y_axis = transform_parent_node_from_current_node["rotation"][:,1].A1
    rotation_z_axis = transform_parent_node_from_current_node["rotation"][:,2].A1
    assert np.allclose(np.cross(rotation_x_axis, rotation_y_axis), rotation_z_axis)

    body_name = actor_name + ":" + kinematic_tree_node_name
    body_element = xml.etree.ElementTree.SubElement(parent_element, "body", attrib={
        "name": body_name,
        "pos": get_mujoco_pos_str(transform_world_from_parent_node["scale"]*transform_parent_node_from_current_node["location"]),
        "xyaxes": get_mujoco_xyaxes_str(transform_parent_node_from_current_node["rotation"])})

    if args.color_mode == "unique_color_per_body":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

    merge_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"]
    for merge_id in merge_ids.keys():

        if args.color_mode == "unique_color_per_merge_id":
            color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

        # TODO: retrieve desired decomposition strategy from the JSON data
        convex_decomposition_strategy = "coacd"

        # we don't use os.path.realpath here because we don't want to convert to an absolute path
        merge_id_obj_dir = os.path.join(
            "..",
            "collision_geometry",
            convex_decomposition_strategy,
            actor_name.replace("/", "."),
            kinematic_tree_node["name"],
            f"merge_id_{int(merge_id):04}") # JSON stores keys as strings, so we need to convert merge_id to an int

        part_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id][convex_decomposition_strategy]["part_ids"]
        for part_id in part_ids:

            # we don't use os.path.realpath here because we don't want to convert to an absolute path
            part_id_obj_path = os.path.join(merge_id_obj_dir, f"part_id_{part_id:04}.obj")

            if args.color_mode == "unique_color_per_geom":
                color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

            part_name = actor_name + ":" + kinematic_tree_node_name + ":" + f"merge_id_{int(merge_id):04}" + ":" + f"part_id_{part_id:04}"
            mesh_name = part_name + ":mesh"
            geom_name = part_name + ":geom"
            mesh_element = xml.etree.ElementTree.SubElement(meshes_element, "mesh", attrib={
                "file": part_id_obj_path,
                "name": mesh_name,
                "scale": get_mujoco_scale_str(transform_world_from_current_node["scale"])})
            geom_element = xml.etree.ElementTree.SubElement(body_element, "geom", attrib={
                "mesh": mesh_name,
                "name": geom_name,
                "rgba": get_mujoco_rgba_str(color),
                "type": "mesh"})

    # Recurse for each child node.
    for child_kinematic_tree_node in kinematic_tree_node["children_nodes"].values():
        add_mujoco_elements_for_kinematic_tree_node(
            actor_name=actor_name,
            kinematic_tree_node=child_kinematic_tree_node["node"],
            meshes_element=meshes_element,
            parent_element=body_element,
            transform_world_from_parent_node=transform_world_from_current_node,
            color=color,
            log_prefix_str=log_prefix_str+"    ")


def get_element_str(element):
    return xml.dom.minidom.parseString(xml.etree.ElementTree.tostring(element)).toprettyxml(indent="    ")


def get_mujoco_pos_str(location):
    return " ".join([ str(value) for value in location.A1 ])

def get_mujoco_xyaxes_str(rotation):
    return " ".join([ str(value) for value in rotation[:,0].A1 ]) + " " + " ".join([ str(value) for value in rotation[:,1].A1 ])

def get_mujoco_scale_str(scale):
    return " ".join([ str(value) for value in np.diag(scale) ])

def get_mujoco_rgba_str(rgb):
    rgba = list(rgb) + [1.0]
    return " ".join([ str(value) for value in rgba ])


if __name__ == '__main__':
    process_scene()
