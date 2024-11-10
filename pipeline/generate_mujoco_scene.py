#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import colorsys
import json
import numpy as np
import os
import pathlib
import posixpath
import spear
import spear.pipeline
import xml.dom.minidom
import xml.etree.ElementTree


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
"""<mujoco model="{0}">

    <option gravity="0 0 -981"/>

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

    collision_geometry_dir = os.path.realpath(os.path.join(args.pipeline_dir, "scenes", args.scene_id, "collision_geometry"))
    actors_json_file = os.path.realpath(os.path.join(collision_geometry_dir, "actors.json"))
    spear.log("Reading JSON file: " + actors_json_file)
    assert os.path.exists(collision_geometry_dir)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    actors = { actor_name: actor_kinematic_tree for actor_name, actor_kinematic_tree in actors_json.items() if actor_name not in ignore_actors }

    meshes_element = xml.etree.ElementTree.Element("meshes")
    bodies_element = xml.etree.ElementTree.Element("bodies")
    color = (0.75, 0.75, 0.75)
    for actor_name, actor_kinematic_tree in actors.items():
        add_mujoco_elements(actor_name, actor_kinematic_tree, meshes_element, bodies_element, color)

    mujoco_scene_dir = os.path.realpath(os.path.join(args.pipeline_dir, "scenes", args.scene_id, "mujoco_scene"))
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
        root_node=True,
        color=color,
        log_prefix_str="    ")

#
# MuJoCo allows for scale transformations at leaf nodes in its kinematic tree representation via the "scale"
# attribute on each mesh asset. But the input kinematic tree representation here (that we compute directly
# from Unreal's component hierarchy) may have scale transformations throughout. So, when computing our
# MuJoCo kinematic tree, we cannot simply copy the position of each node in our input tree. Instead, we need
# to adjust the input node positions to account for the scale transformations that might be present throughout
# our input tree, which are not directly representable in MuJoCo.
#
# To derive adjusted node positions in a MuJoCo kinematic tree, we must first choose a particular target
# reference frame, where we want projected mesh vertices to match in our input tree and the MuJoCo tree. This
# up-front decision is necessary because projected mesh vertices will generally not match as they are projected
# through each tree's various reference frames. But, as we will see here, the projected vertices can be made to
# match at a particular reference frame. In this derivation, we choose the world frame as our target reference
# frame.
# 
# Let v be a mesh vertex in object space, and let u_w be its corresponding location in world space, as computed
# by Unreal. Given a hierarchy of transforms that map to world space from object space, Unreal computes u_w as
# follows.
#
#     u_w = R_wo*S_wo*v + l_wo                            (1)
#
# where R_wo and S_wo are the accumulated rotation and scale matrices that map points to world space from
# object space, and l_wo is the accumulated location vector that maps points to world space from object
# space. See spear.pipeline.compose_transforms(...) for a reference implementation.
#
# For illustrative purposes, we will assume here that we have a tree with a depth of 3, i.e., a tree with a
# root node and some child nodes and some grandchild nodes. We will also assume that frames with numerically
# lower indices are closer to the leaves (i.e., frame 1 is a grandchild, frame 2 is a child, frame 3 is
# the root, etc). Under these assumptions, we get the following expression for u_w.
#
#     u_w = R_w3*R_32*R_21 * S_w3*S_32*S_21 * v + l_wo    (2)
#           --------------   --------------
#               = R_wo           = S_wo
#
# where R_ij and S_ij are the rotation and scale matrices that map points to reference frame i from reference
# frame j.
#
# Now let l_wc be an accumulated location vector that maps points to world space to some current reference
# frame. Unreal computes l_wc as follows.
#
#     l_wc = R_wp*S_wp*l_pc + l_wp                        (3)
#
# where R_wp and S_wp are the rotation and scale matrices that map points to world space from the parent
# reference frame (i.e., the immediate parent of the current reference frame), l_pc is the location vector
# that maps points to the parent reference frame from the current reference frame, and l_wp is the
# accumulated location vector that maps to world space from the parent reference frame.
#
# If we substitute equation (3) into equation (2) and expand for a tree of depth 3, we get the following
# expression for u_w.
#
#     u_w = R_w3*R_32*R_21 * S_w3*S_32*S_21 * v    +
#           R_w3*R_32      * S_w3*S_32      * l_21 +
#           R_w3           * S_w3           * l_32 +
#           I                               * l_w3        (4)
#
# Now let m_c be the position of v in some current reference frame, and let m_p be its corresponding position
# in the parent reference frame, as computed by MuJoCo. Given a transform that maps to the parent reference
# frame from the current reference frame, MuJoCo computes m_p as follows.
#
#     m_p = R_pc*m_c + t_pc                               (5)
#
# where R_pc is the rotation matrix that maps points to the parent reference frame from the current reference
# frame, m_p is the position of v in the parent reference frame, and t_pc is the translation vector that maps
# points to the parent reference frame from the current reference frame.
#
# Let m_w be the world-space position of v, as computed by MuJoCo. If we expand equation (5) for a tree of
# depth 3, we get the following expression for m_w.
# 
#     m_w = R_w3*R_32*R_21 * S              * v    +
#           R_w3*R_32                       * t_21 +
#           R_w3                            * t_32 +
#           I                               * t_w3        (6)
#
# Note that in equation (6), we have included an S matrix because MuJoCo allows us to scale mesh vertices by
# a diagonal scale matrix before applying any other transformations.
#
# Our goal is therefore to choose the diagonal scale matrix S and the vectors t_ij such that m_w == u_w for all
# mesh vertices v. If we set equation (4) equal to equation (6), we arrive at the following expressions for S
# and t_ij.
#
#        S := S_w3*S_32*S_21        = S_w1
#     t_21 := S_w3*S_32      * l_21 = S_w2*l_21
#     t_32 := S_w3           * l_32 = S_w3*l_32
#     t_w3 := I              * l_w3 = I   *l_w3           (7)
#
# Noting the recursive structure in equation (7), we arrive at the following general expressions for S and t_pc
# that apply for all tree depths.
#
#        S := transform_world_from_current_node["scale"]
#     t_pc := transform_world_from_parent_node["scale"]*transform_parent_node_from_current_node["location"] 
#
# We store these variables in the "pos" and "scale" attributes below.
#

def add_mujoco_elements_for_kinematic_tree_node(
    actor_name, kinematic_tree_node, meshes_element, parent_element, transform_world_from_parent_node, root_node, color, log_prefix_str):

    kinematic_tree_node_name = kinematic_tree_node["name"]
    spear.log(log_prefix_str, "Processing kinematic tree node: ", kinematic_tree_node_name)

    transform_parent_node_from_current_node = spear.pipeline.get_transform_from_transform_data(transform_data=kinematic_tree_node["transform_parent_node_from_current_node"])
    transform_world_from_current_node = spear.pipeline.compose_transforms(transforms=[transform_world_from_parent_node, transform_parent_node_from_current_node])

    rotation_x_axis = transform_parent_node_from_current_node["rotation"][:,0].A1
    rotation_y_axis = transform_parent_node_from_current_node["rotation"][:,1].A1
    rotation_z_axis = transform_parent_node_from_current_node["rotation"][:,2].A1
    assert np.allclose(np.cross(rotation_x_axis, rotation_y_axis), rotation_z_axis)

    # TODO: get static-vs-dynamic flag from the JSON data, use better heuristic for avoiding interpenetrations
    if "Meshes/05_chair" not in actor_name:
        body_type = "static"
    else:
        body_type = "dynamic"

    if body_type == "static":
        pos_offset = np.matrix([0.0, 0.0, 0.0]).T
    elif body_type == "dynamic":
        pos_offset = np.matrix([0.0, 0.0, 20.0]).T
    else:
        assert False

    body_name = actor_name + ":" + kinematic_tree_node_name
    body_element = xml.etree.ElementTree.SubElement(parent_element, "body", attrib={
        "name": body_name,
        "pos": get_mujoco_pos_str(transform_world_from_parent_node["scale"]*transform_parent_node_from_current_node["location"] + pos_offset),
        "xyaxes": get_mujoco_xyaxes_str(transform_parent_node_from_current_node["rotation"])})

    if body_type == "dynamic" and root_node:
        xml.etree.ElementTree.SubElement(body_element, "freejoint")

    if args.color_mode == "unique_color_per_body":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

    merge_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"]
    for merge_id in merge_ids.keys():

        if args.color_mode == "unique_color_per_merge_id":
            color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

        # TODO: retrieve desired decomposition strategy from the JSON data
        convex_decomposition_strategy = "coacd"

        # we don't use os.path.realpath here because we don't want to convert to an absolute path
        merge_id_obj_dir = posixpath.join(
            "..",
            "collision_geometry",
            convex_decomposition_strategy,
            actor_name.replace("/", "."),
            kinematic_tree_node["name"],
            f"merge_id_{int(merge_id):04}") # JSON stores keys as strings, so we need to convert merge_id to an int

        part_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id][convex_decomposition_strategy]["part_ids"]
        for part_id in part_ids:

            # we don't use os.path.realpath here because we don't want to convert to an absolute path
            part_id_obj_path = posixpath.join(merge_id_obj_dir, f"part_id_{part_id:04}.obj")

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
            root_node=False,
            color=color,
            log_prefix_str=log_prefix_str+"    ")

def get_element_str(element):
    return xml.dom.minidom.parseString(xml.etree.ElementTree.tostring(element)).toprettyxml(indent="    ")

def get_mujoco_pos_str(pos):
    return " ".join([ str(value) for value in pos.A1 ])

def get_mujoco_xyaxes_str(rotation):
    return " ".join([ str(value) for value in rotation[:,0].A1 ]) + " " + " ".join([ str(value) for value in rotation[:,1].A1 ])

def get_mujoco_scale_str(scale):
    return " ".join([ str(value) for value in np.diag(scale) ])

def get_mujoco_rgba_str(rgb):
    rgba = list(rgb) + [1.0]
    return " ".join([ str(value) for value in rgba ])


if __name__ == "__main__":
    process_scene()
