#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import colorsys
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import trimesh
import spear
import unreal_geometry_utils


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--visual-parity-with-unreal", action="store_true")
parser.add_argument("--ignore-actors", nargs="*")
parser.add_argument("--color-mode", default="unique_color_per_node")
args = parser.parse_args()

assert os.path.exists(args.pipeline_dir)
assert args.color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_node", "unique_color_per_component"]

np.random.seed(0)


def process_scene():

    kinematic_trees_dir = spear.path.realpath(args.pipeline_dir, "kinematic_trees")
    actors_json_file = spear.path.realpath(kinematic_trees_dir, "scene.json")
    spear.log("Reading JSON file: ", actors_json_file)
    assert os.path.exists(actors_json_file)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    actors = actors_json
    if args.ignore_actors is not None:
        actors = { actor_name: actor_kinematic_tree for actor_name, actor_kinematic_tree in actors.items() if actor_kinematic_tree["stable_name"] not in args.ignore_actors }

    color = (0.75, 0.75, 0.75)

    for actor_name, actor_kinematic_tree in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_kinematic_tree(kinematic_tree=actor_kinematic_tree, color=color)

    mayavi.mlab.show()

    spear.log("Done.")

def draw_kinematic_tree(kinematic_tree, color):
    if args.color_mode == "unique_color_per_actor":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
    draw_kinematic_tree_node(
        transform_world_from_parent_node=spear.math.get_identity_transform(),
        kinematic_tree_node=kinematic_tree["root_node"],
        color=color,
        log_prefix_str="    ")

def draw_kinematic_tree_node(transform_world_from_parent_node, kinematic_tree_node, color, log_prefix_str):

    spear.log(f"{log_prefix_str}Processing kinematic tree node: {kinematic_tree_node['name']}")

    transform_parent_node_from_current_node = kinematic_tree_node["transform_parent_node_from_current_node"]
    transform_world_from_current_node = spear.math.compose_transforms(
        transforms=[transform_world_from_parent_node, transform_parent_node_from_current_node], as_spear=True)

    if args.color_mode == "unique_color_per_node":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

    for static_mesh_component_name, static_mesh_component_desc in kinematic_tree_node["static_mesh_components"].items():

        transform_current_node_from_current_component = \
            static_mesh_component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"]
        transform_world_from_current_component = spear.math.compose_transforms(
            transforms=[transform_world_from_current_node, transform_current_node_from_current_component], as_spear=True)

        M_world_from_current_component = spear.math.to_numpy_matrix_from_spear_transform(spear_transform=transform_world_from_current_component, as_matrix=True)

        static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_component_desc["editor_properties"]["static_mesh"]["path"])

        obj_path_suffix = f"{os.path.join(*static_mesh_asset_path.parts[1:])}.obj"
        numerical_parity_obj_path = spear.path.realpath(args.pipeline_dir, "unreal_geometry", "numerical_parity", obj_path_suffix)
        spear.log(f"{log_prefix_str}Reading OBJ file: {numerical_parity_obj_path}")

        mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
        V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
        V_world = M_world_from_current_component*V_current_component
        assert np.allclose(V_world[3,:], 1.0)
        mesh.vertices = V_world.T.A[:,0:3]

        if args.color_mode == "unique_color_per_component":
            color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            mesh.vertices = mesh.vertices[:,[0,2,1]]

        mayavi.mlab.triangular_mesh(
            mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=color, opacity=unreal_geometry_utils.mesh_opacity)

    # Recurse for each child node.
    for child_kinematic_tree_node in kinematic_tree_node["children_nodes"].values():
        draw_kinematic_tree_node(
            transform_world_from_parent_node=transform_world_from_current_node,
            kinematic_tree_node=child_kinematic_tree_node["node"],
            color=color,
            log_prefix_str=f"{log_prefix_str}    ")


if __name__ == "__main__":
    process_scene()
