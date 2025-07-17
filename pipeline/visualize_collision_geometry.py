#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--visual_parity_with_unreal", action="store_true")
parser.add_argument("--ignore_actors", nargs="*")
parser.add_argument("--color_mode", default="unique_color_per_part_id")
args = parser.parse_args()

assert args.color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_node", "unique_color_per_merge_id", "unique_color_per_part_id"]

ignore_actors = []
if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors

np.random.seed(0)

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0,  0.0,  0.0)
c_y_axis = (0.0,  1.0,  0.0)
c_z_axis = (0.0,  0.0,  1.0)

origin_world = np.array([[0.0,   0.0,   0.0]])
x_axis_world = np.array([[100.0, 0.0,   0.0]])
y_axis_world = np.array([[0.0,   100.0, 0.0]])
z_axis_world = np.array([[0.0,   0.0,   100.0]])

# Swap y and z coordinates to match the visual appearance of the Unreal editor.
if args.visual_parity_with_unreal:
    origin_world = origin_world[:,[0,2,1]]
    x_axis_world = x_axis_world[:,[0,2,1]]
    y_axis_world = y_axis_world[:,[0,2,1]]
    z_axis_world = z_axis_world[:,[0,2,1]]


def process_scene():

    collision_geometry_dir = os.path.realpath(os.path.join(args.pipeline_dir, "scenes", args.scene_id, "collision_geometry"))
    actors_json_file = os.path.realpath(os.path.join(collision_geometry_dir, "actors.json"))
    spear.log("Reading JSON file: ", actors_json_file)
    assert os.path.exists(collision_geometry_dir)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         x_axis_world[:,0], x_axis_world[:,1], x_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_x_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         y_axis_world[:,0], y_axis_world[:,1], y_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_y_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         z_axis_world[:,0], z_axis_world[:,1], z_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_z_axis)

    actors = { actor_name: actor_kinematic_tree for actor_name, actor_kinematic_tree in actors_json.items() if actor_name not in ignore_actors }

    color = (0.75, 0.75, 0.75)

    for actor_name, actor_kinematic_tree in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_collision_geometry(actor_name, actor_kinematic_tree, color)

    mayavi.mlab.show()

    spear.log("Done.")

def draw_collision_geometry(actor_name, kinematic_tree, color):
    if args.color_mode == "unique_color_per_actor":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
    draw_collision_geometry_for_kinematic_tree_node(
        actor_name=actor_name,
        transform_world_from_parent_node=spear.utils.pipeline_utils.identity_transform,
        kinematic_tree_node=kinematic_tree["root_node"],
        color=color,
        log_prefix_str="    ")

def draw_collision_geometry_for_kinematic_tree_node(actor_name, transform_world_from_parent_node, kinematic_tree_node, color, log_prefix_str):

    spear.log(log_prefix_str, "Processing kinematic tree node: ", kinematic_tree_node["name"])

    transform_parent_node_from_current_node = spear.utils.pipeline_utils.get_transform_from_transform_data(transform_data=kinematic_tree_node["transform_parent_node_from_current_node"])
    transform_world_from_current_node = spear.utils.pipeline_utils.compose_transforms(transforms=[transform_world_from_parent_node, transform_parent_node_from_current_node])
    M_world_from_current_node = spear.utils.pipeline_utils.get_matrix_from_transform(transform=transform_world_from_current_node)

    if args.color_mode == "unique_color_per_node":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

    merge_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"]
    for merge_id in merge_ids.keys():

        if args.color_mode == "unique_color_per_merge_id":
            color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

        # TODO: retrieve desired decomposition strategy from the JSON data
        convex_decomposition_strategy = "coacd"

        merge_id_obj_dir = os.path.realpath(os.path.join(
            args.pipeline_dir,
            "scenes",
            args.scene_id,
            "collision_geometry",
            convex_decomposition_strategy,
            actor_name.replace("/", "."),
            kinematic_tree_node["name"],
            f"merge_id_{int(merge_id):04}")) # JSON stores keys as strings, so we need to convert merge_id to an int

        part_ids = kinematic_tree_node["pipeline_info"]["generate_collision_geometry"]["merge_ids"][merge_id][convex_decomposition_strategy]["part_ids"]
        for part_id in part_ids:

            part_id_obj_path = os.path.realpath(os.path.join(merge_id_obj_dir, f"part_id_{part_id:04}.obj"))
            spear.log(log_prefix_str, "Reading OBJ file: ", part_id_obj_path)

            mesh = trimesh.load_mesh(part_id_obj_path, process=False, validate=False)
            V_current_node = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
            V_world = M_world_from_current_node*V_current_node
            assert np.allclose(V_world[3,:], 1.0)
            mesh.vertices = V_world.T.A[:,0:3]

            if args.color_mode == "unique_color_per_part_id":
                color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

            # Swap y and z coordinates to match the visual appearance of the Unreal editor.
            if args.visual_parity_with_unreal:
                mesh.vertices = mesh.vertices[:,[0,2,1]]

            mayavi.mlab.triangular_mesh(
                mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=color, opacity=mesh_opacity)

    # Recurse for each child node.
    for child_kinematic_tree_node in kinematic_tree_node["children_nodes"].values():
        draw_collision_geometry_for_kinematic_tree_node(
            actor_name=actor_name,
            transform_world_from_parent_node=transform_world_from_current_node,
            kinematic_tree_node=child_kinematic_tree_node["node"],
            color=color,
            log_prefix_str=f"{log_prefix_str}    ")


if __name__ == "__main__":
    process_scene()
