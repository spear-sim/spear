#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import trimesh
import spear
import spear.pipeline

parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--visual_parity_with_unreal", action="store_true")
parser.add_argument("--ignore_actors")
args = parser.parse_args()

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0,  0.0,  0.0)
c_y_axis = (0.0,  1.0,  0.0)
c_z_axis = (0.0,  0.0,  1.0)
c_face   = (0.75, 0.75, 0.75)

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

    kinematic_trees_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "kinematic_trees"))
    actors_json_file = os.path.realpath(os.path.join(kinematic_trees_dir, "actors.json"))
    assert os.path.exists(kinematic_trees_dir)
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

    actors = [ (actor_name, kinematic_tree) for actor_name, kinematic_tree in actors_json.items() if actor_name not in ignore_actors ]

    for actor_name, actor_kinematic_tree in actors:
        spear.log("Processing actor: ", actor_name)
        root_node = actor_kinematic_tree["root_node"]
        draw_kinematic_tree_nodes(
            transform_world_from_parent_node=spear.pipeline.TRANSFORM_IDENTITY,
            node=root_node,
            log_prefix_str="    ")

    mayavi.mlab.show()

    spear.log("Done.")


def draw_kinematic_tree_nodes(transform_world_from_parent_node, node, log_prefix_str=""):

    spear.log(log_prefix_str, "Processing kinematic tree node: ", node["name"])

    transform_parent_node_from_current_node = \
        spear.pipeline.get_transform_from_transform_data(
            node["transform_parent_node_from_current_node"])
    transform_world_from_current_node = \
        spear.pipeline.compose_transforms([transform_world_from_parent_node, transform_parent_node_from_current_node])

    for static_mesh_component_name, static_mesh_component_desc in node["static_mesh_components"].items():

        transform_current_node_from_current_component = \
            spear.pipeline.get_transform_from_transform_data(
                static_mesh_component_desc["pipeline_info"]["generate_kinematic_trees"]["transform_current_node_from_current_component"])
        transform_world_from_current_component = \
            spear.pipeline.compose_transforms([transform_world_from_current_node, transform_current_node_from_current_component])

        M_world_from_current_component = spear.pipeline.get_matrix_from_transform(transform_world_from_current_component)

        static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_component_desc["editor_properties"]["static_mesh"]["path"])
        assert static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id)

        obj_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".obj"
        numerical_parity_obj_path = \
            os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))
        spear.log(log_prefix_str, "    OBJ file: ", numerical_parity_obj_path)

        mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
        V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
        V_world = M_world_from_current_component*V_current_component
        assert np.allclose(V_world[3,:], 1.0)
        mesh.vertices = V_world.T.A[:,0:3]

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            mesh.vertices = mesh.vertices[:,[0,2,1]]

        mayavi.mlab.triangular_mesh(
            mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=c_face, opacity=mesh_opacity)

    # Recurse for each child node.
    for child_node in node["children_nodes"].values():
        draw_kinematic_tree_nodes(
            transform_world_from_parent_node=transform_world_from_current_node,
            node=child_node,
            log_prefix_str=log_prefix_str+"    ")


if __name__ == '__main__':
    process_scene()
