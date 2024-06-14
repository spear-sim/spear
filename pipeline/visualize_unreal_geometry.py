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
import spear.pipeline


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
parser.add_argument("--scene_id", required=True)
parser.add_argument("--visual_parity_with_unreal", action="store_true")
parser.add_argument("--ignore_actors")
parser.add_argument("--color_mode", default="unique_color_per_component")
args = parser.parse_args()

assert args.color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_component"]

if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors.split(",")
else:
    ignore_actors = []

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]

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

    unreal_metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "actors.json"))
    assert os.path.exists(unreal_metadata_dir)
    spear.log("Reading JSON file: " + actors_json_file)
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

    actors = actors_json
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["root_component"] is not None }
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_name not in ignore_actors }

    color = (0.75, 0.75, 0.75)

    for actor_name, actor_desc in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_actor(actor_desc, color)

    mayavi.mlab.show()

    spear.log("Done.")


def draw_actor(actor_desc, color):
    if args.color_mode == "unique_color_per_actor":
        color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
    draw_component(
        transform_world_from_parent_component=spear.pipeline.TRANSFORM_IDENTITY,
        component_desc=actor_desc["root_component"],
        color=color,
        log_prefix_str="    ")


def draw_component(transform_world_from_parent_component, component_desc, color, log_prefix_str):

    # Only process SceneComponents...
    component_class = component_desc["class"]
    if component_class in scene_component_classes:
        spear.log(log_prefix_str, "Processing SceneComponent: ", component_desc["name"])
        
        transform_world_from_current_component = spear.pipeline.compose_transform_with_component(transform_world_from_parent_component, component_desc)
        M_world_from_current_component = spear.pipeline.get_matrix_from_transform(transform_world_from_current_component)

        # Check the M_world_from_current_component matrix that we computed above against Unreal's 
        # SceneComponent.get_world_transform() function. We store the output from get_world_transform() in our
        # exported JSON file for each SceneComponent, so we can simply compare the matrix we computed above
        # against the stored matrix.

        debug_info_world_transform = spear.pipeline.get_matrix_from_matrix_desc(component_desc["debug_info"]["world_transform"])
        assert np.allclose(M_world_from_current_component, debug_info_world_transform)

        # ...and only attempt to draw StaticMeshComponents...
        if component_class in static_mesh_component_classes:
            spear.log(log_prefix_str, "Component is a StaticMeshComponent.")
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]

            # ...that refer to non-null StaticMesh assets...
            if static_mesh_desc is not None:
                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
                spear.log(log_prefix_str, "StaticMesh asset path: ", static_mesh_asset_path)
    
                # ...that are in the /Game/Scenes/<scene_id> directory.
                if static_mesh_asset_path.parts[:4] == ("/", "Game", "Scenes", args.scene_id):

                    obj_path_suffix = os.path.join(*static_mesh_asset_path.parts[4:]) + ".obj"
                    numerical_parity_obj_path = \
                        os.path.realpath(os.path.join(args.pipeline_dir, args.scene_id, "unreal_geometry", "numerical_parity", obj_path_suffix))
                    spear.log(log_prefix_str, "Reading OBJ file: ", numerical_parity_obj_path)

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
                        mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=color, opacity=mesh_opacity)

        # Recurse for each child component.
        for child_component_desc in component_desc["children_components"].values():
            draw_component(
                transform_world_from_parent_component=transform_world_from_current_component,
                component_desc=child_component_desc,
                color=color,
                log_prefix_str=log_prefix_str+"    ")


if __name__ == "__main__":
    process_scene()
