#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import trimesh
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--visual-parity-with-unreal", action="store_true")
parser.add_argument("--ignore-actors", nargs="*")
args = parser.parse_args()

ignore_actors = []
if args.ignore_actors is not None:
    ignore_actors = args.ignore_actors

scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]

np.random.seed(0)

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0,  0.0,  0.0)
c_y_axis = (0.0,  1.0,  0.0)
c_z_axis = (0.0,  0.0,  1.0)

# Set axes to be be 1 meter, or 100 Unreal units.
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

# Vertex indices of the 12 edges of a bounding box, where each edge connects two corners that differ on exactly
# one axis (using the same corner indexing as spear.pipeline.get_bounding_box's corners_world).
box_edge_indices = np.array([
    [0, 4], [1, 5], [2, 6], [3, 7],
    [0, 2], [1, 3], [4, 6], [5, 7],
    [0, 1], [2, 3], [4, 5], [6, 7]])

edge_opacity = 0.025


def process_scene():

    unreal_metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "scene.json"))
    assert os.path.exists(unreal_metadata_dir)
    spear.log("Reading JSON file: ", actors_json_file)
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
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_name.split(":")[0] not in ignore_actors }
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["editor_properties"]["relevant_for_level_bounds"] }
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["root_component"] is not None }

    # Draw the scene's original triangle meshes in dark grey to highlight the visibility graph drawn on top.
    color = (0.25, 0.25, 0.25)

    for actor_name, actor_desc in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_actor(actor_desc=actor_desc, color=color)

    # Load the free-space points and the mutual-visibility graph (the index pairs of mutually visible points)
    # computed by generate_free_space_points.py and generate_free_space_visibility_graph.py.
    free_space_points_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points", "free_space_points.h5"))
    visibility_graph_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_visibility_graph", "free_space_visibility_graph.h5"))
    assert os.path.exists(free_space_points_file)
    assert os.path.exists(visibility_graph_file)
    spear.log("Reading free-space points file: ", free_space_points_file)
    spear.log("Reading visibility graph file: ", visibility_graph_file)
    with h5py.File(free_space_points_file, "r") as f:
        points = f["scene_points"][:]
    with h5py.File(visibility_graph_file, "r") as f:
        edges = f["visibility_graph_edges"][:]

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if args.visual_parity_with_unreal:
        points = points[:,[0,2,1]]

    # Draw the mutual-visibility edges as faint yellow lines and the free-space points as white markers.
    draw_lines(points=points, lines=edges, color=(1.0, 1.0, 0.0), opacity=edge_opacity, line_width=1.0)
    mayavi.mlab.points3d(points[:,0], points[:,1], points[:,2], color=(1.0, 1.0, 1.0), scale_factor=3.0)

    mayavi.mlab.show()

    spear.log("Done.")

def draw_actor(actor_desc, color):
    draw_component(
        transform_world_from_parent_component=spear.math.identity_transform,
        component_desc=actor_desc["root_component"],
        color=color,
        log_prefix_str="    ")

def draw_component(transform_world_from_parent_component, component_desc, color, log_prefix_str):

    # Only process SceneComponents...
    component_class = component_desc["class"]
    if component_class in scene_component_classes:
        spear.log(f"{log_prefix_str}Processing SceneComponent: {component_desc['stable_name']}")

        transform_parent_from_current_component, is_absolute_location, is_absolute_rotation, is_absolute_scale = \
            spear.pipeline.to_spear_transform_from_json_component(json_component=component_desc)

        transform_world_from_current_component = spear.math.compose_component_transforms(
            transforms=[transform_world_from_parent_component, transform_parent_from_current_component],
            is_absolute_location=[is_absolute_location], is_absolute_rotation=[is_absolute_rotation], is_absolute_scale=[is_absolute_scale],
            as_spear=True)
        M_world_from_current_component = spear.math.to_numpy_matrix_from_spear_transform(spear_transform=transform_world_from_current_component, as_matrix=True)

        # Check the M_world_from_current_component matrix that we computed above against Unreal's
        # SceneComponent.get_world_transform() function. We store the output from get_world_transform() in our
        # exported JSON file for each SceneComponent, so we can simply compare the matrix we computed above
        # against the stored matrix.

        world_transform = spear.pipeline.to_numpy_matrix_from_json_matrix(json_matrix=component_desc["attributes"]["world_transform_as_matrix"], as_matrix=True)
        assert np.allclose(M_world_from_current_component, world_transform)

        # ...and only attempt to draw StaticMeshComponents...
        if component_class in static_mesh_component_classes:
            spear.log(f"{log_prefix_str}Component is a StaticMeshComponent.")
            static_mesh_desc = component_desc["editor_properties"]["static_mesh"]

            # ...that refer to non-null StaticMesh assets.
            if static_mesh_desc is not None:
                static_mesh_asset_path = pathlib.PurePosixPath(static_mesh_desc["path"])
                spear.log(f"{log_prefix_str}StaticMesh asset path: {static_mesh_asset_path}")

                obj_path_suffix = f"{os.path.join(*static_mesh_asset_path.parts[1:])}.obj"
                numerical_parity_obj_path = os.path.realpath(os.path.join(args.pipeline_dir, "unreal_geometry", "numerical_parity", obj_path_suffix))
                spear.log(f"{log_prefix_str}Reading OBJ file: {numerical_parity_obj_path}")

                mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
                V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
                V_world = M_world_from_current_component*V_current_component
                assert np.allclose(V_world[3,:], 1.0)
                mesh.vertices = V_world.T.A[:,0:3]

                # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                if args.visual_parity_with_unreal:
                    mesh.vertices = mesh.vertices[:,[0,2,1]]

                mayavi.mlab.triangular_mesh(mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=color, opacity=mesh_opacity)

                # Compute the component's oriented bounding box from the exported local_bounds and world_transform
                # metadata (the same spear.pipeline.get_bounding_box the free-space pipeline uses) and draw it as a
                # white wireframe.
                bounding_box = spear.pipeline.get_bounding_box(component_desc=component_desc)
                box_corners = bounding_box["corners_world"]

                # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                if args.visual_parity_with_unreal:
                    box_corners = box_corners[:,[0,2,1]]

                draw_lines(points=box_corners, lines=box_edge_indices, color=(1.0, 1.0, 1.0), opacity=1.0, line_width=1.0)

        # Recurse for each child component.
        for child_component_desc in component_desc["children_components"].values():
            draw_component(
                transform_world_from_parent_component=transform_world_from_current_component,
                component_desc=child_component_desc,
                color=color,
                log_prefix_str=f"{log_prefix_str}    ")

def draw_lines(points, lines, color, opacity, line_width):

    line_source = mayavi.mlab.pipeline.scalar_scatter(points[:,0], points[:,1], points[:,2])
    line_source.mlab_source.dataset.lines = lines
    line_source.update()
    mayavi.mlab.pipeline.surface(mayavi.mlab.pipeline.stripper(line_source), color=color, opacity=opacity, line_width=line_width)


if __name__ == "__main__":
    process_scene()
