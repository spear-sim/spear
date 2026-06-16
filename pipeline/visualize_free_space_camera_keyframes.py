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

# The paths are drawn as faint light grey polylines so the camera axes drawn along them stand out.
path_color = (0.7, 0.7, 0.7)
path_opacity = 0.5
path_line_width = 1.0

# Each keyframe's camera orientation is drawn as a small RGB axis triad (red +X look direction, green +Y right, blue
# +Z up) whose arrows are this long, in world units (cm).
camera_axis_length = 30.0

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

    # Draw the scene's original triangle meshes in dark grey to highlight the paths and camera axes drawn on top.
    color = (0.4, 0.4, 0.4)

    for actor_name, actor_desc in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_actor(actor_desc=actor_desc, color=color)

    # Load the planned paths and the camera keyframes computed by generate_free_space_paths.py and
    # generate_free_space_camera_keyframes.py. The keyframes store a camera-to-world rotation matrix at each of a set
    # of normalized times along each path; the camera position at each keyframe is the path's position at that time.
    free_space_paths_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_paths", "free_space_paths.h5"))
    free_space_camera_keyframes_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_keyframes", "free_space_camera_keyframes.h5"))
    assert os.path.exists(free_space_paths_file)
    assert os.path.exists(free_space_camera_keyframes_file)
    spear.log("Reading free-space paths file: ", free_space_paths_file)
    spear.log("Reading free-space camera keyframes file: ", free_space_camera_keyframes_file)
    with h5py.File(free_space_paths_file, "r") as f:
        path_points = f["path_points"][:]
        path_point_times = f["path_point_times"][:]
    with h5py.File(free_space_camera_keyframes_file, "r") as f:
        camera_keyframe_times = f["camera_keyframe_times"][:]
        camera_keyframe_rotations = f["camera_keyframe_orientations"][:]

    # Draw each path as a faint polyline (consecutive points are joined by an edge).
    path_edge_indices = np.column_stack([np.arange(path_points.shape[1] - 1), np.arange(1, path_points.shape[1])])
    for points in path_points:

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            points = points[:,[0,2,1]]

        draw_lines(points=points, lines=path_edge_indices, color=path_color, opacity=path_opacity, line_width=path_line_width)

    # Gather every keyframe's camera position and orientation axes. A keyframe's position is the path's position at
    # the keyframe's normalized time, and its orientation axes are the columns of its camera-to-world rotation matrix
    # (column 0 is the +X look direction, column 1 the +Y right direction, column 2 the +Z up direction).
    keyframe_positions = []
    keyframe_x_axes = []
    keyframe_y_axes = []
    keyframe_z_axes = []
    for path_index in range(path_points.shape[0]):
        positions = get_keyframe_positions(path_points=path_points[path_index], path_point_times=path_point_times[path_index], keyframe_times=camera_keyframe_times[path_index])
        for position, rotation in zip(positions, camera_keyframe_rotations[path_index]):
            keyframe_positions.append(position)
            keyframe_x_axes.append(rotation[:,0])
            keyframe_y_axes.append(rotation[:,1])
            keyframe_z_axes.append(rotation[:,2])
    keyframe_positions = np.array(keyframe_positions)
    keyframe_x_axes = np.array(keyframe_x_axes)
    keyframe_y_axes = np.array(keyframe_y_axes)
    keyframe_z_axes = np.array(keyframe_z_axes)

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if args.visual_parity_with_unreal:
        keyframe_positions = keyframe_positions[:,[0,2,1]]
        keyframe_x_axes = keyframe_x_axes[:,[0,2,1]]
        keyframe_y_axes = keyframe_y_axes[:,[0,2,1]]
        keyframe_z_axes = keyframe_z_axes[:,[0,2,1]]

    # Draw a small RGB axis triad at each keyframe, reusing the same arrow style as the world origin axes above (red
    # for the camera's +X look direction, green for +Y right, blue for +Z up).
    for keyframe_axes, color in [(keyframe_x_axes, c_x_axis), (keyframe_y_axes, c_y_axis), (keyframe_z_axes, c_z_axis)]:
        mayavi.mlab.quiver3d(keyframe_positions[:,0], keyframe_positions[:,1], keyframe_positions[:,2],
                             keyframe_axes[:,0], keyframe_axes[:,1], keyframe_axes[:,2],
                             mode="arrow", scale_factor=camera_axis_length, color=color)

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

        # Recurse for each child component.
        for child_component_desc in component_desc["children_components"].values():
            draw_component(
                transform_world_from_parent_component=transform_world_from_current_component,
                component_desc=child_component_desc,
                color=color,
                log_prefix_str=f"{log_prefix_str}    ")

def get_keyframe_positions(path_points, path_point_times, keyframe_times):

    # The path positions at the given normalized keyframe times, interpolated per axis from the path's sampled points
    # (whose normalized times increase monotonically over [0, 1]).
    return np.column_stack([ np.interp(keyframe_times, path_point_times, path_points[:,axis]) for axis in range(path_points.shape[1]) ])

def draw_lines(points, lines, color, opacity, line_width):

    line_source = mayavi.mlab.pipeline.scalar_scatter(points[:,0], points[:,1], points[:,2])
    line_source.mlab_source.dataset.lines = lines
    line_source.update()
    mayavi.mlab.pipeline.surface(mayavi.mlab.pipeline.stripper(line_source), color=color, opacity=opacity, line_width=line_width)


if __name__ == "__main__":
    process_scene()
