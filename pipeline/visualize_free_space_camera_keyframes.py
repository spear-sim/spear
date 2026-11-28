#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import mayavi.mlab
import numpy as np
import os
import spear
import unreal_geometry_utils


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--visual-parity-with-unreal", action="store_true")
parser.add_argument("--ignore-actors", nargs="*")
args = parser.parse_args()

assert os.path.exists(args.pipeline_dir)

# The paths are drawn as faint light grey polylines so the camera axes drawn along them stand out.
path_color = (0.7, 0.7, 0.7)
path_opacity = 0.5
path_line_width = 1.0

# Each keyframe's camera orientation is drawn as a small RGB axis triad (red +X look direction, green +Y right, blue
# +Z up) whose arrows are this long, in world units (cm).
camera_axis_length = 30.0


def process_scene():

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Draw the scene's original triangle meshes in dark grey to highlight the paths and camera axes drawn on top.
    color = (0.4, 0.4, 0.4)
    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Load the planned paths and the camera keyframes computed by generate_free_space_paths.py and
    # generate_free_space_camera_keyframes.py. The keyframes store a camera-to-world rotation matrix at each of a set
    # of normalized times along each path; the camera position at each keyframe is the path's position at that time.
    free_space_paths_file = spear.path.realpath(args.pipeline_dir, "free_space_paths", "free_space_paths.h5")
    free_space_camera_keyframes_file = spear.path.realpath(args.pipeline_dir, "free_space_camera_keyframes", "free_space_camera_keyframes.h5")
    spear.log("Reading free-space paths file: ", free_space_paths_file)
    assert os.path.exists(free_space_paths_file)
    spear.log("Reading free-space camera keyframes file: ", free_space_camera_keyframes_file)
    assert os.path.exists(free_space_camera_keyframes_file)
    with h5py.File(free_space_paths_file, "r") as f:
        path_points = f["path_points"][:]
        path_point_times = f["path_point_times"][:]
    with h5py.File(free_space_camera_keyframes_file, "r") as f:
        camera_keyframe_times = f["camera_keyframe_times"][:]
        camera_keyframe_rotations = f["camera_keyframe_rotations"][:]

    # Draw each path as a faint polyline (consecutive points are joined by an edge).
    path_edge_indices = np.column_stack([np.arange(path_points.shape[1] - 1), np.arange(1, path_points.shape[1])])
    for points in path_points:

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            points = points[:,[0,2,1]]

        unreal_geometry_utils.draw_lines(points=points, lines=path_edge_indices, color=path_color, opacity=path_opacity, line_width=path_line_width)

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
    for keyframe_axes, color in [(keyframe_x_axes, unreal_geometry_utils.c_x_axis), (keyframe_y_axes, unreal_geometry_utils.c_y_axis), (keyframe_z_axes, unreal_geometry_utils.c_z_axis)]:
        mayavi.mlab.quiver3d(keyframe_positions[:,0], keyframe_positions[:,1], keyframe_positions[:,2],
                             keyframe_axes[:,0], keyframe_axes[:,1], keyframe_axes[:,2],
                             mode="arrow", scale_factor=camera_axis_length, color=color)

    mayavi.mlab.show()

    spear.log("Done.")

def get_keyframe_positions(path_points, path_point_times, keyframe_times):

    # The path positions at the given normalized keyframe times, interpolated per axis from the path's sampled points
    # (whose normalized times increase monotonically over [0, 1]).
    return np.column_stack([ np.interp(keyframe_times, path_point_times, path_points[:,axis]) for axis in range(path_points.shape[1]) ])


if __name__ == "__main__":
    process_scene()
