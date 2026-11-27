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

# The camera paths are drawn as faint light grey polylines so the camera axes drawn along them stand out.
path_color = (0.7, 0.7, 0.7)
path_opacity = 0.5
path_line_width = 1.0

# Number of evenly-spaced frames along each (densely-sampled) camera path at which to draw the camera orientation,
# so the orientation is visible along the path without drawing an axis triad at every sample.
num_visualized_frames = 50

# Each visualized frame's camera orientation is drawn as a small RGB axis triad (red +X look direction, green +Y
# right, blue +Z up) whose arrows are this long, in world units (cm).
camera_axis_length = 30.0


def process_scene():

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Draw the scene's original triangle meshes in dark grey to highlight the camera paths and axes drawn on top.
    color = (0.4, 0.4, 0.4)
    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Load the camera paths computed by generate_free_space_camera_paths.py. Each path has a position and a
    # camera-to-world rotation matrix at each of a dense set of samples (the rotation matrix's columns are the
    # camera's +X look, +Y right, and +Z up directions in world space).
    free_space_camera_paths_file = spear.path.realpath(args.pipeline_dir, "free_space_camera_paths", "free_space_camera_paths.h5")
    spear.log("Reading free-space camera paths file: ", free_space_camera_paths_file)
    assert os.path.exists(free_space_camera_paths_file)
    with h5py.File(free_space_camera_paths_file, "r") as f:
        positions = f["camera_path_positions"][:]
        camera_rotations = f["camera_path_orientations"][:]

    # Draw each camera path as a faint polyline (consecutive samples are joined by an edge).
    path_edge_indices = np.column_stack([np.arange(positions.shape[1] - 1), np.arange(1, positions.shape[1])])
    for path_positions in positions:

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            path_positions = path_positions[:,[0,2,1]]

        unreal_geometry_utils.draw_lines(points=path_positions, lines=path_edge_indices, color=path_color, opacity=path_opacity, line_width=path_line_width)

    # Gather the camera position and orientation axes at an evenly-spaced subset of the samples along every path. A
    # sample's orientation axes are the columns of its camera-to-world rotation matrix (column 0 is the +X look
    # direction, column 1 the +Y right direction, column 2 the +Z up direction).
    frame_indices = np.unique(np.linspace(0.0, positions.shape[1] - 1, num_visualized_frames).round().astype(int))
    frame_positions = []
    frame_x_axes = []
    frame_y_axes = []
    frame_z_axes = []
    for path_index in range(positions.shape[0]):
        for frame_index in frame_indices:
            frame_positions.append(positions[path_index][frame_index])
            frame_x_axes.append(camera_rotations[path_index][frame_index][:,0])
            frame_y_axes.append(camera_rotations[path_index][frame_index][:,1])
            frame_z_axes.append(camera_rotations[path_index][frame_index][:,2])
    frame_positions = np.array(frame_positions)
    frame_x_axes = np.array(frame_x_axes)
    frame_y_axes = np.array(frame_y_axes)
    frame_z_axes = np.array(frame_z_axes)

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if args.visual_parity_with_unreal:
        frame_positions = frame_positions[:,[0,2,1]]
        frame_x_axes = frame_x_axes[:,[0,2,1]]
        frame_y_axes = frame_y_axes[:,[0,2,1]]
        frame_z_axes = frame_z_axes[:,[0,2,1]]

    # Draw a small RGB axis triad at each visualized frame, reusing the same arrow style as the world origin axes
    # above (red for the camera's +X look direction, green for +Y right, blue for +Z up).
    for frame_axes, color in [(frame_x_axes, unreal_geometry_utils.c_x_axis), (frame_y_axes, unreal_geometry_utils.c_y_axis), (frame_z_axes, unreal_geometry_utils.c_z_axis)]:
        mayavi.mlab.quiver3d(frame_positions[:,0], frame_positions[:,1], frame_positions[:,2],
                             frame_axes[:,0], frame_axes[:,1], frame_axes[:,2],
                             mode="arrow", scale_factor=camera_axis_length, color=color)

    mayavi.mlab.show()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
