#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cv2
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

# The network of free-space bounding boxes the paths are planned through is drawn as faint light grey wireframes for
# context (so the corridors the paths run through are visible).
network_box_color = (0.7, 0.7, 0.7)
network_box_opacity = 0.1
network_box_line_width = 1.0

# Each path is drawn as a polyline colored by progress along the path (its evolution from begin to end) using this
# colormap, and the waypoints each path was planned through are drawn as balls colored by the normalized time at which
# the path reaches them using the same colormap (so the first waypoint takes the colormap's start color, the last its
# end color, and the interior waypoints the colors in between). path_colormap_table is turbo as a 256-entry RGBA lookup
# table that we install on each path's line; cv2 returns BGR, so we reverse to RGB and append an opaque alpha channel.
path_colormap_table = np.column_stack([cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_TURBO).reshape(256, 3)[:, [2,1,0]], np.full(256, 255, dtype=np.uint8)])
path_line_width = 5.0
path_waypoint_scale_factor = 8.0


def process_scene():

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    # Draw the scene's original triangle meshes in dark grey to highlight the paths drawn on top.
    color = (0.4, 0.4, 0.4)

    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Load the free-space bounding boxes (to draw the planning network for context) and the planned paths computed by
    # generate_free_space_bounding_boxes.py and generate_free_space_paths.py.
    free_space_bounding_boxes_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_bounding_boxes", "free_space_bounding_boxes.h5"))
    free_space_paths_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_paths", "free_space_paths.h5"))
    spear.log("Reading free-space bounding boxes file: ", free_space_bounding_boxes_file)
    assert os.path.exists(free_space_bounding_boxes_file)
    spear.log("Reading free-space paths file: ", free_space_paths_file)
    assert os.path.exists(free_space_paths_file)
    with h5py.File(free_space_bounding_boxes_file, "r") as f:
        box_mins = f["box_mins"][:]
        box_maxs = f["box_maxs"][:]
        covering_indices = f["box_covering_indices"][:]
        overlapping_indices = f["box_overlapping_indices"][:]
    with h5py.File(free_space_paths_file, "r") as f:
        path_points = f["path_points"][:]
        path_point_times = f["path_point_times"][:]
        path_waypoints = f["path_waypoints"][:]
        path_waypoint_times = f["path_waypoint_times"][:]

    # Draw the planning network (the union of the phase-1 and phase-2 selected boxes) as faint light grey wireframes.
    network_indices = np.union1d(covering_indices, overlapping_indices)
    for box_index in network_indices:
        box_corners = unreal_geometry_utils.get_axis_aligned_bounding_box_corners(box_min=box_mins[box_index], box_max=box_maxs[box_index])

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            box_corners = box_corners[:,[0,2,1]]

        unreal_geometry_utils.draw_lines(points=box_corners, lines=unreal_geometry_utils.box_edge_indices, color=network_box_color, opacity=network_box_opacity, line_width=network_box_line_width)

    # Draw each path as a polyline colored by the normalized time along the path (0 at the begin point, 1 at the end
    # point). We attach the per-point times to the line as a scalar and override the line's lookup table with
    # path_colormap_table, so the path fades through the colormap from start to end. We also draw the waypoints each
    # path was planned through as balls, colored with the same colormap at the normalized time the path reaches each
    # waypoint, so each waypoint's color exactly matches the path where it sits. (The waypoints are not evenly spaced in
    # time, so coloring them by their index in the sequence rather than by their time would mismatch the path at the
    # interior waypoints.)
    for points, point_times, waypoints, waypoint_times in zip(path_points, path_point_times, path_waypoints, path_waypoint_times):

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            points = points[:,[0,2,1]]
            waypoints = waypoints[:,[0,2,1]]

        path_line = mayavi.mlab.plot3d(points[:,0], points[:,1], points[:,2], point_times, tube_radius=None, line_width=path_line_width)
        path_line.module_manager.scalar_lut_manager.lut.table = path_colormap_table

        for waypoint, waypoint_time in zip(waypoints, waypoint_times):
            color = tuple(float(component) / 255.0 for component in path_colormap_table[int(round(np.clip(waypoint_time, 0.0, 1.0) * 255.0)), :3])
            mayavi.mlab.points3d(waypoint[0:1], waypoint[1:2], waypoint[2:3], color=color, scale_factor=path_waypoint_scale_factor)

    mayavi.mlab.show()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
