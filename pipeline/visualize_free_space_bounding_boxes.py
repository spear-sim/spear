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

np.random.seed(0)

# Random per-box translation (in Unreal units) applied when drawing the unselected candidate bounding boxes, used
# to break the symmetry between near-identical duplicate boxes so that heavily-duplicated regions render as a thick
# bundle.
box_jitter = 2.5

# The candidate boxes selected in phase 1 (covering the free-space points) and phase 2 (covering the phase-1 boxes'
# footprints) are drawn with a different colormap per phase (cv2's autumn for phase 1, winter for phase 2), shaded by
# each box's rank in that phase's greedy selection order so the order in which boxes were selected is visible. We only
# sample each colormap up to colormap_light_end rather than all the way to 1.0, which keeps the two phases' colors
# visually distinct. The unselected candidate boxes are drawn in faint light grey. cv2 returns BGR, so we reverse each
# 256-entry lookup table to RGB.
covering_colormap_table = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_AUTUMN).reshape(256, 3)[:, [2,1,0]]
overlapping_colormap_table = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_WINTER).reshape(256, 3)[:, [2,1,0]]
colormap_light_end = 0.7
unselected_box_color = (0.7, 0.7, 0.7)
unselected_box_opacity = 0.1

# Selected boxes are drawn with thicker wireframes than the unselected candidate boxes so they stand out.
selected_box_line_width = 4.0
unselected_box_line_width = 1.0


def process_scene():

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Draw the scene's original triangle meshes in dark grey to highlight the free-space bounding boxes drawn on top.
    color = (0.4, 0.4, 0.4)
    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal)

    # Load the free-space points, the mutual-visibility graph (the index pairs of mutually visible points), and the
    # free-space bounding boxes computed by generate_free_space_points.py, generate_free_space_visibility_graph.py,
    # and generate_free_space_bounding_boxes.py. We draw the points and visibility graph here too, but even more
    # faintly than in visualize_free_space_visibility_graph.py, to give context for the bounding boxes without
    # cluttering them.
    free_space_points_file = spear.path.realpath(args.pipeline_dir, "free_space_points", "free_space_points.h5")
    visibility_graph_file = spear.path.realpath(args.pipeline_dir, "free_space_visibility_graph", "free_space_visibility_graph.h5")
    free_space_bounding_boxes_file = spear.path.realpath(args.pipeline_dir, "free_space_bounding_boxes", "free_space_bounding_boxes.h5")
    spear.log("Reading free-space points file: ", free_space_points_file)
    assert os.path.exists(free_space_points_file)
    spear.log("Reading visibility graph file: ", visibility_graph_file)
    assert os.path.exists(visibility_graph_file)
    spear.log("Reading free-space bounding boxes file: ", free_space_bounding_boxes_file)
    assert os.path.exists(free_space_bounding_boxes_file)
    with h5py.File(free_space_points_file, "r") as f:
        points = f["free_space_points"][:]
    with h5py.File(visibility_graph_file, "r") as f:
        edges = f["visibility_graph_edges"][:]
    with h5py.File(free_space_bounding_boxes_file, "r") as f:
        free_space_box_mins = f["box_mins"][:]
        free_space_box_maxs = f["box_maxs"][:]
        covering_indices = f["box_covering_indices"][:]
        overlapping_indices = f["box_overlapping_indices"][:]

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if args.visual_parity_with_unreal:
        points = points[:,[0,2,1]]

    # Draw the mutual-visibility edges as very faint yellow lines and the free-space points as faint white markers,
    # both fainter than in visualize_free_space_visibility_graph.py so the bounding boxes stay legible on top.
    unreal_geometry_utils.draw_lines(points=points, lines=edges, color=(1.0, 1.0, 0.0), opacity=0.001, line_width=1.0)
    mayavi.mlab.points3d(points[:,0], points[:,1], points[:,2], color=(1.0, 1.0, 1.0), scale_factor=3.0, opacity=0.25)

    # Draw the free-space bounding boxes (axis-aligned, stored as min and max corners) as wireframes. The boxes
    # selected in phase 1 (covering the free-space points) and phase 2 (covering the phase-1 boxes' footprints) are
    # each colored by their rank in that phase's selection order, using a single hue per phase shaded dark to light,
    # so both the phase and the selection order are visible; the unselected candidate boxes are drawn in faint light
    # grey for context. We draw the unselected boxes first so the selected boxes render on top of them.
    is_selected = np.zeros(free_space_box_mins.shape[0], dtype=bool)
    is_selected[covering_indices] = True
    is_selected[overlapping_indices] = True
    unselected_indices = np.flatnonzero(np.logical_not(is_selected))

    # Map each selected box to a color sampled from its phase's colormap at its selection rank.
    box_color_from_index = {}
    for box_index, t in zip(covering_indices, np.linspace(colormap_light_end, 0.0, covering_indices.shape[0])):
        box_color_from_index[box_index] = tuple(float(component) / 255.0 for component in covering_colormap_table[int(round(t * 255.0))])
    for box_index, t in zip(overlapping_indices, np.linspace(colormap_light_end, 0.0, overlapping_indices.shape[0])):
        box_color_from_index[box_index] = tuple(float(component) / 255.0 for component in overlapping_colormap_table[int(round(t * 255.0))])

    for box_index in np.concatenate([unselected_indices, covering_indices, overlapping_indices]):
        box_corners = unreal_geometry_utils.get_axis_aligned_bounding_box_corners(box_min=free_space_box_mins[box_index], box_max=free_space_box_maxs[box_index])

        if box_index in box_color_from_index:
            color = box_color_from_index[box_index]
            opacity = 1.0
            line_width = selected_box_line_width
        else:
            # Jitter only the unselected boxes to break the symmetry between near-identical duplicates; the selected
            # boxes are few and distinct, so we draw them at their true positions.
            box_corners = box_corners + np.random.uniform(low=-box_jitter, high=box_jitter, size=3)
            color = unselected_box_color
            opacity = unselected_box_opacity
            line_width = unselected_box_line_width

        # Swap y and z coordinates to match the visual appearance of the Unreal editor.
        if args.visual_parity_with_unreal:
            box_corners = box_corners[:,[0,2,1]]

        unreal_geometry_utils.draw_lines(points=box_corners, lines=unreal_geometry_utils.box_edge_indices, color=color, opacity=opacity, line_width=line_width)

    mayavi.mlab.show()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
