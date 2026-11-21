#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import mayavi.mlab
import os
import spear
import unreal_geometry_utils


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--visual-parity-with-unreal", action="store_true")
parser.add_argument("--ignore-actors", nargs="*")
args = parser.parse_args()

assert os.path.exists(args.pipeline_dir)

edge_opacity = 0.025


def process_scene():

    unreal_geometry_utils.draw_origin_axes(visual_parity_with_unreal=args.visual_parity_with_unreal)

    actors = unreal_geometry_utils.get_actors(pipeline_dir=args.pipeline_dir, ignore_actors=args.ignore_actors)

    # Draw the scene's original triangle meshes in dark grey to highlight the visibility graph drawn on top.
    color = (0.25, 0.25, 0.25)

    unreal_geometry_utils.draw_actors(pipeline_dir=args.pipeline_dir, actors=actors, color=color, visual_parity_with_unreal=args.visual_parity_with_unreal, draw_bounding_boxes=True)

    # Load the free-space points and the mutual-visibility graph (the index pairs of mutually visible points)
    # computed by generate_free_space_points.py and generate_free_space_visibility_graph.py.
    free_space_points_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points", "free_space_points.h5"))
    visibility_graph_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_visibility_graph", "free_space_visibility_graph.h5"))
    spear.log("Reading free-space points file: ", free_space_points_file)
    assert os.path.exists(free_space_points_file)
    spear.log("Reading visibility graph file: ", visibility_graph_file)
    assert os.path.exists(visibility_graph_file)
    with h5py.File(free_space_points_file, "r") as f:
        points = f["free_space_points"][:]
    with h5py.File(visibility_graph_file, "r") as f:
        edges = f["visibility_graph_edges"][:]

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if args.visual_parity_with_unreal:
        points = points[:,[0,2,1]]

    # Draw the mutual-visibility edges as faint yellow lines and the free-space points as white markers.
    unreal_geometry_utils.draw_lines(points=points, lines=edges, color=(1.0, 1.0, 0.0), opacity=edge_opacity, line_width=1.0)
    mayavi.mlab.points3d(points[:,0], points[:,1], points[:,2], color=(1.0, 1.0, 1.0), scale_factor=3.0)

    mayavi.mlab.show()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
