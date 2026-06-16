#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import json
import numpy as np
import os
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
args = parser.parse_args()

# Component classes that export the local-space bounds we use to build oriented bounding boxes.
static_mesh_component_classes = ["StaticMeshComponent"]

# Tolerance in Unreal world units (cm). visibility_tolerance prevents the bounding boxes containing a segment's
# own endpoints from counting as occluders.
visibility_tolerance = 0.1

# Distance in Unreal world units (cm) by which every obstacle is expanded so the camera frustum's near face is
# guaranteed to lie in free space. See generate_free_space_points.py for the derivation.
expand_bounding_box_eps = 20.0


def process_scene():

    bounding_boxes = get_component_bounding_boxes_for_scene()
    assert len(bounding_boxes) > 0

    # Read the free-space points sampled by generate_free_space_points.py.
    free_space_points_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points", "free_space_points.h5"))
    assert os.path.exists(free_space_points_file)
    spear.log("Reading free-space points file: ", free_space_points_file)
    with h5py.File(free_space_points_file, "r") as f:
        free_space_points = f["scene_points"][:]

    edges = compute_visibility_graph(free_space_points=free_space_points, bounding_boxes=bounding_boxes)
    spear.log(f"Generated {edges.shape[0]} mutual-visibility edges between {free_space_points.shape[0]} free-space points.")

    # Save the visibility graph as the index pairs of mutually visible points. The free-space points themselves
    # are read back from free_space_points.h5 by each downstream stage.
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_visibility_graph"))
    visibility_graph_file = os.path.realpath(os.path.join(free_space_dir, "free_space_visibility_graph.h5"))
    spear.log("Writing visibility graph file: ", visibility_graph_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(visibility_graph_file, "w") as f:
        f.create_dataset("visibility_graph_edges", data=edges)

    spear.log("Done.")

def get_component_bounding_boxes_for_scene():

    unreal_metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "scene.json"))
    spear.log("Reading JSON file: ", actors_json_file)
    assert os.path.exists(unreal_metadata_dir)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    # We only export geometry for actors for which relevant_for_level_bounds is True, so we apply the same
    # filter here, and ignore actors with no root component.
    bounding_boxes = []
    for actor_name, actor_desc in actors_json.items():
        if actor_desc["root_component"] is None or not actor_desc["editor_properties"]["relevant_for_level_bounds"]:
            continue
        spear.log("Processing actor: ", actor_name)
        bounding_boxes.extend(get_component_bounding_boxes(component_desc=actor_desc["root_component"], log_prefix_str="    "))

    return bounding_boxes

def get_component_bounding_boxes(component_desc, log_prefix_str):

    spear.log(f"{log_prefix_str}Processing component: {component_desc['stable_name']}")

    # We only build a bounding box for StaticMeshComponents that have a static mesh assigned.
    bounding_boxes = []
    if component_desc["class"] in static_mesh_component_classes and component_desc["editor_properties"]["static_mesh"] is not None:
        bounding_box = spear.pipeline.get_bounding_box(component_desc=component_desc)
        bounding_box = spear.pipeline.expand_bounding_box(bounding_box=bounding_box, eps=expand_bounding_box_eps)
        bounding_boxes.append(bounding_box)

    # Recurse for each child component.
    if component_desc["children_components"] is not None:
        for child_component_desc in component_desc["children_components"].values():
            child_bounding_boxes = get_component_bounding_boxes(component_desc=child_component_desc, log_prefix_str=f"{log_prefix_str}    ")
            bounding_boxes.extend(child_bounding_boxes)

    return bounding_boxes

def compute_visibility_graph(free_space_points, bounding_boxes):

    num_points = free_space_points.shape[0]

    # Consider every pair of free-space points as a candidate edge.
    i_indices, j_indices = np.triu_indices(num_points, k=1)

    if i_indices.shape[0] == 0:
        return np.zeros((0, 2), dtype=int)

    # Cast a ray from each point i toward each point j and keep the edge if no bounding box lies strictly
    # between them. We transform each ray into a box's local frame and use the slab method to find the ray's
    # entry and exit distances; a box occludes the segment if it is entered after leaving point i and before
    # reaching point j. The component-from-world rotation is rigid, so distances are preserved in world units. We
    # store the rays as column vectors (3 rows, N cols) so the rotation matrix appears on the left.
    ray_deltas = free_space_points[j_indices] - free_space_points[i_indices]
    segment_lengths = np.linalg.norm(ray_deltas, axis=1)
    ray_origins = np.matrix(free_space_points[i_indices]).T
    ray_directions = np.matrix(ray_deltas/segment_lengths[:,np.newaxis]).T

    # We test every ray against every box at once, so when a ray is parallel to a box axis (a zero direction
    # component) we intentionally let the slab divisions below produce inf/nan rather than branching. The min/max
    # over the three slabs that follows resolves those entries correctly, so we suppress the expected
    # divide-by-zero and 0/0 warnings here.
    occluded = np.zeros(i_indices.shape[0], dtype=bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        for bounding_box in bounding_boxes:
            rotation_component_from_world = np.matrix(bounding_box["rotation_component_from_world"])
            translation_component_from_world = bounding_box["translation_component_from_world"]
            half_extent = bounding_box["half_extent"]
            ray_origins_component = (rotation_component_from_world*ray_origins).A + translation_component_from_world[:,np.newaxis]
            ray_directions_component = (rotation_component_from_world*ray_directions).A
            t_lo = (-half_extent[:,np.newaxis] - ray_origins_component)/ray_directions_component
            t_hi = (half_extent[:,np.newaxis] - ray_origins_component)/ray_directions_component
            t_enter = np.minimum(t_lo, t_hi).max(axis=0)
            t_exit = np.maximum(t_lo, t_hi).min(axis=0)
            occluded = np.logical_or(occluded, np.logical_and(np.logical_and(t_exit >= t_enter, t_enter > visibility_tolerance), t_enter < segment_lengths - visibility_tolerance))

    visible = np.logical_not(occluded)
    return np.column_stack([i_indices[visible], j_indices[visible]])


if __name__ == "__main__":
    process_scene()
