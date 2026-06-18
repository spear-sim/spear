#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cv2
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

# The 8 corners of a unit cube in a component's local frame, expressed as signs that scale a box's half-extents.
cube_corners_normalized_component = np.array([
    [-1.0, -1.0, -1.0],
    [-1.0, -1.0,  1.0],
    [-1.0,  1.0, -1.0],
    [-1.0,  1.0,  1.0],
    [ 1.0, -1.0, -1.0],
    [ 1.0, -1.0,  1.0],
    [ 1.0,  1.0, -1.0],
    [ 1.0,  1.0,  1.0]])

# Vertex indices of the 12 edges of a bounding box, where each edge connects two corners that differ on exactly
# one axis (using the same corner indexing as cube_corners_normalized_component above).
box_edge_indices = np.array([
    [0, 4], [1, 5], [2, 6], [3, 7],
    [0, 2], [1, 3], [4, 6], [5, 7],
    [0, 1], [2, 3], [4, 5], [6, 7]])


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

    # Draw the scene's original triangle meshes in dark grey to highlight the free-space bounding boxes drawn on top.
    color = (0.4, 0.4, 0.4)

    for actor_name, actor_desc in actors.items():
        spear.log("Processing actor: ", actor_name)
        draw_actor(actor_desc=actor_desc, color=color)

    # Load the free-space points, the mutual-visibility graph (the index pairs of mutually visible points), and the
    # free-space bounding boxes computed by generate_free_space_points.py, generate_free_space_visibility_graph.py,
    # and generate_free_space_bounding_boxes.py. We draw the points and visibility graph here too, but even more
    # faintly than in visualize_free_space_visibility_graph.py, to give context for the bounding boxes without
    # cluttering them. The occupied oriented bounding boxes are recomputed and drawn per-component from the same
    # metadata as the meshes in draw_component below.
    free_space_points_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points", "free_space_points.h5"))
    visibility_graph_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_visibility_graph", "free_space_visibility_graph.h5"))
    free_space_bounding_boxes_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_bounding_boxes", "free_space_bounding_boxes.h5"))
    assert os.path.exists(free_space_points_file)
    assert os.path.exists(visibility_graph_file)
    assert os.path.exists(free_space_bounding_boxes_file)
    spear.log("Reading free-space points file: ", free_space_points_file)
    spear.log("Reading visibility graph file: ", visibility_graph_file)
    spear.log("Reading free-space bounding boxes file: ", free_space_bounding_boxes_file)
    with h5py.File(free_space_points_file, "r") as f:
        points = f["scene_points"][:]
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
    draw_lines(points=points, lines=edges, color=(1.0, 1.0, 0.0), opacity=0.001, line_width=1.0)
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
        box_corners = get_axis_aligned_bounding_box_corners(box_min=free_space_box_mins[box_index], box_max=free_space_box_maxs[box_index])

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

        draw_lines(points=box_corners, lines=box_edge_indices, color=color, opacity=opacity, line_width=line_width)

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

def get_axis_aligned_bounding_box_corners(box_min, box_max):

    # The free-space bounding boxes are axis-aligned in world space, so their corners are the box center offset by
    # the unit-cube corner signs scaled by the box's half-extent.
    center = (box_min + box_max)/2.0
    half_extent = (box_max - box_min)/2.0
    return cube_corners_normalized_component*half_extent + center

def draw_lines(points, lines, color, opacity, line_width):

    line_source = mayavi.mlab.pipeline.scalar_scatter(points[:,0], points[:,1], points[:,2])
    line_source.mlab_source.dataset.lines = lines
    line_source.update()
    mayavi.mlab.pipeline.surface(mayavi.mlab.pipeline.stripper(line_source), color=color, opacity=opacity, line_width=line_width)


if __name__ == "__main__":
    process_scene()
