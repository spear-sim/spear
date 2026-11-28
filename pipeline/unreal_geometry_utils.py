#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Helper functions for pipeline stages that consume the "unreal_metadata" and "unreal_geometry" directories produced
# by editor/export_scene_metadata.py and editor/export_unreal_geometry.py. Any stage that imports this file can be
# executed any time after those two stages have been executed.

import colorsys
import json
import mayavi.mlab
import numpy as np
import os
import pathlib
import spear
import trimesh


scene_component_classes = ["SceneComponent", "StaticMeshComponent"]
static_mesh_component_classes = ["StaticMeshComponent"]
physics_constraint_component_classes = ["PhysicsConstraintComponent"]


#
# Functions for getting actors from the exported metadata.
#

def get_actors(pipeline_dir, ignore_actors=None):

    unreal_metadata_dir = spear.path.realpath(pipeline_dir, "unreal_metadata")
    actors_json_file = spear.path.realpath(unreal_metadata_dir, "scene.json")
    spear.log("Reading JSON file: ", actors_json_file)
    assert os.path.exists(actors_json_file)
    with open(actors_json_file, "r") as f:
        actors_json = json.load(f)

    # We only export geometry for actors for which relevant_for_level_bounds is True, so we apply the same filter
    # here, and ignore actors with no root component.
    actors = actors_json
    if ignore_actors is not None:
        actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["stable_name"] not in ignore_actors }
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["editor_properties"]["relevant_for_level_bounds"] }
    actors = { actor_name: actor_desc for actor_name, actor_desc in actors.items() if actor_desc["root_component"] is not None }

    return actors


#
# Functions for computing world-space oriented bounding boxes from the exported metadata.
#

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

def get_component_bounding_boxes_for_scene(pipeline_dir, expand_bounding_box_eps):

    bounding_boxes = []
    for actor_name, actor_desc in get_actors(pipeline_dir=pipeline_dir).items():
        spear.log("Processing actor: ", actor_name)
        bounding_boxes.extend(get_component_bounding_boxes(component_desc=actor_desc["root_component"], expand_bounding_box_eps=expand_bounding_box_eps, log_prefix_str="    "))

    return bounding_boxes

def get_component_bounding_boxes(component_desc, expand_bounding_box_eps, log_prefix_str):

    spear.log(f"{log_prefix_str}Processing component: {component_desc['stable_name']}")

    # We only build a bounding box for StaticMeshComponents that have a static mesh assigned.
    bounding_boxes = []
    if component_desc["class"] in static_mesh_component_classes and component_desc["editor_properties"]["static_mesh"] is not None:
        bounding_box = get_bounding_box(component_desc=component_desc)
        bounding_box = expand_bounding_box(bounding_box=bounding_box, eps=expand_bounding_box_eps)
        bounding_boxes.append(bounding_box)

    # Recurse for each child component.
    if component_desc["children_components"] is not None:
        for child_component_desc in component_desc["children_components"].values():
            bounding_boxes.extend(get_component_bounding_boxes(component_desc=child_component_desc, expand_bounding_box_eps=expand_bounding_box_eps, log_prefix_str=f"{log_prefix_str}    "))

    return bounding_boxes

def get_bounding_box(component_desc):

    assert "world_transform" in component_desc["attributes"]
    assert "local_bounds" in component_desc["attributes"]
    assert "static_mesh" in component_desc["editor_properties"]

    # A StaticMeshComponent exports its local-space bounds (local_bounds); combined with its world transform, this
    # gives an oriented bounding box in world space. The exported world transform and local bounds parse into
    # matrices: a (3, 1) translation, a (3, 3) rotation, a (3, 3) diagonal scale, and (3, 1) local bounds.
    transform_world_from_component = spear.json.to_numpy_transform_from_json_transform(json_transform=component_desc["attributes"]["world_transform"], as_matrix=True)
    box_min, box_max = component_desc["attributes"]["local_bounds"]
    box_min = spear.json.to_numpy_array_from_json_vector(json_vector=box_min, as_matrix=True)
    box_max = spear.json.to_numpy_array_from_json_vector(json_vector=box_max, as_matrix=True)

    translation_world = transform_world_from_component["translation"]
    rotation_world_from_component = transform_world_from_component["rotation"]
    rotation_component_from_world = rotation_world_from_component.T
    scale = transform_world_from_component["scale"]

    # The box is axis-aligned in the component's local frame, so its world half-extent is the local half-extent
    # scaled by the component's scale, its world orientation is the component's rotation, and its world center is
    # the local center mapped out to world space. The rotation is rigid, so distances are preserved in world units.
    # We use the absolute value of the scale for the half-extent, because the scale is negative along mirrored axes,
    # but the half-extent must be non-negative. We use the signed scale for the center, because mirroring moves it.
    half_extent = np.abs(scale)*(box_max - box_min)/2.0
    center_world = rotation_world_from_component*scale*(box_min + box_max)/2.0 + translation_world
    translation_component_from_world = -rotation_component_from_world*center_world

    # World-space corners: scale the unit-cube corner signs by the world half-extent, then rotate and translate
    # them out to world space. We keep the corners as column vectors (3 rows, N cols) so the rotation matrix
    # appears on the left.
    corner_offsets_component = np.matrix(cube_corners_normalized_component*half_extent.A1).T
    corners_world = rotation_world_from_component*corner_offsets_component + center_world

    # Compute every piece of derived data we need downstream once, so a bounding box is described by a single dict:
    # the component-from-world rotation and translation (the local-frame containment and slab tests), the
    # world-space center and axes (the oriented-box overlap test), the half-extent, and the world-space corners.
    return {
        "rotation_component_from_world": rotation_component_from_world.A,
        "translation_component_from_world": translation_component_from_world.A1,
        "half_extent": half_extent.A1,
        "center_world": center_world.A1,
        "axes_world": rotation_world_from_component.A,
        "corners_world": corners_world.T.A}

def expand_bounding_box(bounding_box, eps):

    # Return a copy of bounding_box expanded by eps along each of its local axes (in world units), leaving its
    # center and orientation unchanged. A point outside the expanded box is then guaranteed to be more than eps
    # away (in Euclidean distance) from the original box, which the free-space pipeline relies on to keep a
    # clearance of eps around every obstacle.

    half_extent = bounding_box["half_extent"] + eps
    rotation_world_from_component = np.matrix(bounding_box["axes_world"])
    center_world = np.matrix(bounding_box["center_world"]).T

    # Recompute the world-space corners from the expanded half-extent (the same construction as get_bounding_box):
    # scale the unit-cube corner signs by the expanded half-extent, then rotate and translate them out to world
    # space, keeping the corners as column vectors (3 rows, N cols) so the rotation matrix appears on the left.
    corner_offsets_component = np.matrix(cube_corners_normalized_component*half_extent).T
    corners_world = rotation_world_from_component*corner_offsets_component + center_world

    # The center and orientation are unchanged, so the component-from-world rotation and translation carry over.
    return {
        "rotation_component_from_world": bounding_box["rotation_component_from_world"],
        "translation_component_from_world": bounding_box["translation_component_from_world"],
        "half_extent": half_extent,
        "center_world": bounding_box["center_world"],
        "axes_world": bounding_box["axes_world"],
        "corners_world": corners_world.T.A}

def get_axis_aligned_bounding_box_corners(box_min, box_max):

    # The corners of an axis-aligned box are the box center offset by the unit-cube corner signs scaled by the box's
    # half-extent.
    center = (box_min + box_max)/2.0
    half_extent = (box_max - box_min)/2.0
    return cube_corners_normalized_component*half_extent + center


#
# Functions for visualizing the exported geometry with Mayavi. If visual_parity_with_unreal is True, we swap the y
# and z coordinates of everything we draw to match the visual appearance of the Unreal Editor.
#

origin_scale_factor = 1.0
mesh_opacity = 1.0

c_x_axis = (1.0,  0.0,  0.0)
c_y_axis = (0.0,  1.0,  0.0)
c_z_axis = (0.0,  0.0,  1.0)

def draw_origin_axes(visual_parity_with_unreal):

    # Set axes to be 1 meter, or 100 Unreal units.
    origin_world = np.array([[0.0,   0.0,   0.0]])
    x_axis_world = np.array([[100.0, 0.0,   0.0]])
    y_axis_world = np.array([[0.0,   100.0, 0.0]])
    z_axis_world = np.array([[0.0,   0.0,   100.0]])

    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
    if visual_parity_with_unreal:
        origin_world = origin_world[:,[0,2,1]]
        x_axis_world = x_axis_world[:,[0,2,1]]
        y_axis_world = y_axis_world[:,[0,2,1]]
        z_axis_world = z_axis_world[:,[0,2,1]]

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         x_axis_world[:,0], x_axis_world[:,1], x_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_x_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         y_axis_world[:,0], y_axis_world[:,1], y_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_y_axis)

    mayavi.mlab.quiver3d(origin_world[:,0], origin_world[:,1], origin_world[:,2],
                         z_axis_world[:,0], z_axis_world[:,1], z_axis_world[:,2],
                         mode="arrow", scale_factor=origin_scale_factor, color=c_z_axis)

def draw_actors(pipeline_dir, actors, color, visual_parity_with_unreal, color_mode="single_color", draw_bounding_boxes=False):

    # color_mode is "single_color" (draw every mesh in color), "unique_color_per_actor", or
    # "unique_color_per_component". If draw_bounding_boxes is True, we also draw each StaticMeshComponent's oriented
    # bounding box as a white wireframe.
    assert color_mode in ["single_color", "unique_color_per_actor", "unique_color_per_component"]

    for actor_name, actor_desc in actors.items():
        spear.log("Processing actor: ", actor_name)
        actor_color = color
        if color_mode == "unique_color_per_actor":
            actor_color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)
        draw_component(
            pipeline_dir=pipeline_dir,
            transform_world_from_parent_component=spear.math.get_identity_transform(),
            component_desc=actor_desc["root_component"],
            color=actor_color,
            visual_parity_with_unreal=visual_parity_with_unreal,
            color_mode=color_mode,
            draw_bounding_boxes=draw_bounding_boxes,
            log_prefix_str="    ")

def draw_component(pipeline_dir, transform_world_from_parent_component, component_desc, color, visual_parity_with_unreal, color_mode, draw_bounding_boxes, log_prefix_str):

    # Only process SceneComponents...
    component_class = component_desc["class"]
    if component_class in scene_component_classes:
        spear.log(f"{log_prefix_str}Processing SceneComponent: {component_desc['stable_name']}")

        transform_parent_from_current_component, is_absolute_location, is_absolute_rotation, is_absolute_scale = \
            spear.json.to_spear_transform_from_json_component(json_component=component_desc)

        transform_world_from_current_component = spear.math.compose_component_transforms(
            transforms=[transform_world_from_parent_component, transform_parent_from_current_component],
            is_absolute_location=[is_absolute_location], is_absolute_rotation=[is_absolute_rotation], is_absolute_scale=[is_absolute_scale],
            as_spear=True)
        M_world_from_current_component = spear.math.to_numpy_matrix_from_spear_transform(spear_transform=transform_world_from_current_component, as_matrix=True)

        # Check the M_world_from_current_component matrix that we computed above against Unreal's
        # SceneComponent.get_world_transform() function. We store the output from get_world_transform() in our
        # exported JSON file for each SceneComponent, so we can simply compare the matrix we computed above
        # against the stored matrix.
        world_transform = spear.json.to_numpy_matrix_from_json_matrix(json_matrix=component_desc["attributes"]["world_transform_as_matrix"], as_matrix=True)
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
                numerical_parity_obj_path = spear.path.realpath(pipeline_dir, "unreal_geometry", "numerical_parity", obj_path_suffix)
                spear.log(f"{log_prefix_str}Reading OBJ file: {numerical_parity_obj_path}")

                mesh = trimesh.load_mesh(numerical_parity_obj_path, process=False, validate=False)
                V_current_component = np.matrix(np.c_[mesh.vertices, np.ones(mesh.vertices.shape[0])]).T
                V_world = M_world_from_current_component*V_current_component
                assert np.allclose(V_world[3,:], 1.0)
                mesh.vertices = V_world.T.A[:,0:3]

                if color_mode == "unique_color_per_component":
                    color = colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0)

                # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                if visual_parity_with_unreal:
                    mesh.vertices = mesh.vertices[:,[0,2,1]]

                mayavi.mlab.triangular_mesh(
                    mesh.vertices[:,0], mesh.vertices[:,1], mesh.vertices[:,2], mesh.faces, representation="surface", color=color, opacity=mesh_opacity)

                # Compute the component's oriented bounding box from the exported local_bounds and world_transform
                # metadata (the same get_bounding_box the free-space pipeline uses) and draw it as a white wireframe.
                if draw_bounding_boxes:
                    box_corners = get_bounding_box(component_desc=component_desc)["corners_world"]

                    # Swap y and z coordinates to match the visual appearance of the Unreal editor.
                    if visual_parity_with_unreal:
                        box_corners = box_corners[:,[0,2,1]]
                    draw_lines(points=box_corners, lines=box_edge_indices, color=(1.0, 1.0, 1.0), opacity=1.0, line_width=1.0)

        # Recurse for each child component.
        for child_component_desc in component_desc["children_components"].values():
            draw_component(
                pipeline_dir=pipeline_dir,
                transform_world_from_parent_component=transform_world_from_current_component,
                component_desc=child_component_desc,
                color=color,
                visual_parity_with_unreal=visual_parity_with_unreal,
                color_mode=color_mode,
                draw_bounding_boxes=draw_bounding_boxes,
                log_prefix_str=f"{log_prefix_str}    ")

def draw_lines(points, lines, color, opacity, line_width):

    line_source = mayavi.mlab.pipeline.scalar_scatter(points[:,0], points[:,1], points[:,2])
    line_source.mlab_source.dataset.lines = lines
    line_source.update()
    mayavi.mlab.pipeline.surface(mayavi.mlab.pipeline.stripper(line_source), color=color, opacity=opacity, line_width=line_width)
