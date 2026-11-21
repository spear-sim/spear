#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import numpy as np
import os
import spear
import unreal_geometry_utils


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
args = parser.parse_args()

assert os.path.exists(args.pipeline_dir)

np.random.seed(0)

# Number of candidate points to sample uniformly within the scene AABB.
num_candidate_points = 2000

# Tolerance in Unreal world units (cm). inside_tolerance expands each bounding box slightly when testing for
# containment.
inside_tolerance = 0.01

# We expand every obstacle (component bounding box) by eps when testing whether a point is in free space, so that
# the near face of the camera frustum at a free-space point is also guaranteed to lie in free space. The camera's
# pinhole is in free space, but without this expansion the finite near face of its frustum could still poke into an
# obstacle, which makes the rendered images clip against geometry. eps is the distance from the pinhole to the
# farthest corner of the near face, so a clear sphere of radius eps around the pinhole contains the entire near
# face. We compute it once, here, and the other free-space stages just reuse the value. The final images are
# rendered by generate_free_space_camera_path_images.py at 1080p, with a 90 degree horizontal field of view (which
# it sets directly on the capture component), and with Unreal's default near clipping plane (which we never
# override on the capture component):
#
#     final_image_width          = 1920
#     final_image_height         = 1080
#     final_image_fov_horizontal = 90.0    # In degrees.
#     final_image_znear          = 10.0    # Unreal's default near clipping plane GNearClippingPlane, in cm.
#
# The vertical field of view follows from the horizontal field of view and the image aspect ratio (Unreal's
# BuildProjectionMatrix applies the horizontal half-angle to the x axis and scales the y axis by width/height):
#
#     half_fov_horizontal        = (final_image_fov_horizontal*pi/180.0)/2.0
#     half_fov_vertical          = atan(tan(half_fov_horizontal)*final_image_height/final_image_width)
#     final_image_fov_vertical   = half_fov_vertical*2.0*180.0/pi = 58.72 degrees
#
# and the pinhole-to-near-face-corner distance is:
#
#     expand_bounding_box_eps    = final_image_znear*sqrt(1 + tan(half_fov_horizontal)**2 + tan(half_fov_vertical)**2)
#                                = 15.22 cm
#                               ~= 20.0 cm

expand_bounding_box_eps = 20.0


def process_scene():

    bounding_boxes = unreal_geometry_utils.get_component_bounding_boxes_for_scene(pipeline_dir=args.pipeline_dir, expand_bounding_box_eps=expand_bounding_box_eps)
    assert len(bounding_boxes) > 0

    # Sample candidate points uniformly within the AABB of all the oriented bounding boxes' corners.
    corners = np.array([ bounding_box["corners_world"] for bounding_box in bounding_boxes ]).reshape(-1, 3)
    candidate_points = np.random.uniform(low=corners.min(axis=0), high=corners.max(axis=0), size=(num_candidate_points, 3))

    free_space_mask = np.logical_not(are_points_inside_any_bounding_box(points=candidate_points, bounding_boxes=bounding_boxes))
    free_space_points = candidate_points[free_space_mask]
    spear.log(f"Sampled {free_space_points.shape[0]} free-space points out of {num_candidate_points} candidate points.")

    # Save the free-space points. The component oriented bounding boxes are recomputed from unreal_metadata/scene.json
    # by each downstream stage, so we only store the free-space points themselves here.
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_points"))
    free_space_points_file = os.path.realpath(os.path.join(free_space_dir, "free_space_points.h5"))
    spear.log("Writing free-space points file: ", free_space_points_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(free_space_points_file, "w") as f:
        f.create_dataset("free_space_points", data=free_space_points)

    spear.log("Done.")

def are_points_inside_any_bounding_box(points, bounding_boxes):

    # A point is inside the scene geometry if it is inside any (slightly expanded) oriented bounding box. We
    # store the points as columns and transform them into each box's local frame, where the containment test is
    # axis-aligned. Storing the points as columns makes the meaning of each matrix multiplication explicit: it
    # maps a world point to a component-frame point.
    points_world = np.matrix(points).T
    inside = np.zeros(points_world.shape[1], dtype=bool)
    for bounding_box in bounding_boxes:
        rotation_component_from_world = np.matrix(bounding_box["rotation_component_from_world"])
        translation_component_from_world = bounding_box["translation_component_from_world"]
        half_extent = bounding_box["half_extent"]
        points_component = (rotation_component_from_world*points_world).A + translation_component_from_world[:,np.newaxis]
        inside = np.logical_or(inside, np.all(np.abs(points_component) <= half_extent[:,np.newaxis] + inside_tolerance, axis=0))
    return inside


if __name__ == "__main__":
    process_scene()
