#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import numpy as np
import os
import scipy.spatial.transform
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
args = parser.parse_args()

np.random.seed(0)

# Maximum absolute roll (in degrees) added to each keyframe's camera orientation. A roll in [-max_roll_degrees,
# +max_roll_degrees] is sampled independently for each keyframe (about the camera's look direction) before
# interpolating, to add a bit of natural variation to the otherwise perfectly-upright keyframe orientations.
max_roll_degrees = 5.0


def process_scene():

    # Read the planned paths (positions sampled uniformly in time along each path, with their normalized times) and
    # the camera keyframes (a camera-to-world rotation matrix at each of a sparse set of normalized times along each
    # path) computed by generate_free_space_paths.py and generate_free_space_camera_keyframes.py.
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
    assert path_points.shape[0] == camera_keyframe_rotations.shape[0]

    num_keyframes = camera_keyframe_rotations.shape[1]
    assert num_keyframes >= 2

    # For each path, add a small independent random roll to each keyframe orientation, then interpolate the keyframe
    # orientations to the path's (denser) sample times, giving one camera orientation per path sample aligned with
    # that sample's position. We interpolate with a cubic rotation spline, which correctly handles that the keyframes
    # are rotations (not free vectors) and is smooth -- continuous in both angular velocity and acceleration -- rather
    # than the piecewise-constant angular velocity of spherical linear interpolation, which would change the camera's
    # rotation rate abruptly at each keyframe.
    camera_rotations = []
    for path_index in range(path_points.shape[0]):

        # Apply a random roll about each keyframe's look (+X) direction. Composing each keyframe rotation on the right
        # with a rotation about the local X axis rolls the camera about its own look direction.
        roll_degrees = np.random.uniform(-max_roll_degrees, max_roll_degrees, size=num_keyframes)
        keyframe_rotations = scipy.spatial.transform.Rotation.from_matrix(camera_keyframe_rotations[path_index])*scipy.spatial.transform.Rotation.from_euler("x", roll_degrees, degrees=True)

        # Interpolate the keyframe rotations to the path sample times with a cubic rotation spline. The keyframe times
        # span the whole path from 0 to 1, so every sample time lies within the interpolation range.
        interpolated_rotations = scipy.spatial.transform.RotationSpline(camera_keyframe_times[path_index], keyframe_rotations)(path_point_times[path_index])
        camera_rotations.append(interpolated_rotations.as_matrix())

    camera_rotations = np.array(camera_rotations)

    # Save each camera path ready to visualize and render: the position and camera-to-world rotation matrix at each
    # sample, along with the sample's normalized time. The positions and times are the planned path's; the rotations
    # are the interpolated camera orientations. Each rotation matrix maps points (as column vectors) from camera space
    # to world space, following the Unreal convention that the camera's +X is its look direction, +Y its right, and
    # +Z its up (so the matrix's columns are those three directions in world space).
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_paths"))
    free_space_camera_paths_file = os.path.realpath(os.path.join(free_space_dir, "free_space_camera_paths.h5"))
    spear.log("Writing free-space camera paths file: ", free_space_camera_paths_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(free_space_camera_paths_file, "w") as f:
        f.create_dataset("camera_path_times", data=path_point_times)
        f.create_dataset("camera_path_positions", data=path_points)
        f.create_dataset("camera_path_orientations", data=camera_rotations)

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
