#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import h5py
import numpy as np
import os
import spear
import time


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--user-config-files", nargs="*", default=[])
parser.add_argument("--view-selection-mode", default="proportional")
args = parser.parse_args()

# "proportional" selects a view at each location by sampling proportionally to the number of segments it sees;
# "argmax" (useful for debugging) deterministically selects the view that sees the most segments.
assert args.view_selection_mode in ["proportional", "argmax"]

np.random.seed(0)

# Number of locations sampled along each path (evenly spaced in normalized time), and number of candidate views we
# render and choose between at each location.
num_locations = 20
num_candidate_views = 32

# Candidate view directions are sampled on the band of the unit sphere between these pitches (elevation in degrees
# above the horizon). Both must lie strictly inside (-90, 90) so no view direction is vertical (which would make the
# upright orientation degenerate).
min_pitch_degrees = -30.0
max_pitch_degrees = 30.0

# The candidate views are rendered with a square segmentation image at this resolution and horizontal field of view
# (in degrees). The renders are only used to count visible segments, so a small resolution is sufficient.
camera_resolution = 256
camera_fov_degrees = 90.0

# The golden ratio, used to spread the candidate views' azimuths evenly around the sphere (see get_candidate_view_rotations).
golden_ratio = (1.0 + np.sqrt(5.0))/2.0


def process_scene():

    # Read the planned paths computed by generate_free_space_paths.py. Each path is a sequence of points sampled
    # uniformly in time along it, with the corresponding normalized times in [0, 1].
    free_space_paths_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_paths", "free_space_paths.h5"))
    assert os.path.exists(free_space_paths_file)
    spear.log("Reading free-space paths file: ", free_space_paths_file)
    with h5py.File(free_space_paths_file, "r") as f:
        path_points = f["path_points"][:]
        path_point_times = f["path_point_times"][:]

    # The same candidate view orientations are used at every location (only the camera position changes).
    candidate_view_rotations = get_candidate_view_rotations()

    # Launch SPEAR and get the game-scoped services. The user-provided config files specify how to launch the game
    # offscreen and which scene to open (the scene must be the one this export directory was generated from).
    config = spear.get_config(user_config_files=[ os.path.realpath(user_config_file) for user_config_file in args.user_config_files ])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # Spawn a camera sensor and configure its segmentation capture component (a square perspective render at our
    # resolution and field of view). The segmentation service must be initialized before the component so that the
    # rendered object-ids pass contains valid per-component-material segment ids.
    with instance.begin_frame():
        game.segmentation_service.initialize()
        camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        camera_sensor = game.unreal_service.spawn_actor(uclass=camera_sensor_uclass)
        segmentation_component = game.unreal_service.get_component_by_name(actor=camera_sensor, component_name="DefaultSceneRoot.sp_object_ids_uint8_", uclass="USpSceneCaptureComponent2D")
        segmentation_component.Width = camera_resolution
        segmentation_component.Height = camera_resolution
        segmentation_component.ProjectionType = "Perspective"
        segmentation_component.FOVAngle = camera_fov_degrees
        segmentation_component.Initialize()
        segmentation_component.initialize_sp_funcs()
    with instance.end_frame():
        pass

    # Wait for the segmentation service to finish loading its materials, and step a couple of frames to let the
    # render targets initialize, before rendering.
    game.async_loading_service.wait_for_engine_idle()
    instance.step(num_frames=2)

    # For each path, sample num_locations positions along the curve and choose a view at each.
    location_times = np.linspace(0.0, 1.0, num_locations)
    camera_keyframe_times = []
    camera_keyframe_rotations = []
    for path_index in range(path_points.shape[0]):

        locations = np.column_stack([ np.interp(location_times, path_point_times[path_index], path_points[path_index][:,axis]) for axis in range(path_points.shape[2]) ])

        chosen_rotations = []
        for location_index in range(num_locations):
            location = locations[location_index]

            # Render each candidate view from this location and count the number of distinct segments it sees (a
            # segment is a component-material pair; each is a distinct nonzero id in the rendered object-ids pass).
            num_segments = np.zeros(num_candidate_views)
            render_start_time = time.time()
            for view_index in range(num_candidate_views):
                with instance.begin_frame():
                    camera_sensor.K2_SetActorLocationAndRotation(
                        NewLocation=spear.math.to_spear_vector_from_numpy_array(numpy_array=location),
                        NewRotation=spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=candidate_view_rotations[view_index]),
                        bSweep=False, bTeleport=True)
                with instance.end_frame():
                    object_ids_bgra_uint8_image = segmentation_component.read_pixels()["arrays"]["data"]
                    raw_id_image = spear.rendering.get_object_ids_bgra_uint8_as_uint32(object_ids_bgra_uint8=object_ids_bgra_uint8_image)
                    num_segments[view_index] = np.count_nonzero(np.unique(raw_id_image))

            # The rate at which we rendered the candidate views (one rendered frame each), for basic throughput info.
            frames_per_second = num_candidate_views/(time.time() - render_start_time)

            # Select a view. If at least one candidate view sees a segment, select among the candidate views either by
            # sampling proportionally to the number of segments each sees ("proportional") or by taking the view that
            # sees the most ("argmax"); otherwise no view sees anything, so fall back to a uniform random choice.
            if num_segments.sum() > 0.0:
                if args.view_selection_mode == "proportional":
                    view_index = np.random.choice(num_candidate_views, p=num_segments/num_segments.sum())
                elif args.view_selection_mode == "argmax":
                    view_index = int(np.argmax(num_segments))
                else:
                    assert False
            else:
                view_index = np.random.randint(num_candidate_views)

            spear.log(f"Path {path_index}, location {location_index}: chose view {int(view_index)} ({int(num_segments[view_index])} segments, {frames_per_second:.1f} FPS)")
            chosen_rotations.append(candidate_view_rotations[view_index])

        camera_keyframe_times.append(location_times)
        camera_keyframe_rotations.append(np.array(chosen_rotations))

    camera_keyframe_times = np.array(camera_keyframe_times)
    camera_keyframe_rotations = np.array(camera_keyframe_rotations)

    # Terminate the camera sensor and segmentation service, and close the instance.
    with instance.begin_frame():
        pass
    with instance.end_frame():
        segmentation_component.terminate_sp_funcs()
        segmentation_component.Terminate()
        game.unreal_service.destroy_actor(actor=camera_sensor)
        game.segmentation_service.terminate()
    instance.close()

    # Save the chosen camera keyframes for each path as a normalized time and a camera-to-world rotation matrix per location. Each rotation
    # matrix maps points (as column vectors) from camera space to world space, following the Unreal convention that
    # the camera's +X is its look direction, +Y is its right direction, and +Z is its up direction (so the matrix's
    # columns are those three directions in world space). The camera position at each location is recoverable from
    # the path itself at the corresponding normalized time.
    free_space_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_keyframes"))
    free_space_camera_keyframes_file = os.path.realpath(os.path.join(free_space_dir, "free_space_camera_keyframes.h5"))
    spear.log("Writing free-space camera keyframes file: ", free_space_camera_keyframes_file)
    os.makedirs(free_space_dir, exist_ok=True)
    with h5py.File(free_space_camera_keyframes_file, "w") as f:
        f.create_dataset("camera_keyframe_times", data=camera_keyframe_times)
        f.create_dataset("camera_keyframe_orientations", data=camera_keyframe_rotations)

    spear.log("Done.")

def get_candidate_view_rotations():

    # Generate num_candidate_views upright camera orientations whose look directions are spread evenly over the band
    # of the unit sphere between min_pitch_degrees and max_pitch_degrees. This is a spherical Fibonacci lattice
    # restricted to the band: the look directions are evenly spaced in z = sin(pitch) over the band (which is even in
    # area, since a sphere's area element is uniform in z) and evenly spaced in azimuth via the golden angle (so they
    # do not cluster). Each look direction is turned into an upright orientation with no roll: the camera's up
    # direction lies in the vertical plane containing the look direction.
    world_up = np.array([0.0, 0.0, 1.0])
    sin_min_pitch = np.sin(np.deg2rad(min_pitch_degrees))
    sin_max_pitch = np.sin(np.deg2rad(max_pitch_degrees))

    rotations = []
    for view_index in range(num_candidate_views):
        sin_pitch = sin_min_pitch + (sin_max_pitch - sin_min_pitch)*(view_index + 0.5)/num_candidate_views
        cos_pitch = np.sqrt(1.0 - sin_pitch**2)
        azimuth = 2.0*np.pi*((view_index/golden_ratio) % 1.0)

        # The Unreal camera looks along its +X axis, so the look direction is the camera's +X (forward) direction. Its
        # +Y (right) and +Z (up) directions complete an upright, right-handed orthonormal frame.
        forward = np.array([cos_pitch*np.cos(azimuth), cos_pitch*np.sin(azimuth), sin_pitch])
        right = np.cross(world_up, forward)
        right = right/np.linalg.norm(right)
        up = np.cross(forward, right)

        # The camera-to-world rotation matrix's columns are the camera's +X, +Y, +Z directions in world space.
        rotations.append(np.column_stack([forward, right, up]))

    return np.array(rotations)


if __name__ == "__main__":
    process_scene()
