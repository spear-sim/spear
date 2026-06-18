#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cv2
import h5py
import numpy as np
import os
import spear
import time


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline-dir", required=True)
parser.add_argument("--user-config-files", nargs="*", default=[])
parser.add_argument("--preview", action="store_true")
args = parser.parse_args()

# The final rendered images are 1080p, with a horizontal field of view (in degrees) matching the one used to choose
# the camera orientations in generate_free_space_camera_keyframes.py.
final_image_width = 1920
final_image_height = 1080
camera_fov_degrees = 90.0

# In preview mode (--preview) we instead render small images, scaled so their widest dimension is this many pixels
# while keeping the final aspect ratio, for quickly checking the camera paths.
preview_image_max_dimension = 256

# Warm-up frames when rendering.
num_warmup_frames = 30


def process_scene():

    # Render final images at full resolution into a "final" subdirectory, or, in preview mode, smaller images scaled
    # so their widest dimension is preview_image_max_dimension (keeping the final aspect ratio) into a "preview"
    # subdirectory.
    if args.preview:
        if final_image_width >= final_image_height:
            image_width = preview_image_max_dimension
            image_height = round(preview_image_max_dimension*final_image_height/final_image_width)
        else:
            image_height = preview_image_max_dimension
            image_width = round(preview_image_max_dimension*final_image_width/final_image_height)
        images_dir_prefix = "preview"
    else:
        image_width = final_image_width
        image_height = final_image_height
        images_dir_prefix = "final"

    # Read the camera paths computed by generate_free_space_camera_paths.py: a camera position and a camera-to-world
    # rotation matrix at each of a dense set of samples along each path.
    free_space_camera_paths_file = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_paths", "free_space_camera_paths.h5"))
    assert os.path.exists(free_space_camera_paths_file)
    spear.log("Reading free-space camera paths file: ", free_space_camera_paths_file)
    with h5py.File(free_space_camera_paths_file, "r") as f:
        camera_path_positions = f["camera_path_positions"][:]
        camera_path_orientations = f["camera_path_orientations"][:]

    # Launch SPEAR and get the game-scoped services. The user-provided config files specify how to launch the game
    # offscreen and which scene to open (the scene must be the one this export directory was generated from).
    config = spear.get_config(user_config_files=[ os.path.realpath(user_config_file) for user_config_file in args.user_config_files ])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # Spawn a camera sensor and configure its final-tone-curve (RGB) capture component as a perspective render at our
    # resolution and field of view.
    with instance.begin_frame():
        camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        camera_sensor = game.unreal_service.spawn_actor(uclass=camera_sensor_uclass)
        capture_component = game.unreal_service.get_component_by_name(actor=camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")
        capture_component.Width = image_width
        capture_component.Height = image_height
        capture_component.ProjectionType = "Perspective"
        capture_component.FOVAngle = camera_fov_degrees
        capture_component.Initialize()
        capture_component.initialize_sp_funcs()
    with instance.end_frame():
        pass

    # Wait for the engine to finish loading, and step a couple of frames to let the render targets initialize, before rendering.
    game.async_loading_service.wait_for_engine_idle()
    instance.step(num_frames=2)

    images_dir = os.path.realpath(os.path.join(args.pipeline_dir, "free_space_camera_path_images", images_dir_prefix))
    os.makedirs(images_dir, exist_ok=True)

    # Render an image at each sample of each camera path, saving each path's images to its own subdirectory.
    num_frames = camera_path_positions.shape[1]
    for path_index in range(camera_path_positions.shape[0]):

        path_images_dir = os.path.realpath(os.path.join(images_dir, f"{path_index:04}"))
        os.makedirs(path_images_dir, exist_ok=True)

        spear.log(f"Rendering {num_warmup_frames} warm-up frames for path {path_index}...")

        with instance.begin_frame():
            camera_sensor.K2_SetActorLocationAndRotation(
                NewLocation=spear.math.to_spear_vector_from_numpy_array(numpy_array=camera_path_positions[path_index][0]),
                NewRotation=spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=camera_path_orientations[path_index][0]),
                bSweep=False, bTeleport=True)
        with instance.end_frame():
            pass

        instance.step(num_frames=num_warmup_frames)

        spear.log(f"Rendering {num_frames} images for path {path_index}...")

        render_start_time = time.time()
        for frame_index in range(num_frames):
            with instance.begin_frame():
                camera_sensor.K2_SetActorLocationAndRotation(
                    NewLocation=spear.math.to_spear_vector_from_numpy_array(numpy_array=camera_path_positions[path_index][frame_index]),
                    NewRotation=spear.math.to_spear_rotator_from_numpy_matrix(numpy_matrix=camera_path_orientations[path_index][frame_index]),
                    bSweep=False, bTeleport=True)
            with instance.end_frame():
                # The final-tone-curve component renders a BGRA uint8 image, which OpenCV writes directly as a PNG.
                image = capture_component.read_pixels()["arrays"]["data"]
            image_file = os.path.realpath(os.path.join(path_images_dir, f"{frame_index:04}.png"))
            spear.log("Saving image: ", image_file)
            cv2.imwrite(image_file, image)

        frames_per_second = num_frames/(time.time() - render_start_time)
        spear.log(f"Rendered {num_frames} images for path {path_index} ({frames_per_second:.1f} FPS).")

    # Terminate the camera sensor and close the instance.
    with instance.begin_frame():
        pass
    with instance.end_frame():
        capture_component.terminate_sp_funcs()
        capture_component.Terminate()
        game.unreal_service.destroy_actor(actor=camera_sensor)
    instance.close()

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
