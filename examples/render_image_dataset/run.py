#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

#
# In this example, we provide an optimized mode that uses several complementary strategies to increase end-to-end
# rendering throughput by more than 2x. First, we use User Scene Textures (USTs), which enable Unreal to output multiple
# image modalities in a single forward rendering pass. Second, we use our camera sensor's triple-buffered readback mode
# to overlap rendering with GPU-to-staging and staging-to-CPU data copying, so our image data is already sitting in an
# appropriate CPU buffer by the time we call read_pixels(). Third, we use our general-purpose send_async API to enqueue
# work on the game thread without waiting for it to execute, which is possible because our Python code doesn't need to
# wait for the return value of K2_SetActorLocationAndRotation(...) before advancing in the code below. Fourth, we
# overlap saving each image to disk with executing each Unreal frame. In this example, both optimized and non-optimized
# modes use interprocess shared memory to avoid unnecessary data copying between the Unreal application and Python.
#

#
# IMPORTANT: Shared memory volatility contract. When a camera sensor uses shared memory, the arrays returned by
# read_pixels() are not copies. They are NumPy arrays that point directly into shared memory regions that Unreal
# writes to during subsequent readbacks. So these arrays are only safe to read during a window of time that depends on
# the camera sensor's buffering mode.
#
#     - SingleBuffered: Unreal writes to shared memory during each call to read_pixels(), and read_pixels() doesn't
#                       return until the write has finished. So the arrays returned by read_pixels() are safe to read
#                       until the next call to read_pixels().
#
#     - DoubleBuffered and TripleBuffered: Unreal may write to shared memory at any point after each call to
#                       enqueue_read_pixels(), until the next call to read_pixels() returns. So the arrays returned by
#                       read_pixels() are safe to read until the next call to enqueue_read_pixels().
#
# In all buffering modes, calling Terminate() on the camera sensor ends this guarantee, so the arrays returned by
# read_pixels() must not be read after calling Terminate(). Reading them after calling Terminate() is undefined
# behavior, and can crash the Python process.
#
# Reading these arrays outside of this window will silently return data from a different frame, or partially
# overwritten data. If you need an image for longer than this window (e.g., if you want to save it to disk while the
# next frame is executing, or if you want to accumulate images in a list), copy it into an array owned by Python
# immediately after calling read_pixels(). In this example, we index each array with a list of channels, which copies
# the data out of shared memory as a side effect. See the comments in SpSceneCaptureComponent2D.h for more details.
#

parser = argparse.ArgumentParser()
parser.add_argument("--optimized", action="store_true")
args = parser.parse_args()

M_hypersim_camera_from_unreal_camera = np.array([[  0, 1, 0],
                                                 [  0, 0, 1],
                                                 [ -1, 0, 0]], dtype=np.float32)

width = 512
height = 512

scene_desc = \
{
    "name": "apartment_0000"
}

#
# In non-optimized mode, each image comes from a separate component on BP_CameraSensor (component_name). In optimized
# mode, each UST image comes from one of the UserSceneTextures (read_pixels_array_name) on a component on
# BP_CameraSensor_UST (ust_source_component_name), and each non-UST image (ust_source_component_name is None) comes from
# the primary pixel data of the component with the same name on BP_CameraSensor_UST (component_name). If save_raw_data
# is True, we save the raw floating-point data as a float16 .npy file in addition to the visualized .png file. We only
# keep the meaningful channels of each array (channels), so the raw data has the same shape in both modes.
#

image_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "component_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "ust_source_component_name": None,
        "read_pixels_array_name": "data",
        "save_raw_data": False,
        "channels": [0,1,2], # native BGR (drop A)
        "visualize_func": lambda data, R_camera_from_world : data
    },
    {
        "name": "camera_normal",
        "component_name": "DefaultSceneRoot.world_normal_",
        "ust_source_component_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "read_pixels_array_name": "WorldNormal",
        "save_raw_data": True,
        "channels": [0,1,2], # RGB (drop A)
        "visualize_func": lambda data, R_camera_from_world : (np.clip((1.0 + (data @ R_camera_from_world.T @ M_hypersim_camera_from_unreal_camera.T))/2.0, 0.0, 1.0)*255.0).astype(np.uint8)[:,:,[2,1,0]] # RGB [0,1] -> BGR uint8
    },
    {
        "name": "diffuse_color",
        "component_name": "DefaultSceneRoot.diffuse_color_",
        "ust_source_component_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "read_pixels_array_name": "DiffuseColor",
        "save_raw_data": True,
        "channels": [0,1,2], # RGB (drop A)
        "visualize_func": lambda data, R_camera_from_world : (np.clip(data, 0.0, 1.0)*255.0).astype(np.uint8)[:,:,[2,1,0]] # RGB [0,1] -> BGR uint8
    },
    {
        "name": "post_process_input_2",
        "component_name": "DefaultSceneRoot.diffuse_and_specular_post_process_input_2_",
        "ust_source_component_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "read_pixels_array_name": "PostProcessInput2",
        "save_raw_data": True,
        "channels": [0,1,2], # RGB (drop A)
        "visualize_func": lambda data, R_camera_from_world : (np.clip(spear.rendering.tone_map_hypersim(image=data, mask=np.ones(data.shape[:2], dtype=bool)), 0.0, 1.0)*255.0).astype(np.uint8)[:,:,[2,1,0]] # tone-map HDR RGB to [0,1] using all pixels, then RGB [0,1] -> BGR uint8
    },
    {
        "name": "sp_depth_meters",
        "component_name": "DefaultSceneRoot.sp_depth_meters_",
        "ust_source_component_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "read_pixels_array_name": "SpDepthMeters",
        "save_raw_data": True,
        "channels": [0], # R (drop GBA if present)
        "visualize_func": lambda data, R_camera_from_world : cv2.applyColorMap((np.clip((data[:,:,0] - np.min(data[:,:,0])) / np.minimum((np.max(data[:,:,0]) - np.min(data[:,:,0])), 7.5), 0.0, 1.0)*255.0).astype(np.uint8), cv2.COLORMAP_VIRIDIS) # normalize to max depth of 7.5 meters, then colorize
    }
]

if not args.optimized:
    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", "non_optimized", scene_desc["name"]))
else:
    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", "optimized", scene_desc["name"]))


def save_images(camera_pose, arrays):
    R_world_from_camera = spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator={"Pitch": camera_pose["rotation_pitch"], "Yaw": camera_pose["rotation_yaw"], "Roll": camera_pose["rotation_roll"]}, as_matrix=True)
    R_camera_from_world = R_world_from_camera.T.A
    for image_desc in image_descs:
        data = arrays[image_desc["name"]]
        image_file = os.path.realpath(os.path.join(images_dir, image_desc["name"], f"{camera_pose['index']:04d}.png"))
        image = image_desc["visualize_func"](data=data, R_camera_from_world=R_camera_from_world)
        cv2.imwrite(image_file, image)
        if image_desc["save_raw_data"]:
            raw_data_file = os.path.realpath(os.path.join(images_dir, image_desc["name"], f"{camera_pose['index']:04d}.npy"))
            np.save(raw_data_file, data.astype(np.float16))


def get_source_component_name(image_desc):
    if image_desc["ust_source_component_name"] is None:
        return image_desc["component_name"]
    else:
        return image_desc["ust_source_component_name"]


if __name__ == "__main__":

    spear.log("Processing scene: ", scene_desc["name"])

    # create output dir
    for image_desc in image_descs:
        image_dir = os.path.realpath(os.path.join(images_dir, image_desc["name"]))
        if os.path.exists(image_dir):
            spear.log("Directory exists, removing: ", image_dir)
            shutil.rmtree(image_dir, ignore_errors=True)
        os.makedirs(image_dir, exist_ok=True)

    # get camera poses
    camera_poses_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "camera_poses"))
    camera_poses_file = os.path.realpath(os.path.join(camera_poses_dir, f"{scene_desc['name']}.csv"))
    df = pd.read_csv(camera_poses_file, comment="#")
    camera_poses = df.to_records()

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # set cinematic rendering quality
        game.rendering_quality_service.set_default_mrq_rendering_quality()

        if not args.optimized:

            # spawn camera sensor
            bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
            bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

            # initialize components (need to call initialize_sp_funcs() after calling Initialize() because
            # read_pixels() is registered during Initialize())
            for image_desc in image_descs:
                image_desc["component"] = game.unreal_service.get_component_by_name(uclass="USpSceneCaptureComponent2D", actor=bp_camera_sensor, component_name=image_desc["component_name"])
                image_desc["component"].Width = width
                image_desc["component"].Height = height
                image_desc["component"].Initialize()
                image_desc["component"].initialize_sp_funcs()
            components = { image_desc["component_name"]: image_desc["component"] for image_desc in image_descs }

        else:

            # spawn camera sensor
            bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor_UST.BP_CameraSensor_UST_C")
            bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

            # initialize components (need to call initialize_sp_funcs() after calling Initialize() because
            # read_pixels() is registered during Initialize())
            components = {}
            for image_desc in image_descs:
                source_component_name = get_source_component_name(image_desc=image_desc)
                if source_component_name not in components:
                    components[source_component_name] = game.unreal_service.get_component_by_name(uclass="USpSceneCaptureComponent2D", actor=bp_camera_sensor, component_name=source_component_name)
            for component_name, component in components.items():
                component.UserSceneTextureNames = [ image_desc["read_pixels_array_name"] for image_desc in image_descs if component_name == image_desc["ust_source_component_name"] ]
                component.BufferingMode = "TripleBuffered"
                component.Width = width
                component.Height = height
                component.Initialize()
                component.initialize_sp_funcs()

    with instance.end_frame():
        pass

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and inserting
    # an extra frame or two can fix occasional render-to-texture initialization issues (advances a minimum of
    # 4 frames)
    game.async_loading_service.wait_for_engine_idle()

    if len(camera_poses) == 0:

        spear.log("No camera poses, nothing to render.")

    elif not args.optimized:

        #
        # With single-buffered readback, read_pixels() at the end of frame i returns the image that was rendered in
        # frame i. So we render and read back each image in the same frame, and then save it after the frame is
        # finished. Each step happens sequentially, so the engine is idle while we save images, and we're idle while
        # the engine renders.
        #

        start_time_seconds = time.time()

        for camera_pose in camera_poses:

            with instance.begin_frame():
                bp_camera_sensor.K2_SetActorLocationAndRotation(
                    NewLocation={"X": camera_pose["location_x"], "Y": camera_pose["location_y"], "Z": camera_pose["location_z"]},
                    NewRotation={"Pitch": camera_pose["rotation_pitch"], "Yaw": camera_pose["rotation_yaw"], "Roll": camera_pose["rotation_roll"]})

            with instance.end_frame(single_step=True):
                # indexing with a list of channels copies the data out of shared memory
                arrays = { image_desc["name"]: image_desc["component"].read_pixels()["arrays"]["data"][:,:,image_desc["channels"]] for image_desc in image_descs }

            save_images(camera_pose=camera_pose, arrays=arrays)

        end_time_seconds = time.time()

    else:

        #
        # With triple-buffered readback, read_pixels() at the end of frame i returns the image that was rendered in
        # frame i-2. So we render each image in one frame and read it back two frames later. The arrays returned by
        # read_pixels() point to shared memory that gets overwritten during the next frame, so we copy them into NumPy
        # arrays owned by Python immediately after calling read_pixels(). We can then visualize and save the previous
        # frame's images while the engine is executing the current frame, i.e., in between our with
        # instance.begin_frame() and with instance.end_frame() blocks.
        #

        #
        # Priming phase: render the 0th and 1st images without reading anything back, since read_pixels() doesn't
        # return an image until two frames after it was rendered
        #

        # priming frame 0: render the 0th image
        with instance.begin_frame():
            bp_camera_sensor.send_async.K2_SetActorLocationAndRotation(
                NewLocation={"X": camera_poses[0]["location_x"], "Y": camera_poses[0]["location_y"], "Z": camera_poses[0]["location_z"]},
                NewRotation={"Pitch": camera_poses[0]["rotation_pitch"], "Yaw": camera_poses[0]["rotation_yaw"], "Roll": camera_poses[0]["rotation_roll"]})
            for component in components.values():
                component.send_async.enqueue_read_pixels()
        with instance.end_frame(single_step=True):
            pass

        # priming frame 1: render the 1st image (if there is only one camera pose, render the 0th image again)
        with instance.begin_frame():
            if len(camera_poses) > 1:
                bp_camera_sensor.send_async.K2_SetActorLocationAndRotation(
                    NewLocation={"X": camera_poses[1]["location_x"], "Y": camera_poses[1]["location_y"], "Z": camera_poses[1]["location_z"]},
                    NewRotation={"Pitch": camera_poses[1]["rotation_pitch"], "Yaw": camera_poses[1]["rotation_yaw"], "Roll": camera_poses[1]["rotation_roll"]})
            for component in components.values():
                component.send_async.enqueue_read_pixels()
        with instance.end_frame(single_step=True):
            pass

        # don't time the priming frames so we're measuring steady-state throughput
        start_time_seconds = time.time()

        previous_camera_pose = None
        previous_arrays = None

        #
        # Steady-state phase: render an image, read back the image that was rendered two frames ago, and save the
        # image that was read back in the previous frame
        #

        for camera_pose, render_camera_pose in zip(camera_poses[:-2], camera_poses[2:]):

            with instance.begin_frame():

                # use send_async so these calls return immediately instead of blocking until the game thread executes
                # them, which lets us start saving images sooner
                bp_camera_sensor.send_async.K2_SetActorLocationAndRotation(
                    NewLocation={"X": render_camera_pose["location_x"], "Y": render_camera_pose["location_y"], "Z": render_camera_pose["location_z"]},
                    NewRotation={"Pitch": render_camera_pose["rotation_pitch"], "Yaw": render_camera_pose["rotation_yaw"], "Roll": render_camera_pose["rotation_roll"]})
                for component in components.values():
                    component.send_async.enqueue_read_pixels()

            # save images in between begin_frame() and end_frame() to overlap the work with the UE frame
            if previous_arrays is not None:
                save_images(camera_pose=previous_camera_pose, arrays=previous_arrays)

            with instance.end_frame(single_step=True):
                data_bundles = { component_name: component.read_pixels() for component_name, component in components.items() }
                previous_camera_pose = camera_pose
                # indexing with a list of channels copies the data out of shared memory
                previous_arrays = { image_desc["name"]: data_bundles[get_source_component_name(image_desc=image_desc)]["arrays"][image_desc["read_pixels_array_name"]][:,:,image_desc["channels"]] for image_desc in image_descs }

        #
        # Trailing phase: don't render any new images, read back the last two images, and save the remaining images
        #

        # trailing frame 0: read back the second-to-last image, and save the previous image (if there is only one
        # camera pose, the image we read back here is the 0th image, which we read back again in the next frame, so we
        # ignore it)

        with instance.begin_frame():
            for component in components.values():
                component.send_async.enqueue_read_pixels()

        if previous_arrays is not None:
            save_images(camera_pose=previous_camera_pose, arrays=previous_arrays)

        with instance.end_frame(single_step=True):
            data_bundles = { component_name: component.read_pixels() for component_name, component in components.items() }
            if len(camera_poses) > 1:
                previous_camera_pose = camera_poses[-2]
                previous_arrays = { image_desc["name"]: data_bundles[get_source_component_name(image_desc=image_desc)]["arrays"][image_desc["read_pixels_array_name"]][:,:,image_desc["channels"]] for image_desc in image_descs }

        # trailing frame 1: read back the last image, and save the previous image

        with instance.begin_frame():
            for component in components.values():
                component.send_async.enqueue_read_pixels()

        if previous_arrays is not None:
            save_images(camera_pose=previous_camera_pose, arrays=previous_arrays)

        with instance.end_frame(single_step=True):
            data_bundles = { component_name: component.read_pixels() for component_name, component in components.items() }
            previous_camera_pose = camera_poses[-1]
            previous_arrays = { image_desc["name"]: data_bundles[get_source_component_name(image_desc=image_desc)]["arrays"][image_desc["read_pixels_array_name"]][:,:,image_desc["channels"]] for image_desc in image_descs }

        # save the last image
        save_images(camera_pose=previous_camera_pose, arrays=previous_arrays)

        end_time_seconds = time.time()

    instance.step() # needed after the last call to instance.end_frame(single_step=True)

    if len(camera_poses) > 0:
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        num_steps = len(camera_poses)
        spear.log(f"Average frame time for rendering and saving images to disk (optimized={args.optimized}): {(elapsed_time_seconds / num_steps)*1000.0:.4f} ms ({num_steps / elapsed_time_seconds:.4f} FPS)")

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component in components.values():
            component.terminate_sp_funcs()
            component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    # instance.close()

    spear.log("Done.")
