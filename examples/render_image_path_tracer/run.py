
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear

parser = argparse.ArgumentParser()
parser.add_argument("--denoiser", type=str, default="")
parser.add_argument("--num-bounces", type=int, default=8)
parser.add_argument("--num-frames", type=int, default=64)
parser.add_argument("--teaser", action="store_true")
args = parser.parse_args()

assert args.denoiser in ["", "nne", "oidn"]

if args.teaser:
    width = 1920
    height = 1080
else:
    width = None
    height = None

if args.denoiser == "nne":
    denoiser = 1
    denoiser_name = "NNEDenoiser"
elif args.denoiser == "oidn":
    denoiser = 1
    denoiser_name = "OIDN"
elif args.denoiser == "":
    denoiser = 0
    denoiser_name = ""
else:
    assert False


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # force high-res textures for captured images
        game.console_service.set(name="r.Streaming.FullyLoadUsedTextures", value=1)

        # configure the path tracer via console variables

        game.console_service.set(name="r.RayTracing.Enable", value=1)
        game.console_service.set(name="r.RayTracing.SceneCaptures", value=1)

        game.console_service.set(name="r.PathTracing.SamplesPerPixel", value=args.num_frames)
        game.console_service.set(name="r.PathTracing.MaxBounces", value=args.num_bounces)
        game.console_service.set(name="r.PathTracing.Denoiser", value=denoiser)
        game.console_service.set(name="r.PathTracing.Denoiser.Name", value=denoiser_name)

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc, widths=width, heights=height)

        visualize_func = lambda data : data[:,:,[2,1,0]] # BGRA -> RGB
        # visualize_func = lambda data : data

        # enable path tracing and disable camera imperfections because they amplify noise and produce artifacts if we don't denoise
        final_tone_curve_hdr_component.SetShowFlagSettings(InShowFlagSettings=[
            {"ShowFlagName": "PathTracing", "Enabled": True},
            {"ShowFlagName": "CameraImperfections", "Enabled": False}])

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame(single_step=True):
        pass

    # inserting an extra frame or two can fix occasional render-to-texture initialization issues (advances a minimum of 3 frames)
    game.async_loading_service.wait_for_engine_idle()

    # The path tracer accumulates one sample per pixel per rendered frame, exactly like the editor's
    # path-tracing viewport, and stops once it reaches r.PathTracing.SamplesPerPixel (set to args.num_frames
    # above). Moving the camera or anything in the scene invalidates the accumulated samples and restarts
    # from scratch, so we simply render args.num_frames frames in a row without moving anything, which
    # converges the image on the final frame. The engine then applies the denoiser (if one was requested) to
    # the converged result. Note that calling instance.step(num_frames=args.num_frames) without specifying
    # single_step=True advances at least num_frames, but is not guaranteed to advance exactly num_frames.
    spear.log("Path-traced rendering beginning...")
    for i in range(args.num_frames):
        spear.log(f"Rendering frame {i:04d}...")
        instance.step(single_step=True)
    spear.log("Path-traced rendering finished.")

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame(single_step=True):
        data_bundle = final_tone_curve_hdr_component.read_pixels()

    # save image (cv2 writes the native BGR, dropping alpha)
    image_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "image.png"))
    spear.log("Saving image: ", image_file)

    image = visualize_func(data=data_bundle["arrays"]["data"])
    if image.dtype != np.uint8:
        image = (np.clip(image, 0.0, 1.0)*255.0).astype(np.uint8)
    if image.ndim == 3:
        image = image[:, :, [2,1,0]] # RGB -> BGR

    cv2.imwrite(image_file, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Done.")
