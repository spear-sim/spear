#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# This examples renders a path-traced image with a USpSceneCaptureComponent2D (no Movie Render Queue). Same flow as
# examples/render_image, plus the configuration that makes the scene capture use the path tracer:
#   - Hardware ray tracing is enabled (r.RayTracing.Enable=1)
#   - Ray tracing is enabled for the scene-capture view. FSceneView::IsRayTracingAllowedForView() allows ray
#     tracing for a scene capture only when bSceneCaptureUsesRayTracing is set, which comes from
#     r.RayTracing.SceneCaptures or the component's bUseRayTracingIfEnabled.
#   - The PathTracing engine show flag is set on the view (selects the path tracer over the deferred renderer).


import argparse
import os

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")  # enable cv2's EXR codec; must be set before importing cv2

import cv2
import spear
import numpy as np

_NUM_EXTRA_SETTLE_FRAMES = 4

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default=os.path.realpath(os.path.join(os.path.dirname(__file__), "out.png")))
    parser.add_argument("--width", type=int, default=None) # Viewport by default
    parser.add_argument("--height", type=int, default=None) # Viewport by default
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=8) # More than ~12 bounces makes almost no difference in most scenes.
    parser.add_argument("--denoiser", type=str, default=None)  # e.g. "NNEDenoiser" or "OIDN"; plugin must be enabled. No denoising by default.
    parser.add_argument("--manual-accumulate", action="store_true")
    args = parser.parse_args()

    if args.manual_accumulate and args.denoiser is not None:
        print("Error: The --denoiser flag is not compatible with --manual-accumulate. Specify one or the other.")
        exit(1)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    is_hdr = str.endswith(args.out, ".exr")

    spear.log("Info: Starting ...")


    # Initialize

    with instance.begin_frame():

        scene_captures_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.SceneCaptures")
        game.unreal_service.set_console_variable_value(cvar=scene_captures_cvar, value=1)

        samples_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.SamplesPerPixel")
        game.unreal_service.set_console_variable_value(cvar=samples_cvar, value=1 if args.manual_accumulate else args.samples)
        bounces_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.MaxBounces")
        game.unreal_service.set_console_variable_value(cvar=bounces_cvar, value=args.bounces)
        denoiser_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.Denoiser")
        game.unreal_service.set_console_variable_value(cvar=denoiser_cvar, value=1 if args.denoiser else 0)

        # optionally pick the spatial denoiser by the name its plugin registered (the same names shown in the
        # project's Path Tracing denoiser dropdown): "NNEDenoiser" (engine default) or "OIDN" are enabled in
        # SpearSim; the chosen denoiser's plugin must be enabled or the name won't resolve.
        if args.denoiser is not None:
            denoiser_name_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.Denoiser.Name")
            game.unreal_service.set_console_variable_value(cvar=denoiser_name_cvar, value=args.denoiser)

        # disable the in-frame path-tracer progress bar (the red line at the bottom) so it can never bake into the
        # captured image if we read before it clears on its own (Shouldn't happen with the extra settle frames, but just in case)
        progress_display_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.ProgressDisplay")
        game.unreal_service.set_console_variable_value(cvar=progress_display_cvar, value=0)

        # spawn camera sensor and get the final_tone_curve_hdr component (same as examples/render_image)
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        scene_capture_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # match the viewport, --width/--height override the capture resolution
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=scene_capture_component, viewport_desc=viewport_desc, widths=args.width, heights=args.height)

        # enable ray tracing for this component and turn on the PathTracing show flag.
        # SetShowFlagSettings() applies immediately.
        scene_capture_component.bUseRayTracingIfEnabled = True
        scene_capture_component.SetShowFlagSettings(InShowFlagSettings=[
            {"ShowFlagName": "PathTracing", "Enabled": True},
            {"ShowFlagName": "PostProcessing", "Enabled": False},
        ])
        scene_capture_component.CaptureSource = "SCS_FinalColorHDR" # no tonemapping

        scene_capture_component.TextureRenderTargetFormat = "RTF_RGBA16f"
        scene_capture_component.ChannelDataType = "Float16"

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        scene_capture_component.Initialize()
        scene_capture_component.initialize_sp_funcs()

    with instance.end_frame():
        pass
 

    # Render image

    if args.manual_accumulate:
        # manual accumulation: average independent 1-spp frames in Python, forcing a reset each frame.
        # Works even when the scene invalidates the engine's own accumulation every frame (e.g. animated content)
        accumulator = None
        for i in range(args.samples):
            spear.log(f"Info: Path tracing sample {i + 1}/{args.samples} ...")
            with instance.begin_frame():
                kismet_rendering_library = game.get_unreal_object(uclass="UKismetRenderingLibrary")
                kismet_rendering_library.RefreshPathTracingOutput() # reset accumulation before this frame renders
            with instance.end_frame():
                sample = scene_capture_component.read_pixels()["arrays"]["data"].astype(np.float32)
            accumulator = sample if accumulator is None else accumulator + sample
        final_pixels = accumulator / args.samples

    else:
        # Accumulate one path-traced sample per pixel per frame; r.PathTracing.SamplesPerPixel caps this at args.samples.
        for i in range(0, args.samples):
            spear.log(f"Info: Path tracing sample {i + 1}/{args.samples} ...")
            instance.step()

        # Wait for the final pass and possibly the denoiser
        instance.step(num_frames=_NUM_EXTRA_SETTLE_FRAMES)

        # get rendered frame
        with instance.begin_frame():
            pass
        with instance.end_frame():
            final_pixels = scene_capture_component.read_pixels()["arrays"]["data"]
            

    # Save render

    spear.log("Info: Saving image: ", args.out)
    linear_bgr = final_pixels[:,:,[2,1,0]].astype(np.float32) # RGBA float -> BGR float32 (linear HDR)
    if is_hdr:
        cv2.imwrite(args.out, linear_bgr)
    else:
        # the render target is always linear float, so quantize to 8-bit ourselves: clip to the displayable range
        # and apply the sRGB transfer function. This is also what keeps --manual-accumulate correct, since it
        # means we quantize once, after averaging, instead of averaging already-quantized sRGB samples.
        linear_bgr = np.clip(linear_bgr, 0.0, 1.0)
        srgb_bgr = np.where(linear_bgr <= 0.0031308, linear_bgr*12.92, 1.055*np.power(linear_bgr, 1.0/2.4) - 0.055)
        image = np.clip(srgb_bgr*255.0 + 0.5, 0.0, 255.0).astype(np.uint8)
        cv2.imwrite(args.out, image)


    # Shutdown

    with instance.begin_frame():
        pass
    with instance.end_frame():
        scene_capture_component.terminate_sp_funcs()
        scene_capture_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Info: Done.")
