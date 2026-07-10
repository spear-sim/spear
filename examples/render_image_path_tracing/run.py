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
import cv2
import os
import spear
import sys

_NUM_EXTRA_SETTLE_FRAMES = 4

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=str, default=os.path.realpath(os.path.join(os.path.dirname(__file__), "out.png")))
    parser.add_argument("--width", type=int, default=None) # Viewport by default
    parser.add_argument("--height", type=int, default=None) # Viewport by default
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--bounces", type=int, default=8) # More than ~12 bounces makes almost no difference in most scenes.
    parser.add_argument("--denoiser", type=str, default=None)  # e.g. "NNEDenoiser" or "OIDN"; plugin must be enabled. No denoising by default.
    args = parser.parse_args()

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    spear.log("Info: Starting ...")

    # initialize actors and components
    with instance.begin_frame():

        # enable hardware ray tracing
        ray_tracing_enable_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.Enable")
        game.unreal_service.set_console_variable_value(cvar=ray_tracing_enable_cvar, value=1)

        # enable ray tracing for scene-capture views (the cvar half of the gate; the per-component half is set below)
        scene_captures_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.SceneCaptures")
        game.unreal_service.set_console_variable_value(cvar=scene_captures_cvar, value=1)

        # path tracer quality: samples per pixel (caps accumulation), max light bounces, denoiser
        samples_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.SamplesPerPixel")
        game.unreal_service.set_console_variable_value(cvar=samples_cvar, value=args.samples)
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

        # don't denoise the alpha channel; r.OIDN.DenoiseAlpha only exists if the OIDN plugin is enabled, so this
        # is a no-op (rather than an error) when OIDN isn't installed or another denoiser is selected
        oidn_denoise_alpha_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.OIDN.DenoiseAlpha")
        if oidn_denoise_alpha_cvar:
            game.unreal_service.set_console_variable_value(cvar=oidn_denoise_alpha_cvar, value=0)

        # disable the in-frame path-tracer progress bar (the red line at the bottom) so it can never bake into the
        # captured image if we read before it clears on its own (Shouldn't happen with the extra settle frames, but just in case)
        progress_display_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.ProgressDisplay")
        game.unreal_service.set_console_variable_value(cvar=progress_display_cvar, value=0)

        # spawn camera sensor and get the final_tone_curve_hdr component (same as examples/render_image)
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # match the viewport (FOV, post-processing settings, etc); --width/--height override the capture resolution
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc, widths=args.width, heights=args.height)

        # enable ray tracing for this component and turn on the PathTracing show flag; SetShowFlagSettings() applies immediately
        final_tone_curve_hdr_component.bUseRayTracingIfEnabled = True
        final_tone_curve_hdr_component.SetShowFlagSettings(InShowFlagSettings=[{"ShowFlagName": "PathTracing", "Enabled": True}])

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame():
        pass

    # accumulate one path-traced sample per pixel per frame; r.PathTracing.SamplesPerPixel caps this at args.samples.
    # It is possible to read_pixels after every sample, accumulate, and manually invalidate the view.
    # However that way you cannot easily use the built-in denoiser, s use the manual accumulation only in
    # cases when the scene has movement/animation or you want to do custom motion blur and similar effects.
    for i in range(0, args.samples):
        spear.log(f"Info: Path tracing sample {i + 1}/{args.samples} ...")
        instance.step()

    # the sample count is capped now, but the denoiser's final pass (and its resolve into the scene-capture render
    # target) lags a frame behind the last sample;
    instance.step(num_frames=_NUM_EXTRA_SETTLE_FRAMES)

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        data_bundle = final_tone_curve_hdr_component.read_pixels()

    # save image (cv2 writes the native BGR, dropping alpha)
    spear.log("Info: Saving image: ", args.out)
    cv2.imwrite(args.out, data_bundle["arrays"]["data"][:,:,[0,1,2]])

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Info: Done.")
