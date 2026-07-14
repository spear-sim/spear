#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

# This example renders a path-traced image with a USpSceneCaptureComponent2D (no Movie Render Queue). It is the
# same flow as examples/render_image, except the scene capture is routed through Unreal's offline path tracer
# instead of the real-time renderer.
# r.RayTracing.Enable=1 is set in SpearSim's DefaultEngine.ini because it is a read-only CVar that must be set at startup.

import argparse
import cv2
import os
import spear

parser = argparse.ArgumentParser()
parser.add_argument("--teaser", action="store_true")
parser.add_argument("--num-frames", type=int, default=64)
parser.add_argument("--bounces", type=int, default=8) # More than ~12 bounces makes almost no difference in most scenes.
parser.add_argument("--denoiser", type=str, default=None) # e.g. "NNEDenoiser" or "OIDN"; the denoiser's plugin must be enabled in SpearSim.uproject. No denoising by default.
args = parser.parse_args()

if args.teaser:
    width = 1920
    height = 1080
else:
    width = None
    height = None


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # Configure the path tracer via runtime CVars (see the note at the top of this file for why we don't use
        # user_config.yaml). r.RayTracing.SceneCaptures=1 lets a scene-capture view use ray tracing at all, which is
        # required for the path tracer to run here. The path tracer accumulates one sample per pixel per rendered
        # frame and stops at r.PathTracing.SamplesPerPixel, so we set that cap equal to the number of frames we
        # render below (see the rendering loop).
        scene_captures_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.RayTracing.SceneCaptures")
        game.unreal_service.set_console_variable_value(cvar=scene_captures_cvar, value=1)
        samples_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.SamplesPerPixel")
        game.unreal_service.set_console_variable_value(cvar=samples_cvar, value=args.num_frames)
        bounces_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.MaxBounces")
        game.unreal_service.set_console_variable_value(cvar=bounces_cvar, value=args.bounces)

        # Disable the in-frame path-tracer progress bar (the red line at the bottom) so it can never bake into the
        # captured image if we happen to read before it clears on its own.
        progress_display_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.ProgressDisplay")
        game.unreal_service.set_console_variable_value(cvar=progress_display_cvar, value=0)

        # The engine applies the spatial denoiser to the converged image once accumulation reaches
        # r.PathTracing.SamplesPerPixel. Selecting a denoiser by name requires its plugin to be enabled in
        # SpearSim.uproject; SpearSim enables "NNEDenoiser" (the engine default) and "OIDN".
        denoiser_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.Denoiser")
        game.unreal_service.set_console_variable_value(cvar=denoiser_cvar, value=1 if args.denoiser is not None else 0)
        if args.denoiser is not None:
            denoiser_name_cvar = game.unreal_service.find_console_variable_by_name(console_variable_name="r.PathTracing.Denoiser.Name")
            game.unreal_service.set_console_variable_value(cvar=denoiser_name_cvar, value=args.denoiser)

        # spawn camera sensor and get the final_tone_curve_hdr component (same as examples/render_image)
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        final_tone_curve_hdr_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name="DefaultSceneRoot.final_tone_curve_hdr_", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=final_tone_curve_hdr_component, viewport_desc=viewport_desc, widths=width, heights=height)

        # route this scene-capture view through the path tracer: allow it to use ray tracing and turn on the
        # PathTracing show flag. SetShowFlagSettings() applies immediately.
        final_tone_curve_hdr_component.bUseRayTracingIfEnabled = True

        final_tone_curve_hdr_component.SetShowFlagSettings(InShowFlagSettings=[
            {"ShowFlagName": "PathTracing", "Enabled": True},
            # The chromatic aberration and lens distortion amplifies the noise and produces artifacts when we don't denoise.
            {"ShowFlagName": "CameraImperfections", "Enabled": False},
        ])

        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        final_tone_curve_hdr_component.Initialize()
        final_tone_curve_hdr_component.initialize_sp_funcs()

    with instance.end_frame():
        pass

    # The path tracer accumulates one additional sample per pixel per rendered frame, exactly like the editor's
    # path-tracing viewport, and stops once it reaches r.PathTracing.SamplesPerPixel (set to args.num_frames above).
    # Moving the camera or anything in the scene invalidates the accumulated samples and restarts from scratch, so we
    # simply render args.num_frames frames in a row without moving anything, which converges the image on the final
    # frame. The engine then applies the denoiser (if one was requested) to the converged result.
    spear.log("Path-traced rendering beginning...")

    for i in range(args.num_frames):
        spear.log(f"Rendering frame {i + 1}/{args.num_frames} ...")
        instance.step()

    # let the final converged frame (and the denoiser, if enabled) settle before we read it back
    spear.log("Path-traced rendering finished.")

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        data_bundle = final_tone_curve_hdr_component.read_pixels()

    # save image (cv2 writes the native BGR, dropping alpha)
    image_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "image.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, data_bundle["arrays"]["data"][:,:,[0,1,2]])

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        final_tone_curve_hdr_component.terminate_sp_funcs()
        final_tone_curve_hdr_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Done.")
