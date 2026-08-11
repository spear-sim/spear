#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import shutil
import spear

capture_component_name = "final_tone_curve_hdr_"

user_scene_texture_names = [
    "PathTracingAlbedo",
    "PathTracingDenoisedRadiance",
    "PathTracingNormal",
    "PathTracingRadiance",
    "PathTracingVariance",
    "SceneDepth"
]

parser = argparse.ArgumentParser()
parser.add_argument("--denoiser", default="")
parser.add_argument("--num-bounces", type=int, default=8)
parser.add_argument("--num-frames", type=int, default=64)
parser.add_argument("--num-warmup-frames", type=int, default=4)
parser.add_argument("--filter-width", type=float, default=3.0)
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


def normalize(image):
    image = image.astype(np.float32)
    return (image - np.min(image)) / (np.max(image) - np.min(image) + 1e-8)

def as_uint8(image):
    if image.dtype == np.uint8:
        return image[:,:,[0,1,2]]
    else:
        return np.clip(255.0*image[:,:,[0,1,2]], 0.0, 255.0).astype(np.uint8)

process_fns = {
    "data":                        as_uint8,
    "PathTracingAlbedo":           lambda image : as_uint8(image[:,:,[2,1,0]]),
    "PathTracingDenoisedRadiance": lambda image : as_uint8(image[:,:,[2,1,0]]),
    "PathTracingNormal":           lambda image : as_uint8(0.5*image[:,:,[2,1,0]] + 0.5),
    "PathTracingRadiance":         lambda image : as_uint8(image[:,:,[2,1,0]]),
    "PathTracingVariance":         lambda image : as_uint8(normalize(image[:,:,[0,0,0]])),
    "SceneDepth":                  lambda image : as_uint8(normalize(image[:,:,[0,0,0]]))
}


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
    if os.path.exists(images_dir):
        spear.log("Directory exists, removing: ", images_dir)
        shutil.rmtree(images_dir, ignore_errors=True)
    os.makedirs(images_dir, exist_ok=True)

    # initialize actors and components
    with instance.begin_frame():

        # force high-res textures for captured images
        game.console_service.set(name="r.Streaming.FullyLoadUsedTextures", value=1)

        # When path tracing begins, Unreal begins asynchronously building various Nanite data structures, and
        # as these data structures finish building, they internally reset the progress of the path tracer.
        # Unfortunately Unreal doesn't expose any way to check if these asynchronous build tasks are complete,
        # so we proceed with a heuristic approach that attempts to front-load the build tasks as much as
        # possible, and then we render a conservative number of warm-up frames to ensure the build tasks are
        # complete before we start rendering a final image.

        game.console_service.set(name="r.RayTracing.Nanite.ForceUpdateVisible", value=1)                     # force all Nanite builds to be scheduled on a single frame
        game.console_service.set(name="r.RayTracing.Nanite.MaxBuiltPrimitivesPerFrame", value=256*1024*1024) # set the max number of built Nanite primitives per frame to be very large (default is 8*1024*1024)
        game.console_service.set(name="r.RayTracing.Nanite.MaxStagingBufferSizeMB", value=2048)              # set the size of the Nanite staging buffer to be large (default is 1024)

        # configure the path tracer via console variables

        game.console_service.set(name="r.RayTracing.Enable", value=1)
        game.console_service.set(name="r.RayTracing.SceneCaptures", value=1)
        game.console_service.set(name="r.PathTracing.ProgressDisplay", value=0)

        game.console_service.set(name="r.PathTracing.SamplesPerPixel", value=args.num_frames)
        game.console_service.set(name="r.PathTracing.MaxBounces", value=args.num_bounces)
        game.console_service.set(name="r.PathTracing.Denoiser", value=denoiser)
        game.console_service.set(name="r.PathTracing.Denoiser.Name", value=denoiser_name)

        # Filter Width controls how much primary rays are jittered, so it can be used to effectively
        # turn off anti-aliasing for primary rays.
        game.console_service.set(name="r.PathTracing.FilterWidth", value=args.filter_width)

        # spawn camera sensor and get the final_tone_curve_hdr component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name=f"/SpContent/Blueprints/BP_CameraSensorPathTracer_UST.BP_CameraSensorPathTracer_UST_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        capture_component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=f"DefaultSceneRoot.{capture_component_name}", uclass="USpSceneCaptureComponent2D")

        # configure the final_tone_curve_hdr component to match the viewport (width, height, FOV, post-processing settings, etc)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=capture_component, viewport_desc=viewport_desc, widths=width, heights=height)

        # enable all the available UserSceneTexture buffers on our capture component; need to call
        # initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        capture_component.UserSceneTextureNames = user_scene_texture_names
        capture_component.Initialize()
        capture_component.initialize_sp_funcs()

        sp_scene_view_state_interface = game.get_unreal_object(uclass="USpSceneViewStateInterface")

    with instance.end_frame(single_step=True):
        pass
    # inserting an extra frame or two can fix occasional render-to-texture initialization issues (advances a minimum of 3 frames)
    game.async_loading_service.wait_for_engine_idle()

    spear.log("Rendering warm-up frames...")

    # advance an extra few frames to give Nanite a chance to finish building its data structures
    instance.step(num_frames=args.num_warmup_frames)

    # force Nanite to stop building its data structures so it can't invalidate the path tracer any more
    with instance.begin_frame():
        game.console_service.set(name="r.RayTracing.Nanite.Update", value=0)
    with instance.end_frame():
        pass

    spear.log("Finished rendering warm-up frames.")

    # The path tracer accumulates one sample per pixel per rendered frame, exactly like the editor's
    # path-tracing viewport, and stops once it reaches r.PathTracing.SamplesPerPixel (set to args.num_frames
    # above). Moving the camera or anything in the scene invalidates the accumulated samples and restarts
    # from scratch. We poll the internal path tracer accumulation counter to ensure that the image has been
    # fully rendered. This way we avoid potential loading issues. The engine then applies the denoiser (if
    # one was requested) to the converged result.

    spear.log("Path-traced rendering beginning...")

    for i in range(args.num_frames):

        with instance.begin_frame():
            # Explicitly reset the path tracer's accumulated samples on the first frame. Nothing moves
            # in this example, but frames rendered above (e.g. during wait_for_engine_idle()) already accumulated
            # samples against this component's persistent view state, so without this reset, sample_index would
            # start ahead of 0 below.
            if i == 0:
                capture_component.RequestPathTracerReset()

        with instance.end_frame(single_step=True):
            view_states = capture_component.GetViewStates()
            assert len(view_states) == 1
            view_state = view_states[0]
            sample_index = sp_scene_view_state_interface.GetPathTracingSampleIndex(ViewState=view_state)
            assert sample_index == i + 1

    spear.log("Path-traced rendering finished.")

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame(single_step=True):
        data_bundle = capture_component.read_pixels()

    for name, image in data_bundle["arrays"].items():
        image_file = os.path.realpath(os.path.join(images_dir, f"{name}.png"))
        spear.log("Saving image: ", image_file)
        cv2.imwrite(image_file, process_fns[name](image))

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        capture_component.terminate_sp_funcs()
        capture_component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Done.")
