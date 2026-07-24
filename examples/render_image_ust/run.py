#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear


# Each UST camera sensor selectable via --blueprint: its blueprint asset, its capture passes (components)
# selectable via --component, and the UserSceneTexture "universe" available on every pass. Enabling a buffer
# name adds its post-process material instance to the pass and reads it back under the same key.
camera_sensor_descs = \
{
    "lumen":
    {
        "blueprint_name": "BP_CameraSensor_UST",
        "component_names":
        [
            "final_tone_curve_hdr_",
            "final_tone_curve_hdr_no_taa_",
            "lighting_only_"
        ],
        "user_scene_texture_names":
        [
            "CustomStencil",
            "DiffuseColor",
            "MaterialAO",
            "Metallic",
            "PostProcessInput2",
            "Roughness",
            "SceneDepth",
            "SpDepthMeters",
            "SpecularForLighting",
            "SpViewNormal",
            "SpWorldPosition",
            "WorldNormal"
        ]
    },
    "path_tracer":
    {
        "blueprint_name": "BP_CameraSensorPathTracer_UST",
        "component_names":
        [
            "final_tone_curve_hdr_",
            "lighting_only_"
        ],
        "user_scene_texture_names":
        [
            "PathTracingAlbedo",
            "PathTracingDenoisedRadiance",
            "PathTracingNormal",
            "PathTracingRadiance",
            "PathTracingVariance",
            "SceneDepth"
        ]
    }
}

parser = argparse.ArgumentParser()
parser.add_argument("--blueprint", default="lumen")
parser.add_argument("--component", default="final_tone_curve_hdr_")
parser.add_argument("--num-frames", type=int, default=64)
args = parser.parse_args()
assert args.blueprint in camera_sensor_descs

camera_sensor_desc = camera_sensor_descs[args.blueprint]
assert args.component in camera_sensor_desc["component_names"]

blueprint_name = camera_sensor_desc["blueprint_name"]
component_name = args.component
enabled_names = camera_sensor_desc["user_scene_texture_names"]


# Per-buffer image processing: turn a raw readback array into a cv2-ready (BGR, uint8) image. Each buffer has
# its own quirks, so this is the one place to tweak them: channel order (the uint8 beauty is BGRA and written
# as-is; the float16 UST buffers are RGBA, so we swap to BGR), value range (color and [0,1] scalar buffers are
# clipped, while depth/variance/position/id buffers have arbitrary ranges and are min/max-normalized), and
# encoding (normals are remapped from [-1,1]).

def normalize(image):
    image = image.astype(np.float32)
    return (image - np.min(image)) / (np.max(image) - np.min(image) + 1e-8)

def to_uint8(image):
    return np.clip(255.0*image, 0.0, 255.0).astype(np.uint8)

def process_data(image):
    # main capture: the uint8 beauty is already BGRA; a float pass color is RGBA, so swap to BGR
    if image.dtype == np.uint8:
        return image[:,:,[0,1,2]]
    else:
        return to_uint8(image[:,:,[2,1,0]])

process_fns = \
{
    "data":                        process_data,
    "CustomStencil":               lambda image: to_uint8(normalize(image[:,:,[0,0,0]])),
    "DiffuseColor":                lambda image: to_uint8(image[:,:,[2,1,0]]),
    "MaterialAO":                  lambda image: to_uint8(image[:,:,[0,0,0]]),
    "Metallic":                    lambda image: to_uint8(image[:,:,[0,0,0]]),
    "PostProcessInput2":           lambda image: to_uint8(image[:,:,[2,1,0]]),
    "Roughness":                   lambda image: to_uint8(image[:,:,[0,0,0]]),
    "SceneDepth":                  lambda image: to_uint8(normalize(image[:,:,[0,0,0]])),
    "SpDepthMeters":               lambda image: to_uint8(normalize(image[:,:,[0,0,0]])),
    "SpecularForLighting":         lambda image: to_uint8(image[:,:,[0,0,0]]),
    "SpViewNormal":                lambda image: to_uint8(0.5*image[:,:,[2,1,0]] + 0.5),
    "SpWorldPosition":             lambda image: to_uint8(normalize(image[:,:,[2,1,0]])),
    "WorldNormal":                 lambda image: to_uint8(0.5*image[:,:,[2,1,0]] + 0.5),
    "PathTracingAlbedo":           lambda image: to_uint8(image[:,:,[2,1,0]]),
    "PathTracingNormal":           lambda image: to_uint8(0.5*image[:,:,[2,1,0]] + 0.5),
    "PathTracingDenoisedRadiance": lambda image: to_uint8(image[:,:,[2,1,0]]),
    "PathTracingRadiance":         lambda image: to_uint8(image[:,:,[2,1,0]]),
    "PathTracingVariance":         lambda image: to_uint8(normalize(image[:,:,[0,0,0]]))
}


def save_image(image_file, name, image):
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, process_fns.get(name, process_data)(image))


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # spawn the camera sensor and initialize the selected component with the full UST universe enabled
    with instance.begin_frame():

        # force high-res textures for captured images
        game.console_service.set(name="r.Streaming.FullyLoadUsedTextures", value=1)

        game.console_service.set(name="r.RayTracing.Enable", value=1)
        game.console_service.set(name="r.RayTracing.SceneCaptures", value=1)

        game.console_service.set(name="r.PathTracing.SamplesPerPixel", value=args.num_frames)
        game.console_service.set(name="r.PathTracing.MaxBounces", value=8)
        game.console_service.set(name="r.PathTracing.Denoiser", value=1) # use the default denoiser

        # spawn camera sensor and get the selected component
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name=f"/SpContent/Blueprints/{blueprint_name}.{blueprint_name}_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)
        component = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=f"DefaultSceneRoot.{component_name}", uclass="USpSceneCaptureComponent2D")

        # configure the component to match the viewport (location, rotation, FOV, aspect ratio)
        viewport_desc = game.rendering_service.get_current_viewport_desc()
        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=component, viewport_desc=viewport_desc)

        # enable every UST buffer in the universe, then initialize (read_pixels() is registered during Initialize())
        component.UserSceneTextureNames = enabled_names
        component.Initialize()
        component.initialize_sp_funcs()

    with instance.end_frame(single_step=True):
        pass

    # inserting an extra frame or two can fix occasional render-to-texture initialization issues (advances a minimum of 3 frames)
    game.async_loading_service.wait_for_engine_idle()

    # The path tracer accumulates one sample per pixel per rendered frame and stops at r.PathTracing.SamplesPerPixel
    # (set to args.num_frames above). Nothing in the scene moves, so rendering num_frames frames in a row converges
    # the path-traced image; for the non-path-traced sensor these frames simply let temporal anti-aliasing settle.
    spear.log("Rendering ", args.num_frames, " frames...")
    for i in range(args.num_frames):
        instance.step(single_step=True)

    # read back the main capture ("data") plus every enabled UST buffer
    with instance.begin_frame():
        pass
    with instance.end_frame(single_step=True):
        arrays = component.read_pixels()["arrays"]

    # save each returned buffer as a separate image
    for name, image in arrays.items():
        image_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "images", f"{args.blueprint}.{args.component}{name}.png"))
        os.makedirs(os.path.dirname(image_file), exist_ok=True)
        save_image(image_file, name, image)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        component.terminate_sp_funcs()
        component.Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

    instance.close()

    spear.log("Done.")
