#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import colorsys
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import PIL
import pprint
import re
import shutil
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--teaser", action="store_true")
args = parser.parse_args()

if args.teaser:
    width = 1920
    height = 1080
else:
    width = None
    height = None

np.random.seed(0)

R_camera_from_world = None
M_hypersim_camera_from_unreal_camera = np.array([[  0, 1, 0],
                                                 [  0, 0, 1],
                                                 [ -1, 0, 0]], dtype=np.float32)

# foreground_actor_name = "Meshes/05_chair/LivingRoom_Chair_02"
# foreground_actor_name = "Meshes/40_otherprop/Vase_03"
foreground_actor_name = "Meshes/35_lamp/Ceiling_LivingRoom_Lights"

semantic_instance_categories = [
    # "Meshes/01_wall",
    # "Meshes/02_floor",
    "Meshes/03_cabinet",
    "Meshes/05_chair",
    "Meshes/06_sofa",
    "Meshes/07_table",
    "Meshes/08_door",
    "Meshes/09_window",
    "Meshes/10_bookshelf",
    "Meshes/11_picture",
    "Meshes/16_curtain",
    "Meshes/18_pillow",
    "Meshes/19_mirror",
    "Meshes/20_floormat",
    # "Meshes/22_ceiling",
    "Meshes/34_sink",
    "Meshes/35_lamp",
    "Meshes/38_otherstructure",
    "Meshes/40_otherprop"]

component_descs = \
[
    {
        "name": "diffuse_and_specular_post_process_input_2",
        "long_name": "DefaultSceneRoot.diffuse_and_specular_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "diffuse_color",
        "long_name": "DefaultSceneRoot.diffuse_color_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "diffuse_only_post_process_input_2",
        "long_name": "DefaultSceneRoot.diffuse_only_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "lighting_only_diffuse_color",
        "long_name": "DefaultSceneRoot.lighting_only_diffuse_color_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "lighting_only_post_process_input_2",
        "long_name": "DefaultSceneRoot.lighting_only_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "material_ao",
        "long_name": "DefaultSceneRoot.material_ao_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0)
    },
    {
        "name": "metallic",
        "long_name": "DefaultSceneRoot.metallic_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0)
    },
    {
        "name": "roughness",
        "long_name": "DefaultSceneRoot.roughness_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0)
    },
    {
        "name": "sp_camera_normal",
        "long_name": "DefaultSceneRoot.world_normal_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip((1.0 + (data[:,:,[0,1,2]] @ R_camera_from_world.T @ M_hypersim_camera_from_unreal_camera.T))/2.0, 0.0, 1.0)
    },
    {
        "name": "sp_depth_meters",
        "long_name": "DefaultSceneRoot.sp_depth_meters_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip((data[:,:,0] - np.min(data[:,:,0])) / np.minimum((np.max(data[:,:,0]) - np.min(data[:,:,0])), 7.5), 0.0, 1.0) # normalize to max depth of 7.5 meters
    },
    {
        "name": "sp_object_ids_float16",
        "long_name": "DefaultSceneRoot.sp_object_ids_float16_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "sp_object_ids_uint8",
        "long_name": "DefaultSceneRoot.sp_object_ids_uint8_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "sp_unlit_float16",
        "long_name": "DefaultSceneRoot.sp_unlit_float16_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "sp_unlit_uint8",
        "long_name": "DefaultSceneRoot.sp_unlit_uint8_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "sp_world_position",
        "long_name": "DefaultSceneRoot.sp_world_position_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]]/100.0, 0.0, 1.0)
    },
    {
        "name": "specular_for_lighting",
        "long_name": "DefaultSceneRoot.specular_for_lighting_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0)
    }
]


if __name__ == "__main__":

    # create output dir
    images_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "images"))
    if os.path.exists(images_dir):
        spear.log("Directory exists, removing: ", images_dir)
        shutil.rmtree(images_dir, ignore_errors=True)
    os.makedirs(images_dir, exist_ok=True)

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # initialize actors and components
    with instance.begin_frame():

        # initialize segmentation service
        game.segmentation_service.initialize()

        # spawn camera sensor
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor.BP_CameraSensor_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # get camera components

        final_tone_curve_hdr_component = None

        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=component_desc["long_name"], uclass="USpSceneCaptureComponent2D")

            # get final_tone_curve_hdr_component
            if component_desc["name"] == "final_tone_curve_hdr":
                final_tone_curve_hdr_component = component_desc["component"]

        assert final_tone_curve_hdr_component is not None

        # configure components to match the viewport (width, height, FOV, post-processing settings, etc)

        viewport_info = game.rendering_service.get_current_viewport_info()

        R_world_from_camera = spear.to_numpy_matrix_from_rotator(rotator=viewport_info["camera_rotation"], as_matrix=True)
        R_camera_from_world = R_world_from_camera.T.A

        w = width if width is not None else viewport_info["viewport_size_x"]
        h = height if height is not None else viewport_info["viewport_size_y"]
        components = [ desc["component"] for desc in component_descs ]
        widths = [ w*desc["spatial_supersampling_factor"] for desc in component_descs ]
        heights = [ h*desc["spatial_supersampling_factor"] for desc in component_descs ]

        game.rendering_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_info=viewport_info, widths=widths, heights=heights)

        # need to call Initialize() after calling game.segmentation_service.initialize()
        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame or two can fix occasional render-to-texture initialization issues
    instance.step(num_frames=2)

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            data_bundle = component_desc["component"].read_pixels()
            component_desc["data"] = data_bundle["arrays"]["data"]

        # create component desc map for easier bookkeeping later
        component_desc_map = { desc["name"]: desc for desc in component_descs }

        object_ids_bgra_uint8_image = component_desc_map["sp_object_ids_uint8"]["data"]
        proxy_id_image, proxy_id_descs = game.segmentation_service.get_segmentation_data(object_ids_bgra_uint8_image=object_ids_bgra_uint8_image)

    # get actor names and handles
    actor_names = [ proxy_id_desc["actorName"] for proxy_id_desc in proxy_id_descs ]
    actor_handles = [ proxy_id_desc["actor"] for proxy_id_desc in proxy_id_descs ]
    assert actor_names[0] == ""
    assert actor_handles[0] == 0

    # get material handles
    material_handles = [ proxy_id_desc["material"] for proxy_id_desc in proxy_id_descs ]
    assert material_handles[0] == 0

    # get unique actors
    unique_actor_handles, unique_actor_handles_index, unique_actor_handles_inverse = np.unique(actor_handles, return_index=True, return_inverse=True)
    unique_actor_names = [ actor_names[i] for i in unique_actor_handles_index ]

    # get per-unique-actor semantic IDs
    unique_actor_semantic_ids = np.array([ 0 if not re.search(r"Meshes/(\d+)_", name) else int(re.search(r"Meshes/(\d+)_", name).group(1)) for name in unique_actor_names ])
    semantic_id_image = unique_actor_semantic_ids[unique_actor_handles_inverse[proxy_id_image]]

    # get per-unique-actor semantic instance IDs
    unique_actor_is_semantic_instance = np.array([ any([ name.startswith(category) for category in semantic_instance_categories ]) for name in unique_actor_names ])
    num_semantic_instances = np.count_nonzero(unique_actor_is_semantic_instance) + 1
    unique_actor_semantic_instance_ids = np.zeros(len(unique_actor_handles), dtype=np.uint32)
    unique_actor_semantic_instance_ids[unique_actor_is_semantic_instance] = np.arange(1, num_semantic_instances)
    semantic_instance_id_image = unique_actor_semantic_instance_ids[unique_actor_handles_inverse[proxy_id_image]]

    # get unique materials
    unique_material_handles, unique_material_handles_inverse = np.unique(material_handles, return_inverse=True)
    num_materials = len(unique_material_handles)
    material_id_image = unique_material_handles_inverse[proxy_id_image]

    # get foreground actor
    assert foreground_actor_name in unique_actor_names
    foreground_actor_proxy_ids = np.where([ foreground_actor_name == actor_name for actor_name in actor_names])[0]
    spear.log("foreground_actor_proxy_ids: ", foreground_actor_proxy_ids)
    spear.log("foreground_actor_proxy_descs:")
    for proxy_id in foreground_actor_proxy_ids:
        pprint.pprint(proxy_id_descs[proxy_id], depth=1)

    # get colors for semantic IDs from CSV
    df = pd.read_csv(os.path.realpath(os.path.join(os.path.dirname(__file__), "semantic_label_descs.csv")), comment="#")
    df.columns = df.columns.str.strip()
    semantic_colors = np.vstack([[0,0,0], df[["semantic_color_r", "semantic_color_g", "semantic_color_b"]].to_numpy()]).astype(dtype=np.uint8)

    # generate colors for semantic instance IDs
    semantic_instance_colors = np.zeros((num_semantic_instances, 3), dtype=np.uint8)
    for i in range(1, num_semantic_instances):
        semantic_instance_colors[i] = np.round(np.array(colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0))*255.0).astype(np.uint8)

    # generate colors for unique materials
    material_colors = np.zeros((num_materials, 3), dtype=np.uint8)
    for i in range(1, num_materials):
        material_colors[i] = np.round(np.array(colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0))*255.0).astype(np.uint8)

    # save an image for each component using the component's visualizer function
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, f"{component_desc['name']}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)

    # foreground
    foreground = np.isin(proxy_id_image, foreground_actor_proxy_ids)*255
    image_file = os.path.realpath(os.path.join(images_dir, "foreground.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, foreground)

    # semantic
    semantic_ids = semantic_id_image
    semantic = semantic_colors[semantic_ids]
    image_file = os.path.realpath(os.path.join(images_dir, "semantic.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, semantic)

    # semantic instance
    semantic_instance_ids = semantic_instance_id_image
    semantic_instance = semantic_instance_colors[semantic_instance_ids]
    image_file = os.path.realpath(os.path.join(images_dir, "semantic_instance.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, semantic_instance)

    # material
    material_ids = material_id_image
    material = material_colors[material_id_image]
    image_file = os.path.realpath(os.path.join(images_dir, "material.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, material)

    # diffuse_reflectance
    gamma = 1.0/2.2
    spatial_supersampling_factor = component_desc_map["diffuse_color"]["spatial_supersampling_factor"]
    diffuse_reflectance = component_desc_map["diffuse_color"]["data"]
    diffuse_reflectance = np.power(np.maximum(diffuse_reflectance, 0), gamma)
    diffuse_reflectance = np.clip(diffuse_reflectance, 0.0, 1.0)
    diffuse_reflectance = PIL.Image.fromarray((diffuse_reflectance*255.0).astype(np.uint8))
    diffuse_reflectance = diffuse_reflectance.resize((diffuse_reflectance.width // spatial_supersampling_factor, diffuse_reflectance.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_reflectance = np.asarray(diffuse_reflectance)
    image_file = os.path.realpath(os.path.join(images_dir, "diffuse_reflectance.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, diffuse_reflectance)

    # diffuse_illumination
    spatial_supersampling_factor = component_desc_map["lighting_only_diffuse_color"]["spatial_supersampling_factor"]
    mask = proxy_id_image != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    lighting_only_diffuse_color = component_desc_map["lighting_only_diffuse_color"]["data"][:,:,[0,1,2]].astype(np.float32)
    lighting_only_post_process_input_2 = component_desc_map["lighting_only_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    diffuse_illumination = spear.rendering.tone_map_hypersim(image=lighting_only_post_process_input_2/lighting_only_diffuse_color, mask=mask)
    diffuse_illumination = np.clip(diffuse_illumination, 0.0, 1.0)
    diffuse_illumination = PIL.Image.fromarray((diffuse_illumination*255.0).astype(np.uint8))
    diffuse_illumination = diffuse_illumination.resize((diffuse_illumination.width // spatial_supersampling_factor, diffuse_illumination.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_illumination = np.asarray(diffuse_illumination)
    image_file = os.path.realpath(os.path.join(images_dir, "diffuse_illumination.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, diffuse_illumination)

    # diffuse_and_specular
    spatial_supersampling_factor = component_desc_map["diffuse_color"]["spatial_supersampling_factor"]
    mask = proxy_id_image != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    diffuse_and_specular = component_desc_map["diffuse_and_specular_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    tone_map_result_dict = spear.rendering.tone_map_hypersim(image=diffuse_and_specular, mask=mask, as_dict=True) # get scale and gamma from tone-mapping diffuse_and_specular
    diffuse_and_specular, scale, gamma = tone_map_result_dict["image_tone_map"], tone_map_result_dict["scale"], tone_map_result_dict["gamma"]
    diffuse_and_specular = np.clip(diffuse_and_specular, 0.0, 1.0)
    diffuse_and_specular = PIL.Image.fromarray((diffuse_and_specular*255.0).astype(np.uint8))
    diffuse_and_specular = diffuse_and_specular.resize((diffuse_and_specular.width // spatial_supersampling_factor, diffuse_and_specular.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_and_specular = np.asarray(diffuse_and_specular)
    image_file = os.path.realpath(os.path.join(images_dir, "diffuse_and_specular.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, diffuse_and_specular)

    # diffuse_only
    spatial_supersampling_factor = component_desc_map["diffuse_color"]["spatial_supersampling_factor"]
    diffuse_only = component_desc_map["diffuse_only_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    diffuse_only = np.power(np.maximum(scale*diffuse_only, 0), gamma) # use previous scale and gamma
    diffuse_only = np.clip(diffuse_only, 0.0, 1.0)
    diffuse_only = PIL.Image.fromarray((diffuse_only*255.0).astype(np.uint8))
    diffuse_only = diffuse_only.resize((diffuse_only.width // spatial_supersampling_factor, diffuse_only.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_only = np.asarray(diffuse_only)
    image_file = os.path.realpath(os.path.join(images_dir, "diffuse_only.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, diffuse_only)

    # residual
    spatial_supersampling_factor = component_desc_map["diffuse_color"]["spatial_supersampling_factor"]
    diffuse_and_specular = component_desc_map["diffuse_and_specular_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    diffuse_only = component_desc_map["diffuse_only_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    residual = diffuse_and_specular - diffuse_only
    residual = np.power(np.maximum(1.0*residual, 0.0), gamma) # use previous scale and gamma
    residual = np.clip(residual, 0.0, 1.0)
    residual = PIL.Image.fromarray((residual*255.0).astype(np.uint8))
    residual = residual.resize((residual.width // spatial_supersampling_factor, residual.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    residual = np.asarray(residual)
    image_file = os.path.realpath(os.path.join(images_dir, "residual.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, residual)

    # terminate actors and components
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            component_desc["component"].terminate_sp_funcs()
            component_desc["component"].Terminate()
        game.unreal_service.destroy_actor(actor=bp_camera_sensor)

        # terminate segmentation service
        game.segmentation_service.terminate()

    spear.log("Done.")
