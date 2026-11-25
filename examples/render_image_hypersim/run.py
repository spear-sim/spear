#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import colorsys
import cv2
import numpy as np
import os
import pandas as pd
import PIL.Image
import re
import shutil
import spear


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
        "name": "final_tone_curve_hdr_",
        "spatial_supersampling_factor": 1,
        "read_primary_pixel_data": True,
        "user_scene_texture_names": ["MaterialAO", "Metallic", "Roughness", "SpecularForLighting", "SpDepthMeters", "SpWorldPosition", "WorldNormal"],
        "visualize_funcs":
        {
            "data":                lambda data : data[:,:,[2,1,0]], # BGRA to RGB
            "MaterialAO":          lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0),
            "Metallic":            lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0),
            "Roughness":           lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0),
            "SpecularForLighting": lambda data : np.clip(data[:,:,[0,0,0]], 0.0, 1.0),
            "SpDepthMeters":       lambda data : np.clip((data[:,:,0] - np.min(data[:,:,0])) / np.minimum((np.max(data[:,:,0]) - np.min(data[:,:,0])), 7.5), 0.0, 1.0), # normalize to max depth of 7.5 meters
            "SpWorldPosition":     lambda data : np.clip(data[:,:,[0,1,2]]/100.0, 0.0, 1.0),
            "WorldNormal":         lambda data : np.clip((1.0 + (data[:,:,[0,1,2]] @ R_camera_from_world.T @ M_hypersim_camera_from_unreal_camera.T))/2.0, 0.0, 1.0) # rotate world-space normals into Hypersim camera space
        }
    },
    {
        "name": "diffuse_and_specular_",
        "spatial_supersampling_factor": 2,
        "read_primary_pixel_data": False,
        "user_scene_texture_names": ["PostProcessInput2"],
        "visualize_funcs":
        {
            "PostProcessInput2": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
        }
    },
    {
        "name": "diffuse_only_",
        "spatial_supersampling_factor": 2,
        "read_primary_pixel_data": False,
        "user_scene_texture_names": ["DiffuseColor", "PostProcessInput2"],
        "visualize_funcs":
        {
            "DiffuseColor":      lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0),
            "PostProcessInput2": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
        }
    },
    {
        "name": "lighting_only_",
        "spatial_supersampling_factor": 2,
        "read_primary_pixel_data": False,
        "user_scene_texture_names": ["DiffuseColor", "PostProcessInput2"],
        "visualize_funcs":
        {
            "DiffuseColor":      lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0),
            "PostProcessInput2": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
        }
    },
    {
        "name": "sp_object_ids_uint8_",
        "spatial_supersampling_factor": 1,
        "read_primary_pixel_data": True,
        "user_scene_texture_names": [],
        "visualize_funcs":
        {
            "data": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
        }
    },
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

        # force high-res textures for captured images
        game.console_service.set_cvar(name="r.Streaming.FramesForFullUpdate", value=0)
        game.console_service.set_cvar(name="r.Streaming.FullyLoadUsedTextures", value=1)

        # # force Cinematic rendering quality levels for captured images (except use Epic quality for GI otherwise Lumen's internally allocated textures might be too big depending on viewport size)
        # previous_quality_levels = game.cdo.USpScalability.GetQualityLevels()
        # cinematic_quality_levels = game.cdo.USpQualityLevelsUtils.QualityLevelsFromSingleQualityLevel(QualityLevel="Cinematic")
        # cinematic_quality_levels_epic_gi = game.cdo.USpQualityLevelsUtils.SetGlobalIlluminationQuality(QualityLevels=cinematic_quality_levels, QualityLevel="Epic", as_dict=True)["QualityLevels"]
        # game.rendering_quality_service.log_quality_levels(label="Previous quality levels:", quality_levels=previous_quality_levels)
        # game.rendering_quality_service.log_quality_levels(label="New quality levels:", quality_levels=cinematic_quality_levels_epic_gi)
        # game.cdo.USpScalability.SetQualityLevels(QualityLevels=cinematic_quality_levels_epic_gi)

        # # we set cinematic_quality_settings=False because we already do our own customization above
        # game.rendering_quality_service.set_default_mrq_rendering_quality(cinematic_quality_settings=False)

        # alternatively we can just set default MRQ quality if we know our viewport isn't too big
        game.rendering_quality_service.set_default_mrq_rendering_quality()

        # initialize segmentation service
        game.segmentation_service.initialize()

        # spawn camera sensor
        bp_camera_sensor_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_CameraSensor_UST.BP_CameraSensor_UST_C")
        bp_camera_sensor = game.unreal_service.spawn_actor(uclass=bp_camera_sensor_uclass)

        # get camera components

        final_tone_curve_hdr_component = None

        for component_desc in component_descs:
            component_desc["component"] = game.unreal_service.get_component_by_name(actor=bp_camera_sensor, component_name=f"DefaultSceneRoot.{component_desc['name']}", uclass="USpSceneCaptureComponent2D")

            # get final_tone_curve_hdr_component
            if component_desc["name"] == "final_tone_curve_hdr_":
                final_tone_curve_hdr_component = component_desc["component"]

        assert final_tone_curve_hdr_component is not None

        # configure components to match the viewport (width, height, FOV, post-processing settings, etc)

        viewport_desc = game.viewport_service.get_current_viewport_desc()
        R_world_from_camera = spear.math.to_numpy_matrix_from_spear_rotator(spear_rotator=viewport_desc["camera_rotation"], as_matrix=True)
        R_camera_from_world = R_world_from_camera.T.A
        w = width if width is not None else viewport_desc["viewport_size_x"]
        h = height if height is not None else viewport_desc["viewport_size_y"]
        components = [ desc["component"] for desc in component_descs ]
        widths = [ w*desc["spatial_supersampling_factor"] for desc in component_descs ]
        heights = [ h*desc["spatial_supersampling_factor"] for desc in component_descs ]

        game.viewport_service.align_camera_with_viewport(camera_sensor=bp_camera_sensor, camera_components=components, viewport_desc=viewport_desc, widths=widths, heights=heights)

        # need to call Initialize() after calling game.segmentation_service.initialize()
        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].UserSceneTextureNames = component_desc["user_scene_texture_names"]
            component_desc["component"].bReadPrimaryPixelData = component_desc["read_primary_pixel_data"]
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and inserting
    # an extra frame or two can fix occasional render-to-texture initialization issues, and we need to wait
    # in case the segmentation service is still loading its materials (advances a minimum of 4 frames)
    game.async_loading_service.wait_for_engine_idle()

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            data_bundle = component_desc["component"].read_pixels()
            component_desc["arrays"] = data_bundle["arrays"]

        # create component desc map for easier bookkeeping later
        component_desc_map = { desc["name"]: desc for desc in component_descs }

        object_ids_bgra_uint8_image = component_desc_map["sp_object_ids_uint8_"]["arrays"]["data"]
        proxy_id_image, proxy_id_descs = game.segmentation_service.get_segmentation_data(object_ids_bgra_uint8_image=object_ids_bgra_uint8_image)

    # get actor names and pointers
    actor_names = [ proxy_id_desc["ActorStableName"] for proxy_id_desc in proxy_id_descs ]
    actor_ptrs = [ proxy_id_desc["Actor"].ptr for proxy_id_desc in proxy_id_descs ]
    assert actor_names[0] == ""
    assert actor_ptrs[0] == 0

    # get material pointers
    material_ptrs = [ proxy_id_desc["NonProxyMaterial"].ptr for proxy_id_desc in proxy_id_descs ]
    assert material_ptrs[0] == 0

    # get unique actors
    unique_actor_ptrs, unique_actor_ptrs_index, unique_actor_ptrs_inverse = np.unique(actor_ptrs, return_index=True, return_inverse=True)
    unique_actor_names = [ actor_names[i] for i in unique_actor_ptrs_index ]

    # get per-unique-actor semantic IDs
    unique_actor_semantic_ids = np.array([ 0 if not re.search(r"Meshes/(\d+)_", name) else int(re.search(r"Meshes/(\d+)_", name).group(1)) for name in unique_actor_names ])
    semantic_id_image = unique_actor_semantic_ids[unique_actor_ptrs_inverse[proxy_id_image]]

    # get per-unique-actor semantic instance IDs
    unique_actor_is_semantic_instance = np.array([ any([ name.startswith(category) for category in semantic_instance_categories ]) for name in unique_actor_names ])
    num_semantic_instances = np.count_nonzero(unique_actor_is_semantic_instance) + 1
    unique_actor_semantic_instance_ids = np.zeros(len(unique_actor_ptrs), dtype=np.uint32)
    unique_actor_semantic_instance_ids[unique_actor_is_semantic_instance] = np.arange(1, num_semantic_instances)
    semantic_instance_id_image = unique_actor_semantic_instance_ids[unique_actor_ptrs_inverse[proxy_id_image]]

    # get unique materials
    unique_material_ptrs, unique_material_ptrs_inverse = np.unique(material_ptrs, return_inverse=True)
    num_materials = len(unique_material_ptrs)
    material_id_image = unique_material_ptrs_inverse[proxy_id_image]

    # get foreground actor
    assert foreground_actor_name in unique_actor_names
    foreground_actor_proxy_ids = np.where([ foreground_actor_name == actor_name for actor_name in actor_names])[0]
    spear.log("foreground_actor_proxy_ids: ", foreground_actor_proxy_ids)
    spear.log("foreground_actor_proxy_descs:")
    for proxy_id in foreground_actor_proxy_ids:
        spear.pprint(obj=proxy_id_descs[proxy_id], depth=1)

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
        for name, data in component_desc["arrays"].items():
            image_file = os.path.realpath(os.path.join(images_dir, f"{component_desc['name']}.{name}.png"))
            image = component_desc["visualize_funcs"][name](data=data)
            if image.dtype != np.uint8:
                image = (np.clip(image, 0.0, 1.0)*255.0).astype(np.uint8)
            if image.ndim == 3:
                image = image[:, :, [2,1,0]] # RGB -> BGR
            elif image.ndim == 2:
                image = cv2.applyColorMap(image, cv2.COLORMAP_VIRIDIS) # match matplotlib's default colormap for single-channel images
            spear.log("Saving image: ", image_file)
            cv2.imwrite(image_file, image)

    # foreground
    foreground = np.isin(proxy_id_image, foreground_actor_proxy_ids)*255
    image_file = os.path.realpath(os.path.join(images_dir, "_foreground.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, cv2.applyColorMap(foreground.astype(np.uint8), cv2.COLORMAP_VIRIDIS)) # match matplotlib's default colormap for single-channel images

    # semantic
    semantic = semantic_colors[semantic_id_image]
    image_file = os.path.realpath(os.path.join(images_dir, "_semantic.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, semantic[:,:,[2,1,0]]) # RGB -> BGR

    # semantic instance
    semantic_instance = semantic_instance_colors[semantic_instance_id_image]
    image_file = os.path.realpath(os.path.join(images_dir, "_semantic_instance.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, semantic_instance[:,:,[2,1,0]]) # RGB -> BGR

    # material
    material = material_colors[material_id_image]
    image_file = os.path.realpath(os.path.join(images_dir, "_material.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, material[:,:,[2,1,0]]) # RGB -> BGR

    # diffuse_reflectance
    gamma = 1.0/2.2
    spatial_supersampling_factor = component_desc_map["diffuse_only_"]["spatial_supersampling_factor"]
    diffuse_reflectance = component_desc_map["diffuse_only_"]["arrays"]["DiffuseColor"][:,:,[0,1,2]].astype(np.float32)
    diffuse_reflectance = np.power(np.maximum(diffuse_reflectance, 0), gamma)
    diffuse_reflectance = np.clip(diffuse_reflectance, 0.0, 1.0)
    diffuse_reflectance = PIL.Image.fromarray((diffuse_reflectance*255.0).astype(np.uint8))
    diffuse_reflectance = diffuse_reflectance.resize((diffuse_reflectance.width // spatial_supersampling_factor, diffuse_reflectance.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_reflectance = np.asarray(diffuse_reflectance)
    image_file = os.path.realpath(os.path.join(images_dir, "_diffuse_reflectance.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, diffuse_reflectance[:,:,[2,1,0]]) # RGB -> BGR

    # diffuse_illumination
    spatial_supersampling_factor = component_desc_map["lighting_only_"]["spatial_supersampling_factor"]
    mask = proxy_id_image != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    lighting_only_diffuse_color = component_desc_map["lighting_only_"]["arrays"]["DiffuseColor"][:,:,[0,1,2]].astype(np.float32)
    lighting_only_post_process_input_2 = component_desc_map["lighting_only_"]["arrays"]["PostProcessInput2"][:,:,[0,1,2]].astype(np.float32)
    diffuse_illumination = spear.rendering.tone_map_hypersim(image=lighting_only_post_process_input_2/lighting_only_diffuse_color, mask=mask)
    diffuse_illumination = np.clip(diffuse_illumination, 0.0, 1.0)
    diffuse_illumination = PIL.Image.fromarray((diffuse_illumination*255.0).astype(np.uint8))
    diffuse_illumination = diffuse_illumination.resize((diffuse_illumination.width // spatial_supersampling_factor, diffuse_illumination.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_illumination = np.asarray(diffuse_illumination)
    image_file = os.path.realpath(os.path.join(images_dir, "_diffuse_illumination.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, diffuse_illumination[:,:,[2,1,0]]) # RGB -> BGR

    # diffuse_and_specular
    spatial_supersampling_factor = component_desc_map["diffuse_and_specular_"]["spatial_supersampling_factor"]
    mask = proxy_id_image != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    diffuse_and_specular = component_desc_map["diffuse_and_specular_"]["arrays"]["PostProcessInput2"][:,:,[0,1,2]].astype(np.float32)
    tone_map_result_dict = spear.rendering.tone_map_hypersim(image=diffuse_and_specular, mask=mask, as_dict=True) # get scale and gamma from tone-mapping diffuse_and_specular and use repeatedly
    diffuse_and_specular, tone_map_scale, tone_map_gamma = tone_map_result_dict["image_tone_map"], tone_map_result_dict["scale"], tone_map_result_dict["gamma"]
    diffuse_and_specular = np.clip(diffuse_and_specular, 0.0, 1.0)
    diffuse_and_specular = PIL.Image.fromarray((diffuse_and_specular*255.0).astype(np.uint8))
    diffuse_and_specular = diffuse_and_specular.resize((diffuse_and_specular.width // spatial_supersampling_factor, diffuse_and_specular.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_and_specular = np.asarray(diffuse_and_specular)
    image_file = os.path.realpath(os.path.join(images_dir, "_diffuse_and_specular.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, diffuse_and_specular[:,:,[2,1,0]]) # RGB -> BGR

    # diffuse_only
    spatial_supersampling_factor = component_desc_map["diffuse_only_"]["spatial_supersampling_factor"]
    diffuse_only = component_desc_map["diffuse_only_"]["arrays"]["PostProcessInput2"][:,:,[0,1,2]].astype(np.float32)
    diffuse_only = np.power(np.maximum(tone_map_scale*diffuse_only, 0), tone_map_gamma) # use previous scale and gamma
    diffuse_only = np.clip(diffuse_only, 0.0, 1.0)
    diffuse_only = PIL.Image.fromarray((diffuse_only*255.0).astype(np.uint8))
    diffuse_only = diffuse_only.resize((diffuse_only.width // spatial_supersampling_factor, diffuse_only.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_only = np.asarray(diffuse_only)
    image_file = os.path.realpath(os.path.join(images_dir, "_diffuse_only.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, diffuse_only[:,:,[2,1,0]]) # RGB -> BGR

    # residual
    assert component_desc_map["diffuse_and_specular_"]["spatial_supersampling_factor"] == component_desc_map["diffuse_only_"]["spatial_supersampling_factor"]
    spatial_supersampling_factor = component_desc_map["diffuse_and_specular_"]["spatial_supersampling_factor"]
    diffuse_and_specular = component_desc_map["diffuse_and_specular_"]["arrays"]["PostProcessInput2"][:,:,[0,1,2]].astype(np.float32)
    diffuse_only = component_desc_map["diffuse_only_"]["arrays"]["PostProcessInput2"][:,:,[0,1,2]].astype(np.float32)
    residual = diffuse_and_specular - diffuse_only
    residual = np.power(np.maximum(tone_map_scale*residual, 0.0), tone_map_gamma) # use previous scale and gamma
    residual = np.clip(residual, 0.0, 1.0)
    residual = PIL.Image.fromarray((residual*255.0).astype(np.uint8))
    residual = residual.resize((residual.width // spatial_supersampling_factor, residual.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    residual = np.asarray(residual)
    image_file = os.path.realpath(os.path.join(images_dir, "_residual.png"))
    spear.log("Saving image: ", image_file)
    cv2.imwrite(image_file, residual[:,:,[2,1,0]]) # RGB -> BGR

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

        # restore rendering quality
        game.rendering_quality_service.restore_rendering_quality()

        # # restore quality levels from our own customization
        # game.rendering_quality_service.log_quality_levels(label="Restoring quality levels:", quality_levels=previous_quality_levels)
        # game.cdo.USpScalability.SetQualityLevels(QualityLevels=previous_quality_levels)

    spear.log("Done.")
