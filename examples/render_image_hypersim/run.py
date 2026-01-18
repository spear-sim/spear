#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import colorsys
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import PIL
import re
import shutil
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--teaser", action="store_true")
args = parser.parse_args()

R_camera_from_world = None
M_hypersim_camera_from_unreal_camera = np.array([[  0, 1, 0],
                                                 [  0, 0, 1],
                                                 [ -1, 0, 0]], dtype=np.float32)

foreground_actor_name      = "Meshes/05_chair/LivingRoom_Chair_02"
foreground_actor_proxy_ids = None

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
    "Meshes/40_otherprop"
]

component_descs = \
[
    {
        "name": "final_tone_curve_hdr",
        "long_name": "DefaultSceneRoot.final_tone_curve_hdr_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : data[:,:,[2,1,0]] # BGRA to RGB
    },
    {
        "name": "material_ao",
        "long_name": "DefaultSceneRoot.material_ao_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "metallic",
        "long_name": "DefaultSceneRoot.metallic_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
    },
    {
        "name": "roughness",
        "long_name": "DefaultSceneRoot.roughness_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]], 0.0, 1.0)
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
        "name": "sp_world_position",
        "long_name": "DefaultSceneRoot.sp_world_position_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.clip(data[:,:,[0,1,2]]/100.0, 0.0, 1.0)
    },
    {
        "name": "object_ids",
        "long_name": "DefaultSceneRoot.object_ids_",
        "spatial_supersampling_factor": 1,
        "visualize_func": lambda data : np.isin(data.view(np.uint32).reshape(data.shape[:2]) & 0x00ffffff, foreground_actor_proxy_ids)*255 # reinterpret little-endian BGRA as uint32 and set A=0x00
    },
    {
        "name": "diffuse_color",
        "long_name": "DefaultSceneRoot.diffuse_color_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data, 0.0, 1.0)
    },
    {
        "name": "diffuse_only_post_process_input_2",
        "long_name": "DefaultSceneRoot.diffuse_only_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(2.0*data, 0.0, 1.0)
    },
    {
        "name": "diffuse_and_specular_post_process_input_2",
        "long_name": "DefaultSceneRoot.diffuse_and_specular_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(2.0*data, 0.0, 1.0)
    },
    {
        "name": "lighting_only_diffuse_color",
        "long_name": "DefaultSceneRoot.lighting_only_diffuse_color_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(data, 0.0, 1.0)
    },
    {
        "name": "lighting_only_post_process_input_2",
        "long_name": "DefaultSceneRoot.lighting_only_post_process_input_2_",
        "spatial_supersampling_factor": 2,
        "visualize_func": lambda data : np.clip(2.0*data, 0.0, 1.0)
    }
]


def tone_map_hypersim(img, mask, as_tuple=None):

    img = img.astype(np.float64)

    gamma                             = 1.0/2.2   # standard gamma correction exponent
    inv_gamma                         = 1.0/gamma
    percentile                        = 90        # we want this percentile brightness value in the unmodified image...
    brightness_nth_percentile_desired = 0.8       # ...to be this bright after scaling

    spear.log("np.min(img): ", np.min(img))
    spear.log("np.max(img): ", np.max(img))
    spear.log("np.count_nonzero(mask): ", np.count_nonzero(mask))

    if np.count_nonzero(mask) == 0:
        scale = 1.0 # if there are no valid pixels, then set scale to 1.0
    else:
        brightness      = 0.3*img[:,:,0] + 0.59*img[:,:,1] + 0.11*img[:,:,2] # "CCIR601 YIQ" method for computing brightness
        brightness_mask = brightness[mask]

        eps                               = 0.0001 # if the nth percentile brightness value in the unmodified image is less than this, set the scale to 0.0 to avoid divide-by-zero
        brightness_nth_percentile_current = np.percentile(brightness_mask, percentile)
        spear.log("brightness_nth_percentile_current: ", brightness_nth_percentile_current)

        if brightness_nth_percentile_current < eps:
            scale = 0.0
        else:

            # Snavely uses the following expression in the code at https://github.com/snavely/pbrs_tonemapper/blob/master/tonemap_rgbe.py:
            # scale = np.exp(np.log(brightness_nth_percentile_desired)*inv_gamma - np.log(brightness_nth_percentile_current))
            #
            # Our expression below is equivalent, but is more intuitive, because it follows more directly from the expression:
            # (scale*brightness_nth_percentile_current)^gamma = brightness_nth_percentile_desired

            scale = np.power(brightness_nth_percentile_desired, inv_gamma) / brightness_nth_percentile_current

    spear.log("scale: ", scale)

    img_tone_map = np.power(np.maximum(scale*img, 0.0), gamma)

    spear.log("np.min(img_tone_map):", np.min(img_tone_map))
    spear.log("np.max(img_tone_map):", np.max(img_tone_map))

    if as_tuple is None:
        return img_tone_map
    else:
        assert as_tuple
        return (img_tone_map, scale, gamma)


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

        # need to spawn an ASpObjectIdsProxyComponentManager to get object IDs
        sp_object_ids_proxy_component_manager = game.unreal_service.spawn_actor(uclass="ASpObjectIdsProxyComponentManager")
        sp_object_ids_proxy_component_manager.Initialize()
        component_and_material_descs = sp_object_ids_proxy_component_manager.GetComponentAndMaterialDescs()

        # force refreshing all object IDs to make sure that a different set of uint8 IDs will be rendered correctly
        # for i in range(100):
        #     sp_object_ids_proxy_component_manager.Update()

        # find actors and components
        actors = game.unreal_service.find_actors_as_dict()
        semantic_instances = { name: actor for name, actor in actors.items() if any([ name.startswith(c) for c in semantic_instance_categories ]) }
        semantic_instance_uobjects = np.array([-1] + [ semantic_instances[k].uobject for k in sorted(semantic_instances.keys()) ])

        # get proxy ID maps
        proxy_id_to_component_map = { desc["id"]: game.get_unreal_object(uobject=desc["component"]) for desc in component_and_material_descs }
        proxy_id_to_actor_map = { proxy_id: component.GetOwner() for proxy_id, component in proxy_id_to_component_map.items() }
        proxy_id_to_semantic_instance_id_map = { proxy_id: int(np.where(actor.uobject == semantic_instance_uobjects)[0][0]) for proxy_id, actor in proxy_id_to_actor_map.items() if len(np.where(actor.uobject == semantic_instance_uobjects)[0]) > 0 }
        proxy_id_to_actor_name_map = { proxy_id: game.unreal_service.get_stable_name_for_actor(actor=actor) for proxy_id, actor in proxy_id_to_actor_map.items() if game.unreal_service.has_stable_name(actor=actor) }
        proxy_id_to_semantic_id_map = { proxy_id: int(re.search(r"Meshes/(\d+)_", stable_name).group(1)) for proxy_id, stable_name in proxy_id_to_actor_name_map.items() if re.search(r"Meshes/(\d+)_", stable_name) is not None }

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
        
        engine = game.engine_globals_service.get_engine()
        game_viewport_client = engine.GameViewport.get()

        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
        player_camera_manager = player_controller.PlayerCameraManager.get()
        view_target_pov = player_camera_manager.ViewTarget.POV.get()
        R_world_from_camera = spear.to_numpy_matrix_from_rotator(rotator=view_target_pov["rotation"], as_matrix=True)
        R_camera_from_world = R_world_from_camera.T.A

        post_process_volume_settings = None
        post_process_volumes = game.unreal_service.find_actors_by_class(uclass="APostProcessVolume")
        if len(post_process_volumes) == 1:
            post_process_volume = post_process_volumes[0]
            spear.log("Found unique post-process volume: ", post_process_volume)
            post_process_volume_settings = post_process_volume.Settings.get()

        # GetViewportSize(...) modifies arguments in-place, so we need as_dict=True so all arguments get returned
        sp_game_viewport = game.get_unreal_object(uclass="USpGameViewportClient")
        return_values = sp_game_viewport.GetViewportSize(GameViewportClient=game_viewport_client, as_dict=True)

        if args.teaser:
            viewport_size_x = 1920
            viewport_size_y = 1080
        else:
            viewport_size_x = return_values["ViewportSize"]["x"]
            viewport_size_y = return_values["ViewportSize"]["y"]

        viewport_aspect_ratio = viewport_size_x/viewport_size_y # see Engine/Source/Editor/UnrealEd/Private/EditorViewportClient.cpp:2130 for evidence that Unreal's aspect ratio convention is x/y
        fov = view_target_pov["fOV"]*math.pi/180.0 # this adjustment is necessary to compute an FOV value that matches the game viewport
        half_fov = fov/2.0
        half_fov_adjusted = math.atan(math.tan(half_fov)*viewport_aspect_ratio/view_target_pov["aspectRatio"])
        fov_adjusted = half_fov_adjusted*2.0
        fov_adjusted_degrees = fov_adjusted*180.0/math.pi

        bp_camera_sensor.K2_SetActorLocation(NewLocation=view_target_pov["location"])
        bp_camera_sensor.K2_SetActorRotation(NewRotation=view_target_pov["rotation"])

        for component_desc in component_descs:
            component_desc["component"].Width = viewport_size_x*component_desc["spatial_supersampling_factor"]
            component_desc["component"].Height = viewport_size_y*component_desc["spatial_supersampling_factor"]
            component_desc["component"].FOVAngle = fov_adjusted_degrees

        if post_process_volume_settings is not None:
            final_tone_curve_hdr_component.PostProcessSettings = post_process_volume_settings

        # need to call Initialize() after spawning our ASpObjectIdsProxyComponentManager instance
        # need to call initialize_sp_funcs() after calling Initialize() because read_pixels() is registered during Initialize()
        for component_desc in component_descs:
            component_desc["component"].Initialize()
            component_desc["component"].initialize_sp_funcs()

    with instance.end_frame():
        pass # we could get rendered data here, but the rendered image will look better if we let temporal anti-aliasing etc accumulate additional information across frames

    # let temporal anti-aliasing etc accumulate additional information across multiple frames, and
    # inserting an extra frame can fix occasional render-to-texture initialization issues on macOS
    for i in range(1):
        instance.flush()

    # get rendered frame
    with instance.begin_frame():
        pass
    with instance.end_frame():
        for component_desc in component_descs:
            data_bundle = component_desc["component"].read_pixels()
            component_desc["data"] = data_bundle["arrays"]["data"]

    # get proxy IDs for foreground actor
    foreground_actor = actors[foreground_actor_name]
    foreground_actor_proxy_ids = np.array([ proxy_id for proxy_id, actor_name in proxy_id_to_actor_name_map.items() if actor_name == foreground_actor_name ])
    spear.log("foreground_actor_proxy_ids: ", foreground_actor_proxy_ids)

    # get colors for semantic IDs
    df = pd.read_csv(os.path.realpath(os.path.join(os.path.dirname(__file__), "semantic_label_descs.csv")), comment="#")
    df.columns = df.columns.str.strip()
    semantic_colors = np.vstack([[0,0,0], df[["semantic_color_r", "semantic_color_g", "semantic_color_b"]].to_numpy()]).astype(dtype=np.uint8)

    # generate colors for semantic instance IDs
    num_semantic_instances = len(semantic_instance_uobjects)
    semantic_instance_colors = np.zeros((num_semantic_instances, 3), dtype=np.uint8)
    for i in range(1, num_semantic_instances):
        semantic_instance_colors[i] = np.round(np.array(colorsys.hsv_to_rgb(np.random.uniform(), 0.8, 1.0))*255.0).astype(np.uint8)

    # save an image for each component using the component's visualizer function
    for component_desc in component_descs:
        data = component_desc["data"]
        image_file = os.path.realpath(os.path.join(images_dir, f"{component_desc['name']}.png"))
        image = component_desc["visualize_func"](data=data)
        spear.log("Saving image: ", image_file)
        plt.imsave(image_file, image)

    # create component desc map for saving more advanced derived images
    component_desc_map = { desc["name"]: desc for desc in component_descs }

    # semantic
    proxy_ids = component_desc_map["object_ids"]["data"].view(np.uint32).reshape(component_desc_map["object_ids"]["data"].shape[:2]) & 0x00ffffff
    semantic_ids = np.zeros_like(proxy_ids)
    for proxy_id, semantic_id in proxy_id_to_semantic_id_map.items():
        semantic_ids[proxy_id == proxy_ids] = semantic_id
    semantic = semantic_colors[semantic_ids]
    image_file = os.path.realpath(os.path.join(images_dir, "semantic.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, semantic)

    # semantic instance
    proxy_ids = component_desc_map["object_ids"]["data"].view(np.uint32).reshape(component_desc_map["object_ids"]["data"].shape[:2]) & 0x00ffffff
    semantic_instance_ids = np.zeros_like(proxy_ids)
    for proxy_id, semantic_instance_id in proxy_id_to_semantic_instance_id_map.items():
        semantic_instance_ids[proxy_id == proxy_ids] = semantic_instance_id
    semantic_instance = semantic_instance_colors[semantic_instance_ids]
    image_file = os.path.realpath(os.path.join(images_dir, "semantic_instance.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, semantic_instance)

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
    proxy_ids = component_desc_map["object_ids"]["data"].view(np.uint32).reshape(component_desc_map["object_ids"]["data"].shape[:2]) & 0x00ffffff
    mask = proxy_ids != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    lighting_only_diffuse_color = component_desc_map["lighting_only_diffuse_color"]["data"][:,:,[0,1,2]].astype(np.float32)
    lighting_only_post_process_input_2 = component_desc_map["lighting_only_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    diffuse_illumination = tone_map_hypersim(img=lighting_only_post_process_input_2/lighting_only_diffuse_color, mask=mask)
    diffuse_illumination = np.clip(diffuse_illumination, 0.0, 1.0)
    diffuse_illumination = PIL.Image.fromarray((diffuse_illumination*255.0).astype(np.uint8))
    diffuse_illumination = diffuse_illumination.resize((diffuse_illumination.width // spatial_supersampling_factor, diffuse_illumination.height // spatial_supersampling_factor), resample=PIL.Image.Resampling.LANCZOS)
    diffuse_illumination = np.asarray(diffuse_illumination)
    image_file = os.path.realpath(os.path.join(images_dir, "diffuse_illumination.png"))
    spear.log("Saving image: ", image_file)
    plt.imsave(image_file, diffuse_illumination)

    # diffuse_and_specular
    spatial_supersampling_factor = component_desc_map["diffuse_color"]["spatial_supersampling_factor"]
    proxy_ids = component_desc_map["object_ids"]["data"].view(np.uint32).reshape(component_desc_map["object_ids"]["data"].shape[:2]) & 0x00ffffff
    mask = proxy_ids != 0
    mask = np.repeat(np.repeat(mask, spatial_supersampling_factor, axis=0), spatial_supersampling_factor, axis=1)
    diffuse_and_specular = component_desc_map["diffuse_and_specular_post_process_input_2"]["data"][:,:,[0,1,2]].astype(np.float32)
    diffuse_and_specular, scale, gamma = tone_map_hypersim(img=diffuse_and_specular, mask=mask, as_tuple=True) # get scale and gamma from tone-mapping diffuse_and_specular
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

        sp_object_ids_proxy_component_manager.Terminate()
        game.unreal_service.destroy_actor(actor=sp_object_ids_proxy_component_manager)

    spear.log("Done.")
