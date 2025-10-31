#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import posixpath
import spear
import spear.utils.editor_utils
import unreal


width = 512
height = 512
fov_angle = 90.0
spatial_supersampling_factor = 2

engine_show_flag_settings = {}

#
# final_tone_curve_hdr
#

engine_show_flag_settings["final_tone_curve_hdr"] = []
engine_show_flag_settings["final_tone_curve_hdr"] = engine_show_flag_settings["final_tone_curve_hdr"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)
]

#
# Without anti-aliasing
#

engine_show_flag_settings["without_lighting"] = []

# FEngineShowFlags::DisableAdvancedFeatures()
# Excluded:
#     unreal.EngineShowFlagsSetting(show_flag_name="PostProcessMaterial", enabled=False)

engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="LensFlares", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="OnScreenDebug", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="EyeAdaptation", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ColorGrading", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="CameraImperfections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DepthOfField", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Vignette", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Grain", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="SeparateTranslucency", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenPercentage", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenSpaceReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientOcclusion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="IndirectLightingCache", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightShafts", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HighResScreenshotMask", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HMDDistortion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="StereoRendering", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DistanceFieldAO", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricFog", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricLightmap", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenGlobalIllumination", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MegaLights", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VirtualShadowMapPersistentData", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ShaderPrint", enabled=False)
]

# UMoviePipelineObjectIdRenderPass::GetViewShowFlags(...)
# Excluded:
#     unreal.EngineShowFlagsSetting(show_flag_name="PostProcessing", enabled=False)
#     unreal.EngineShowFlagsSetting(show_flag_name="PostProcessMaterial", enabled=False)

engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenPercentage", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HitProxies", enabled=False)
]

# disable "General Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="AntiAliasing", enabled=False)
]

# disable "Advanced Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

# disable "Light Types Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="SkyLighting", enabled=False)
]

# disable "Lighting Components Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientOcclusion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DynamicShadows", enabled=False)
]

# disable "Lighting Features Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientCubemap", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DistanceFieldAO", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightFunctions", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightShafts", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ReflectionEnvironment", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenSpaceReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TexturedLightProfiles", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricFog", enabled=False)
]

# disable "Post Processing Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="Bloom", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="EyeAdaptation", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LocalExposure", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MotionBlur", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ToneCurve", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Tonemapper", enabled=False)
]

# disable "Hidden Show Flags" that are visible in the editor UI for scene capture components
engine_show_flag_settings["without_lighting"] = engine_show_flag_settings["without_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="Lighting", enabled=False)
]

#
# We intentially disable temporal antialiasing below. Otherwise, e.g., /SpContent/Materials/PPM_DiffuseColor
# will be jittered across frames but not actually anti-aliased on any particular frame. On the other hand,
# /SpContent/Materials/PPM_PostProcessInput2 won't be jittered across frames but will be anti-aliased on each
# frame. This means we can't, e.g., divide PPM_PostProcessInput2 by PPM_DiffuseColor to estimate irradiance.
# So we choose to disable temporal antialiasing, and implement our own anti-aliasing strategy with spatial
# supersampling, similar to the strategy used in the MPPC_DefaultConfigWithLighting movie render queue config
# we're attempting to replicate here.
#

#
# With lighting
#

engine_show_flag_settings["with_lighting"] = []
engine_show_flag_settings["with_lighting"] = engine_show_flag_settings["with_lighting"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

#
# With lighting (diffuse only)
#

engine_show_flag_settings["with_lighting_diffuse_only"] = []
engine_show_flag_settings["with_lighting_diffuse_only"] = engine_show_flag_settings["with_lighting_diffuse_only"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="Specular", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

#
# Lighting only
#

engine_show_flag_settings["lighting_only"] = []
engine_show_flag_settings["lighting_only"] = engine_show_flag_settings["lighting_only"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="LightingOnlyOverride", enabled=True),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

blueprint_desc = \
{
    "blueprint_name": "BP_CameraSensor",
    "blueprint_dir": "/SpContent/Blueprints",
    "component_descs":
    [
        {
            "name": "final_tone_curve_hdr_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["final_tone_curve_hdr"]
        },
        {
            "name": "custom_stencil_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_CustomStencil",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "material_ao_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_MaterialAO",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "metallic_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_Metallic",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "roughness_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_Roughness",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "scene_depth_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_SceneDepth",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "sp_camera_normal_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_SpCameraNormal",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "sp_depth_meters_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_SpDepthMeters",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "sp_world_position_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_SpWorldPosition",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "object_ids_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PostProcessInput2",
            "allowed_proxy_component_modalities": ["object_ids"],
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "world_normal_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_WorldNormal",
            "show_flag_settings": engine_show_flag_settings["without_lighting"]
        },
        {
            "name": "diffuse_color_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_DiffuseColor",
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["with_lighting_diffuse_only"]
        },
        {
            "name": "diffuse_only_post_process_input_2_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PostProcessInput2",
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["with_lighting_diffuse_only"],
            "use_scene_view_extension": True
        },
        {
            "name": "diffuse_and_specular_post_process_input_2_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PostProcessInput2",
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["with_lighting"]
        },
        {
            "name": "lighting_only_diffuse_color_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_DiffuseColor",
            "allowed_primitive_proxy_component_modalities": ["lighting_only"],
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["lighting_only"],
            "use_scene_view_extension": True
        },
        {
            "name": "lighting_only_post_process_input_2_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PostProcessInput2",
            "allowed_primitive_proxy_component_modalities": ["lighting_only"],
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["lighting_only"],
            "use_scene_view_extension": True
        }
    ]
}

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # Explicitly load "/SpContent" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with -run=pythonscript
    # as opposed to -ExecutePythonScript.
    asset_registry.scan_paths_synchronous(paths=["/SpContent"])

    # remove existing blueprint
    blueprint_path = posixpath.join(blueprint_desc["blueprint_dir"], blueprint_desc["blueprint_name"])
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path=blueprint_path):
        spear.log("Asset exists, removing: ", blueprint_path)
        success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=blueprint_path)
        assert success

    # create blueprint
    spear.log("Creating blueprint: ", blueprint_path)
    blueprint_asset = spear.utils.editor_utils.create_blueprint_asset(
        asset_name=blueprint_desc["blueprint_name"],
        package_dir=blueprint_desc["blueprint_dir"],
        parent_class=unreal.Actor)

    blueprint_subobject_descs = spear.utils.editor_utils.get_subobject_descs_for_blueprint_asset(blueprint_asset=blueprint_asset)
    assert len(blueprint_subobject_descs) == 2
    assert isinstance(blueprint_subobject_descs[0]["object"], unreal.Actor)          # the 0th entry always refers to the actor itself
    assert isinstance(blueprint_subobject_descs[1]["object"], unreal.SceneComponent) # the 1st entry must be the root component in this case because there are only 2 entries

    # create SpStableNameComponent
    component_name = "sp_stable_name_component_"
    spear.log("Creating component: ", component_name)
    parent_data_handle = blueprint_subobject_descs[0]["data_handle"] # actor
    sp_stable_name_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
        blueprint_asset=blueprint_asset,
        parent_data_handle=parent_data_handle,
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create SpSceneCaptureComponent2Ds
    for component_desc in blueprint_desc["component_descs"]:

        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        sp_scene_capture_component_2d_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
            blueprint_asset=blueprint_asset,
            parent_data_handle=parent_data_handle,
            subobject_name=component_desc["name"],
            subobject_class=unreal.SpSceneCaptureComponent2D)

        sp_scene_capture_component_2d = sp_scene_capture_component_2d_desc["object"]

        # SpSceneCaptureComponent2D properties (required)

        sp_scene_capture_component_2d.set_editor_property(name="width", value=component_desc["width"])
        sp_scene_capture_component_2d.set_editor_property(name="height", value=component_desc["height"])
        sp_scene_capture_component_2d.set_editor_property(name="num_channels_per_pixel", value=component_desc["num_channels_per_pixel"])
        sp_scene_capture_component_2d.set_editor_property(name="channel_data_type", value=component_desc["channel_data_type"])
        sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_format", value=component_desc["texture_render_target_format"])
        sp_scene_capture_component_2d.set_editor_property(name="capture_source", value=component_desc["capture_source"])

        # SpSceneCaptureComponent2D properties (optional)

        if "material_path" in component_desc:
            material = unreal.load_asset(name=component_desc["material_path"])
            sp_scene_capture_component_2d.set_editor_property(name="material", value=material)

        if "use_scene_view_extension" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="use_scene_view_extension", value=component_desc["use_scene_view_extension"])

        # SceneCaptureComponent2D properties (optional)

        if "fov_angle" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="fov_angle", value=component_desc["fov_angle"])

        if "allowed_proxy_component_modalities" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="allowed_proxy_component_modalities", value=component_desc["allowed_proxy_component_modalities"])

        # SceneCaptureComponent properties (optional)

        if "show_flag_settings" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="show_flag_settings", value=component_desc["show_flag_settings"])

        # PostProcessingSettings properties (optional)

        if "dynamic_global_illumination_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
            post_process_settings.set_editor_property(name="override_dynamic_global_illumination_method", value=True)
            post_process_settings.set_editor_property(name="dynamic_global_illumination_method", value=component_desc["dynamic_global_illumination_method"])

        if "reflection_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
            post_process_settings.set_editor_property(name="override_reflection_method", value=True)
            post_process_settings.set_editor_property(name="reflection_method", value=component_desc["reflection_method"])

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
