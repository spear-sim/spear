#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import posixpath
import spear
import unreal


width = 512
height = 512
fov_angle = 90.0
spatial_supersampling_factor = 2

engine_show_flag_settings = {}

#
# final_tone_curve_hdr
#

engine_show_flag_settings["final_tone_curve_hdr_with_path_tracing"] = []
engine_show_flag_settings["final_tone_curve_hdr_with_path_tracing"] = engine_show_flag_settings["final_tone_curve_hdr_with_path_tracing"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="PathTracing", enabled=True)
]

#
# The disable_all_but_allow_post_processing_material settings are identical to disable_all
# (see create_asset_bp_camera_sensor.py) except we allow post-processing materials.
#

engine_show_flag_settings["disable_all_but_allow_post_processing_material"] = []
engine_show_flag_settings["disable_all_but_allow_post_processing_material"] = engine_show_flag_settings["disable_all_but_allow_post_processing_material"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientCubemap", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientOcclusion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="AntiAliasing", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Atmosphere", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Bloom", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="CameraImperfections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ColorGrading", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DepthOfField", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DistanceFieldAO", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DynamicShadows", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="EyeAdaptation", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Fog", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Grain", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HighResScreenshotMask", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HitProxies", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HMDDistortion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="IndirectLightingCache", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LensFlares", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightFunctions", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Lighting", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightShafts", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LocalExposure", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenGlobalIllumination", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MegaLights", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MotionBlur", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="OnScreenDebug", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ReflectionEnvironment", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenPercentage", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenSpaceReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="SeparateTranslucency", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ShaderPrint", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="SkyLighting", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="StereoRendering", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TexturedLightProfiles", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ToneCurve", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Tonemapper", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Vignette", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VirtualShadowMapPersistentData", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricFog", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricLightmap", enabled=False)
]

#
# The lighting_only_disable_all_but_allow_post_processing_material settings are identical to disable_all
# (see create_asset_bp_camera_sensor.py) except enable the lighting-only override and we allow
# post-processing materials.
#

engine_show_flag_settings["lighting_only_disable_all_but_allow_post_processing_material"] = []
engine_show_flag_settings["lighting_only_disable_all_but_allow_post_processing_material"] = engine_show_flag_settings["lighting_only_disable_all_but_allow_post_processing_material"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientCubemap", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="AmbientOcclusion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="AntiAliasing", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Atmosphere", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Bloom", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="CameraImperfections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ColorGrading", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DepthOfField", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DistanceFieldAO", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="DynamicShadows", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="EyeAdaptation", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Fog", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Grain", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HighResScreenshotMask", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HitProxies", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="HMDDistortion", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="IndirectLightingCache", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LensFlares", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightFunctions", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Lighting", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LightingOnlyOverride", enabled=True),
    unreal.EngineShowFlagsSetting(show_flag_name="LightShafts", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LocalExposure", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenGlobalIllumination", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="LumenReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MegaLights", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="MotionBlur", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="OnScreenDebug", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ReflectionEnvironment", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenPercentage", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ScreenSpaceReflections", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="SeparateTranslucency", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ShaderPrint", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="SkyLighting", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="StereoRendering", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TexturedLightProfiles", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="ToneCurve", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Tonemapper", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="Vignette", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VirtualShadowMapPersistentData", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricFog", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="VolumetricLightmap", enabled=False)
]

#
# Lighting only (with path tracing)
#

engine_show_flag_settings["lighting_only_with_path_tracing"] = []
engine_show_flag_settings["lighting_only_with_path_tracing"] = engine_show_flag_settings["lighting_only_with_path_tracing"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="LightingOnlyOverride", enabled=True),
    unreal.EngineShowFlagsSetting(show_flag_name="PathTracing", enabled=True)
]

#
# With lighting (with path tracing)
#

engine_show_flag_settings["with_lighting_with_path_tracing"] = []
engine_show_flag_settings["with_lighting_with_path_tracing"] = engine_show_flag_settings["with_lighting_with_path_tracing"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="PathTracing", enabled=True)
]

blueprint_desc = \
{
    "blueprint_name": "BP_CameraSensorPathTracer",
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
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "show_flag_settings": engine_show_flag_settings["final_tone_curve_hdr_with_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB
        },
        {
            "name": "diffuse_color_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_DiffuseColor",
            "show_flag_settings": engine_show_flag_settings["disable_all_but_allow_post_processing_material"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F
        },
        {
            "name": "diffuse_and_specular_path_tracing_radiance_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT32,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PathTracingRadiance",
            "show_flag_settings": engine_show_flag_settings["with_lighting_with_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F
        },
        {
            "name": "diffuse_and_specular_path_tracing_variance_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT32,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PathTracingVariance",
            "show_flag_settings": engine_show_flag_settings["with_lighting_with_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F
        },
        {
            "name": "lighting_only_diffuse_color_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_DiffuseColor",
            "show_flag_settings": engine_show_flag_settings["lighting_only_disable_all_but_allow_post_processing_material"],
            "use_scene_view_extension": True,
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F
        },
        {
            "name": "lighting_only_path_tracing_radiance_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT32,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PathTracingRadiance",
            "show_flag_settings": engine_show_flag_settings["lighting_only_with_path_tracing"],
            "use_scene_view_extension": True,
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F
        },
        {
            "name": "specular_only_path_tracing_radiance_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT32,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_PathTracingRadiance",
            "show_flag_settings": engine_show_flag_settings["with_lighting_with_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F,
            "post_process_settings":
            [
                {"property": "override_path_tracing_include_diffuse", "value": True},
                {"property": "path_tracing_include_diffuse", "value": False},
                {"property": "override_path_tracing_include_indirect_diffuse", "value": True},
                {"property": "path_tracing_include_indirect_diffuse", "value": False}
            ]
        },
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
    blueprint_asset = spear.editor.create_blueprint_asset(
        asset_name=blueprint_desc["blueprint_name"],
        package_dir=blueprint_desc["blueprint_dir"],
        parent_class=unreal.Actor)

    blueprint_subobject_descs = spear.editor.get_subobject_descs_for_blueprint_asset(blueprint_asset=blueprint_asset)
    assert len(blueprint_subobject_descs) == 2
    assert isinstance(blueprint_subobject_descs[0]["object"], unreal.Actor)          # the 0th entry always refers to the actor itself
    assert isinstance(blueprint_subobject_descs[1]["object"], unreal.SceneComponent) # the 1st entry must be the root component in this case because there are only 2 entries

    # create SpStableNameComponent
    component_name = "sp_stable_name_component_"
    spear.log("Creating component: ", component_name)
    parent_data_handle = blueprint_subobject_descs[0]["data_handle"] # actor
    sp_stable_name_component_desc = spear.editor.add_new_subobject_to_blueprint_asset(
        blueprint_asset=blueprint_asset,
        parent_data_handle=parent_data_handle,
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create SpSceneCaptureComponent2Ds
    for component_desc in blueprint_desc["component_descs"]:

        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        sp_scene_capture_component_2d_desc = spear.editor.add_new_subobject_to_blueprint_asset(
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
        sp_scene_capture_component_2d.set_editor_property(name="capture_source", value=component_desc["capture_source"])

        # SpSceneCaptureComponent2D properties (optional)

        if "material_path" in component_desc:
            material = unreal.load_asset(name=component_desc["material_path"])
            sp_scene_capture_component_2d.set_editor_property(name="material", value=material)

        if "use_scene_view_extension" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="use_scene_view_extension", value=component_desc["use_scene_view_extension"])

        # SpSceneCaptureComponent2D properties for texture render target (optional)

        if "override_texture_render_target_format" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="override_texture_render_target_format", value=component_desc["override_texture_render_target_format"])
            sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_format", value=component_desc["texture_render_target_format"])

        if "override_texture_render_target_srgb" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="override_texture_render_target_srgb", value=component_desc["override_texture_render_target_srgb"])
            sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_srgb", value=component_desc["texture_render_target_srgb"])

        if "override_texture_render_target_force_linear_gamma" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="override_texture_render_target_force_linear_gamma", value=component_desc["override_texture_render_target_force_linear_gamma"])
            sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_force_linear_gamma", value=component_desc["texture_render_target_force_linear_gamma"])

        if "override_texture_render_target_gamma" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="override_texture_render_target_gamma", value=component_desc["override_texture_render_target_gamma"])
            sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_gamma", value=component_desc["texture_render_target_gamma"])

        # SceneCaptureComponent2D properties (optional)

        if "fov_angle" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="fov_angle", value=component_desc["fov_angle"])

        # SceneCaptureComponent properties (optional)

        if "show_flag_settings" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="show_flag_settings", value=component_desc["show_flag_settings"])

        # PostProcessingSettings properties (optional)

        if "post_process_settings" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
            for post_process_setting in component_desc["post_process_settings"]:
                post_process_settings.set_editor_property(name=post_process_setting["property"], value=post_process_setting["value"])

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
