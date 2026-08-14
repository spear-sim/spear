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

# The universe of possible UserSceneTexture buffers that can be extracted by each capture component. Each name
# here adds a MI_PPM_<name>_UST material instance (see create_asset_mi_ppm_ust_instances.py) to the capture
# component via the UserSceneTextures property. Each PPM is configured to route its output to a buffer named
# <name>_UST, and the capture component knows how to extract data from these buffers. In order to enable the
# capture component to extract data from a buffer, the user must add <name> to the component's UserSceneTextureNames
# property at runtime.

user_scene_texture_material_names = \
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

engine_show_flag_settings = {}

#
# Show flags for specific components
#

# final_tone_curve_hdr

engine_show_flag_settings["final_tone_curve_hdr"] = []
engine_show_flag_settings["final_tone_curve_hdr"] = engine_show_flag_settings["final_tone_curve_hdr"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)
]

# diffuse_and_specular

engine_show_flag_settings["diffuse_and_specular"] = []
engine_show_flag_settings["diffuse_and_specular"] = engine_show_flag_settings["diffuse_and_specular"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

# diffuse_only

engine_show_flag_settings["diffuse_only"] = []
engine_show_flag_settings["diffuse_only"] = engine_show_flag_settings["diffuse_only"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="Specular", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

# lighting_only

engine_show_flag_settings["lighting_only"] = []
engine_show_flag_settings["lighting_only"] = engine_show_flag_settings["lighting_only"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="LightingOnlyOverride", enabled=True),
    unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
]

# sp_object_ids_uint8

#
# For the sp_object_ids_uint8 settings, we mimic various locations in the UE source code that turn off show
# flags, we disable various show flags that are visible in the editor:
#
#   - FEngineShowFlags::DisableAdvancedFeatures()
#   - UMoviePipelineObjectIdRenderPass::GetViewShowFlags(...)
#   - General Show Flags, Advanced Show Flags, Light Types Show Flags, Lighting Components Show Flags,
#     Lighting Features Show Flags, Post Processing Show Flags, Hidden Show Flags
#

engine_show_flag_settings["sp_object_ids_uint8"] = []
engine_show_flag_settings["sp_object_ids_uint8"] = engine_show_flag_settings["sp_object_ids_uint8"] + \
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
    unreal.EngineShowFlagsSetting(show_flag_name="PostProcessing", enabled=False),
    unreal.EngineShowFlagsSetting(show_flag_name="PostProcessMaterial", enabled=False),
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

blueprint_desc = \
{
    "blueprint_name": "BP_CameraSensor_UST",
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
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["final_tone_curve_hdr"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB
        },
        {
            "name": "diffuse_and_specular_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["diffuse_and_specular"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB
        },
        {
            "name": "diffuse_only_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["diffuse_only"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB
        },
        {
            "name": "lighting_only_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": engine_show_flag_settings["lighting_only"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB
        },
        {
            "name": "sp_object_ids_uint8_",
            "width": width,
            "height": height,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "show_flag_settings": engine_show_flag_settings["sp_object_ids_uint8"],
            "mesh_proxy_component_manager_class": unreal.SpObjectIdsProxyComponentManager,
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8,
            "override_texture_render_target_srgb": True,
            "texture_render_target_srgb": False,
            "override_texture_render_target_force_linear_gamma": True,
            "texture_render_target_force_linear_gamma": True,
            "override_texture_render_target_gamma": True,
            "texture_render_target_gamma": 1.0
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

    # Build the UserSceneTexture universe shared by every capture component. We read each buffer's InternalName
    # from its own material instance override so it is guaranteed to match what the material actually writes; a
    # mismatch would assert on the render thread and take the whole engine down.
    user_scene_texture_material_descs = {}
    for user_scene_texture_material_name in user_scene_texture_material_names:
        material_instance_path = posixpath.join("/SpContent/Materials", "MI_PPM_" + user_scene_texture_material_name + "_UST")
        material_instance = unreal.load_asset(name=material_instance_path)
        assert isinstance(material_instance, unreal.MaterialInstanceConstant)

        user_scene_texture_overrides = material_instance.get_editor_property(name="user_scene_texture_overrides")
        assert len(user_scene_texture_overrides) == 1
        internal_name = str(user_scene_texture_overrides[0].get_editor_property(name="value"))
        assert internal_name != ""

        material_desc = unreal.SpUserSceneTextureMaterialDesc()
        material_desc.set_editor_property(name="material", value=material_instance)
        material_desc.set_editor_property(name="internal_name", value=internal_name)
        material_desc.set_editor_property(name="resolution_divisor_width", value=1)
        material_desc.set_editor_property(name="resolution_divisor_height", value=1)

        # the public key is derived from the material name and matches InternalName
        user_scene_texture_material_descs[user_scene_texture_material_name] = material_desc

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

        if "mesh_proxy_component_manager_class" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="mesh_proxy_component_manager_class", value=component_desc["mesh_proxy_component_manager_class"])

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

        if "dynamic_global_illumination_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
            post_process_settings.set_editor_property(name="override_dynamic_global_illumination_method", value=True)
            post_process_settings.set_editor_property(name="dynamic_global_illumination_method", value=component_desc["dynamic_global_illumination_method"])

        if "reflection_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
            post_process_settings.set_editor_property(name="override_reflection_method", value=True)
            post_process_settings.set_editor_property(name="reflection_method", value=component_desc["reflection_method"])

        # UserSceneTexture universe (applied to every component; enabled set left empty)

        sp_scene_capture_component_2d.set_editor_property(name="user_scene_texture_material_descs", value=user_scene_texture_material_descs)
        sp_scene_capture_component_2d.set_editor_property(name="user_scene_texture_names", value=[])

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
