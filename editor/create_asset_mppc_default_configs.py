#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import posixpath
import spear
import spear.utils.editor_utils
import unreal


use_custom_playback_range = False
custom_start_frame = 0
custom_end_frame = 1

config_descs = \
{
    "without_lighting" :
    {
        "name": f"MPPC_DefaultConfigWithoutLighting",
        "dir": "/SpContent/Cinematic"
    },
    "with_lighting" :
    {
        "name": f"MPPC_DefaultConfigWithLighting",
        "dir": "/SpContent/Cinematic"
    },
    "with_path_tracer" :
    {
        "name": f"MPPC_DefaultConfigWithPathTracer",
        "dir": "/SpContent/Cinematic"
    }
}

post_process_material_descs = \
[
    {
        "name": "custom_stencil",
        "path": "/SpContent/Materials/PPM_CustomStencil",
    },
    {
        "name": "diffuse_color",
        "path": "/SpContent/Materials/PPM_DiffuseColor",
    },
    {
        "name": "material_ao",
        "path": "/SpContent/Materials/PPM_MaterialAO",
    },
    {
        "name": "metallic",
        "path": "/SpContent/Materials/PPM_Metallic",
    },
    {
        "name": "post_process_input_2",
        "path": "/SpContent/Materials/PPM_PostProcessInput2",
    },
    {
        "name": "roughness",
        "path": "/SpContent/Materials/PPM_Roughness",
    },
    {
        "name": "scene_depth",
        "path": "/SpContent/Materials/PPM_SceneDepth",
    },
    {
        "name": "sp_camera_normal",
        "path": "/SpContent/Materials/PPM_SpCameraNormal",
    },
    {
        "name": "sp_depth_meters",
        "path": "/SpContent/Materials/PPM_SpDepthMeters",
    },
    {
        "name": "sp_world_position",
        "path": "/SpContent/Materials/PPM_SpWorldPosition",
    },
    {
        "name": "world_normal",
        "path": "/SpContent/Materials/PPM_WorldNormal",
    }
]

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # Explicitly load "/SpContent" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with -run=pythonscript
    # as opposed to -ExecutePythonScript.
    asset_registry.scan_paths_synchronous(paths=["/SpContent"])

    # load post-process materials
    for post_process_material_desc in post_process_material_descs:
        post_process_material_desc["post_process_pass"] = unreal.MoviePipelinePostProcessPass()
        post_process_material_desc["post_process_pass"].set_editor_property(name="enabled", value=True)
        post_process_material_desc["post_process_pass"].set_editor_property(name="material", value=unreal.load_asset(name=post_process_material_desc["path"]))
        post_process_material_desc["post_process_pass"].set_editor_property(name="name", value="." + posixpath.split(post_process_material_desc["path"])[1])

    # convert to dict
    post_process_material_descs = { post_process_material_desc["name"]: post_process_material_desc for post_process_material_desc in post_process_material_descs }

    #
    #
    # Without lighting
    #
    #

    config_desc = config_descs["without_lighting"]

    # remove existing config
    config_path = posixpath.join(config_desc["dir"], config_desc["name"])
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path=config_path):
        spear.log("Asset exists, removing: ", config_path)
        success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=config_path)
        assert success

    # create config
    spear.log("Creating config: ", config_path)
    config = asset_tools.create_asset(asset_name=config_desc["name"], package_path=config_desc["dir"], asset_class=unreal.MoviePipelinePrimaryConfig, factory=None)
    assert isinstance(config, unreal.MoviePipelinePrimaryConfig)

    # needed to behave identically to a manually created config in the MRQ editor pane
    unreal.SpMoviePipelineConfigBase.set_display_name(config=config, display_name=config.get_name())

    #
    # Exports
    #

    # exr Sequence [16bit]
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineImageSequenceOutput_EXR, include_disabled_settings=False, exact_match=True)

    #
    # Rendering
    #

    # Deferred Rendering (SPEAR customizations) - use to output data that doesn't make sense to average across pixels

    setting = config.find_or_add_setting_by_class(class_=unreal.SpMoviePipelineDeferredPass, include_disabled_settings=False, exact_match=True)

    additional_post_process_materials = \
    [
        post_process_material_descs["material_ao"]["post_process_pass"],
        post_process_material_descs["metallic"]["post_process_pass"],
        post_process_material_descs["roughness"]["post_process_pass"],
        post_process_material_descs["scene_depth"]["post_process_pass"],
        post_process_material_descs["sp_camera_normal"]["post_process_pass"],
        post_process_material_descs["sp_depth_meters"]["post_process_pass"],
        post_process_material_descs["sp_world_position"]["post_process_pass"],
        post_process_material_descs["world_normal"]["post_process_pass"]
    ]

    console_variables = []
    engine_show_flag_settings = []

    # FEngineShowFlags::DisableAdvancedFeatures()
    # Excluded:
    #     unreal.EngineShowFlagsSetting(show_flag_name="PostProcessMaterial", enabled=False)

    engine_show_flag_settings = engine_show_flag_settings + \
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

    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="ScreenPercentage", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="HitProxies", enabled=False)
    ]

    # disable "General Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="AntiAliasing", enabled=False)
    ]

    # disable "Advanced Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=False)
    ]

    # disable "Light Types Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="SkyLighting", enabled=False)
    ]

    # disable "Lighting Components Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="AmbientOcclusion", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="DynamicShadows", enabled=False)
    ]

    # disable "Lighting Features Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
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
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="Bloom", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="EyeAdaptation", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="LocalExposure", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="MotionBlur", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="ToneCurve", enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name="Tonemapper", enabled=False)
    ]

    # disable "Hidden Show Flags" that are visible in the editor UI for scene capture components
    engine_show_flag_settings = engine_show_flag_settings + \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="Lighting", enabled=False)
    ]

    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="console_variables", value=console_variables)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    setting.set_editor_property(name="engine_show_flag_settings", value=engine_show_flag_settings)

    #
    # Settings
    #

    # Color
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineColorSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="disable_tone_curve", value=True)

    # Output
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineOutputSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="output_directory", value=unreal.DirectoryPath(path=os.path.join("{project_dir}", "Saved", "MovieRenders", config_desc["name"])))
    if use_custom_playback_range:
        setting.set_editor_property(name="use_custom_playback_range", value=True)
        setting.set_editor_property(name="custom_start_frame", value=custom_start_frame)
        setting.set_editor_property(name="custom_end_frame", value=custom_end_frame)

    # save config
    spear.log("Saving config: ", config)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=config)

    #
    #
    # With lighting
    #
    #

    config_desc = config_descs["with_lighting"]

    # remove existing config
    config_path = posixpath.join(config_desc["dir"], config_desc["name"])
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path=config_path):
        spear.log("Asset exists, removing: ", config_path)
        success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=config_path)
        assert success

    # create config
    spear.log("Creating config: ", config_path)
    config = asset_tools.create_asset(asset_name=config_desc["name"], package_path=config_desc["dir"], asset_class=unreal.MoviePipelinePrimaryConfig, factory=None)
    assert isinstance(config, unreal.MoviePipelinePrimaryConfig)

    # needed to behave identically to a manually created config in the MRQ editor pane
    unreal.SpMoviePipelineConfigBase.set_display_name(config=config, display_name=config.get_name())


    #
    # Exports
    #

    # exr Sequence [16bit]
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineImageSequenceOutput_EXR, include_disabled_settings=False, exact_match=True)


    #
    # Rendering
    #

    # Deferred Rendering - use to store buffers that make sense to average across pixels but that don't change when applying SPEAR customizations
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineDeferredPassBase, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["diffuse_color"]["post_process_pass"]
    ]
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)

    
    # Deferred Rendering (SPEAR customizations extra pass 0) - disable material AO, disable specular
    setting = config.find_or_add_setting_by_class(class_=unreal.SpMoviePipelineDeferredPass_ExtraPass0, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    console_variables = \
    [
        unreal.MoviePipelineConsoleVariableEntry(name="r.Lumen.ScreenProbeGather.MaterialAO", value=0.0) # disable material AO
    ]
    engine_show_flag_settings = \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="Specular", enabled=False) # disable specular
    ]
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    setting.set_editor_property(name="console_variables", value=console_variables)
    setting.set_editor_property(name="engine_show_flag_settings", value=engine_show_flag_settings)


    # Deferred Rendering (SPEAR customizations extra pass 1) - leave specular enabled, disable material AO
    setting = config.find_or_add_setting_by_class(class_=unreal.SpMoviePipelineDeferredPass_ExtraPass1, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    console_variables = \
    [
        unreal.MoviePipelineConsoleVariableEntry(name="r.Lumen.ScreenProbeGather.MaterialAO", value=0.0) # disable material AO
    ]
    engine_show_flag_settings = []
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    setting.set_editor_property(name="console_variables", value=console_variables)
    setting.set_editor_property(name="engine_show_flag_settings", value=engine_show_flag_settings)


    # Deferred Rendering (SPEAR customizations extra pass 2) - leave material AO enabled, disable specular
    setting = config.find_or_add_setting_by_class(class_=unreal.SpMoviePipelineDeferredPass_ExtraPass2, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    engine_show_flag_settings = \
    [
        unreal.EngineShowFlagsSetting(show_flag_name="Specular", enabled=False) # disable specular
    ]
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    setting.set_editor_property(name="console_variables", value=console_variables)
    setting.set_editor_property(name="engine_show_flag_settings", value=engine_show_flag_settings)


    # Deferred Rendering (SPEAR customizations extra pass 3) - leave material AO enabled, leave specular enabled
    setting = config.find_or_add_setting_by_class(class_=unreal.SpMoviePipelineDeferredPass_ExtraPass3, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    console_variables = []
    engine_show_flag_settings = []
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    setting.set_editor_property(name="console_variables", value=console_variables)
    setting.set_editor_property(name="engine_show_flag_settings", value=engine_show_flag_settings)


    # Deferred Rendering (Lighting Only)
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineDeferredPass_LightingOnly, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["diffuse_color"]["post_process_pass"],
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)


    # Object Ids (Limited)
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineObjectIdRenderPass, include_disabled_settings=False, exact_match=True)


    #
    # Settings
    #

    # Anti-aliasing
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineAntiAliasingSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="spatial_sample_count", value=7)

    # Color
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineColorSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="disable_tone_curve", value=True)

    # Output
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineOutputSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="output_directory", value=unreal.DirectoryPath(path=os.path.join("{project_dir}", "Saved", "MovieRenders", config_desc["name"])))
    if use_custom_playback_range:
        setting.set_editor_property(name="use_custom_playback_range", value=True)
        setting.set_editor_property(name="custom_start_frame", value=custom_start_frame)
        setting.set_editor_property(name="custom_end_frame", value=custom_end_frame)


    # save config
    spear.log("Saving config: ", config)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=config)

    spear.log("Done.")


    #
    #
    # With path tracing
    #
    #

    config_desc = config_descs["with_path_tracer"]

    # remove existing config
    config_path = posixpath.join(config_desc["dir"], config_desc["name"])
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path=config_path):
        spear.log("Asset exists, removing: ", config_path)
        success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=config_path)
        assert success

    # create config
    spear.log("Creating config: ", config_path)
    config = asset_tools.create_asset(asset_name=config_desc["name"], package_path=config_desc["dir"], asset_class=unreal.MoviePipelinePrimaryConfig, factory=None)
    assert isinstance(config, unreal.MoviePipelinePrimaryConfig)

    # needed to behave identically to a manually created config in the MRQ editor pane
    unreal.SpMoviePipelineConfigBase.set_display_name(config=config, display_name=config.get_name())

    #
    # Exports
    #

    # exr Sequence [16bit]
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineImageSequenceOutput_EXR, include_disabled_settings=False, exact_match=True)

    #
    # Rendering
    #

    # Path Tracer
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineDeferredPass_PathTracer, include_disabled_settings=False, exact_match=True)
    additional_post_process_materials = \
    [
        post_process_material_descs["post_process_input_2"]["post_process_pass"]
    ]
    setting.set_editor_property(name="additional_post_process_materials", value=additional_post_process_materials)
    setting.set_editor_property(name="disable_multisample_effects", value=True)
    
    #
    # Settings
    #

    # Anti-aliasing
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineAntiAliasingSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="spatial_sample_count", value=7)

    # Color
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineColorSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="disable_tone_curve", value=True)

    # Output
    setting = config.find_or_add_setting_by_class(class_=unreal.MoviePipelineOutputSetting, include_disabled_settings=False, exact_match=True)
    setting.set_editor_property(name="output_directory", value=unreal.DirectoryPath(path=os.path.join("{project_dir}", "Saved", "MovieRenders", config_desc["name"])))
    if use_custom_playback_range:
        setting.set_editor_property(name="use_custom_playback_range", value=True)
        setting.set_editor_property(name="custom_start_frame", value=custom_start_frame)
        setting.set_editor_property(name="custom_end_frame", value=custom_end_frame)

    # save config
    spear.log("Saving config: ", config)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=config)

    spear.log("Done.")
