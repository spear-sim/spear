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
    "PathTracingAlbedo",
    "PathTracingDenoisedRadiance",
    "PathTracingNormal",
    "PathTracingRadiance",
    "PathTracingVariance",
    "SceneDepth"
]

engine_show_flag_settings = {}

#
# Show flags for specific components
#

# lighting_only

engine_show_flag_settings["lighting_only"] = []
engine_show_flag_settings["lighting_only"] = engine_show_flag_settings["lighting_only"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="LightingOnlyOverride", enabled=True),
    unreal.EngineShowFlagsSetting(show_flag_name="PathTracing", enabled=True)
]

#
# Show flags for multiple components
#

# enable_path_tracing

engine_show_flag_settings["enable_path_tracing"] = []
engine_show_flag_settings["enable_path_tracing"] = engine_show_flag_settings["enable_path_tracing"] + \
[
    unreal.EngineShowFlagsSetting(show_flag_name="PathTracing", enabled=True)
]

blueprint_desc = \
{
    "blueprint_name": "BP_CameraSensorPathTracer_UST",
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
            "show_flag_settings": engine_show_flag_settings["enable_path_tracing"],
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
            "show_flag_settings": engine_show_flag_settings["enable_path_tracing"],
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
            "show_flag_settings": engine_show_flag_settings["enable_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "post_process_settings":
            [
                {"property": "override_path_tracing_include_emissive", "value": True},
                {"property": "path_tracing_include_emissive", "value": False},
                {"property": "override_path_tracing_include_specular", "value": True},
                {"property": "path_tracing_include_specular", "value": False},
                {"property": "override_path_tracing_include_indirect_specular", "value": True},
                {"property": "path_tracing_include_indirect_specular", "value": False},
                {"property": "override_path_tracing_include_volume", "value": True},
                {"property": "path_tracing_include_volume", "value": False},
                {"property": "override_path_tracing_include_indirect_volume", "value": True},
                {"property": "path_tracing_include_indirect_volume", "value": False}
            ]
        },
        {
            "name": "lighting_only_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "show_flag_settings": engine_show_flag_settings["lighting_only"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
        },
        {
            "name": "specular_only_",
            "width": width*spatial_supersampling_factor,
            "height": height*spatial_supersampling_factor,
            "fov_angle": fov_angle,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "show_flag_settings": engine_show_flag_settings["enable_path_tracing"],
            "override_texture_render_target_format": True,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "post_process_settings":
            [
                {"property": "override_path_tracing_include_diffuse", "value": True},
                {"property": "path_tracing_include_diffuse", "value": False},
                {"property": "override_path_tracing_include_indirect_diffuse", "value": True},
                {"property": "path_tracing_include_indirect_diffuse", "value": False}
            ]
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

        # SpSceneCaptureComponent2D properties for texture render target (optional)

        if "override_texture_render_target_format" in component_desc:
            sp_scene_capture_component_2d.set_editor_property(name="override_texture_render_target_format", value=component_desc["override_texture_render_target_format"])
            sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_format", value=component_desc["texture_render_target_format"])

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

        # UserSceneTexture universe (must be enabled by setting UserSceneTextureNames at runtime)

        sp_scene_capture_component_2d.set_editor_property(name="user_scene_texture_material_descs", value=user_scene_texture_material_descs)
        sp_scene_capture_component_2d.set_editor_property(name="user_scene_texture_names", value=[])

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
