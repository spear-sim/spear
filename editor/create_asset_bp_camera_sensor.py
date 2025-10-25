#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import posixpath
import spear
import spear.utils.editor_utils
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_CameraSensor",
    "blueprint_dir": "/SpContent/Blueprints",
    "component_descs":
    [
        {
            "name": "ambient_occlusion_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_AmbientOcclusion.PPM_AmbientOcclusion",
            "fov_angle": 90.0,
        },
        {
            "name": "depth_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpContent/Materials/PPM_Depth.PPM_Depth",
            "fov_angle": 90.0,
        },
        {
            "name": "diffuse_color_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_DiffuseColor.PPM_DiffuseColor",
            "fov_angle": 90.0,
        },
        {
            "name": "final_tone_curve_hdr_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "fov_angle": 90.0,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": [unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)]
        },
        {
            "name": "metallic_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_Metallic.PPM_Metallic",
            "fov_angle": 90.0,
        },
        {
            "name": "normal_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpContent/Materials/PPM_Normal.PPM_Normal",
            "fov_angle": 90.0,
        },
        {
            "name": "roughness_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_Roughness.PPM_Roughness",
            "fov_angle": 90.0,
        },
        {
            "name": "segmentation_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpContent/Materials/PPM_Segmentation.PPM_Segmentation",
            "fov_angle": 90.0,
        },
        {
            "name": "specular_color_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.FLOAT16,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA16F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_COLOR_HDR,
            "material_path": "/SpContent/Materials/PPM_SpecularColor.PPM_SpecularColor",
            "fov_angle": 90.0,
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

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
