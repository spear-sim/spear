#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import posixpath
import spear
import spear.editor
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_Camera_Sensor",
    "blueprint_path": "/SpComponents/Blueprints",
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
            "material_path": "/SpComponents/Materials/PPM_AmbientOcclusion.PPM_AmbientOcclusion",
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
            "material_path": "/SpComponents/Materials/PPM_Depth.PPM_Depth",
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
            "material_path": "/SpComponents/Materials/PPM_DiffuseColor.PPM_DiffuseColor",
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
            "material_path": "/SpComponents/Materials/PPM_Metallic.PPM_Metallic",
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
            "material_path": "/SpComponents/Materials/PPM_Normal.PPM_Normal",
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
            "material_path": "/SpComponents/Materials/PPM_Roughness.PPM_Roughness",
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
            "material_path": "/SpComponents/Materials/PPM_Segmentation.PPM_Segmentation",
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
            "material_path": "/SpComponents/Materials/PPM_SpecularColor.PPM_SpecularColor",
            "fov_angle": 90.0,
        },
    ]
}

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # explicitly load "/SpComponents" into the asset registry, since it won't be loaded by default if we are
    # running from the command-line
    asset_registry.scan_paths_synchronous(paths=["/SpComponents"])

    # remove existing blueprint
    blueprint_path = posixpath.join(blueprint_desc["blueprint_path"], blueprint_desc["blueprint_name"])
    if unreal.EditorAssetLibrary.does_asset_exist(blueprint_path):
        spear.log(f"Asset exists, removing: {blueprint_path}")
        success = unreal.EditorAssetLibrary.delete_asset(blueprint_path)
        assert success

    # create blueprint
    spear.log(f"Creating blueprint: {blueprint_path}")
    blueprint_asset, blueprint_subobject_descs = spear.editor.create_blueprint(
        asset_name=blueprint_desc["blueprint_name"],
        package_path=blueprint_desc["blueprint_path"])

    # create SpStableNameComponent
    component_name = "sp_stable_name_component_"
    spear.log(f"Creating component: ", component_name)
    sp_stable_name_component_desc = spear.editor.add_new_subobject(
        blueprint_asset=blueprint_asset,
        parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create SpSceneCaptureComponent2Ds
    for component_desc in blueprint_desc["component_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])
        sp_scene_capture_component_2d_desc = spear.editor.add_new_subobject(
            blueprint_asset=blueprint_asset,
            parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
            subobject_name=component_desc["name"],
            subobject_class=unreal.SpSceneCaptureComponent2D)

        sp_scene_capture_component_2d = sp_scene_capture_component_2d_desc["object"]

        # SpSceneCaptureComponent2D properties (required)

        sp_scene_capture_component_2d.set_editor_property("width", component_desc["width"])
        sp_scene_capture_component_2d.set_editor_property("height", component_desc["height"])
        sp_scene_capture_component_2d.set_editor_property("num_channels_per_pixel", component_desc["num_channels_per_pixel"])
        sp_scene_capture_component_2d.set_editor_property("channel_data_type", component_desc["channel_data_type"])
        sp_scene_capture_component_2d.set_editor_property("texture_render_target_format", component_desc["texture_render_target_format"])
        sp_scene_capture_component_2d.set_editor_property("capture_source", component_desc["capture_source"])

        # SpSceneCaptureComponent2D properties (optional)

        if "material_path" in component_desc:
            material = unreal.load_asset(component_desc["material_path"])
            sp_scene_capture_component_2d.set_editor_property("material", material)

        # SceneCaptureComponent2D properties (optional)

        if "fov_angle" in component_desc:
            sp_scene_capture_component_2d.set_editor_property("fov_angle", component_desc["fov_angle"])

        # SceneCaptureComponent properties (optional)

        if "show_flag_settings" in component_desc:
            sp_scene_capture_component_2d.set_editor_property("show_flag_settings", component_desc["show_flag_settings"])

        # PostProcessingSettings properties (optional)

        if "dynamic_global_illumination_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property("post_process_settings")
            post_process_settings.set_editor_property("override_dynamic_global_illumination_method", True)
            post_process_settings.set_editor_property("dynamic_global_illumination_method", component_desc["dynamic_global_illumination_method"])

        if "reflection_method" in component_desc:
            post_process_settings = sp_scene_capture_component_2d.get_editor_property("post_process_settings")
            post_process_settings.set_editor_property("override_reflection_method", True)
            post_process_settings.set_editor_property("reflection_method", component_desc["reflection_method"])

    # save blueprint
    spear.log(f"Saving blueprint: {blueprint_path}")
    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log("Done.")
