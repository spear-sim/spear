#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import posixpath
import spear
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_Camera_Sensor",
    "blueprint_path": "/SpComponents/Blueprints",
    "component_descs":
    [
        {
            "name": "final_tone_curve_hdr",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpFuncArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "fov_angle": 90.0,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": [unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)]
        },
        {
            "name": "depth",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpFuncArrayDataType.FLOAT32,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpServices/Materials/PPM_Depth.PPM_Depth",
            "fov_angle": 90.0,
        },
        {
            "name": "normal",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpFuncArrayDataType.FLOAT32,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA32F,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpServices/Materials/PPM_Normal.PPM_Normal",
            "fov_angle": 90.0,
        },
        {
            "name": "segmentation",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpFuncArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "material_path": "/SpServices/Materials/PPM_Segmentation.PPM_Segmentation",
            "fov_angle": 90.0,
        }
    ]
}

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)


if __name__ == "__main__":

    # create blueprint type
    blueprint_path = posixpath.join(blueprint_desc["blueprint_path"], blueprint_desc["blueprint_name"])
    if unreal.EditorAssetLibrary.does_asset_exist(blueprint_path):
        spear.log(f"Asset exists, removing: {blueprint_path}")
        success = unreal.EditorAssetLibrary.delete_asset(blueprint_path)
        assert success

    spear.log(f"Creating blueprint: {blueprint_path}")
     
    blueprint_factory = unreal.BlueprintFactory()
    blueprint_factory.set_editor_property("parent_class", unreal.Actor)

    blueprint_asset = asset_tools.create_asset(asset_name=blueprint_desc["blueprint_name"], package_path=blueprint_desc["blueprint_path"], asset_class=None, factory=blueprint_factory)
    assert isinstance(blueprint_asset, unreal.Blueprint)

    subobject_data_handles = subobject_data_subsystem.k2_gather_subobject_data_for_blueprint(blueprint_asset)

    assert len(subobject_data_handles) == 2        
    for subobject_data_handle in subobject_data_handles:
        assert unreal.SubobjectDataBlueprintFunctionLibrary.is_handle_valid(subobject_data_handle)
        subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(subobject_data_handle)
        assert unreal.SubobjectDataBlueprintFunctionLibrary.is_valid(subobject_data)

    actor_subobject_data_handle = subobject_data_handles[0]
    actor_subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(actor_subobject_data_handle)
    actor = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(actor_subobject_data)
    assert isinstance(actor, unreal.Actor)

    scene_component_subobject_data_handle = subobject_data_handles[1]
    scene_component_subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(scene_component_subobject_data_handle)
    scene_component = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(scene_component_subobject_data)
    assert isinstance(scene_component, unreal.SceneComponent)

    # create SpStableNameComponent
    component_name = "sp_stable_name_component"
    spear.log(f"Creating component: ", component_name)

    add_new_subobject_params = unreal.AddNewSubobjectParams(
        parent_handle=scene_component_subobject_data_handle,
        new_class=unreal.SpStableNameComponent,
        blueprint_context=blueprint_asset)
    sp_stable_name_component_subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(add_new_subobject_params)
    assert fail_reason.is_empty()
    subobject_data_subsystem.rename_subobject(sp_stable_name_component_subobject_data_handle, unreal.Text(component_name))

    sp_stable_name_component_subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(sp_stable_name_component_subobject_data_handle)
    sp_stable_name_component = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(sp_stable_name_component_subobject_data)
    assert isinstance(sp_stable_name_component, unreal.SpStableNameComponent)

    # create SpSceneCaptureComponent2Ds
    for component_desc in blueprint_desc["component_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])

        add_new_subobject_params = unreal.AddNewSubobjectParams(
            parent_handle=scene_component_subobject_data_handle,
            new_class=unreal.SpSceneCaptureComponent2D,
            blueprint_context=blueprint_asset)
        sp_scene_capture_component_2d_subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(add_new_subobject_params)
        assert fail_reason.is_empty()
        subobject_data_subsystem.rename_subobject(sp_scene_capture_component_2d_subobject_data_handle, unreal.Text(component_desc["name"]))

        sp_scene_capture_component_2d_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(sp_scene_capture_component_2d_subobject_data_handle)
        sp_scene_capture_component_2d = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(sp_scene_capture_component_2d_data)
        assert isinstance(sp_scene_capture_component_2d, unreal.SpSceneCaptureComponent2D)

        # SpSceneCaptureComponent2D properties (required)
        sp_scene_capture_component_2d.set_editor_property("width", component_desc["width"])
        sp_scene_capture_component_2d.set_editor_property("height", component_desc["height"])
        sp_scene_capture_component_2d.set_editor_property("num_channels_per_pixel", component_desc["num_channels_per_pixel"])
        sp_scene_capture_component_2d.set_editor_property("channel_data_type", component_desc["channel_data_type"])
        sp_scene_capture_component_2d.set_editor_property("texture_render_target_format", component_desc["texture_render_target_format"])

        # SceneCaptureComponent2D properties (required)
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

    spear.log(f"Saving blueprint: {blueprint_path}")

    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log(f"Done.")
