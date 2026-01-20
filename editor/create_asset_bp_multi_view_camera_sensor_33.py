#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import posixpath
import spear
import spear.utils.editor_utils
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_MultiViewCameraSensor_33",
    "blueprint_dir": "/SpContent/Blueprints"
}

component_common_desc = \
{
    "width": 640,
    "height": 360,
    "fov_angle": 90.0,
    "num_channels_per_pixel": 4,
    "channel_data_type": unreal.SpArrayDataType.U_INT8,
    "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
    "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
    "show_flag_settings": [unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)],
    "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
    "reflection_method": unreal.ReflectionMethod.LUMEN
}

component_descs = \
[
    { "name": "final_tone_curve_hdr_00_", "location": unreal.Vector(x=0, y=-40, z=-40) },
    { "name": "final_tone_curve_hdr_01_", "location": unreal.Vector(x=0, y=0,   z=-40) },
    { "name": "final_tone_curve_hdr_02_", "location": unreal.Vector(x=0, y=+40, z=-40) },
    { "name": "final_tone_curve_hdr_10_", "location": unreal.Vector(x=0, y=-40, z=0)   },
    { "name": "final_tone_curve_hdr_11_", "location": unreal.Vector(x=0, y=0,   z=0)   },
    { "name": "final_tone_curve_hdr_12_", "location": unreal.Vector(x=0, y=+40, z=0)   },
    { "name": "final_tone_curve_hdr_20_", "location": unreal.Vector(x=0, y=-40, z=+40) },
    { "name": "final_tone_curve_hdr_21_", "location": unreal.Vector(x=0, y= 0,  z=+40) },
    { "name": "final_tone_curve_hdr_22_", "location": unreal.Vector(x=0, y=+40, z=+40) }
]

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
    for i, component_desc in enumerate(component_descs):

        component_name = component_desc["name"]
        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        sp_scene_capture_component_2d_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
            blueprint_asset=blueprint_asset,
            parent_data_handle=parent_data_handle,
            subobject_name=component_desc["name"],
            subobject_class=unreal.SpSceneCaptureComponent2D)

        sp_scene_capture_component_2d = sp_scene_capture_component_2d_desc["object"]

        # SpSceneCaptureComponent2D properties

        sp_scene_capture_component_2d.set_editor_property(name="relative_location", value=component_desc["location"])

        sp_scene_capture_component_2d.set_editor_property(name="width", value=component_common_desc["width"])
        sp_scene_capture_component_2d.set_editor_property(name="height", value=component_common_desc["height"])
        sp_scene_capture_component_2d.set_editor_property(name="fov_angle", value=component_common_desc["fov_angle"])
        sp_scene_capture_component_2d.set_editor_property(name="num_channels_per_pixel", value=component_common_desc["num_channels_per_pixel"])
        sp_scene_capture_component_2d.set_editor_property(name="channel_data_type", value=component_common_desc["channel_data_type"])
        sp_scene_capture_component_2d.set_editor_property(name="texture_render_target_format", value=component_common_desc["texture_render_target_format"])
        sp_scene_capture_component_2d.set_editor_property(name="capture_source", value=component_common_desc["capture_source"])
        sp_scene_capture_component_2d.set_editor_property(name="show_flag_settings", value=component_common_desc["show_flag_settings"])

        # PostProcessingSettings properties

        post_process_settings = sp_scene_capture_component_2d.get_editor_property(name="post_process_settings")
        post_process_settings.set_editor_property(name="override_dynamic_global_illumination_method", value=True)
        post_process_settings.set_editor_property(name="dynamic_global_illumination_method", value=component_common_desc["dynamic_global_illumination_method"])
        post_process_settings.set_editor_property(name="override_reflection_method", value=True)
        post_process_settings.set_editor_property(name="reflection_method", value=component_common_desc["reflection_method"])

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
