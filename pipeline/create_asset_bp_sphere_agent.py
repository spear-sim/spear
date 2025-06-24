#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import posixpath
import spear
import spear.editor_utils
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_Sphere_Agent",
    "blueprint_path": "/SpComponents/Blueprints",
    "static_mesh_component_descs":
    [
        {
            "name": "sphere_", # use underscore to avoid FName conflict
            "static_mesh_path": "/Engine/BasicShapes/Sphere.Sphere",
            "material_path": "/Game/StarterContent/Materials/M_Ceramic_Tile_Checker.M_Ceramic_Tile_Checker",
            "location": {"X": 0.0, "Y": 0.0, "Z": 0.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.4, "Y": 0.4, "Z": 0.4},
            "simulate_physics": True
        },
        {
            "name": "camera_mesh_",
            "static_mesh_path": "/Engine/EditorMeshes/MatineeCam_SM.MatineeCam_SM",
            "material_path": "/Engine/EditorMaterials/MatineeCam_mat.MatineeCam_mat",
            "location": {"X": 0.0, "Y": 0.0, "Z": 35.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.25, "Y": 0.25, "Z": 0.25}
        },
        {
            "name": "x_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Red.MI_BasicShapeMaterial_Inst_Red",
            "location": {"X": 12.5, "Y": 0.0, "Z": 0.0},
            "rotation": {"Pitch": 90.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25},
            "collision_profile_name": "NoCollision"
        },
        {
            "name": "y_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Green.MI_BasicShapeMaterial_Inst_Green",
            "location": {"X": 0.0, "Y": 12.5, "Z": 0.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 90.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25},
            "collision_profile_name": "NoCollision"
        },
        {
            "name": "z_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Blue.MI_BasicShapeMaterial_Inst_Blue",
            "location": {"X": 0.0, "Y": 0.0, "Z": 12.5},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25},
            "collision_profile_name": "NoCollision"
        }
    ],
    "sp_scene_capture_component_2d_descs":
    [
        {
            "name": "final_tone_curve_hdr_",
            "width": 512,
            "height": 512,
            "num_channels_per_pixel": 4,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "location": {"X": 0.0, "Y": 0.0, "Z": 45.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "fov_angle": 90.0,
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "show_flag_settings": [unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)],
        },
    ]
}

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # Explicitly load "/SpComponents" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with -run=pythonscript
    # as opposed to -ExecutePythonScript.
    asset_registry.scan_paths_synchronous(["/SpComponents"])

    # remove existing blueprint
    blueprint_path = posixpath.join(blueprint_desc["blueprint_path"], blueprint_desc["blueprint_name"])
    if unreal.EditorAssetLibrary.does_asset_exist(blueprint_path):
        spear.log(f"Asset exists, removing: {blueprint_path}")
        success = unreal.EditorAssetLibrary.delete_asset(blueprint_path)
        assert success

    # create blueprint
    spear.log(f"Creating blueprint: {blueprint_path}")
    blueprint_asset, blueprint_subobject_descs = spear.editor_utils.create_blueprint(
        asset_name=blueprint_desc["blueprint_name"],
        package_path=blueprint_desc["blueprint_path"])

    # create SpStableNameComponent
    component_name = "sp_stable_name_component"
    spear.log(f"Creating component: ", component_name)
    sp_stable_name_component_desc = spear.editor_utils.add_new_subobject(
        blueprint_asset=blueprint_asset,
        parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create SpUpdateTransformComponent
    component_name = "sp_update_transform_component"
    spear.log(f"Creating component: ", component_name)
    sp_update_transform_component_desc = spear.editor_utils.add_new_subobject(
        blueprint_asset=blueprint_asset,
        parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
        subobject_name=component_name,
        subobject_class=unreal.SpUpdateTransformComponent)

    sp_update_transform_component = sp_update_transform_component_desc["object"]
    sp_update_transform_component.set_editor_property("source_component_path", "DefaultSceneRoot.sphere_")
    sp_update_transform_component.set_editor_property("destination_component_path", "DefaultSceneRoot")
    sp_update_transform_component.set_editor_property("set_world_location", True)

    # create SpBasicKeyboardControlComponent
    component_name = "sp_basic_keyboard_control_component"
    spear.log(f"Creating component: ", component_name)
    sp_basic_keyboard_control_component_desc = spear.editor_utils.add_new_subobject(
        blueprint_asset=blueprint_asset,
        parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
        subobject_name="sp_basic_keyboard_control_component",
        subobject_class=unreal.SpBasicKeyboardControlComponent)

    sp_basic_keyboard_control_component = sp_basic_keyboard_control_component_desc["object"]
    sp_basic_keyboard_control_component.set_editor_property("add_rotation_component_path", "DefaultSceneRoot")
    sp_basic_keyboard_control_component.set_editor_property("add_force_target_component_path", "DefaultSceneRoot.sphere_")
    sp_basic_keyboard_control_component.set_editor_property("add_force_rotation_component_path", "DefaultSceneRoot")

    sp_user_input_component = sp_basic_keyboard_control_component.get_editor_property("sp_user_input_component")
    sp_user_input_component.set_editor_property("handle_user_input", True)

    #
    # create StaticMeshComponents
    #

    for component_desc in blueprint_desc["static_mesh_component_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])
        static_mesh_component_desc = spear.editor_utils.add_new_subobject(
            blueprint_asset=blueprint_asset,
            parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
            subobject_name=component_desc["name"],
            subobject_class=unreal.StaticMeshComponent)

        static_mesh_component = static_mesh_component_desc["object"]
        static_mesh = unreal.load_asset(component_desc["static_mesh_path"])
        material = unreal.load_asset(component_desc["material_path"])

        static_mesh_component.set_editor_property("static_mesh", static_mesh)
        static_mesh_component.set_material(0, material)

        static_mesh_component.set_relative_location(
            new_location=unreal.Vector(component_desc["location"]["X"], component_desc["location"]["Y"], component_desc["location"]["Z"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["Pitch"], yaw=component_desc["rotation"]["Yaw"], roll=component_desc["rotation"]["Roll"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_scale3d(
            new_scale3d=unreal.Vector(component_desc["scale3d"]["X"], component_desc["scale3d"]["Y"], component_desc["scale3d"]["Z"]))

        if "simulate_physics" in component_desc:
            static_mesh_component.set_simulate_physics(component_desc["simulate_physics"])

        if "collision_profile_name" in component_desc:
            static_mesh_component.set_collision_profile_name(component_desc["collision_profile_name"])

    #
    # create SpSceneCaptureComponent2Ds
    #

    for component_desc in blueprint_desc["sp_scene_capture_component_2d_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])
        sp_scene_capture_component_2d_desc = spear.editor_utils.add_new_subobject(
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

        sp_scene_capture_component_2d.set_relative_location(
            new_location=unreal.Vector(component_desc["location"]["X"], component_desc["location"]["Y"], component_desc["location"]["Z"]), sweep=False, teleport=False)
        sp_scene_capture_component_2d.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["Pitch"], yaw=component_desc["rotation"]["Yaw"], roll=component_desc["rotation"]["Roll"]), sweep=False, teleport=False)

        # SpSceneCaptureComponent2D properties (optional)

        if "material_path" in component_desc:
            material = unreal.load_asset(component_desc["material_path"])
            sp_scene_capture_component_2d.set_editor_property("material", material)

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
