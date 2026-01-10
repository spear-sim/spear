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
    "blueprint_name": "BP_SphereAgent",
    "blueprint_dir": "/SpContent/Blueprints",
    "static_mesh_component_descs":
    [
        {
            "name": "sphere_",
            "static_mesh_path": "/Engine/BasicShapes/Sphere.Sphere",
            "material_path": "/Game/StarterContent/Materials/M_Ceramic_Tile_Checker.M_Ceramic_Tile_Checker",
            "location": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.4, "y": 0.4, "z": 0.4},
            "simulate_physics": True
        },
        {
            "name": "camera_mesh_",
            "static_mesh_path": "/Engine/EditorMeshes/MatineeCam_SM.MatineeCam_SM",
            "material_path": "/Engine/EditorMaterials/MatineeCam_mat.MatineeCam_mat",
            "location": {"x": 0.0, "y": 0.0, "z": 35.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.25, "y": 0.25, "z": 0.25}
        },
        {
            "name": "x_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Red.MI_BasicShapeMaterial_Inst_Red",
            "location": {"x": 12.5, "y": 0.0, "z": 0.0},
            "rotation": {"pitch": 90.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25},
            "collision_profile_name": "NoCollision"
        },
        {
            "name": "y_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Green.MI_BasicShapeMaterial_Inst_Green",
            "location": {"x": 0.0, "y": 12.5, "z": 0.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 90.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25},
            "collision_profile_name": "NoCollision"
        },
        {
            "name": "z_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Blue.MI_BasicShapeMaterial_Inst_Blue",
            "location": {"x": 0.0, "y": 0.0, "z": 12.5},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25},
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
            "fov_angle": 90.0,
            "channel_data_type": unreal.SpArrayDataType.U_INT8,
            "texture_render_target_format": unreal.TextureRenderTargetFormat.RTF_RGBA8_SRGB,
            "capture_source": unreal.SceneCaptureSource.SCS_FINAL_TONE_CURVE_HDR,
            "location": {"x": 0.0, "y": 0.0, "z": 45.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "dynamic_global_illumination_method": unreal.DynamicGlobalIlluminationMethod.LUMEN,
            "reflection_method": unreal.ReflectionMethod.LUMEN,
            "primitive_render_mode": unreal.SceneCapturePrimitiveRenderMode.PRM_RENDER_SCENE_PRIMITIVES,
            "show_flag_settings": [unreal.EngineShowFlagsSetting(show_flag_name="TemporalAA", enabled=True)],
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

    # create SpUpdateTransformComponent
    component_name = "sp_update_transform_component_"
    spear.log("Creating component: ", component_name)
    parent_data_handle = blueprint_subobject_descs[0]["data_handle"] # actor
    sp_update_transform_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
        blueprint_asset=blueprint_asset,
        parent_data_handle=parent_data_handle,
        subobject_name=component_name,
        subobject_class=unreal.SpUpdateTransformComponent)

    sp_update_transform_component = sp_update_transform_component_desc["object"]
    sp_update_transform_component.set_editor_property(name="source_component_path", value="DefaultSceneRoot.sphere_")
    sp_update_transform_component.set_editor_property(name="destination_component_path", value="DefaultSceneRoot")
    sp_update_transform_component.set_editor_property(name="set_world_location", value=True)

    # create SpBasicKeyboardControlComponent
    component_name = "sp_basic_keyboard_control_component_"
    spear.log("Creating component: ", component_name)
    parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
    sp_basic_keyboard_control_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
        blueprint_asset=blueprint_asset,
        parent_data_handle=parent_data_handle,
        subobject_name=component_name,
        subobject_class=unreal.SpBasicKeyboardControlComponent)

    sp_basic_keyboard_control_component = sp_basic_keyboard_control_component_desc["object"]
    sp_basic_keyboard_control_component.set_editor_property(name="add_rotation_component_path", value="DefaultSceneRoot")
    sp_basic_keyboard_control_component.set_editor_property(name="add_force_target_component_path", value="DefaultSceneRoot.sphere_")
    sp_basic_keyboard_control_component.set_editor_property(name="add_force_rotation_component_path", value="DefaultSceneRoot")

    sp_user_input_component = sp_basic_keyboard_control_component.get_editor_property(name="sp_user_input_component")
    sp_user_input_component.set_editor_property(name="handle_user_input", value=True)

    #
    # create StaticMeshComponents
    #

    for component_desc in blueprint_desc["static_mesh_component_descs"]:

        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        static_mesh_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
            blueprint_asset=blueprint_asset,
            parent_data_handle=parent_data_handle,
            subobject_name=component_desc["name"],
            subobject_class=unreal.StaticMeshComponent)

        static_mesh_component = static_mesh_component_desc["object"]
        static_mesh = unreal.load_asset(name=component_desc["static_mesh_path"])
        material = unreal.load_asset(name=component_desc["material_path"])

        static_mesh_component.set_editor_property(name="static_mesh", value=static_mesh)
        static_mesh_component.set_material(element_index=0, material=material)

        static_mesh_component.set_relative_location(
            new_location=unreal.Vector(x=component_desc["location"]["x"], y=component_desc["location"]["y"], z=component_desc["location"]["z"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["pitch"], yaw=component_desc["rotation"]["yaw"], roll=component_desc["rotation"]["roll"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_scale3d(
            new_scale3d=unreal.Vector(x=component_desc["scale3d"]["x"], y=component_desc["scale3d"]["y"], z=component_desc["scale3d"]["z"]))

        if "simulate_physics" in component_desc:
            static_mesh_component.set_simulate_physics(component_desc["simulate_physics"])

        if "collision_profile_name" in component_desc:
            static_mesh_component.set_collision_profile_name(component_desc["collision_profile_name"])

    #
    # create SpSceneCaptureComponent2Ds
    #

    for component_desc in blueprint_desc["sp_scene_capture_component_2d_descs"]:

        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        sp_scene_capture_component_2d_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
            blueprint_asset=blueprint_asset,
            parent_data_handle=parent_data_handle,
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
            new_location=unreal.Vector(component_desc["location"]["x"], component_desc["location"]["y"], component_desc["location"]["z"]), sweep=False, teleport=False)
        sp_scene_capture_component_2d.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["pitch"], yaw=component_desc["rotation"]["yaw"], roll=component_desc["rotation"]["roll"]), sweep=False, teleport=False)

        # SpSceneCaptureComponent2D properties (optional)

        if "material_path" in component_desc:
            material = unreal.load_asset(name=component_desc["material_path"])
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

    # compile blueprint
    unreal.BlueprintEditorLibrary.compile_blueprint(blueprint=blueprint_asset)

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log("Done.")
