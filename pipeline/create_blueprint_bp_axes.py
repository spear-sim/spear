#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import posixpath
import spear
import unreal


blueprint_desc = \
{
    "blueprint_name": "BP_Axes",
    "blueprint_path": "/SpComponents/Blueprints",
    "component_descs":
    [
        {
            "name": "origin",
            "static_mesh_path": "/Engine/BasicShapes/Sphere.Sphere",
            "material_path": "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial",
            "location": {"X": 0.0, "Y": 0.0, "Z": 0.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.05, "Y": 0.05, "Z": 0.05}
        },
        {
            "name": "x_axis",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Red.MI_BasicShapeMaterial_Inst_Red",
            "location": {"X": 12.5, "Y": 0.0, "Z": 0.0},
            "rotation": {"Pitch": 90.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25}
        },
        {
            "name": "y_axis",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Green.MI_BasicShapeMaterial_Inst_Green",
            "location": {"X": 0.0, "Y": 12.5, "Z": 0.0},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 90.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25}
        },
        {
            "name": "z_axis",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpComponents/Materials/MI_BasicShapeMaterial_Inst_Blue.MI_BasicShapeMaterial_Inst_Blue",
            "location": {"X": 0.0, "Y": 0.0, "Z": 12.5},
            "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
            "scale3d": {"X": 0.025, "Y": 0.025, "Z": 0.25}
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

    # create StaticMeshComponents
    for component_desc in blueprint_desc["component_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])

        add_new_subobject_params = unreal.AddNewSubobjectParams(
            parent_handle=scene_component_subobject_data_handle,
            new_class=unreal.StaticMeshComponent,
            blueprint_context=blueprint_asset)
        static_mesh_component_subobject_data_handle, fail_reason = subobject_data_subsystem.add_new_subobject(add_new_subobject_params)
        assert fail_reason.is_empty()
        subobject_data_subsystem.rename_subobject(static_mesh_component_subobject_data_handle, unreal.Text(component_desc["name"]))

        static_mesh_component_subobject_data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(static_mesh_component_subobject_data_handle)
        static_mesh_component = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(static_mesh_component_subobject_data)
        assert isinstance(static_mesh_component, unreal.StaticMeshComponent)

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

    spear.log(f"Saving blueprint: {blueprint_path}")

    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log(f"Done.")
