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
    "blueprint_name": "BP_Axes",
    "blueprint_dir": "/SpContent/Blueprints",
    "component_descs":
    [
        {
            "name": "origin_",
            "static_mesh_path": "/Engine/BasicShapes/Sphere.Sphere",
            "material_path": "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial",
            "location": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.05, "y": 0.05, "z": 0.05}
        },
        {
            "name": "x_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Red.MI_BasicShapeMaterial_Inst_Red",
            "location": {"x": 12.5, "y": 0.0, "z": 0.0},
            "rotation": {"pitch": 90.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25}
        },
        {
            "name": "y_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Green.MI_BasicShapeMaterial_Inst_Green",
            "location": {"x": 0.0, "y": 12.5, "z": 0.0},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 90.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25}
        },
        {
            "name": "z_axis_",
            "static_mesh_path": "/Engine/BasicShapes/Cylinder.Cylinder",
            "material_path": "/SpContent/Materials/MI_BasicShapeMaterial_Inst_Blue.MI_BasicShapeMaterial_Inst_Blue",
            "location": {"x": 0.0, "y": 0.0, "z": 12.5},
            "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "scale3d": {"x": 0.025, "y": 0.025, "z": 0.25}
        }
    ]
}

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
subobject_data_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)


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
    assert isinstance(blueprint_subobject_descs[1]["object"], unreal.SceneComponent) # the 1st entry always refers to the actor's root component

    # create SpStableNameComponent
    component_name = "sp_stable_name_component_"
    spear.log("Creating component: ", component_name)
    parent_data_handle = blueprint_subobject_descs[0]["data_handle"] # actor
    sp_stable_name_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
        blueprint_asset=blueprint_asset,
        parent_data_handle=parent_data_handle,
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create StaticMeshComponents
    for component_desc in blueprint_desc["component_descs"]:

        spear.log("Creating component: ", component_desc["name"])
        parent_data_handle = blueprint_subobject_descs[1]["data_handle"] # root component
        static_mesh_component_desc = spear.utils.editor_utils.add_new_subobject_to_blueprint_asset(
            blueprint_asset=blueprint_asset,
            parent_data_handle=parent_data_handle,
            subobject_name=component_desc["name"],
            subobject_class=unreal.StaticMeshComponent)

        static_mesh = unreal.load_asset(name=component_desc["static_mesh_path"])
        material = unreal.load_asset(name=component_desc["material_path"])

        static_mesh_component = static_mesh_component_desc["object"]
        static_mesh_component.set_editor_property(name="static_mesh", value=static_mesh)
        static_mesh_component.set_material(element_index=0, material=material)

        static_mesh_component.set_relative_location(
            new_location=unreal.Vector(x=component_desc["location"]["x"], y=component_desc["location"]["y"], z=component_desc["location"]["z"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["pitch"], yaw=component_desc["rotation"]["yaw"], roll=component_desc["rotation"]["roll"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_scale3d(
            new_scale3d=unreal.Vector(x=component_desc["scale3d"]["x"], y=component_desc["scale3d"]["y"], z=component_desc["scale3d"]["z"]))

    # save blueprint
    spear.log("Saving blueprint: ", blueprint_path)
    editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    spear.log("Done.")
