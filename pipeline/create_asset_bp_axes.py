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
    blueprint_asset, blueprint_subobject_descs = spear.editor.create_blueprint(
        asset_name=blueprint_desc["blueprint_name"],
        package_path=blueprint_desc["blueprint_path"])

    # create SpStableNameComponent
    component_name = "sp_stable_name_component"
    spear.log(f"Creating component: ", component_name)
    sp_stable_name_component_desc = spear.editor.add_new_subobject(
        blueprint_asset=blueprint_asset,
        parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
        subobject_name=component_name,
        subobject_class=unreal.SpStableNameComponent)

    # create StaticMeshComponents
    for component_desc in blueprint_desc["component_descs"]:

        spear.log(f"Creating component: ", component_desc["name"])
        static_mesh_component_desc = spear.editor.add_new_subobject(
            blueprint_asset=blueprint_asset,
            parent_data_handle=blueprint_subobject_descs["root_component"]["data_handle"],
            subobject_name=component_desc["name"],
            subobject_class=unreal.StaticMeshComponent)

        static_mesh = unreal.load_asset(component_desc["static_mesh_path"])
        material = unreal.load_asset(component_desc["material_path"])

        static_mesh_component = static_mesh_component_desc["object"]
        static_mesh_component.set_editor_property("static_mesh", static_mesh)
        static_mesh_component.set_material(0, material)

        static_mesh_component.set_relative_location(
            new_location=unreal.Vector(component_desc["location"]["X"], component_desc["location"]["Y"], component_desc["location"]["Z"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_rotation(
            new_rotation=unreal.Rotator(pitch=component_desc["rotation"]["Pitch"], yaw=component_desc["rotation"]["Yaw"], roll=component_desc["rotation"]["Roll"]), sweep=False, teleport=False)
        static_mesh_component.set_relative_scale3d(
            new_scale3d=unreal.Vector(component_desc["scale3d"]["X"], component_desc["scale3d"]["Y"], component_desc["scale3d"]["Z"]))

    # save blueprint
    spear.log(f"Saving blueprint: {blueprint_path}")
    editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    spear.log("Done.")
