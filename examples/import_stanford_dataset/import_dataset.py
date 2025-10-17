#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import pandas as pd
import posixpath
import shutil
import spear
import spear.utils.editor_utils
import unreal


mesh_descs = \
[
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "bunny", "mesh.gltf")),
        "static_mesh_name": "SM_Bunny",
        "static_mesh_dir": "/Game/Stanford/Meshes",
        "material_name": "M_Metal_Rust",
        "material_dir": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Bunny",
        "blueprint_dir": "/Game/Stanford/Blueprints"
    },
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "happy", "mesh.gltf")),
        "static_mesh_name": "SM_Happy",
        "static_mesh_dir": "/Game/Stanford/Meshes",
        "material_name": "M_Ground_Grass",
        "material_dir": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Happy",
        "blueprint_dir": "/Game/Stanford/Blueprints",
    },
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "dragon", "mesh.gltf")),
        "static_mesh_name": "SM_Dragon",
        "static_mesh_dir": "/Game/Stanford/Meshes",
        "material_name": "M_Basic_Floor",
        "material_dir": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Dragon",
        "blueprint_dir": "/Game/Stanford/Blueprints",
    }
]

content_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "..", "cpp", "unreal_projects", "SpearSim", "Content"))
cook_dirs = [os.path.join(content_dir, "Stanford")]
include_assets = [os.path.join("SpearSim", "Content", "Stanford", "**", "*.*")]

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # create output dir
    csv_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "csv"))
    if os.path.exists(csv_dir):
        spear.log("Directory exists, removing: ", csv_dir)
        shutil.rmtree(csv_dir, ignore_errors=True)
    os.makedirs(csv_dir, exist_ok=True)

    # import data into Unreal project
    for mesh_desc in mesh_descs:

        static_mesh_tmp_path = posixpath.join(mesh_desc["static_mesh_dir"], "mesh")
        static_mesh_path = posixpath.join(mesh_desc["static_mesh_dir"], mesh_desc["static_mesh_name"])

        # import mesh, note that asset_import_task.destination_name seems to be ignored, so we need to rename after import
        spear.log(f"Importing {mesh_desc['gltf_file']} to {static_mesh_tmp_path}...")

        asset_import_task = unreal.AssetImportTask()
        asset_import_task.set_editor_property(name="async_", value=False)
        asset_import_task.set_editor_property(name="automated", value=True)
        asset_import_task.set_editor_property(name="destination_path", value=mesh_desc["static_mesh_dir"])
        asset_import_task.set_editor_property(name="filename", value=mesh_desc["gltf_file"])
        asset_import_task.set_editor_property(name="replace_existing", value=True)
        asset_import_task.set_editor_property(name="replace_existing_settings", value=True)
        asset_import_task.set_editor_property(name="save", value=True)

        asset_tools.import_asset_tasks(import_tasks=[asset_import_task])

        # rename
        if unreal.EditorAssetLibrary.does_asset_exist(asset_path=static_mesh_path):
            spear.log("Asset exists, removing: ", static_mesh_path)
            success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=static_mesh_path)
            assert success

        spear.log(f"Renaming asset: {static_mesh_tmp_path} -> {static_mesh_path}")

        asset = unreal.load_asset(name=static_mesh_tmp_path)
        assert asset is not None

        asset_rename_data = unreal.AssetRenameData()
        asset_rename_data.set_editor_property(name="asset", value=asset)
        asset_rename_data.set_editor_property(name="new_name", value=mesh_desc["static_mesh_name"])
        asset_rename_data.set_editor_property(name="new_package_path", value=mesh_desc["static_mesh_dir"])

        asset_tools.rename_assets(assets_and_names=[asset_rename_data])

        # remove existing blueprint
        blueprint_path = posixpath.join(mesh_desc["blueprint_dir"], mesh_desc["blueprint_name"])
        if unreal.EditorAssetLibrary.does_asset_exist(asset_path=blueprint_path):
            spear.log("Asset exists, removing: ", blueprint_path)
            success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=blueprint_path)
            assert success

        # create blueprint
        spear.log("Creating blueprint: ", blueprint_path)
        blueprint_asset = spear.utils.editor_utils.create_blueprint_asset(
            asset_name=mesh_desc["blueprint_name"],
            package_dir=mesh_desc["blueprint_dir"],
            parent_class=unreal.StaticMeshActor)

        blueprint_subobject_descs = spear.utils.editor_utils.get_subobject_descs_for_blueprint_asset(blueprint_asset=blueprint_asset)
        assert len(blueprint_subobject_descs) == 2
        assert isinstance(blueprint_subobject_descs[0]["object"], unreal.StaticMeshActor)     # the 0th entry always refers to the actor itself
        assert isinstance(blueprint_subobject_descs[1]["object"], unreal.StaticMeshComponent) # the 1st entry always refers to the actor's root component

        # we could add new components here by calling spear.utils.editor_utils.add_new_subobject(...) but in this case it isn't necessary

        actor = blueprint_subobject_descs[0]["object"]
        static_mesh_component = blueprint_subobject_descs[1]["object"]
        static_mesh_path = posixpath.join(mesh_desc["static_mesh_dir"], mesh_desc['static_mesh_name'])
        static_mesh = unreal.load_asset(name=static_mesh_path)
        material_path = posixpath.join(mesh_desc["material_dir"], mesh_desc['material_name'])
        material = unreal.load_asset(name=material_path)
        static_mesh_component.set_editor_property(name="static_mesh", value=static_mesh)
        static_mesh_component.set_material(element_index=0, material=material)

        # actor.get_actor_bounds(...) returns an empty bounding box here, so we use the static_mesh_component.get_local_bounds(...) instead
        bounds_min, bounds_max = static_mesh_component.get_local_bounds()
        bounds_origin = (bounds_max + bounds_min) / 2.0
        bounds_half_extent = (bounds_max - bounds_min) / 2.0
        pivot_offset = unreal.Vector(x=bounds_origin.x, y=bounds_origin.y, z=bounds_origin.z - bounds_half_extent.z)
        actor.set_editor_property(name="pivot_offset", value=pivot_offset)

        spear.log("Saving blueprint: ", blueprint_path)
        editor_asset_subsystem.save_loaded_asset(asset_to_save=blueprint_asset)

    # create cook dirs file
    cook_dirs_file = os.path.realpath(os.path.join(csv_dir, "cook_dirs.csv"))
    spear.log("Writing cook dirs file: :", cook_dirs_file)
    df = pd.DataFrame(columns=["cook_dirs"], data={"cook_dirs": cook_dirs})
    df.to_csv(cook_dirs_file, index=False)

    # create include assets file
    include_assets_file = os.path.realpath(os.path.join(csv_dir, "include_assets.csv"))
    spear.log("Writing include assets file: ", include_assets_file)
    df = pd.DataFrame(columns=["include_assets"], data={"include_assets": include_assets})
    df.to_csv(include_assets_file, index=False)

    spear.log("Done.")
