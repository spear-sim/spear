#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import pandas as pd
import posixpath
import spear
import spear.editor
import unreal


mesh_descs = \
[
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "bunny", "mesh.gltf")),
        "static_mesh_name": "SM_Bunny",
        "static_mesh_path": "/Game/Stanford/Meshes",
        "material_name": "M_Metal_Rust",
        "material_path": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Bunny",
        "blueprint_path": "/Game/Stanford/Blueprints"
    },
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "happy", "mesh.gltf")),
        "static_mesh_name": "SM_Happy",
        "static_mesh_path": "/Game/Stanford/Meshes",
        "material_name": "M_Ground_Grass",
        "material_path": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Happy",
        "blueprint_path": "/Game/Stanford/Blueprints",
    },
    {
        "gltf_file": os.path.realpath(os.path.join(os.path.dirname(__file__), "gltf", "dragon", "mesh.gltf")),
        "static_mesh_name": "SM_Dragon",
        "static_mesh_path": "/Game/Stanford/Meshes",
        "material_name": "M_Basic_Floor",
        "material_path": "/Game/StarterContent/Materials",
        "blueprint_name": "BP_Dragon",
        "blueprint_path": "/Game/Stanford/Blueprints",
    }
]

content_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "..", "cpp", "unreal_projects", "SpearSim", "Content"))
cook_dirs = [os.path.join(content_dir, "Stanford")]
include_assets = [os.path.join("SpearSim", "Content", "Stanford", "**", "*.*")]

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # import data into Unreal project
    for mesh_desc in mesh_descs:

        static_mesh_tmp_path = posixpath.join(mesh_desc["static_mesh_path"], "mesh")
        static_mesh_path = posixpath.join(mesh_desc["static_mesh_path"], mesh_desc["static_mesh_name"])

        # import mesh, note that asset_import_task.destination_name seems to be ignored, so we need to rename after import
        spear.log(f"Importing {mesh_desc['gltf_file']} to {static_mesh_tmp_path}...")

        asset_import_task = unreal.AssetImportTask()
        asset_import_task.set_editor_property("async_", False)
        asset_import_task.set_editor_property("automated", True)
        asset_import_task.set_editor_property("destination_path", mesh_desc["static_mesh_path"])
        asset_import_task.set_editor_property("filename", mesh_desc["gltf_file"])
        asset_import_task.set_editor_property("replace_existing", True)
        asset_import_task.set_editor_property("replace_existing_settings", True)
        asset_import_task.set_editor_property("save", True)

        asset_tools.import_asset_tasks([asset_import_task])

        # rename
        if unreal.EditorAssetLibrary.does_asset_exist(static_mesh_path):
            spear.log(f"Asset exists, removing: {static_mesh_path}")
            success = unreal.EditorAssetLibrary.delete_asset(static_mesh_path)
            assert success

        spear.log(f"Renaming {static_mesh_tmp_path} to {static_mesh_path}")

        asset = unreal.load_asset(static_mesh_tmp_path)
        assert asset is not None

        asset_rename_data = unreal.AssetRenameData()
        asset_rename_data.set_editor_property("asset", asset)
        asset_rename_data.set_editor_property("new_name", mesh_desc["static_mesh_name"])
        asset_rename_data.set_editor_property("new_package_path", mesh_desc["static_mesh_path"])

        asset_tools.rename_assets([asset_rename_data])

        # remove existing blueprint
        blueprint_path = posixpath.join(mesh_desc["blueprint_path"], mesh_desc["blueprint_name"])
        if unreal.EditorAssetLibrary.does_asset_exist(blueprint_path):
            spear.log(f"Asset exists, removing: {blueprint_path}")
            success = unreal.EditorAssetLibrary.delete_asset(blueprint_path)
            assert success

        # create blueprint
        spear.log(f"Creating blueprint: {blueprint_path}")
        blueprint_asset, blueprint_subobject_descs = spear.editor.create_blueprint(
            asset_name=mesh_desc["blueprint_name"],
            package_path=mesh_desc["blueprint_path"],
            actor_class=unreal.StaticMeshActor)

        actor = blueprint_subobject_descs["actor"]["object"]
        assert isinstance(actor, unreal.StaticMeshActor)

        static_mesh_component = blueprint_subobject_descs["root_component"]["object"]
        assert isinstance(static_mesh_component, unreal.StaticMeshComponent)

        # we could add new components here by calling spear.editor.add_new_subobject(...) but in this case it isn't necessary

        static_mesh_path = posixpath.join(mesh_desc["static_mesh_path"], mesh_desc["static_mesh_name"] + "." + mesh_desc["static_mesh_name"])
        static_mesh = unreal.load_asset(static_mesh_path)

        material_path = posixpath.join(mesh_desc["material_path"], mesh_desc["material_name"] + "." + mesh_desc["material_name"])
        material = unreal.load_asset(material_path)

        static_mesh_component.set_editor_property("static_mesh", static_mesh)
        static_mesh_component.set_material(0, material)

        # actor.get_actor_bounds(...) returns an empty bounding box here, so we use the static_mesh_component.get_local_bounds(...) instead
        bounds_min, bounds_max = static_mesh_component.get_local_bounds()
        bounds_origin = (bounds_max + bounds_min) / 2.0
        bounds_half_extent = (bounds_max - bounds_min) / 2.0
        pivot_offset = unreal.Vector(bounds_origin.x, bounds_origin.y, bounds_origin.z - bounds_half_extent.z)
        actor.set_editor_property("pivot_offset", pivot_offset)

        spear.log(f"Saving blueprint: {blueprint_path}")
        editor_asset_subsystem.save_loaded_asset(blueprint_asset)

    # create cook dirs file
    cook_dirs_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "cook_dirs.csv"))
    spear.log(f"Writing cook dirs file: {cook_dirs_file}")
    df = pd.DataFrame(columns=["cook_dirs"], data={"cook_dirs": cook_dirs})
    df.to_csv(cook_dirs_file, index=False)

    # create include assets file
    include_assets_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "include_assets.csv"))
    spear.log(f"Writing include assets file: {include_assets_file}")
    df = pd.DataFrame(columns=["include_assets"], data={"include_assets": include_assets})
    df.to_csv(include_assets_file, index=False)

    spear.log(f"Done.")
