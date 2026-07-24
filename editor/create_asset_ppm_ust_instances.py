#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import posixpath
import spear
import unreal


material_dir = "/SpContent/Materials"
source_material_name_prefix = "PPM_"
material_instance_name_prefix = "MI_"
material_instance_name_suffix = "_UST"

asset_registry = unreal.AssetRegistryHelpers.get_asset_registry()
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
editor_asset_subsystem = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)


if __name__ == "__main__":

    # Explicitly load "/SpContent" into the asset registry, since it won't be loaded by default if we are
    # running as a commandlet, i.e., when the editor is invoked from the command-line with -run=pythonscript
    # as opposed to -ExecutePythonScript.
    asset_registry.scan_paths_synchronous(paths=["/SpContent"])

    # For each PPM_* post-process material, create a lightweight material instance that redirects the
    # material's output to a UserSceneTexture, so a single SpSceneCaptureComponent2D can read the buffer back.
    # We use a material instance rather than a full material copy because the override is resolved on the render
    # proxy at render time, so the instance reuses the parent material's compiled shader with no recompile, and
    # any future edit to the parent material is inherited automatically.
    for asset_path in unreal.EditorAssetLibrary.list_assets(directory_path=material_dir, recursive=False, include_folder=False):

        source_material_name = posixpath.split(asset_path)[1].split(".")[0]
        if not source_material_name.startswith(source_material_name_prefix):
            continue

        source_material_path = posixpath.join(material_dir, source_material_name)
        source_material = unreal.load_asset(name=source_material_path)
        assert isinstance(source_material, unreal.Material)

        # The UserSceneTexture output name is the source material name without the "PPM_" prefix (e.g.,
        # PPM_DiffuseColor -> DiffuseColor). This is the name that FSpUserSceneTextureMaterialDesc::InternalName
        # must match on the SPEAR side.
        user_scene_texture_name = source_material_name[len(source_material_name_prefix):]

        # The material instance keeps the full parent name so its parentage is obvious (e.g.,
        # PPM_DiffuseColor -> MI_PPM_DiffuseColor_UST).
        material_instance_name = material_instance_name_prefix + source_material_name + material_instance_name_suffix
        material_instance_path = posixpath.join(material_dir, material_instance_name)

        # remove existing material instance
        if unreal.EditorAssetLibrary.does_asset_exist(asset_path=material_instance_path):
            spear.log("Asset exists, removing: ", material_instance_path)
            success = unreal.EditorAssetLibrary.delete_asset(asset_path_to_delete=material_instance_path)
            assert success

        # create material instance
        spear.log("Creating material instance: ", material_instance_path)
        material_instance = asset_tools.create_asset(
            asset_name=material_instance_name,
            package_path=material_dir,
            asset_class=unreal.MaterialInstanceConstant,
            factory=unreal.MaterialInstanceConstantFactoryNew())
        assert isinstance(material_instance, unreal.MaterialInstanceConstant)

        # set the parent material
        unreal.MaterialEditingLibrary.set_material_instance_parent(instance=material_instance, new_parent=source_material)

        # Override the UserSceneTexture output. A UserSceneTextureOverride whose key is "None" overrides the
        # material's output, redirecting it from the default SceneColor output to the named UserSceneTexture.
        user_scene_texture_override = unreal.UserSceneTextureOverride()
        user_scene_texture_override.set_editor_property(name="key", value=unreal.Name("None"))
        user_scene_texture_override.set_editor_property(name="value", value=unreal.Name(user_scene_texture_name))
        material_instance.set_editor_property(name="user_scene_texture_overrides", value=[user_scene_texture_override])

        # The UserTextureDivisor parameter is (0, 0) by default and cannot be overriden by a material instance, however
        # that's not a problem because the renderer clamps it to >= 1 (PostProcessMaterial.cpp), so output is full-res.

        # update and save the material instance
        unreal.MaterialEditingLibrary.update_material_instance(instance=material_instance)
        spear.log("Saving material instance: ", material_instance_path)
        editor_asset_subsystem.save_loaded_asset(asset_to_save=material_instance)

    spear.log("Done.")
