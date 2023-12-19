import argparse
import glob
import json
import os

import unreal

KUJIALE_MATERIAL_FIELD_MAPPING = {
    "color": "BaseColor",
    "gloss": "Gloss",
    "metallic": "Metallic",
    "specular": "Specular",
    "emission": "Emissive",
}


def import_asset(src, dst):
    source_data = unreal.InterchangeManager.create_source_data(src)
    import_asset_parameters = unreal.ImportAssetParameters()
    import_asset_parameters.is_automated = True

    ic_mng = unreal.InterchangeManager.get_interchange_manager_scripted()
    ic_mng.import_asset(dst, source_data, import_asset_parameters)


def import_material_asset(material_id, material_json, scene_dir):
    AssetTools = unreal.AssetToolsHelpers.get_asset_tools()

    mi_asset_path = f"{scene_dir}/Materials/{material_id}/MI_{material_id}"
    if unreal.EditorAssetLibrary.does_asset_exist(mi_asset_path):
        mi_asset = unreal.EditorAssetLibrary.load_asset(mi_asset_path)
    else:
        mi_asset = AssetTools.create_asset(f"MI_{material_id}", f"{scene_dir}/Materials/{material_id}", unreal.MaterialInstanceConstant,
                                           unreal.MaterialInstanceConstantFactoryNew())

    mtl = material_json["materials"][material_id]

    if mtl['alpha'] != 1.0:
        base_mtl = unreal.EditorAssetLibrary.find_asset_data("/Game/Kujiale/Materials/M_TranslucentMaterial")
    else:
        base_mtl = unreal.EditorAssetLibrary.find_asset_data("/Game/Kujiale/Materials/M_BaseMaterial")
    unreal.MaterialEditingLibrary.set_material_instance_parent(mi_asset, base_mtl.get_asset())  # set parent material

    for mtl_field, ue_field in KUJIALE_MATERIAL_FIELD_MAPPING.items():
        if mtl_field in mtl:
            value = mtl[mtl_field]
            if isinstance(value, list):
                unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(mi_asset, f"{ue_field}_Color", unreal.LinearColor(value[0], value[1], value[2]))
            elif isinstance(value, float):
                unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(mi_asset, f"{ue_field}_Color", unreal.LinearColor(value, value, value))
            else:
                unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(mi_asset, f"Is{ue_field}Tex", 1)
                new_texture = unreal.EditorAssetLibrary.load_asset(f"{scene_dir}/Materials/{material_id}/T_{material_id}_{mtl_field}")
                unreal.MaterialEditingLibrary.set_material_instance_texture_parameter_value(mi_asset, f"{ue_field}_Tex", new_texture)
    if "emissionIntensity" in mtl:
        unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(mi_asset, "EmissiveIntensity", mtl["emissionIntensity"])
    if mtl['alpha'] != 1.0:
        unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(mi_asset, "Opacity", mtl["alpha"])


def create_component(parent_handle, actor_type):
    so_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    sub_handle, fail_reason = so_subsystem.add_new_subobject(unreal.AddNewSubobjectParams(
        parent_handle=parent_handle,
        new_class=actor_type,
    ))
    blueprint_function_library = unreal.SubobjectDataBlueprintFunctionLibrary
    new_component = blueprint_function_library.get_object(blueprint_function_library.get_data(sub_handle))
    return new_component


def spawn_actor(mesh_id, material_references, scene_dir):
    new_actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))

    new_actor.set_folder_path("Meshes/0000_unknown")

    SM_Dummy = unreal.EditorAssetLibrary.load_asset("/Game/Kujiale/Meshes/SM_Dummy")
    new_actor.root_component.set_editor_property("static_mesh", SM_Dummy)
    new_actor.root_component.set_editor_property("use_default_collision", True)
    new_actor.root_component.set_editor_property("can_ever_affect_navigation", False)
    new_actor.root_component.set_editor_property("relative_scale3d", unreal.Vector(0.1, 0.1, 0.1))

    so_subsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    root_component_handle = so_subsystem.k2_gather_subobject_data_for_instance(new_actor)[0]

    component_index = 0
    for component_name, material_id in material_references.items():
        static_mesh_asset = unreal.EditorAssetLibrary.load_asset(f"{scene_dir}/Meshes/Clutter/{mesh_id}/{component_name}")
        material_instance = unreal.EditorAssetLibrary.load_asset(f"{scene_dir}/materials/{material_id}/MI_{material_id}")

        new_component = create_component(root_component_handle, unreal.StaticMeshComponent)
        new_component.set_static_mesh(static_mesh_asset)
        new_component.set_material(0, material_instance)
        new_component.rename(f"mesh_{component_index:04d}")
        component_index += 1


def generate_kujiale_asset(mesh_id, data_dir, scene_dir):
    # import static mesh
    import_asset(f"{data_dir}/{mesh_id}/model.obj", f"{scene_dir}/Meshes/Clutter/{mesh_id}")

    # import material instances and textures
    for material_folder in glob.glob(f"{data_dir}/{mesh_id}/materials/*/"):
        material_id = os.path.basename(os.path.dirname(material_folder))
        for texture in glob.glob(f"{material_folder}/PBRtextures/*"):
            texture_base_name = os.path.basename(texture)
            if texture_base_name.startswith("T_") and "diffuse" not in texture_base_name:
                import_asset(texture, f"{scene_dir}/Materials/{material_id}/")
        import_material_asset(material_id, json.load(open(f"{material_folder}/material.json")), scene_dir)

    # spawn StaticMeshActor in current scene
    spawn_actor(mesh_id, json.load(open(f"{data_dir}/{mesh_id}/reference.json", mode='r')), scene_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh_id", default="ZDB6BFJVAIM7CPTUKE888888", help="kujiale mesh_id")
    parser.add_argument("--data_dir", default="F:/intel/spearsim-cleanup/download_new_asset/data", help='local data directory')
    parser.add_argument("--scene_dir", default="/Game/Scenes/Kujiale_0005", help="unreal scene directory")
    args = parser.parse_args()

    generate_kujiale_asset(args.mesh_id, data_dir=args.data_dir, scene_dir=args.scene_dir)
