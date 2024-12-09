#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import numpy as np
import pandas as pd
import spear


def get_transforms(df, transform_names):
    transforms = {}
    for transform_name in transform_names:
        transform = {
            "translation": {"X": df[transform_name + "_location_x"], "Y": df[transform_name + "_location_y"], "Z": df[transform_name + "_location_z"]},
            "rotation": {"x": df[transform_name + "_rotation_x"], "y": df[transform_name + "_rotation_y"], "z": df[transform_name + "_rotation_z"],
                         "w": df[transform_name + "_rotation_w"]},
            "scale3D": {"X": df[transform_name + "_scale3D_x"], "Y": df[transform_name + "_scale3D_y"], "Z": df[transform_name + "_scale3D_z"]}}
        transforms[transform_name] = transform
    return transforms


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():
        gameplay_statics_uclass = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        poseable_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.PoseableMeshComponent")
        set_skinned_asset_and_update_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkinnedAssetAndUpdate")
        set_bone_transform_by_name_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")

        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
        actor_uclass = instance.unreal_service.get_static_class(class_name="AActor")
        poseable_mesh_actor = instance.unreal_service.spawn_actor_from_uclass(
            uclass=actor_uclass,
            location={"X": 0.0, "Y": 100.0, "Z": 150.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "third_person_actor", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

        # create UPoseableMeshComponent and setup skeletal mesh
        poseable_mesh_component = instance.unreal_service.create_component_on_actor_by_class(
            owner=poseable_mesh_actor,
            component_class=poseable_mesh_component_uclass,
            component_name="poseable_mesh_component")
        instance.unreal_service.call_function(
            uobject=poseable_mesh_component,
            ufunction=set_skinned_asset_and_update_func,
            args={"NewMesh": spear.func_utils.to_ptr(manny_simple_uobject)})

    with instance.end_frame():
        pass

    df = pd.read_csv(args.actions_file)
    for row in df.to_records(index=False):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            actions = get_transforms(row, )
            for bone_name, transform in actions.items():
                result = instance.unreal_service.call_function(
                    uobject=poseable_mesh_component,
                    ufunction=set_bone_transform_by_name_func,
                    args={"BoneName": bone_name, "InTransform": transform, "BoneSpace": "WorldSpace"})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    spear.log("Done.")
