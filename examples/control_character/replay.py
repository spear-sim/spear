#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import pandas as pd
import spear


def get_transforms(df, transform_names):
    transforms = {}
    for transform_name in transform_names:
        transform = {
            "translation": {"x": df[transform_name + "_translation_x"], "y": df[transform_name + "_translation_y"], "z": df[transform_name + "_translation_z"]},
            "rotation": {"x": df[transform_name + "_rotation_x"], "y": df[transform_name + "_rotation_y"], "z": df[transform_name + "_rotation_z"],
                         "w": df[transform_name + "_rotation_w"]},
            "scale3D": {"x": df[transform_name + "_scale3D_x"], "y": df[transform_name + "_scale3D_y"], "z": df[transform_name + "_scale3D_z"]}}
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

        # we could use get_static_class(...) here, as we do in our other examples, but we want to include
        # an example of how load_class(...) can be used to accomplish the same thing

        # find functions
        gameplay_statics_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.GameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")

        poseable_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.PoseableMeshComponent")
        set_skinned_asset_and_update_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkinnedAssetAndUpdate")
        set_bone_transform_by_name_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")
        get_num_bones_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetNumBones")
        get_bone_name_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetBoneName")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        actor_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Actor")
        poseable_mesh_actor = instance.unreal_service.spawn_actor_from_uclass(
            uclass=actor_uclass,
            location={"X": 0.0, "Y": 100.0, "Z": 150.0},
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "character", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})

        # create UPoseableMeshComponent and setup skeletal mesh
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
        poseable_mesh_component = instance.unreal_service.create_component_on_actor_by_class(
            owner=poseable_mesh_actor,
            component_class=poseable_mesh_component_uclass,
            component_name="poseable_mesh_component")
        instance.unreal_service.call_function(
            uobject=poseable_mesh_component,
            ufunction=set_skinned_asset_and_update_func,
            args={"NewMesh": spear.to_ptr(manny_simple_uobject)})

        bone_names = []
        return_values = instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=get_num_bones_func)
        poseable_mesh_component_num_bones = return_values['ReturnValue']
        for i in range(poseable_mesh_component_num_bones):
            return_values = instance.unreal_service.call_function(
                uobject=poseable_mesh_component,
                ufunction=get_bone_name_func,
                args={"BoneIndex": i})
            bone_name = return_values['ReturnValue']
            bone_names.append(bone_name)

    with instance.end_frame():
        pass

    df = pd.read_csv(args.actions_file)
    for row in df.to_records():
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            bone_transforms = get_transforms(row, bone_names)
            for bone_name, transform in bone_transforms.items():
                result = instance.unreal_service.call_function(
                    uobject=poseable_mesh_component,
                    ufunction=set_bone_transform_by_name_func,
                    args={"BoneName": bone_name, "InTransform": transform, "BoneSpace": "WorldSpace"})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    spear.log("Done.")
