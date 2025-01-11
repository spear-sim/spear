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
            "translation": {
                "x": df[transform_name + "_translation_x"],
                "y": df[transform_name + "_translation_y"],
                "z": df[transform_name + "_translation_z"]},
            "rotation": {
                "x": df[transform_name + "_rotation_x"],
                "y": df[transform_name + "_rotation_y"],
                "z": df[transform_name + "_rotation_z"],
                "w": df[transform_name + "_rotation_w"]},
            "scale3D": {
                "x": df[transform_name + "_scale3D_x"],
                "y": df[transform_name + "_scale3D_y"],
                "z": df[transform_name + "_scale3D_z"]}}
        transforms[transform_name] = transform
    return transforms


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--transforms_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "transforms.csv")))
    args = parser.parse_args()

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    df = pd.read_csv(args.transforms_file)

    with instance.begin_frame():

        # We could use get_static_class(...) here to obtain a handle to the UClass object for each Unreal
        # type, as we do in our other examples. But for demonstration purposes, we use load_class(...)
        # instead. Using load_class(...) can be useful when the Unreal type has not been registered ahead of
        # time using the UnrealClassRegistrar system in our C++ code.

        # find functions
        actor_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Actor")
        set_actor_transform_func = instance.unreal_service.find_function_by_name(uclass=actor_uclass, function_name="K2_SetActorTransform")

        gameplay_statics_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.GameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")

        poseable_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.PoseableMeshComponent")
        get_bone_name_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetBoneName")
        get_num_bones_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetNumBones")
        set_bone_transform_by_name_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")
        set_skinned_asset_and_update_func = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkinnedAssetAndUpdate")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        # there is no point specifying a location and rotation yet, because the spawned actor won't have a
        # root scene component, so we will need to set its location later
        actor_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Actor")
        actor = instance.unreal_service.spawn_actor_from_uclass(
            uclass=actor_uclass,
            spawn_parameters={"Name": "character", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})

        # create UPoseableMeshComponent and setup skeletal mesh
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
        poseable_mesh_component = instance.unreal_service.create_scene_component_by_class_from_actor(
            scene_component_class=poseable_mesh_component_uclass,
            owner=actor,
            scene_component_name="poseable_mesh_component")
        instance.unreal_service.call_function(
            uobject=poseable_mesh_component,
            ufunction=set_skinned_asset_and_update_func,
            args={"NewMesh": spear.to_ptr(manny_simple_uobject)})

        # get bone names
        bone_names = []
        return_values = instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=get_num_bones_func)
        poseable_mesh_component_num_bones = return_values["ReturnValue"]
        for i in range(poseable_mesh_component_num_bones):
            return_values = instance.unreal_service.call_function(
                uobject=poseable_mesh_component,
                ufunction=get_bone_name_func,
                args={"BoneIndex": i})
            bone_name = return_values["ReturnValue"]
            print(bone_name)
            bone_names.append(bone_name)

    with instance.end_frame():
        pass

    for row in df.to_records():
        with instance.begin_frame():

            bone_transforms = get_transforms(row, bone_names)

            # even though we're setting bone transforms in world-space, we need to update the actor's
            # transform to avoid Unreal renderer from frustum culling the skeletal mesh
            root_transform = {}
            if "root" in bone_transforms.keys():
                root_transform = bone_transforms["root"]
            elif "Root" in bone_transforms.keys():
                root_transform = bone_transforms["Root"]
            instance.unreal_service.call_function(uobject=actor, ufunction=set_actor_transform_func, args={"NewTransform": root_transform})

            # set bone transforms
            for bone_name, transform in bone_transforms.items():
                result = instance.unreal_service.call_function(
                    uobject=poseable_mesh_component,
                    ufunction=set_bone_transform_by_name_func,
                    args={"BoneName": bone_name, "InTransform": transform, "BoneSpace": "WorldSpace"})

        with instance.end_frame():
            pass

    spear.log("Done.")
