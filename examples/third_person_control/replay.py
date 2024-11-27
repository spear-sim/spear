import argparse
import os

import numpy as np
import pandas as pd
import spear


def get_action(row):
    bone_names = [name.split(".")[0] for name in row.dtype.names][::10]
    fields = [name[name.find(".") + 1:] for name in row.dtype.names][:10]
    data = np.array([row[name] for name in row.dtype.names], dtype=np.float64).reshape(-1, 10)
    bone_transforms = {}
    for bone_name, value in dict(zip(bone_names, data)).items():
        transform = {"rotation": {}, "translation": {}, "scale3D": {}}
        for key, value in dict(zip(fields, value)).items():
            field, axis = key.split(".")
            transform[field][axis] = value
        bone_transforms[bone_name] = transform
    return bone_transforms


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():
        gameplay_statics_uclass = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        aactor_uclass = instance.unreal_service.get_static_class(class_name="AActor")
        poseable_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.PoseableMeshComponent")

        set_game_paused_ufunc = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")
        set_skinned_asset_and_update_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkinnedAssetAndUpdate")
        set_bone_transform_by_name_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        poseable_mesh_actor = instance.unreal_service.spawn_actor_from_uclass(
            uclass=aactor_uclass,
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
            ufunction=set_skinned_asset_and_update_ufunc,
            args={"NewMesh": spear.func_utils.to_ptr(manny_simple_uobject)})

    with instance.end_frame():
        pass

    df = pd.read_csv(args.actions_file)
    for row in df.to_records(index=False):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": False})

            actions = get_action(row)
            for bone_name, transform in actions.items():
                result = instance.unreal_service.call_function(
                    uobject=poseable_mesh_component,
                    ufunction=set_bone_transform_by_name_ufunc,
                    args={
                        "BoneName": bone_name,
                        "InTransform": transform,
                        "BoneSpace": "WorldSpace"
                    })

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": True})

    print("Done.")
