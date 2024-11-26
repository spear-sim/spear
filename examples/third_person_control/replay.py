import argparse
import json
import os

import numpy as np
import pandas as pd
import spear


def get_action(row):
    bone_names = [name.split(".")[0] for name in row.dtype.names][::10]
    fields = [name[name.find(".") + 1:] for name in row.dtype.names][:10]
    data = np.array([row[name] for name in row.dtype.names], dtype=np.float64).reshape(-1, 10)
    result = {}
    for bone_name, value in dict(zip(bone_names, data)).items():
        result[bone_name] = dict(zip(fields, value))
    return result


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
        bp_replay_actor_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Game/ThirdPerson/Blueprints/BP_posable.BP_posable_C")

        set_game_paused_ufunc = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")
        set_skeletal_mesh_asset_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkeletalMeshAsset")
        set_bone_trnasform_by_name_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")
        get_num_bones_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetNumBones")
        get_bone_name_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetBoneName")
        get_bone_transform_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetBoneTransform")
        set_bone_transform_by_name_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")
        set_bone_location_by_name_ufunc = instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneLocationByName")

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0,
                                                                   name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        bp_third_person_actor1 = instance.unreal_service.spawn_actor_from_uclass(
            uclass=bp_replay_actor_uclass,
            location={"X": 0.0, "Y": 100.0, "Z": 150.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "third_person_actor", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        print("bp_third_person_actor1", bp_third_person_actor1)
        # poseable_mesh_component = instance.unreal_service.create_component_on_actor2(bp_third_person_actor1, "poseable_mesh_component", poseable_mesh_component_uclass
        poseable_mesh_component = instance.unreal_service.get_component_by_class(bp_third_person_actor1, poseable_mesh_component_uclass)
        print("poseable_mesh_component", poseable_mesh_component)
        # instance.unreal_service.call_function(poseable_mesh_component, set_skeletal_mesh_asset_ufunc, {"NewMesh": spear.func_utils.to_ptr(manny_simple_uobject)})

        # get all bone names
        skeletal_mesh_component_num_bones1 = instance.unreal_service.call_function(poseable_mesh_component, get_num_bones_ufunc)['ReturnValue']
        print("skeletal_mesh_component_num_bones1", skeletal_mesh_component_num_bones1)
        actor1_bone_names = []
        for i in range(skeletal_mesh_component_num_bones1):
            bone_name = instance.unreal_service.call_function(poseable_mesh_component, get_bone_name_ufunc, {"BoneIndex": i})['ReturnValue']
            print("bone_name", bone_name)
            actor1_bone_names.append(bone_name)
    with instance.end_frame():
        pass
    df = json.load(open(args.actions_file))
    for row in df:
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": False})
            actions = row
            for bone_name, transform in actions.items():
                result = instance.unreal_service.call_function(poseable_mesh_component, set_bone_transform_by_name_ufunc, {
                    "BoneName": bone_name,
                    "BoneSpace": "WorldSpace",
                    "InTransform": transform
                })
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": True})
            pass
    print("Done.")
