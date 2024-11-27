#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os

import numpy as np
import pandas as pd

import spear


def get_data_frame(skeletal_mesh_transforms):
    columns = np.array([
        name + "." + field + "." + axis
        for name, transform in skeletal_mesh_transforms.items()
        for field, vector in transform.items()
        for axis, val in vector.items()
    ]).ravel()
    data = np.array([
        value
        for name, transform in skeletal_mesh_transforms.items()
        for field, vector in transform.items()
        for axis, value in vector.items()
    ]).reshape(1, -1)
    return pd.DataFrame(columns=columns, data=data)


if __name__ == "__main__":
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
        controller_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0,
                                                               name="/Script/Engine.Controller")
        bp_third_person_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0,
                                                                    name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
        character_movement_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0,
                                                                                 name="/Script/Engine.CharacterMovementComponent")
        skeletal_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0,
                                                                            name="/Script/Engine.SkeletalMeshComponent")

        set_game_paused_ufunc = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")
        set_movement_mode_ufunc = instance.unreal_service.find_function_by_name(uclass=character_movement_component_uclass, function_name="SetMovementMode")
        set_skeletal_mesh_asset_ufunc = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="SetSkeletalMeshAsset")
        get_num_bones_ufunc = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetNumBones")
        get_bone_name_ufunc = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneName")
        get_bone_transform_ufunc = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneTransform")

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
        bp_third_person_actor1 = instance.unreal_service.spawn_actor_from_uclass(
            uclass=bp_third_person_uclass,
            location={"X": 0.0, "Y": 100.0, "Z": 150.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "third_person_actor1", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )
        bp_third_person_actor2 = instance.unreal_service.spawn_actor_from_uclass(
            uclass=bp_third_person_uclass,
            location={"X": 0.0, "Y": 200.0, "Z": 150.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "third_person_actor2", "SpawnCollisionHandlingOverride": "AlwaysSpawn"}
        )

        # set bRunPhysicsWithNoController to True to control pawn without controller
        character_movement_component1 = instance.unreal_service.get_component_by_class(actor=bp_third_person_actor1, uclass=character_movement_component_uclass)
        character_movement_component2 = instance.unreal_service.get_component_by_class(actor=bp_third_person_actor2, uclass=character_movement_component_uclass)

        instance.unreal_service.set_object_properties_for_uobject(uobject=character_movement_component1, properties={"bRunPhysicsWithNoController": True})
        instance.unreal_service.set_object_properties_for_uobject(uobject=character_movement_component2, properties={"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component1, ufunction=set_movement_mode_ufunc, args={"NewMovementMode": "MOVE_Walking"})
        instance.unreal_service.call_function(uobject=character_movement_component2, ufunction=set_movement_mode_ufunc, args={"NewMovementMode": "MOVE_Walking"})

        # initialize input component
        instance.enhanced_input_service.setup_enhanced_input_component(actor=bp_third_person_actor1)
        instance.enhanced_input_service.setup_enhanced_input_component(actor=bp_third_person_actor2)

        # get all bone names
        skeletal_mesh_component1 = instance.unreal_service.get_component_by_class(actor=bp_third_person_actor1, uclass=skeletal_mesh_component_uclass)
        instance.unreal_service.call_function(
            uobject=skeletal_mesh_component1,
            ufunction=set_skeletal_mesh_asset_ufunc,
            args={"NewMesh": spear.func_utils.to_ptr(manny_simple_uobject)})
        actor1_bone_names = []
        skeletal_mesh_component_num_bones1 = instance.unreal_service.call_function(
            uobject=skeletal_mesh_component1,
            ufunction=get_num_bones_ufunc)['ReturnValue']
        for i in range(skeletal_mesh_component_num_bones1):
            actor1_bone_names.append(instance.unreal_service.call_function(
                uobject=skeletal_mesh_component1,
                ufunction=get_bone_name_ufunc,
                args={"BoneIndex": i})['ReturnValue'])

    with instance.end_frame():
        pass

    df = pd.DataFrame()

    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": False})

        instance.enhanced_input_service.inject_input_for_actor(
            actor=bp_third_person_actor1,
            input_action_name="IA_Jump",
            trigger_event="Started",
            input_action_value={},
            input_action_instance={})
        instance.enhanced_input_service.inject_input_for_actor(
            actor=bp_third_person_actor2,
            input_action_name="IA_Jump",
            trigger_event="Started",
            input_action_value={},
            input_action_instance={})

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": True})

        skeletal_mesh_transforms = {}
        for index, bone_name in enumerate(actor1_bone_names):
            skeletal_mesh_transforms[bone_name] = instance.unreal_service.call_function(
                uobject=skeletal_mesh_component1,
                ufunction=get_bone_transform_ufunc,
                args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})['ReturnValue']
        df = pd.concat([df, get_data_frame(skeletal_mesh_transforms)])

    for _ in range(200):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": False})

            instance.enhanced_input_service.inject_input_for_actor(
                actor=bp_third_person_actor1,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": -1, "Y": 1, "Z": 0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})
            instance.enhanced_input_service.inject_input_for_actor(
                actor=bp_third_person_actor2,
                input_action_name="IA_Move",
                trigger_event="Triggered",
                input_action_value={"ValueType": "Axis2D", "Value": {"X": 0, "Y": 1, "Z": 0}},
                input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": True})

            skeletal_mesh_transforms = {}
            for index, bone_name in enumerate(actor1_bone_names):
                skeletal_mesh_transforms[bone_name] = instance.unreal_service.call_function(
                    uobject=skeletal_mesh_component1,
                    ufunction=get_bone_transform_ufunc,
                    args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})['ReturnValue']
            df = pd.concat([df, get_data_frame(skeletal_mesh_transforms)])

    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": False})

        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_ufunc, args={"bPaused": True})

            skeletal_mesh_transforms = {}
            for index, bone_name in enumerate(actor1_bone_names):
                skeletal_mesh_transforms[bone_name] = instance.unreal_service.call_function(
                    uobject=skeletal_mesh_component1,
                    ufunction=get_bone_transform_ufunc,
                    args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})['ReturnValue']
            df = pd.concat([df, get_data_frame(skeletal_mesh_transforms)])

    df.to_csv(args.actions_file, float_format="%.5f", mode="w", index=False)
    # close the instance
    instance.close()

    spear.log("Done.")
