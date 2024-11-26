#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os

import pandas
import spear

import pandas as pd

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
        controller_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Controller")
        bp_third_person_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0,
                                                                    name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
        character_movement_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.CharacterMovementComponent")
        skeletal_mesh_component_class = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.SkeletalMeshComponent")
        SKM_Manny_Simple = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")
        set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_class, function_name="SetMovementMode")
        set_skeletal_mesh_asset_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_class, function_name="SetSkeletalMeshAsset")
        get_num_bones_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_class, function_name="GetNumBones")
        get_bone_name_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_class, function_name="GetBoneName")
        get_bone_transform_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_class, function_name="GetBoneTransform")

        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)
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
        character_movement_component1 = instance.unreal_service.get_component_by_class(bp_third_person_actor1, character_movement_component_class)
        character_movement_component2 = instance.unreal_service.get_component_by_class(bp_third_person_actor2, character_movement_component_class)

        instance.unreal_service.set_object_properties_for_uobject(character_movement_component1, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.set_object_properties_for_uobject(character_movement_component2, {"bRunPhysicsWithNoController": True})
        instance.unreal_service.call_function(uobject=character_movement_component1, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})
        instance.unreal_service.call_function(uobject=character_movement_component2, ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

        # initialize input component
        instance.enhanced_input_service.setup_enhanced_input_component(bp_third_person_actor1)
        instance.enhanced_input_service.setup_enhanced_input_component(bp_third_person_actor2)

        # get all bone names
        skeletal_mesh_component1 = instance.unreal_service.get_component_by_class(bp_third_person_actor1, skeletal_mesh_component_class)
        skeletal_mesh_component_num_bones1 = instance.unreal_service.call_function(skeletal_mesh_component1, get_num_bones_func)['ReturnValue']
        actor1_bone_names = []
        for i in range(skeletal_mesh_component_num_bones1):
            bone_name = instance.unreal_service.call_function(skeletal_mesh_component1, get_bone_name_func, {"BoneIndex": i})['ReturnValue']
            actor1_bone_names.append(bone_name)

    with instance.end_frame():
        pass


    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

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

        for index, bone_name in enumerate(actor1_bone_names):
            bone_transform = instance.unreal_service.call_function(skeletal_mesh_component1, get_bone_transform_func, {"GetBoneTransform": bone_name})['ReturnValue']
            print("bone_transform", bone_name, bone_transform)
        # TODO get actors

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(200):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

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
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # close the instance
    instance.close()

    spear.log("Done.")
