#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import numpy as np
import pandas as pd
import spear


character_descs = \
[ 
    {
        "name": "character_0",
        "location": {"X": 900.0, "Y": 950.0, "Z": 100.0},
        "rotation": {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        "move_input_action_value": {"ValueType": "Axis2D", "Value": {"X": -1.0, "Y": 1.0, "Z": 0.0}}
    },
    {
        "name": "character_1",
        "location": {"X": 900.0, "Y": 1250.0, "Z": 100.0},
        "rotation": {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        "move_input_action_value": {"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 1.0, "Z": 0.0}}
    }
]


def get_data_frame(transforms):
    columns = []
    data = []
    for transform_name, transform in transforms.items():
        cols = [
            transform_name + "_translation_x",
            transform_name + "_translation_y",
            transform_name + "_translation_z",
            transform_name + "_rotation_x",
            transform_name + "_rotation_y",
            transform_name + "_rotation_z",
            transform_name + "_rotation_w",
            transform_name + "_scale3D_x",
            transform_name + "_scale3D_y",
            transform_name + "_scale3D_z"]
        columns.extend(cols)
        vals = [
            transform["translation"]["x"],
            transform["translation"]["y"],
            transform["translation"]["z"],
            transform["rotation"]["x"],
            transform["rotation"]["y"],
            transform["rotation"]["z"],
            transform["rotation"]["w"],
            transform["scale3D"]["x"],
            transform["scale3D"]["y"],
            transform["scale3D"]["z"]]
        data.extend(vals)
    df = pd.DataFrame(columns=columns, data=np.array(data).reshape(1, -1))
    return df


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():

        # We could use get_static_class(...) here to obtain a handle to each UClass object below, as we do in
        # our other examples. But for demonstration purposes, we are choosing to use use load_object(...)
        # instead. Using load_object(...) can be useful when the Unreal type has not been registered ahead of
        # time using the UnrealClassRegistrar system in our C++ code.

        # find functions
        gameplay_statics_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.GameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")

        actor_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.Actor")
        create_input_component_func = instance.unreal_service.find_function_by_name(uclass=actor_uclass, function_name="CreateInputComponent")

        character_movement_component_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.CharacterMovementComponent")
        set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_uclass, function_name="SetMovementMode")

        skeletal_mesh_component_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.SkeletalMeshComponent")
        get_bone_name_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneName")
        get_bone_transform_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneTransform")
        get_num_bones_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetNumBones")
        set_skeletal_mesh_asset_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="SetSkeletalMeshAsset")

        enhanced_input_component_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/EnhancedInput.EnhancedInputComponent")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        #
        # initialize characters
        #

        # load classes and objects
        bp_character_uclass = instance.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        characters = []
        for character_desc in character_descs:

            character = {}

            # copy data from character_desc to simplify indexing later
            character["name"] = character_desc["name"]
            character["move_input_action_value"] = character_desc["move_input_action_value"]

            # spawn character and get handles to components
            character["actor"] = instance.unreal_service.spawn_actor_from_class(
                uclass=bp_character_uclass,
                location=character_desc["location"],
                rotation=character_desc["rotation"],
                spawn_parameters={"Name": character_desc["name"], "SpawnCollisionHandlingOverride": "AlwaysSpawn"})
            character["movement_component"] = instance.unreal_service.get_component_by_class(actor=character["actor"], uclass=character_movement_component_uclass)
            character["skeletal_mesh_component"] = instance.unreal_service.get_component_by_class(actor=character["actor"], uclass=skeletal_mesh_component_uclass)

            # configure character to be controlled without possessing
            instance.unreal_service.set_object_properties_for_uobject(uobject=character["movement_component"], properties={"bRunPhysicsWithNoController": True})
            instance.unreal_service.call_function(uobject=character["movement_component"], ufunction=set_movement_mode_func, args={"NewMovementMode": "MOVE_Walking"})

            # initialize input component
            instance.unreal_service.call_function(
                uobject=character["actor"],
                ufunction=create_input_component_func,
                args={"InputComponentToCreate": spear.to_ptr(enhanced_input_component_uclass)})
            character["input_component"] = instance.unreal_service.get_component_by_class(actor=character["actor"], uclass=enhanced_input_component_uclass)
            instance.input_service.setup_player_input_component(actor=character["actor"], input_component=character["input_component"])

            # update skeletal mesh
            instance.unreal_service.call_function(
                uobject=character["skeletal_mesh_component"],
                ufunction=set_skeletal_mesh_asset_func,
                args={"NewMesh": spear.to_ptr(manny_simple_uobject)})

            # get bone names
            character["bone_names"] = []
            return_values = instance.unreal_service.call_function(uobject=character["skeletal_mesh_component"], ufunction=get_num_bones_func)
            skeletal_mesh_component_num_bones = return_values["ReturnValue"]
            for i in range(skeletal_mesh_component_num_bones):
                return_values = instance.unreal_service.call_function(
                    uobject=character["skeletal_mesh_component"],
                    ufunction=get_bone_name_func,
                    args={"BoneIndex": i})
                bone_name = return_values["ReturnValue"]
                character["bone_names"].append(bone_name)

            # initialize data frame for storing bone transforms
            character["data_frame"] = pd.DataFrame()

            characters.append(character)

    with instance.end_frame():
        pass

    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        # jump
        for character in characters:
            instance.enhanced_input_service.inject_input_for_actor(
                actor=character["actor"],
                input_action_name="IA_Jump",
                trigger_event="Started",
                input_action_value={},
                input_action_instance={})

    with instance.end_frame():

        # record bone transforms
        for character in characters:
            transforms = {}
            for i, bone_name in enumerate(character["bone_names"]):
                return_values = instance.unreal_service.call_function(
                    uobject=character["skeletal_mesh_component"],
                    ufunction=get_bone_transform_func,
                    args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})
                transforms[bone_name] = return_values["ReturnValue"]
            character["data_frame"] = pd.concat([character["data_frame"], get_data_frame(transforms)])

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(200):
        
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            # move
            for character in characters:
                instance.enhanced_input_service.inject_input_for_actor(
                    actor=character["actor"],
                    input_action_name="IA_Move",
                    trigger_event="Triggered",
                    input_action_value=character["move_input_action_value"],
                    input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():

            # record bone transforms
            for character in characters:
                transforms = {}
                for i, bone_name in enumerate(character["bone_names"]):
                    return_values = instance.unreal_service.call_function(
                        uobject=character["skeletal_mesh_component"],
                        ufunction=get_bone_transform_func,
                        args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})
                    transforms[bone_name] = return_values["ReturnValue"]
                character["data_frame"] = pd.concat([character["data_frame"], get_data_frame(transforms)])

            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # unpause now that we're finished recording
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    with instance.end_frame():
        pass

    # save bone transforms in a separate CSV file for each character
    for character in characters:
        transforms_file = os.path.realpath(os.path.join(os.path.dirname(__file__), character["name"] + ".csv"))
        character["data_frame"].to_csv(transforms_file, float_format="%.5f", mode="w", index=False)

    spear.log("Done.")
