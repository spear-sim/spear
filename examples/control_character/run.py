#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import numpy as np
import pandas as pd
import spear


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

    parser = argparse.ArgumentParser()
    parser.add_argument("--transforms_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "transforms.csv")))
    args = parser.parse_args()

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    df = pd.DataFrame()

    with instance.begin_frame():

        # We could use get_static_class(...) here to obtain a handle to the UClass object for each Unreal
        # type, as we do in our other examples. But for demonstration purposes, we use load_class(...)
        # instead. Using load_class(...) can be useful when the Unreal type has not been registered ahead of
        # time using the UnrealClassRegistrar system in our C++ code.

        # find functions
        gameplay_statics_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.GameplayStatics")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")

        actor_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.Actor")
        create_input_component_func = instance.unreal_service.find_function_by_name(uclass=actor_uclass, function_name="CreateInputComponent")

        character_movement_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.CharacterMovementComponent")
        set_movement_mode_func = instance.unreal_service.find_function_by_name(uclass=character_movement_component_uclass, function_name="SetMovementMode")

        skeletal_mesh_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/Engine.SkeletalMeshComponent")
        get_bone_name_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneName")
        get_bone_transform_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetBoneTransform")
        get_num_bones_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="GetNumBones")
        set_skeletal_mesh_asset_func = instance.unreal_service.find_function_by_name(uclass=skeletal_mesh_component_uclass, function_name="SetSkeletalMeshAsset")

        enhanced_input_component_uclass = instance.unreal_service.load_class(class_name="UObject", outer=0, name="/Script/EnhancedInput.EnhancedInputComponent")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        #
        # initialize characters
        #

        character_descs = [
            {"name": "character_0", "location": {"X": 900.0, "Y": 950.0, "Z": 100.0}, "rotation": {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}},
            {"name": "character_1", "location": {"X": 900.0, "Y": 1250.0, "Z": 100.0}, "rotation": {"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}}]
        num_characters = len(character_descs)

        bp_character_uclass = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")

        characters = []
        for character_desc in character_descs:

            # spawn character
            character = {}
            character["actor"] = instance.unreal_service.spawn_actor_from_uclass(
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

            characters.append(character)

        # update skeletal mesh
        manny_simple_uobject = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")
        instance.unreal_service.call_function(
            uobject=characters[0]["skeletal_mesh_component"],
            ufunction=set_skeletal_mesh_asset_func,
            args={"NewMesh": spear.to_ptr(manny_simple_uobject)})

        # get bone names
        for character in characters:
            bone_names = []
            return_values = instance.unreal_service.call_function(uobject=character["skeletal_mesh_component"], ufunction=get_num_bones_func)
            skeletal_mesh_component_num_bones = return_values["ReturnValue"]
            for i in range(skeletal_mesh_component_num_bones):
                return_values = instance.unreal_service.call_function(
                    uobject=character["skeletal_mesh_component"],
                    ufunction=get_bone_name_func,
                    args={"BoneIndex": i})
                bone_name = return_values["ReturnValue"]
                bone_names.append(bone_name)
            character["bone_names"] = bone_names

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

        # record bone transforms for character 0
        transforms = {}
        for i, bone_name in enumerate(characters[0]["bone_names"]):
            return_values = instance.unreal_service.call_function(
                uobject=characters[0]["skeletal_mesh_component"],
                ufunction=get_bone_transform_func,
                args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})
            transforms[bone_name] = return_values["ReturnValue"]
        df = pd.concat([df, get_data_frame(transforms)])

        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    for _ in range(200):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

            # move in a different direction for each character
            input_action_values = [
                {"ValueType": "Axis2D", "Value": {"X": -1.0, "Y": 1.0, "Z": 0.0}},
                {"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 1.0, "Z": 0.0}}]

            for i in range(num_characters):
                input_action_value = input_action_values[i]
                character = characters[i]
                instance.enhanced_input_service.inject_input_for_actor(
                    actor=character["actor"],
                    input_action_name="IA_Move",
                    trigger_event="Triggered",
                    input_action_value=input_action_values[i],
                    input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        with instance.end_frame():

            # record bone transforms for character 0
            transforms = {}
            for i, bone_name in enumerate(characters[0]["bone_names"]):
                return_values = instance.unreal_service.call_function(
                    uobject=characters[0]["skeletal_mesh_component"],
                    ufunction=get_bone_transform_func,
                    args={"InBoneName": bone_name, "TransformSpace": "RTS_World"})
                transforms[bone_name] = return_values["ReturnValue"]
            df = pd.concat([df, get_data_frame(transforms)])

            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # unpause now that we're finished recording
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    with instance.end_frame():
        pass

    df.to_csv(args.transforms_file, float_format="%.5f", mode="w", index=False)

    spear.log("Done.")
