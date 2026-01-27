#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import numpy as np
import pandas as pd
import shutil
import spear


character_descs = \
[ 
    {
        "name": "character_0",
        "location": {"X": 0.0, "Y": -100.0, "Z": 303.0},
        "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
        "move_input_action_value": {"ValueType": "Axis2D", "Value": {"X": -1.0, "Y": 1.0, "Z": 0.0}}
    },
    {
        "name": "character_1",
        "location": {"X": 0.0, "Y": 100.0, "Z": 303.0},
        "rotation": {"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
        "move_input_action_value": {"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 1.0, "Z": 0.0}}
    }
]


def get_data_frame(transforms):
    columns = []
    data = []
    for transform_name, transform in transforms.items():
        cols = [
            f"{transform_name}_translation_x",
            f"{transform_name}_translation_y",
            f"{transform_name}_translation_z",
            f"{transform_name}_rotation_x",
            f"{transform_name}_rotation_y",
            f"{transform_name}_rotation_z",
            f"{transform_name}_rotation_w",
            f"{transform_name}_scale3D_x",
            f"{transform_name}_scale3D_y",
            f"{transform_name}_scale3D_z"]
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
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    # create output dir
    character_poses_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "character_poses"))
    if os.path.exists(character_poses_dir):
        spear.log("Directory exists, removing: ", character_poses_dir)
        shutil.rmtree(character_poses_dir, ignore_errors=True)
    os.makedirs(character_poses_dir, exist_ok=True)

    with instance.begin_frame():

        #
        # initialize characters
        #

        # Load classes. For Blueprint types, we need to use load_class(...), because these types don't have
        # a class name that is visible to the Unreal property system before they are loaded. In contrast, for
        # C++ types, we can use the slightly less verbose get_static_class(...), because C++ types have a
        # class name that is visible to the Unreal property system, and can be located using that name.

        bp_character_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter.BP_ThirdPersonCharacter_C")
        enhanced_input_component_uclass = game.unreal_service.get_static_class(uclass="UEnhancedInputComponent")

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # load objects
        manny_simple = game.unreal_service.load_object(uclass="USkeletalMesh", name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        characters = []
        for character_desc in character_descs:

            character = {}

            # copy data from character_desc to simplify indexing later
            character["name"] = character_desc["name"]
            character["move_input_action_value"] = character_desc["move_input_action_value"]

            # spawn character and get components
            character["actor"] = game.unreal_service.spawn_actor(
                uclass=bp_character_uclass,
                location=character_desc["location"],
                rotation=character_desc["rotation"],
                spawn_parameters={"SpawnCollisionHandlingOverride": "AlwaysSpawn"})
            character["movement_component"] = game.unreal_service.get_component_by_class(actor=character["actor"], uclass="UCharacterMovementComponent")
            character["skeletal_mesh_component"] = game.unreal_service.get_component_by_class(actor=character["actor"], uclass="USkeletalMeshComponent")

            # configure character to be controlled without possessing
            character["movement_component"].bRunPhysicsWithNoController = True
            character["movement_component"].SetMovementMode(NewMovementMode="MOVE_Walking")

            # initialize input component
            character["actor"].CreateInputComponent(InputComponentToCreate=enhanced_input_component_uclass)
            character["input_component"] = game.unreal_service.get_component_by_class(actor=character["actor"], uclass=enhanced_input_component_uclass)
            instance.input_service.setup_player_input_component(actor=character["actor"], input_component=character["input_component"])

            # update skeletal mesh
            character["skeletal_mesh_component"].SetSkeletalMeshAsset(NewMesh=manny_simple)

            # get bone names
            character["bone_names"] = [ character["skeletal_mesh_component"].GetBoneName(BoneIndex=i) for i in range(character["skeletal_mesh_component"].GetNumBones()) ]

            # initialize data frame for storing bone transforms
            character["data_frame"] = pd.DataFrame()

            characters.append(character)

    with instance.end_frame():
        pass

    # jump
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
        for character in characters:
            instance.enhanced_input_service.inject_input_for_actor(
                actor=character["actor"],
                input_action_name="IA_Jump",
                trigger_event="Started",
                input_action_value={},
                input_action_instance={})

    # record bone transforms
    with instance.end_frame():
        for character in characters:
            transforms = { bone_name: character["skeletal_mesh_component"].GetBoneTransform(InBoneName=bone_name, TransformSpace="RTS_World") for bone_name in character["bone_names"] }
            character["data_frame"] = pd.concat([character["data_frame"], get_data_frame(transforms)])
        gameplay_statics.SetGamePaused(bPaused=True)

    for _ in range(100):
        
        # move
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
            for character in characters:
                instance.enhanced_input_service.inject_input_for_actor(
                    actor=character["actor"],
                    input_action_name="IA_Move",
                    trigger_event="Triggered",
                    input_action_value=character["move_input_action_value"],
                    input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        # record bone transforms
        with instance.end_frame():
            for character in characters:
                transforms = { bone_name: character["skeletal_mesh_component"].GetBoneTransform(InBoneName=bone_name, TransformSpace="RTS_World") for bone_name in character["bone_names"] }
                character["data_frame"] = pd.concat([character["data_frame"], get_data_frame(transforms)])
            gameplay_statics.SetGamePaused(bPaused=True)

    # unpause now that we're finished recording
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
    with instance.end_frame():
        pass

    # save bone transforms in a separate CSV file for each character
    for character in characters:
        transforms_file = os.path.realpath(os.path.join(character_poses_dir, f"{character['name']}.csv"))
        character["data_frame"].to_csv(transforms_file, mode="w", index=False)

    spear.log("Done.")
