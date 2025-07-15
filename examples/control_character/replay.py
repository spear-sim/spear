#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pandas as pd
import spear


character_descs = [{"name": "character_0" }, {"name": "character_1"}]


def get_transforms(df, transform_names):
    transforms = {}
    for transform_name in transform_names:
        transform = {
            "translation": {
                "x": df[f"{transform_name}_translation_x"],
                "y": df[f"{transform_name}_translation_y"],
                "z": df[f"{transform_name}_translation_z"]},
            "rotation": {
                "x": df[f"{transform_name}_rotation_x"],
                "y": df[f"{transform_name}_rotation_y"],
                "z": df[f"{transform_name}_rotation_z"],
                "w": df[f"{transform_name}_rotation_w"]},
            "scale3D": {
                "x": df[f"{transform_name}_scale3D_x"],
                "y": df[f"{transform_name}_scale3D_y"],
                "z": df[f"{transform_name}_scale3D_z"]}}
        transforms[transform_name] = transform
    return transforms


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)
    game = instance.get_game()

    with instance.begin_frame():

        # We could use get_static_class(...) here to obtain a handle to each UClass object below, as we do in
        # our other examples. But for demonstration purposes, we are choosing to use load_object(...) instead.
        # Using load_object(...) can be useful when the Unreal type has not been registered ahead of time
        # using the UnrealClassRegistrar system in our C++ code.

        # find functions
        actor_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.Actor")
        set_actor_transform_func = game.unreal_service.find_function_by_name(uclass=actor_uclass, function_name="K2_SetActorTransform")

        gameplay_statics_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.GameplayStatics")
        set_game_paused_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_uclass, function_name="SetGamePaused")

        poseable_mesh_component_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.PoseableMeshComponent")
        get_bone_name_func = game.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetBoneName")
        get_num_bones_func = game.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="GetNumBones")
        set_bone_transform_by_name_func = game.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetBoneTransformByName")
        set_skinned_asset_and_update_func = game.unreal_service.find_function_by_name(uclass=poseable_mesh_component_uclass, function_name="SetSkinnedAssetAndUpdate")

        # get UGameplayStatics default object
        gameplay_statics_default_object = game.unreal_service.get_default_object(uclass=gameplay_statics_uclass, create_if_needed=False)

        # load classes and objects
        actor_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/Engine.Actor")
        manny_simple_uobject = game.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        characters = []
        for character_desc in character_descs:

            character = {}

            transforms_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "character_poses", f"{character_desc['name']}.csv"))
            character["data_frame"] = pd.read_csv(transforms_file)

            # there is no point specifying a location and rotation yet, because the spawned actor won't have a
            # root scene component, so we will need to set its location later
            character["actor"] = game.unreal_service.spawn_actor_from_class(
                uclass=actor_uclass,
                spawn_parameters={"Name": character_desc["name"], "SpawnCollisionHandlingOverride": "AlwaysSpawn"})

            # create UPoseableMeshComponent and setup skeletal mesh
            character["poseable_mesh_component"] = game.unreal_service.create_scene_component_by_class_from_actor(
                scene_component_class=poseable_mesh_component_uclass,
                owner=character["actor"],
                scene_component_name="poseable_mesh_component")
            game.unreal_service.call_function(
                uobject=character["poseable_mesh_component"],
                ufunction=set_skinned_asset_and_update_func,
                args={"NewMesh": spear.to_ptr(handle=manny_simple_uobject)})

            # get bone names
            character["bone_names"] = []
            return_values = game.unreal_service.call_function(uobject=character["poseable_mesh_component"], ufunction=get_num_bones_func)
            poseable_mesh_component_num_bones = return_values["ReturnValue"]
            for i in range(poseable_mesh_component_num_bones):
                return_values = game.unreal_service.call_function(
                    uobject=character["poseable_mesh_component"],
                    ufunction=get_bone_name_func,
                    args={"BoneIndex": i})
                bone_name = return_values["ReturnValue"]
                character["bone_names"].append(bone_name)

            characters.append(character)

    with instance.end_frame():
        pass

    # get number of replay frames from the 0th character's data frame
    assert len(characters) > 0
    num_frames = characters[0]["data_frame"].shape[0]
    for character in characters:
        assert character["data_frame"].shape[0] == num_frames

    for i in range(num_frames):

        with instance.begin_frame():
            for character in characters:
                row = character["data_frame"].iloc[i]
                bone_transforms = get_transforms(row, character["bone_names"])

                # Even though we're setting bone transforms in world-space, we need to update the actor's
                # transform, otherwise Unreal's default frustum culling behavior will cull the skeletal mesh
                # and it will flicker and eventually become invisible in a counterintuitive way. Note that
                # the capitalization of some bone names change when running in the editor, presumably because
                # of the case-insensitivity of FName objects in Unreal. This unpleasant engine behavior means
                # that if we execute run.py in the editor, we must take care to also execute replay.py in the
                # editor. This is why we check for "root" and "Root" below.

                root_transform = {}
                if "root" in bone_transforms.keys():
                    root_transform = bone_transforms["root"]
                elif "Root" in bone_transforms.keys():
                    root_transform = bone_transforms["Root"]
                game.unreal_service.call_function(uobject=character["actor"], ufunction=set_actor_transform_func, args={"NewTransform": root_transform})

                # set bone transforms
                for bone_name, transform in bone_transforms.items():
                    result = game.unreal_service.call_function(
                        uobject=character["poseable_mesh_component"],
                        ufunction=set_bone_transform_by_name_func,
                        args={"BoneName": bone_name, "InTransform": transform, "BoneSpace": "WorldSpace"})

        with instance.end_frame():
            pass

    spear.log("Done.")
