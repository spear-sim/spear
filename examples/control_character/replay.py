#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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

        # load objects
        manny_simple = game.unreal_service.load_object(uclass="UObject", name="/Game/Characters/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple")

        characters = []
        for character_desc in character_descs:

            character = {}

            transforms_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "character_poses", f"{character_desc['name']}.csv"))
            character["data_frame"] = pd.read_csv(transforms_file, comment="#")

            # there is no point specifying a location and rotation yet, because the spawned actor won't have a
            # root scene component, so we will need to set its location later
            character["actor"] = game.unreal_service.spawn_actor(uclass="AActor", spawn_parameters={"Name": character_desc["name"], "SpawnCollisionHandlingOverride": "AlwaysSpawn"})

            # create UPoseableMeshComponent and setup skeletal mesh
            character["poseable_mesh_component"] = game.unreal_service.create_scene_component_for_actor(uclass="UPoseableMeshComponent", owner=character["actor"], scene_component_name="poseable_mesh_component")
            character["poseable_mesh_component"].SetSkinnedAssetAndUpdate(NewMesh=manny_simple)

            # get bone names
            character["bone_names"] = [ character["poseable_mesh_component"].GetBoneName(BoneIndex=i) for i in range(character["poseable_mesh_component"].GetNumBones()) ]

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
                args = {"NewTransform": root_transform}
                character["actor"].K2_SetActorTransform(NewTransform=root_transform)

                # set bone transforms
                for bone_name, transform in bone_transforms.items():
                    character["poseable_mesh_component"].SetBoneTransformByName(BoneName=bone_name, InTransform=transform, BoneSpace="WorldSpace")

        with instance.end_frame():
            pass

    spear.log("Done.")
