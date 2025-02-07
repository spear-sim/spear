#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import readchar
import spear


if __name__ == "__main__":

    spear.log("Initializing SPEAR instance...")

    # create SPEAR instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    spear_instance = spear.Instance(config)

    spear.log("Finished initializing SPEAR instance.")

    spear_instance.engine_service.begin_tick()

    # get Unreal actors and functions
    actor = spear_instance.unreal_service.find_actor_by_name(class_name="AActor", name="Debug/MixamoActor") #"Debug/Manny"
    poseable_mesh_component = spear_instance.unreal_service.get_component_by_type(class_name="UPoseableMeshComponent", actor=actor)

    gameplay_statics_static_class = spear_instance.unreal_service.get_static_class(class_name="UGameplayStatics")
    poseable_mesh_component_static_class = spear_instance.unreal_service.get_static_class(class_name="UPoseableMeshComponent")

    gameplay_statics_default_object = spear_instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class)

    set_game_paused_func = spear_instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, name="SetGamePaused")
    get_bone_transform_by_name_func = spear_instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_static_class, name="GetBoneTransformByName")
    set_bone_transform_by_name_func = spear_instance.unreal_service.find_function_by_name(uclass=poseable_mesh_component_static_class, name="SetBoneTransformByName")

    # the game starts paused by default, so unpause the game, because bone transforms won't visually update otherwise
    args = {"bPaused": False}
    spear_instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args=args)

    spear_instance.engine_service.tick()
    spear_instance.engine_service.end_tick()

    bone_names = ["Head", "LeftHand", "RightHand"] # ["head", "hand_l", "hand_r"]
    skeleton_mapping = {
        "Hips": "pelvis",
        "Spine": "spine1",
        "Spine1": "spine2",
        "Spine2": "spine3",
        "Neck": "neck",
        "Head": "head",
        "LeftShoulder": "left_collar",
        "LeftArm": "left_shoulder",
        "LeftForeArm": "left_elbow",
        "LeftHand": "left_wrist",
        "RightShoulder": "right_collar",
        "RightArm": "right_shoulder",
        "RightForeArm": "right_elbow",
        "RightHand": "right_wrist",
        "LeftUpLeg": "left_hip",
        "LeftLeg": "left_knee",
        "LeftFoot": "left_ankle",
        "LeftToeBase": "left_foot",
        "RightUpLeg": "right_hip",
        "RightLeg": "right_knee",
        "RightFoot": "right_ankle",
        "RightToeBase": "right_foot",
    }

    quit = False
    while not quit:

        key = readchar.readkey()
        spear.log("Received key: ", key)

        if key == "0":

            spear.log("Getting pose data for bones: ", bone_names)

            spear_instance.engine_service.begin_tick()

            for bone_name in skeleton_mapping.keys():
                args = {"BoneName": bone_name, "BoneSpace": "ComponentSpace"}
                return_values = spear_instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=get_bone_transform_by_name_func, args=args)
                spear.log(return_values)

            spear_instance.engine_service.tick()
            spear_instance.engine_service.end_tick()

        elif key == "1":

            spear.log("Decreasing scale of bones: ", bone_names)

            spear_instance.engine_service.begin_tick()

            for bone_name in bone_names:
                args = {"BoneName": bone_name, "BoneSpace": "ComponentSpace"}
                return_values = spear_instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=get_bone_transform_by_name_func, args=args)

                args = {"BoneName": bone_name, "InTransform": return_values["ReturnValue"], "BoneSpace": "ComponentSpace"}
                args["InTransform"]["scale3D"]["x"] = 0.9*args["InTransform"]["scale3D"]["x"]
                args["InTransform"]["scale3D"]["y"] = 0.9*args["InTransform"]["scale3D"]["y"]
                args["InTransform"]["scale3D"]["z"] = 0.9*args["InTransform"]["scale3D"]["z"]
                spear_instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=set_bone_transform_by_name_func, args=args)

            spear_instance.engine_service.tick()
            spear_instance.engine_service.end_tick()

        elif key == "2":

            spear.log("Increasing scale of bones: ", bone_names)

            spear_instance.engine_service.begin_tick()

            for bone_name in bone_names:
                args = {"BoneName": bone_name, "BoneSpace": "ComponentSpace"}
                return_values = spear_instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=get_bone_transform_by_name_func, args=args)

                args = {"BoneName": bone_name, "InTransform": return_values["ReturnValue"], "BoneSpace": "ComponentSpace"}
                args["InTransform"]["scale3D"]["x"] = 1.1*args["InTransform"]["scale3D"]["x"]
                args["InTransform"]["scale3D"]["y"] = 1.1*args["InTransform"]["scale3D"]["y"]
                args["InTransform"]["scale3D"]["z"] = 1.1*args["InTransform"]["scale3D"]["z"]
                spear_instance.unreal_service.call_function(uobject=poseable_mesh_component, ufunction=set_bone_transform_by_name_func, args=args)

            spear_instance.engine_service.tick()
            spear_instance.engine_service.end_tick()

        elif key == readchar.key.ESC:
            quit = True

    spear_instance.close()

    spear.log("Done.")
