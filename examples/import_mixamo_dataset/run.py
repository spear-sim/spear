#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        # spawn Crouch To Stand animation
        location = {"X": 425.0, "Y": 275.0, "Z": 25.0}
        bp_mixamo_path = "/Game/Mixamo/Crouch_To_Stand/Crouch_To_Stand/BP_Crouch_To_Stand_Anim.BP_Crouch_To_Stand_Anim_C"
        bp_mixamo_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name=bp_mixamo_path)
        bp_mixamo = game.unreal_service.spawn_actor_from_class(uclass=bp_mixamo_uclass, location=location)

        # spawn Jump animation
        location = {"X": 225.0, "Y": 475.0, "Z": 30.0}
        bp_mixamo_path = "/Game/Mixamo/Jump/Jump/BP_Jump_Anim.BP_Jump_Anim_C"
        bp_mixamo_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name=bp_mixamo_path)
        bp_mixamo = game.unreal_service.spawn_actor_from_class(uclass=bp_mixamo_uclass, location=location)

    with instance.end_frame():
        pass

    spear.log("Done.")
