#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


if __name__ == "__main__":

    # create instance
    instance = spear.Instance()
    game = instance.get_game()
    
    with instance.begin_frame():

        # spawn Crouch To Stand animation
        location = {"X": 425.0, "Y": 275.0, "Z": 25.0}
        bp_mixamo_path = "/Game/Mixamo/Crouch_To_Stand/Crouch_To_Stand/BP_Crouch_To_Stand_Anim.BP_Crouch_To_Stand_Anim_C"
        bp_mixamo_uclass = game.unreal_service.load_class(uclass="AActor", name=bp_mixamo_path)
        bp_mixamo = game.unreal_service.spawn_actor(uclass=bp_mixamo_uclass, location=location)

        # spawn Jump animation
        location = {"X": 225.0, "Y": 475.0, "Z": 30.0}
        bp_mixamo_path = "/Game/Mixamo/Jump/Jump/BP_Jump_Anim.BP_Jump_Anim_C"
        bp_mixamo_uclass = game.unreal_service.load_class(uclass="AActor", name=bp_mixamo_path)
        bp_mixamo = game.unreal_service.spawn_actor(uclass=bp_mixamo_uclass, location=location)

    with instance.end_frame():
        pass

    spear.log("Done.")
