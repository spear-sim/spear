#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import math
import os
import spear
import time


_ANIMATION_DURATION = 10.0


if __name__ == "__main__":

    # create instance
    instance = spear.Instance()
    game = instance.get_game()

    with instance.begin_frame():
        # if a BP_CharacterDisplay actor is already in the level, use it; otherwise spawn one. This way, we
        # don't need to destroy the character once we're done animating it.
        bp_character_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/ExampleContent/Animation_Basics/BP_CharacterDisplay.BP_CharacterDisplay_C")
        characters = game.unreal_service.find_actors_by_class(uclass=bp_character_uclass)
        if len(characters) > 0:
            character = characters[0]
        else:
            character = game.unreal_service.spawn_actor(
                uclass=bp_character_uclass,
                location={"X": 480.0, "Y": -650.0, "Z": 10.0},
                rotation={"Pitch": 0.0, "Yaw": 0.0, "Roll": 0.0},
                spawn_parameters={"SpawnCollisionHandlingOverride": "AlwaysSpawn"})

        # get skeletal mesh component
        skeletal_mesh_component = game.unreal_service.get_component_by_class(actor=character, uclass="USkeletalMeshComponent")

        # set the anim class at runtime, then get the resulting anim instance so we can drive its variables
        abp_blendspaces_uclass = game.unreal_service.load_class(uclass="UAnimInstance", name="/Game/ExampleContent/Animation_Basics/1-5_ABP_Blendspaces.1-5_ABP_Blendspaces_C")
        skeletal_mesh_component.SetAnimInstanceClass(NewClass=abp_blendspaces_uclass)
        anim_instance = skeletal_mesh_component.GetAnimInstance()

    with instance.end_frame():
        pass

    spear.log(f"Animating character for {_ANIMATION_DURATION} seconds...")

    start_time = time.time()
    current_time = start_time

    while True:
        previous_time = current_time
        current_time = time.time()
        elapsed_time = current_time - start_time

        t = elapsed_time
        s = 4.0*t
        cs = math.cos(s)

        with instance.begin_frame():
            anim_instance.Speed1D = 0.5*(cs+1.0)
        with instance.end_frame():
            pass

        if elapsed_time >= _ANIMATION_DURATION:
            break

    spear.log(f"Finished animation.")
    spear.log("Done.")
