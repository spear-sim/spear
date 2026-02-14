#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import time


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        # load classes
        bp_bunny_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Stanford/Blueprints/BP_Bunny.BP_Bunny_C")
        bp_happy_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Stanford/Blueprints/BP_Happy.BP_Happy_C")
        bp_dragon_uclass = game.unreal_service.load_class(uclass="AActor", name="/Game/Stanford/Blueprints/BP_Dragon.BP_Dragon_C")

        # spawn actors
        bp_bunny = game.unreal_service.spawn_actor(uclass=bp_bunny_uclass, location={"X": 80.0, "Y": 210.0, "Z": 15.0})
        bp_happy = game.unreal_service.spawn_actor(uclass=bp_happy_uclass, location={"X": 80.0, "Y": 280.0, "Z": 5.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 90.0})
        bp_dragon = game.unreal_service.spawn_actor(uclass=bp_dragon_uclass, location={"X": 80.0, "Y": 350.0, "Z": 5.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 90.0})

        # set scale for actors
        bp_bunny.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})
        bp_happy.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})
        bp_dragon.SetActorScale3D(NewScale3D={"X": 4.0, "Y": 4.0, "Z": 4.0})

        # # make the HDRI backdrop a bit brighter for a paper figure
        # hdri_backdrop = game.unreal_service.find_actor_by_name(actor_name="Rendering/HDRIBackdrop", uclass="AActor")
        # skylight = hdri_backdrop.Skylight.get()
        # skylight_old_intensity = skylight.Intensity.get()
        # skylight_new_intensity = skylight_old_intensity + 2.5
        # skylight.SetIntensity(NewIntensity=skylight_new_intensity)

    with instance.end_frame():
        pass

    spear.log("Done.")
