#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear
import time


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    # get UGameplayStatics
    with instance.begin_frame():
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # Call OpenLevel. It is recommended to call OpenLevel in a "with instance.end_frame()" block, rather than
    # in a "with instance.begin_frame()" block. Calling OpenLevel in a "with instance.begin_frame()" does
    # work, but it requires the user to set the SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS config
    # parameter conservatively due to various implementation details in begin_frame() and end_frame().

    spear.log("Opening level: /Game/SPEAR/Scenes/debug_0000/Maps/debug_0000.debug_0000")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        gameplay_statics.OpenLevel(LevelName="/Game/SPEAR/Scenes/debug_0000/Maps/debug_0000.debug_0000", bAbsolute=True, Options="")

    # Calling OpenLevel invalidates the old game object, so get a new one here. This call will block until
    # the new game object is ready.
    game = instance.get_game(wait=True, wait_max_time_seconds=10.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

    with instance.begin_frame():
        # spawn object
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

        # because this level has an ASpStableNameManager, we can find actors by name even if an actor doesn't have a USpStableNameComponent
        player_start = game.unreal_service.find_actor_by_name(uclass="APlayerStart", actor_name="Settings/PlayerStart")
        spear.log("player_start:", player_start)
    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # call OpenLevel again
    spear.log("Opening level: /Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000.apartment_0000")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        gameplay_statics.OpenLevel(LevelName="/Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000.apartment_0000", bAbsolute=True, Options="")

    # get a new game object again
    game = instance.get_game(wait=True, wait_max_time_seconds=10.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

    spear.log("Done.")
