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

    is_with_editor = instance.engine_globals_service.is_with_editor()

    if is_with_editor:
        with instance.begin_frame():
            message_log_names = game.console_service.get_message_log_names()
            assert "MapCheck" in message_log_names
            spear.log("MapCheck:")
            for m in game.console_service.get_message_log_messages(name="MapCheck"):
                spear.log_no_prefix("    ", m)
        with instance.end_frame():
            pass

    # sleep for a few seconds
    spear.log("Sleeping...")
    time.sleep(5.0)

    # Call OpenLevel. It is recommended to call OpenLevel() in a "with instance.end_frame()" block, rather
    # than in a "with instance.begin_frame()" block. Calling OpenLevel in a "with instance.begin_frame()"
    # does work, but it requires the user to set the SPEAR.CLIENT.INTERNAL_TIMEOUT_SECONDS config
    # parameter conservatively due to various implementation details in begin_frame() and end_frame(). It is
    # required to call OpenLevel() inside a "with game.invalidate()" block, as we do here, and OpenLevel()
    # must be the last call inside the block.

    spear.log("Opening level: /Game/ThirdPerson/Maps/ThirdPersonMap")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        with game.invalidate():
            game.cdo.UGameplayStatics.OpenLevel(LevelName="/Game/ThirdPerson/Maps/ThirdPersonMap", bAbsolute=True, Options="")

    # Calling OpenLevel destroys the old game world, so get the game object again here to bind it to the new
    # game world. This call will block until the new game world is ready.
    game = instance.get_game(wait_for_world_initialized=True, wait_for_world_initialized_max_time_seconds=10.0, wait_for_world_initialized_sleep_time_seconds=1.0)

    if is_with_editor:
        with instance.begin_frame():
            message_log_names = game.console_service.get_message_log_names()
            assert "MapCheck" in message_log_names
            spear.log("MapCheck:")
            for m in game.console_service.get_message_log_messages(name="MapCheck"):
                spear.log_no_prefix("    ", m)
        with instance.end_frame():
            pass

    with instance.begin_frame():

        # unpause the game
        game.cdo.UGameplayStatics.SetGamePaused(bPaused=False)

        # spawn object
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})

    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # call OpenLevel again
    spear.log("Opening level: /Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        with game.invalidate():
            game.cdo.UGameplayStatics.OpenLevel(LevelName="/Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000", bAbsolute=True, Options="")

    # get the game object again
    game = instance.get_game(wait_for_world_initialized=True, wait_for_world_initialized_max_time_seconds=10.0, wait_for_world_initialized_sleep_time_seconds=1.0)
    instance.warm_up(time_seconds=5.0)

    if is_with_editor:
        with instance.begin_frame():
            message_log_names = game.console_service.get_message_log_names()
            assert "MapCheck" in message_log_names
            spear.log("MapCheck:")
            for m in game.console_service.get_message_log_messages(name="MapCheck"):
                spear.log_no_prefix("    ", m)
        with instance.end_frame():
            pass

    spear.log("output_log_messages:")
    output_log_messages = game.console_service.flush_output_log_messages()
    assert output_log_messages.num_evicted == 0
    for m in output_log_messages.messages:
        spear.log_no_prefix("    ", m)

    spear.log("Done.")
