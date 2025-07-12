#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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
    
    # find UGameplayStatics default object and OpenLevel function
    with instance.begin_frame():
        gameplay_statics_static_class = game.unreal_service.get_static_class(class_name="UGameplayStatics")
        open_level_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="OpenLevel")
        gameplay_statics_default_object = game.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)
    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # Call OpenLevel. It is recommended to call OpenLevel in a "with instance.end_frame()" block, rather than
    # in a "with instance.begin_frame()" block. Calling OpenLevel in a "with instance.begin_frame()" does
    # work, but it requires the user to set the SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS config
    # parameter conservatively due to various implementation details in begin_frame() and end_frame().

    spear.log("Opening level: /Game/Spear/Scenes/debug_0000/Maps/debug_0000.debug_0000")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        args = {"LevelName": "/Game/Spear/Scenes/debug_0000/Maps/debug_0000.debug_0000", "bAbsolute": True, "Options": ""}
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=open_level_func, args=args)

    # Calling OpenLevel invalidates the old game object, so get a new one here. This call will block until
    # the new game object is ready.
    game = instance.get_game(wait=True, wait_max_time_seconds=10.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

    # spawn object
    with instance.begin_frame():
        bp_axes_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor_from_class(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})
    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # call OpenLevel again
    spear.log("Opening level: /Game/Spear/Scenes/apartment_0000/Maps/apartment_0000.apartment_0000")
    with instance.begin_frame():
        pass
    with instance.end_frame():
        args = {"LevelName": "/Game/Spear/Scenes/apartment_0000/Maps/apartment_0000.apartment_0000", "bAbsolute": True, "Options": ""}
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=open_level_func, args=args)

    # get a new game object again
    game = instance.get_game(wait=True, wait_max_time_seconds=10.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

    spear.log("Done.")
