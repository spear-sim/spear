#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear
import time


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    editor = instance.get_editor()

    with instance.begin_frame():
        # find ULevelEditorSubsystem functions
        level_editor_subsystem_static_class = editor.unreal_service.get_static_class(class_name="ULevelEditorSubsystem")
        editor_request_begin_play_func = editor.unreal_service.find_function_by_name(uclass=level_editor_subsystem_static_class, function_name="EditorRequestBeginPlay")
        editor_request_end_play_func = editor.unreal_service.find_function_by_name(uclass=level_editor_subsystem_static_class, function_name="EditorRequestEndPlay")

        # get ULevelEditorSubsystem
        level_editor_subsystem = editor.unreal_service.get_editor_subsystem_by_type(class_name="ULevelEditorSubsystem")

        # execute console commands
        editor.unreal_service.execute_console_command("stat fps")
        editor.unreal_service.execute_console_command("py import unreal; unreal.log('Calling function: ULevelEditorSubsystem::EditorRequestBeginPlay()');")

        # programmatically press play in the editor by calling ULevelEditorSubsystem::EditorRequestBeginPlay()
        editor.unreal_service.call_function(uobject=level_editor_subsystem, ufunction=editor_request_begin_play_func)

    with instance.end_frame():
        pass

    # now that we have programmatically pressed play, we can access the live game
    game = instance.get_game()

    # spawn object
    with instance.begin_frame():
        bp_axes_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/SpComponents/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor_from_class(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})
    with instance.end_frame():
        pass

    # sleep for a few seconds
    time.sleep(5.0)

    # the game will be invalidated as soon as we programmatically press stop, so we set game to None
    game = None

    with instance.begin_frame():
        # execute console command
        editor.unreal_service.execute_console_command("stat fps")

        # programmatically press stop in the editor by calling ULevelEditorSubsystem::EditorRequestEndPlay()
        editor.unreal_service.call_function(uobject=level_editor_subsystem, ufunction=editor_request_end_play_func)

    with instance.end_frame():
        pass

    with instance.begin_frame():
        # find ASpMessageQueueManager functions and default object
        sp_message_queue_manager_static_class = editor.unreal_service.get_static_class(class_name="ASpMessageQueueManager")
        create_queue_func = editor.unreal_service.find_function_by_name(uclass=sp_message_queue_manager_static_class, function_name="CreateQueue")
        destroy_queue_func = editor.unreal_service.find_function_by_name(uclass=sp_message_queue_manager_static_class, function_name="DestroyQueue")
        get_queue_length_func = editor.unreal_service.find_function_by_name(uclass=sp_message_queue_manager_static_class, function_name="GetQueueLength")
        pop_message_from_front_of_queue_func = editor.unreal_service.find_function_by_name(uclass=sp_message_queue_manager_static_class, function_name="PopMessageFromFrontOfQueue")
        sp_message_queue_manager_default_object = editor.unreal_service.get_default_object(uclass=sp_message_queue_manager_static_class, create_if_needed=False)

        # create a named queue to facilitate communication between this Python script and take_high_res_screenshot.py
        editor.unreal_service.call_function(uobject=sp_message_queue_manager_default_object, ufunction=create_queue_func, args={"queue_name": "take_screenshot"})

        # run take_high_res_screenshot.py
        take_screenshot_py_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "take_screenshot.py"))
        editor.unreal_service.execute_console_command(f"py unreal.log('Executing Python file: {take_screenshot_py_file}');")
        editor.unreal_service.execute_console_command(f"py {take_screenshot_py_file}")

    with instance.end_frame():
        pass

    # wait until take_high_res_screenshot.py pushes a message to the named queue created above
    take_high_res_screenshot_executing = True
    while take_high_res_screenshot_executing:
        with instance.begin_frame():
            return_values = editor.unreal_service.call_function(uobject=sp_message_queue_manager_default_object, ufunction=get_queue_length_func, args={"queue_name": "take_screenshot"})
            queue_length = return_values["ReturnValue"]
            take_high_res_screenshot_executing = queue_length == 0
        with instance.end_frame():
            pass
        if take_high_res_screenshot_executing:
            spear.log("Waiting for take_high_res_screenshot.py to finish executing...")
            time.sleep(1.0)

    with instance.begin_frame():
        # retrieve message pushed by take_high_res_screenshot.py
        return_values = editor.unreal_service.call_function(uobject=sp_message_queue_manager_default_object, ufunction=pop_message_from_front_of_queue_func, args={"queue_name": "take_screenshot"})
        message = return_values["ReturnValue"]

        # destroy queue
        editor.unreal_service.call_function(uobject=sp_message_queue_manager_default_object, ufunction=destroy_queue_func, args={"queue_name": "take_screenshot"})

    with instance.end_frame():
        pass

    spear.log("Message from take_screenshot.py:")
    spear.log_no_prefix(message)

    # instance.close()
    
    spear.log("Done.")
