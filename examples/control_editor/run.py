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
    editor = instance.get_editor()

    with instance.begin_frame():
        # get ULevelEditorSubsystem
        level_editor_subsystem = editor.unreal_service.get_editor_subsystem(uclass="ULevelEditorSubsystem")

        # get USpMessageQueueManager
        sp_message_queue_manager = editor.get_unreal_object(uclass="USpMessageQueueManager")

        # execute console commands
        editor.unreal_service.execute_console_command(command="stat fps")
        editor.unreal_service.execute_console_command(command="py import unreal")

    with instance.end_frame():
        pass

    # programmatically press play in the editor
    with instance.begin_frame():
        editor.unreal_service.execute_console_command(command="py unreal.log('Calling function: ULevelEditorSubsystem::EditorRequestBeginPlay()')")
        level_editor_subsystem.EditorRequestBeginPlay()
    with instance.end_frame():
        pass

    # now that we have programmatically pressed play, we can access the live game
    game = instance.get_game()

    # spawn object in the live game
    with instance.begin_frame():
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})
    with instance.end_frame():
        pass

    # sleep for a few seconds so we can see the newly spawned object in the live game
    time.sleep(5.0)

    # the game will be invalidated as soon as we programmatically press stop, so we set game to None here
    game = None

    # programmatically press stop in the editor
    with instance.begin_frame():
        editor.unreal_service.execute_console_command(command="py unreal.log('Calling function: ULevelEditorSubsystem::EditorRequestEndPlay()')")
        level_editor_subsystem.EditorRequestEndPlay()
    with instance.end_frame():
        pass

    with instance.begin_frame():
        # create a message queue named "take_screenshot" to facilitate communication between this Python script and take_screenshot.py
        sp_message_queue_manager.CreateQueue(QueueName="take_screenshot")

        # run take_screenshot.py
        take_screenshot_py_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "take_screenshot.py"))
        editor.unreal_service.execute_console_command(command=f"py unreal.log('Executing Python file: {take_screenshot_py_file}');")
        editor.unreal_service.execute_console_command(command=f"py {take_screenshot_py_file}")

    with instance.end_frame():
        pass

    # wait until take_screenshot.py pushes a message to the queue created above
    take_screenshot_executing = True
    while take_screenshot_executing:
        with instance.begin_frame():
            queue_length = sp_message_queue_manager.GetQueueLength(QueueName="take_screenshot")
            take_screenshot_executing = queue_length == 0
        with instance.end_frame():
            pass
        if take_screenshot_executing:
            spear.log("Waiting for take_screenshot.py to finish executing...")
            time.sleep(1.0)

    with instance.begin_frame():
        # execute console command
        editor.unreal_service.execute_console_command(command="stat fps")

        # retrieve message pushed by take_screenshot.py
        message = sp_message_queue_manager.PopMessageFromFrontOfQueue(QueueName="take_screenshot")

        # destroy named queue
        sp_message_queue_manager.DestroyQueue(QueueName="take_screenshot")

    with instance.end_frame():
        pass

    spear.log("Message from take_screenshot.py:")
    spear.log_no_prefix(message)

    # instance.close()
    
    spear.log("Done.")
