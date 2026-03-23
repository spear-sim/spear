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

        # execute console command
        editor.unreal_service.execute_console_command(command="stat fps")

    with instance.end_frame():
        pass

    # programmatically press play in the editor
    with instance.begin_frame():
        editor.editor_python_service.execute_statement(statement="unreal.log('Calling function: ULevelEditorSubsystem::EditorRequestBeginPlay()')")
        level_editor_subsystem.EditorRequestBeginPlay()
    with instance.end_frame():
        pass

    # now that we have programmatically pressed play, we can access the live game
    game = instance.get_game()

    spear.log("Spawning actor...")

    # spawn object in the live game
    with instance.begin_frame():
        bp_axes_uclass = game.unreal_service.load_class(uclass="AActor", name="/SpContent/Blueprints/BP_Axes.BP_Axes_C")
        bp_axes = game.unreal_service.spawn_actor(uclass=bp_axes_uclass, location={"X": -10.0, "Y": 280.0, "Z": 50.0})
    with instance.end_frame():
        pass

    # sleep for a few seconds so we can see the newly spawned object in the live game
    spear.log("Sleeping for 5.0 seconds...")
    time.sleep(5.0)

    # the game will be invalidated as soon as we programmatically press stop, so we set game to None here
    game = None

    # programmatically press stop in the editor
    with instance.begin_frame():
        editor.editor_python_service.execute_statement(statement="unreal.log('Calling function: ULevelEditorSubsystem::EditorRequestEndPlay()')")
        level_editor_subsystem.EditorRequestEndPlay()
    with instance.end_frame():
        pass

    # run take_screenshot.py and wait for it to finish
    take_screenshot_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "take_screenshot.py"))
    editor.editor_python_service.execute_file_across_frames(file=take_screenshot_file, args_string="")

    with instance.begin_frame():
        editor.unreal_service.execute_console_command(command="stat fps")
    with instance.end_frame():
        pass

    spear.log("Done.")
