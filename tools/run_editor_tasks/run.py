#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import json
import os
import posixpath
import spear
import time


parser = argparse.ArgumentParser()
parser.add_argument("--tasks-file", required=True)
parser.add_argument("--config-files", nargs="*")
parser.add_argument("--when-finished", default="force_close") # iterating over scenes seems to cause Unreal to hang when shutting down, so we force close by default here
args = parser.parse_args()

assert args.when_finished in ["keep_open", "close", "force_close"]


def merge_tasks(tasks):
    merged = {}
    for task in tasks:
        merged.update(task)
    return merged


def print_task(task, raw_task, prefix="    "):
    if "map" in task:
        spear.log(f"{prefix}map:                  {task['map']}{' (from default task)' if 'map' not in raw_task else ''}")
    if "wait_for_engine_idle" in task:
        spear.log(f"{prefix}wait_for_engine_idle: {task['wait_for_engine_idle']}{' (from default task)' if 'wait_for_engine_idle' not in raw_task else ''}")
    if "script" in task:
        spear.log(f"{prefix}script:               {task['script']}{' (from default task)' if 'script' not in raw_task else ''}")
    if "args" in task:
        spear.log(f"{prefix}args:                 {task['args']}{' (from default task)' if 'args' not in raw_task else ''}")
    if "across_frames" in task:
        spear.log(f"{prefix}across_frames:        {task['across_frames']}{' (from default task)' if 'across_frames' not in raw_task else ''}")


if __name__ == "__main__":

    with open(args.tasks_file) as f:
        task_descs = json.load(f)

    if "default_task" in task_descs:
        default_task = task_descs["default_task"]
    else:
        default_task = {}

    if "tasks" in task_descs:
        raw_tasks = task_descs["tasks"]
    else:
        raw_tasks = []

    if len(raw_tasks) > 0:

        # get task with default values
        initial_task = merge_tasks([default_task, raw_tasks[0]])

        # get config
        user_config_files = [os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))]
        if args.config_files is not None:
            user_config_files = user_config_files + args.config_files
        config = spear.get_config(user_config_files=user_config_files)

        # modify config params
        if "map" in initial_task:
            current_map = initial_task["map"]
            map_path = current_map
            map_name = f"{map_path}.{posixpath.basename(map_path)}"
            config.defrost()
            config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_EDITOR_STARTUP_MAP = True
            config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EDITOR_STARTUP_MAP = map_name
            config.freeze()
        else:
            current_map = None

        # create instance
        spear.configure_system(config=config)
        instance = spear.Instance(config=config)

        if "wait_for_engine_idle" in initial_task and not initial_task["wait_for_engine_idle"]:
            editor = instance.get_editor(wait_for_engine_idle=False)
        else:
            editor = instance.get_editor()

        with instance.begin_frame():
            kismet_system_library = editor.get_unreal_object(uclass="UKismetSystemLibrary")
            level_editor_subsystem = editor.unreal_service.get_editor_subsystem(uclass="ULevelEditorSubsystem")
            unreal_editor_subsystem = editor.unreal_service.get_editor_subsystem(uclass="UUnrealEditorSubsystem")
        with instance.end_frame():
            pass

        for raw_task in raw_tasks:

            # get task with default values
            task = merge_tasks([default_task, raw_task])

            spear.log(f"Task:")
            print_task(task=task, raw_task=raw_task, prefix="    ")

            # optionally load a map if it is specified and it is different from current map.

            if "map" in task and task["map"] != current_map:
                map_path = task["map"]
                map_name = f"{map_path}.{posixpath.basename(map_path)}"

                with instance.begin_frame():
                    pass
                with instance.end_frame():
                    editor.invalidate() # need to call invalidate() before calling OpenLevel()
                    level_editor_subsystem.LoadLevel(AssetPath=map_path)

                # Calling LoadLevel invalidates the old editor object, so get a new one here. This call will block
                # until the new editor object is ready.

                if "wait_for_engine_idle" in task and not task["wait_for_engine_idle"]:
                    editor = instance.get_editor(wait_for_engine_idle=False)
                else:
                    editor = instance.get_editor()

                # make sure the requested map was actually loaded.
                with instance.begin_frame():
                    editor_world = unreal_editor_subsystem.GetEditorWorld()
                    editor_world_name = kismet_system_library.GetObjectName(Object=editor_world)
                with instance.end_frame():
                    pass

                assert editor_world_name == posixpath.basename(map_path)
                current_map = map_path

            # optionally execute a script.

            if "script" in task:
                script = task["script"]

            if "args" in task:
                args_string = task["args"]
            else:
                args_string = ""

            if "across_frames" in task:
                assert "script" in task
                across_frames = task["across_frames"]
            else:
                across_frames = False

            if "script" in task:
                if across_frames:
                    with instance.begin_frame():
                        editor.python_service.execute_file_across_frames(file=script, args_string=args_string, execution_scope="Private")
                    with instance.end_frame():
                        pass
                else:
                    with instance.begin_frame():
                        editor.python_service.execute_file(file=script, args_string=args_string, execution_scope="Private")
                    with instance.end_frame():
                        pass

        # Give the editor a bit of extra time before shutting down to prevent data races with the PakPlatformFile
        # system, which can cause the editor to hang after iterating over scenes.
        time.sleep(1.0)

        if args.when_finished == "close":
            instance.close()
        elif args.when_finished == "force_close":
            instance.close(force=True)

    spear.log("Done.")
