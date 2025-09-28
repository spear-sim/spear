#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import posixpath
import pandas as pd
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--maps_file", required=True)
parser.add_argument("--script")
parser.add_argument("--user_config_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script


if __name__ == "__main__":

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " ".join(unknown_args)

    maps = pd.read_csv(args.maps_file)["maps"].tolist()
    if len(maps) > 0:

        # get config
        config = spear.get_config(user_config_files=[args.user_config_file])

        # modify config params
        config.defrost()
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_EDITOR_STARTUP_MAP = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EDITOR_STARTUP_MAP = f"{maps[0]}.{posixpath.basename(maps[0])}"
        config.freeze()

        # create instance
        spear.configure_system(config=config)
        instance = spear.Instance(config=config)
        editor = instance.get_editor()

        # find functions and default objects
        with instance.begin_frame():
            kismet_system_library_static_class = editor.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
            level_editor_subsystem_static_class = editor.unreal_service.get_static_class(class_name="ULevelEditorSubsystem")
            unreal_editor_subsystem_static_class = editor.unreal_service.get_static_class(class_name="UUnrealEditorSubsystem")

            get_editor_world_func = editor.unreal_service.find_function_by_name(uclass=unreal_editor_subsystem_static_class, function_name="GetEditorWorld")
            get_object_name_func = editor.unreal_service.find_function_by_name(uclass=kismet_system_library_static_class, function_name="GetObjectName")
            load_level_func = editor.unreal_service.find_function_by_name(uclass=level_editor_subsystem_static_class, function_name="LoadLevel")

            kismet_system_library_default_object = editor.unreal_service.get_default_object(uclass=kismet_system_library_static_class, create_if_needed=False)
            level_editor_subsystem = editor.unreal_service.get_editor_subsystem_by_type(class_name="ULevelEditorSubsystem")
            unreal_editor_subsystem = editor.unreal_service.get_editor_subsystem_by_type(class_name="UUnrealEditorSubsystem")

        with instance.end_frame():
            pass

        for i, m in enumerate(maps):

            # Call LoadLevel. It is recommended to call LoadLevel in a "with instance.end_frame()" block, rather than
            # in a "with instance.begin_frame()" block. Calling LoadLevel in a "with instance.begin_frame()" does
            # work, but it requires the user to set the SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS config
            # parameter conservatively due to various implementation details in begin_frame() and end_frame().

            if i > 0:
                map_name = f"{m}.{posixpath.basename(m)}"
                spear.log("Loading map: ", map_name)

                with instance.begin_frame():
                    pass
                with instance.end_frame():
                    func_args = {"AssetPath": m}
                    editor.unreal_service.call_function(uobject=level_editor_subsystem, ufunction=load_level_func, args=func_args)

                # Calling LoadLevel invalidates the old editor object, so get a new one here. This call will block
                # until the new editor object is ready.

                editor = instance.get_editor(wait=True, wait_max_time_seconds=1000.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

            # Make sure the requested map was actually loaded.

            with instance.begin_frame():
                return_values = editor.unreal_service.call_function(uobject=unreal_editor_subsystem, ufunction=get_editor_world_func)
                editor_world = spear.to_handle(string=return_values["ReturnValue"])
                func_args = {"Object": spear.to_ptr(handle=editor_world)}
                return_values = editor.unreal_service.call_function(uobject=kismet_system_library_default_object, ufunction=get_object_name_func, args=func_args)
                editor_world_name = return_values["ReturnValue"]
            with instance.end_frame():
                pass

            assert editor_world_name == posixpath.basename(m)

            # Execute editor script.

            if args.script is not None:
                with instance.begin_frame():
                    cmd = f"py {args.script} {unknown_arg_string}".strip()
                    spear.log("Executing command: ", cmd)
                    editor.unreal_service.execute_console_command(cmd)
                with instance.end_frame():
                    pass

        instance.close()
    
    spear.log("Done.")
