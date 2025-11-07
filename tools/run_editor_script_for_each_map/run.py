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
import time


parser = argparse.ArgumentParser()
parser.add_argument("--maps_file", required=True)
parser.add_argument("--script")
parser.add_argument("--user_config_file")
parser.add_argument("--wait_for_assets", action="store_true")
parser.add_argument("--when_finished", default="force_kill") # iterating over scenes seems to cause Unreal to hang when shutting down, so we force kill by default here
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script

assert args.when_finished in ["keep_open", "close", "force_close"]


if __name__ == "__main__":

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " ".join(unknown_args)

    maps = pd.read_csv(args.maps_file)["maps"].tolist()
    if len(maps) > 0:

        # get config
        user_config_files = [os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))]
        if args.user_config_file is not None:
            user_config_files.append(args.user_config_file)
        config = spear.get_config(user_config_files=user_config_files)

        # modify config params
        config.defrost()
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.OVERRIDE_EDITOR_STARTUP_MAP = True
        config.SP_SERVICES.INITIALIZE_ENGINE_SERVICE.EDITOR_STARTUP_MAP = f"{maps[0]}.{posixpath.basename(maps[0])}"
        config.freeze()

        # create instance
        spear.configure_system(config=config)
        instance = spear.Instance(config=config)
        editor = instance.get_editor()

        with instance.begin_frame():
    
            # find functions and objects

            # The UAssetRegistryHelpers class can't be registered like other classes because it doesn't
            # expose its StaticClass() method in a way that is accessible from other modules. So we obtain
            # its static class using load_object(...) instead, which can be useful in cases where a class
            # isn't registered with our UnrealClassRegistry system ahead of time.

            asset_registry_helpers_uclass = editor.unreal_service.load_object(class_name="UClass", outer=0, name="/Script/AssetRegistry.AssetRegistryHelpers")
            get_asset_registry_func = editor.unreal_service.find_function_by_name(uclass=asset_registry_helpers_uclass, function_name="GetAssetRegistry")

            # Use get_static_class(...) for our other classes.

            asset_registry_static_class = editor.unreal_service.get_static_class(class_name="IAssetRegistry")
            is_loading_assets_func = editor.unreal_service.find_function_by_name(uclass=asset_registry_static_class, function_name="IsLoadingAssets")

            kismet_system_library_static_class = editor.unreal_service.get_static_class(class_name="UKismetSystemLibrary")
            get_object_name_func = editor.unreal_service.find_function_by_name(uclass=kismet_system_library_static_class, function_name="GetObjectName")

            level_editor_subsystem_static_class = editor.unreal_service.get_static_class(class_name="ULevelEditorSubsystem")
            load_level_func = editor.unreal_service.find_function_by_name(uclass=level_editor_subsystem_static_class, function_name="LoadLevel")

            level_streaming_static_class = editor.unreal_service.get_static_class(class_name="ULevelStreaming")
            is_level_loaded_func = editor.unreal_service.find_function_by_name(uclass=level_streaming_static_class, function_name="IsLevelLoaded")
            is_level_visible_func = editor.unreal_service.find_function_by_name(uclass=level_streaming_static_class, function_name="IsLevelVisible")

            sp_asset_compiling_manager_static_class = editor.unreal_service.get_static_class(class_name="USpAssetCompilingManager")
            get_num_remaining_assets_func = editor.unreal_service.find_function_by_name(uclass=sp_asset_compiling_manager_static_class, function_name="GetNumRemainingAssets")

            sp_level_streaming_static_class = editor.unreal_service.get_static_class(class_name="USpLevelStreaming")
            should_be_visible_func = editor.unreal_service.find_function_by_name(uclass=sp_level_streaming_static_class, function_name="ShouldBeVisible")

            sp_world_static_class = editor.unreal_service.get_static_class(class_name="USpWorld")
            get_streaming_levels_func = editor.unreal_service.find_function_by_name(uclass=sp_world_static_class, function_name="GetStreamingLevels")

            unreal_editor_subsystem_static_class = editor.unreal_service.get_static_class(class_name="UUnrealEditorSubsystem")
            get_editor_world_func = editor.unreal_service.find_function_by_name(uclass=unreal_editor_subsystem_static_class, function_name="GetEditorWorld")

            # get default objects
            asset_registry_helpers_default_object = editor.unreal_service.get_default_object(uclass=asset_registry_helpers_uclass, create_if_needed=False)
            kismet_system_library_default_object = editor.unreal_service.get_default_object(uclass=kismet_system_library_static_class, create_if_needed=False)
            sp_asset_compiling_manager_default_object = editor.unreal_service.get_default_object(uclass=sp_asset_compiling_manager_static_class, create_if_needed=False)
            sp_level_streaming_default_object = editor.unreal_service.get_default_object(uclass=sp_level_streaming_static_class, create_if_needed=False)
            sp_world_default_object = editor.unreal_service.get_default_object(uclass=sp_world_static_class, create_if_needed=False)

            # get subsystems
            level_editor_subsystem = editor.unreal_service.get_editor_subsystem_by_type(class_name="ULevelEditorSubsystem")
            unreal_editor_subsystem = editor.unreal_service.get_editor_subsystem_by_type(class_name="UUnrealEditorSubsystem")

            # get asset registry
            return_values = editor.unreal_service.call_function(uobject=asset_registry_helpers_default_object, ufunction=get_asset_registry_func)
            asset_registry_string = return_values["ReturnValue"]
            asset_registry = spear.to_handle(string=asset_registry_string)

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
                editor_world_string = return_values["ReturnValue"]
                editor_world = spear.to_handle(string=editor_world_string)
                func_args = {"Object": spear.to_ptr(handle=editor_world)}
                return_values = editor.unreal_service.call_function(uobject=kismet_system_library_default_object, ufunction=get_object_name_func, args=func_args)
                editor_world_name = return_values["ReturnValue"]

            with instance.end_frame():
                pass

            assert editor_world_name == posixpath.basename(m)

            # Optionally wait for all assets to finish loading.

            if args.wait_for_assets:

                editor_idle = False
                while not editor_idle:

                    with instance.begin_frame():
                        return_values = editor.unreal_service.call_function(uobject=sp_asset_compiling_manager_default_object, ufunction=get_num_remaining_assets_func)
                        num_remaining_assets = return_values["ReturnValue"]

                        is_async_loading = instance.engine_service.is_async_loading()

                        func_args = {"World": spear.to_ptr(handle=editor_world)}
                        return_values = editor.unreal_service.call_function(uobject=sp_world_default_object, ufunction=get_streaming_levels_func, args=func_args)

                        are_streaming_levels_loading = False
                        streaming_levels = [ spear.to_handle(string=string) for string in return_values["ReturnValue"] ]
                        for streaming_level in streaming_levels:
                            func_args = {"Object": spear.to_ptr(handle=streaming_level)}
                            return_values = editor.unreal_service.call_function(uobject=kismet_system_library_default_object, ufunction=get_object_name_func, args=func_args)
                            streaming_level_name = return_values["ReturnValue"]

                            return_values = editor.unreal_service.call_function(uobject=streaming_level, ufunction=is_level_loaded_func)
                            is_level_loaded = return_values["ReturnValue"]

                            return_values = editor.unreal_service.call_function(uobject=sp_level_streaming_default_object, ufunction=should_be_visible_func, args={"LevelStreaming": spear.to_ptr(streaming_level)})
                            should_be_visible = return_values["ReturnValue"]

                            return_values = editor.unreal_service.call_function(uobject=streaming_level, ufunction=is_level_visible_func)
                            is_level_visible = return_values["ReturnValue"]

                            spear.log(f"Streaming level: {streaming_level_name} (is_level_loaded={is_level_loaded}, should_be_visible={should_be_visible}, is_level_visible={is_level_visible}")

                            if not is_level_loaded:
                                are_streaming_levels_loading = True

                            if should_be_visible and not is_level_visible:
                                are_streaming_levels_loading = True

                        return_values = editor.unreal_service.call_function(uobject=asset_registry, ufunction=is_loading_assets_func)
                        is_loading_assets = return_values["ReturnValue"]

                        editor_idle = num_remaining_assets == 0 and not is_async_loading and not are_streaming_levels_loading and not is_loading_assets

                        spear.log(f"num_remaining_assets={num_remaining_assets}, is_async_loading={is_async_loading}, are_streaming_levels_loading={are_streaming_levels_loading}, is_loading_assets={is_loading_assets}")

                    with instance.end_frame():
                        pass

                    if not editor_idle:
                        time.sleep(1.0)

            # Execute editor script.

            if args.script is not None:
                with instance.begin_frame():
                    cmd = f"py {args.script} {unknown_arg_string}".strip()
                    spear.log("Executing command: ", cmd)
                    editor.unreal_service.execute_console_command(cmd)
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
