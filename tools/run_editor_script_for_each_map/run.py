#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import os
import posixpath
import pandas as pd
import spear
import time


parser = argparse.ArgumentParser()
parser.add_argument("--maps-file", required=True)
parser.add_argument("--script")
parser.add_argument("--user-config-file")
parser.add_argument("--wait-for-assets", action="store_true")
parser.add_argument("--when-finished", default="force_close") # iterating over scenes seems to cause Unreal to hang when shutting down, so we force close by default here
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script

assert args.when_finished in ["keep_open", "close", "force_close"]


if __name__ == "__main__":

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " ".join(unknown_args)

    maps = pd.read_csv(args.maps_file, comment="#")["maps"].tolist()

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
    
            # get default objects
            asset_registry_helpers = editor.get_unreal_object(uclass="UAssetRegistryHelpers")
            kismet_system_library = editor.get_unreal_object(uclass="UKismetSystemLibrary")
            navigation_system_v1 = editor.get_unreal_object(uclass="UNavigationSystemV1")
            sp_asset_compiling_manager = editor.get_unreal_object(uclass="USpAssetCompilingManager")
            sp_level_streaming = editor.get_unreal_object(uclass="USpLevelStreaming")
            sp_navigation_system_v1 = editor.get_unreal_object(uclass="USpNavigationSystemV1")
            sp_world = editor.get_unreal_object(uclass="USpWorld")

            # get subsystems
            level_editor_subsystem = editor.unreal_service.get_editor_subsystem(uclass="ULevelEditorSubsystem")
            unreal_editor_subsystem = editor.unreal_service.get_editor_subsystem(uclass="UUnrealEditorSubsystem")

            # get asset registry
            asset_registry_handle = asset_registry_helpers.GetAssetRegistry(as_handle=True)
            asset_registry_uclass = editor.unreal_service.get_static_class(uclass="IAssetRegistry")
            asset_registry = editor.get_unreal_object(uobject=asset_registry_handle, uclass=asset_registry_uclass)

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
                    level_editor_subsystem.LoadLevel(AssetPath=m)

                # Calling LoadLevel invalidates the old editor object, so get a new one here. This call will block
                # until the new editor object is ready.

                editor = instance.get_editor(wait=True, wait_max_time_seconds=1000.0, wait_sleep_time_seconds=1.0, warm_up=True, warm_up_time_seconds=5.0, warm_up_num_frames=1)

            # Make sure the requested map was actually loaded.
            with instance.begin_frame():
                editor_world = unreal_editor_subsystem.GetEditorWorld()
                editor_world_name = kismet_system_library.GetObjectName(Object=editor_world)
                streaming_levels = sp_world.GetStreamingLevels(World=editor_world, as_unreal_object=True) # return type is "TArray" with no inner type, so we need as_unreal_object=True
            with instance.end_frame():
                pass

            assert editor_world_name == posixpath.basename(m)

            # Optionally wait for all assets to finish loading.

            if args.wait_for_assets:

                # we need to get the navigation system every time we load a new level
                with instance.begin_frame():
                    navigation_system = navigation_system_v1.GetNavigationSystem()
                with instance.end_frame():
                    pass
                                       
                editor_idle = False
                while not editor_idle:

                    with instance.begin_frame():
                        is_async_loading = instance.engine_globals_service.is_async_loading()

                        is_loading_assets = asset_registry.IsLoadingAssets()
                        num_remaining_assets = sp_asset_compiling_manager.GetNumRemainingAssets()
                        num_remaining_build_tasks = sp_navigation_system_v1.GetNumRemainingBuildTasks(NavigationSystem=navigation_system)

                        are_streaming_levels_loading = False
                        for streaming_level in streaming_levels:
                            streaming_level_name = kismet_system_library.GetObjectName(Object=streaming_level)

                            is_level_loaded = streaming_level.IsLevelLoaded()
                            is_level_visible = streaming_level.IsLevelVisible()
                            should_be_visible = sp_level_streaming.ShouldBeVisible(LevelStreaming=streaming_level)

                            spear.log(f"Streaming level: {streaming_level_name} (is_level_loaded={is_level_loaded}, should_be_visible={should_be_visible}, is_level_visible={is_level_visible}")

                            if not is_level_loaded:
                                are_streaming_levels_loading = True # if the level isn't loaded, then we're still loading

                            if should_be_visible and not is_level_visible:
                                are_streaming_levels_loading = True # if the level should be visible but it isn't, then we're still loading

                        editor_idle = not is_async_loading and not is_loading_assets and num_remaining_assets == 0 and num_remaining_build_tasks == 0 and not are_streaming_levels_loading

                        spear.log(f"editor_idle={editor_idle} (is_async_loading={is_async_loading}, is_loading_assets={is_loading_assets}, num_remaining_assets={num_remaining_assets}, num_remaining_build_tasks={num_remaining_build_tasks}, are_streaming_levels_loading={are_streaming_levels_loading})")

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
