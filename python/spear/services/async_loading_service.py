#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import time
import spear


class AsyncLoadingService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, engine_globals_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()
        assert engine_globals_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self._engine_globals_service = engine_globals_service

        self._sp_asset_compiling_manager = None
        self._sp_distance_field_async_queue = None
        self._sp_level_streaming = None
        self._sp_navigation_system_v1 = None
        self._sp_shader_compiling_manager = None
        self._sp_streaming_manager = None
        self._sp_world = None
        self._asset_registry = None
        self._navigation_system = None

    def initialize(self):
        self._sp_asset_compiling_manager = self.get_unreal_object(uclass="USpAssetCompilingManager")
        self._sp_distance_field_async_queue = self.get_unreal_object(uclass="USpDistanceFieldAsyncQueue")
        self._sp_level_streaming = self.get_unreal_object(uclass="USpLevelStreaming")
        self._sp_navigation_system_v1 = self.get_unreal_object(uclass="USpNavigationSystemV1")
        self._sp_shader_compiling_manager = self.get_unreal_object(uclass="USpShaderCompilingManager")
        self._sp_streaming_manager = self.get_unreal_object(uclass="USpStreamingManager")
        self._sp_world = self.get_unreal_object(uclass="USpWorld")

        asset_registry_helpers = self.get_unreal_object(uclass="UAssetRegistryHelpers")
        asset_registry_uclass = self.unreal_service.get_static_class(uclass="IAssetRegistry")
        asset_registry_handle = asset_registry_helpers.GetAssetRegistry(as_handle=True)
        self._asset_registry = self.get_unreal_object(uobject=asset_registry_handle, uclass=asset_registry_uclass)

        self._navigation_system = self._sp_navigation_system_v1.GetNavigationSystem()

    #
    # Catch-all functions
    #

    def is_engine_idle(self, verbose=False):

        async_loading_idle = self.is_async_loading_idle()
        asset_registry_idle = self.is_asset_registry_idle()
        distance_field_async_queue_idle, num_outstanding_distance_field_tasks, is_distance_field_async_queue_initialized = self.is_distance_field_async_queue_idle()
        asset_compiling_manager_idle, num_remaining_assets = self.is_asset_compiling_manager_idle()
        navigation_system_idle, num_remaining_build_tasks = self.is_navigation_system_idle()
        shader_compiling_manager_idle, num_remaining_shader_jobs, is_shader_compiling_manager_initialized = self.is_shader_compiling_manager_idle()
        streaming_manager_idle, num_wanting_streaming_resources = self.is_streaming_manager_idle()
        streaming_levels_idle = self.are_streaming_levels_idle()

        engine_idle = \
            async_loading_idle and \
            asset_registry_idle and \
            distance_field_async_queue_idle and \
            asset_compiling_manager_idle and \
            navigation_system_idle and \
            shader_compiling_manager_idle and \
            streaming_manager_idle and \
            streaming_levels_idle

        if verbose:
            if not engine_idle or not is_distance_field_async_queue_initialized or not is_shader_compiling_manager_initialized:
                spear.log(f"engine_idle = {engine_idle}")
                spear.log(f"    is_async_loading                     = {not async_loading_idle}")
                spear.log(f"    is_loading_assets                    = {not asset_registry_idle}")
                spear.log(f"    num_outstanding_distance_field_tasks = {num_outstanding_distance_field_tasks} (initialized = {is_distance_field_async_queue_initialized})")
                spear.log(f"    num_remaining_assets                 = {num_remaining_assets}")
                spear.log(f"    num_remaining_build_tasks            = {num_remaining_build_tasks}")
                spear.log(f"    num_remaining_shader_jobs            = {num_remaining_shader_jobs} (initialized = {is_shader_compiling_manager_initialized})")
                spear.log(f"    num_wanting_streaming_resources      = {num_wanting_streaming_resources}")
                spear.log(f"    are_streaming_levels_loading         = {not streaming_levels_idle}")

        return engine_idle

    def wait_for_engine_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_engine_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_engine_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_engine_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    #
    # Fine-grained functions
    #

    def is_asset_compiling_manager_idle(self, verbose=False):
        num_remaining_assets = self._sp_asset_compiling_manager.GetNumRemainingAssets()
        asset_compiling_manager_idle = num_remaining_assets == 0
        if verbose:
            spear.log(f"num_remaining_assets = {num_remaining_assets}")
        return asset_compiling_manager_idle, num_remaining_assets

    def is_asset_registry_idle(self, verbose=False):
        is_loading_assets = self._asset_registry.IsLoadingAssets()
        asset_registry_idle = not is_loading_assets
        if verbose:
            spear.log(f"is_loading_assets = {is_loading_assets}")
        return asset_registry_idle

    def is_async_loading_idle(self, verbose=False):
        is_async_loading = self._engine_globals_service.is_async_loading()
        async_loading_idle = not is_async_loading
        if verbose:
            spear.log(f"is_async_loading = {is_async_loading}")
        return async_loading_idle

    def is_distance_field_async_queue_idle(self, verbose=False):
        return_values = self._sp_distance_field_async_queue.GetNumOutstandingTasks(as_dict=True)
        num_outstanding_distance_field_tasks = return_values["ReturnValue"]
        is_distance_field_async_queue_initialized = return_values["bIsInitialized"]
        distance_field_async_queue_idle = num_outstanding_distance_field_tasks == 0
        if verbose:
            spear.log(f"num_outstanding_distance_field_tasks = {num_outstanding_distance_field_tasks} (initialized = {is_distance_field_async_queue_initialized})")
        return distance_field_async_queue_idle, num_outstanding_distance_field_tasks, is_distance_field_async_queue_initialized

    def is_navigation_system_idle(self, verbose=False):
        num_remaining_build_tasks = self._sp_navigation_system_v1.GetNumRemainingBuildTasks(NavigationSystem=self._navigation_system)
        navigation_system_idle = num_remaining_build_tasks == 0
        if verbose:
            spear.log(f"num_remaining_build_tasks = {num_remaining_build_tasks}")
        return navigation_system_idle, num_remaining_build_tasks

    def is_shader_compiling_manager_idle(self, verbose=False):
        return_values = self._sp_shader_compiling_manager.GetNumRemainingJobs(as_dict=True)
        num_remaining_shader_jobs = return_values["ReturnValue"]
        is_shader_compiling_manager_initialized = return_values["bIsInitialized"]
        shader_compiling_manager_idle = num_remaining_shader_jobs == 0
        if verbose:
            spear.log(f"num_remaining_shader_jobs = {num_remaining_shader_jobs} (initialized = {is_shader_compiling_manager_initialized})")
        return shader_compiling_manager_idle, num_remaining_shader_jobs, is_shader_compiling_manager_initialized

    def are_streaming_levels_idle(self, verbose=False):
        are_streaming_levels_loading = False
        world = self.get_world()
        streaming_levels = self._sp_world.GetStreamingLevels(World=spear.to_ptr(handle=world))
        for streaming_level in streaming_levels:
            is_level_loaded = streaming_level.IsLevelLoaded()
            is_level_visible = streaming_level.IsLevelVisible()
            should_be_visible = self._sp_level_streaming.ShouldBeVisible(LevelStreaming=streaming_level)
            if not is_level_loaded:
                are_streaming_levels_loading = True
            if should_be_visible and not is_level_visible:
                are_streaming_levels_loading = True
        streaming_levels_idle = not are_streaming_levels_loading
        if verbose:
            spear.log(f"are_streaming_levels_loading = {are_streaming_levels_loading}")
        return streaming_levels_idle

    def is_streaming_manager_idle(self, verbose=False):
        num_wanting_streaming_resources = self._sp_streaming_manager.GetNumWantingResources()
        streaming_manager_idle = num_wanting_streaming_resources == 0
        if verbose:
            spear.log(f"num_wanting_streaming_resources = {num_wanting_streaming_resources}")
        return streaming_manager_idle, num_wanting_streaming_resources

    def wait_for_asset_compiling_manager_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_asset_compiling_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_asset_compiling_manager_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_asset_compiling_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_asset_registry_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_asset_registry_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_asset_registry_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_asset_registry_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_async_loading_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_async_loading_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_async_loading_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_async_loading_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_distance_field_async_queue_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_distance_field_async_queue_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_distance_field_async_queue_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_distance_field_async_queue_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_navigation_system_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_navigation_system_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_navigation_system_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_navigation_system_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_shader_compiling_manager_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_shader_compiling_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_shader_compiling_manager_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_shader_compiling_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_streaming_levels_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.are_streaming_levels_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_streaming_levels_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.are_streaming_levels_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_streaming_manager_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        self._wait_for(func=self.is_streaming_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def wait_for_streaming_manager_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_streaming_manager_idle, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    #
    # Private helper functions
    #

    def _wait_for(self, func, max_time_seconds, sleep_time_seconds):
        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for function to return true: {func_name}")
        spear.log(f"Waiting for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")

        start_time_seconds = time.time()
        elapsed_time_seconds = time.time() - start_time_seconds

        success = False
        while not success:
            assert time.time() - start_time_seconds < max_time_seconds
            with engine_service.begin_frame():
                result = func(verbose=True)
                success = result[0] if isinstance(result, tuple) else result
            with engine_service.end_frame():
                pass
            if not success:
                time.sleep(sleep_time_seconds)
            elapsed_time_seconds = time.time() - start_time_seconds

        spear.log(f"Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")

    def _wait_for_in_editor_script(self, func, max_time_seconds, sleep_time_seconds):
        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for function to return true: {func_name}")
        spear.log(f"Waiting for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")

        start_time_seconds = time.time()
        elapsed_time_seconds = time.time() - start_time_seconds

        success = False
        while not success:
            assert time.time() - start_time_seconds < max_time_seconds
            with engine_service.begin_frame():
                result = func(verbose=True)
                success = result[0] if isinstance(result, tuple) else result
            yield
            with engine_service.end_frame():
                pass
            yield
            if not success:
                time.sleep(sleep_time_seconds)
            elapsed_time_seconds = time.time() - start_time_seconds

        spear.log(f"Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")
