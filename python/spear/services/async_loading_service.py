#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import time
import spear


class AsyncLoadingService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, engine_globals_service, config):
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

    def is_engine_idle(self):

        is_async_loading = self._engine_globals_service.is_async_loading()
        is_loading_assets = self._asset_registry.IsLoadingAssets()
        is_compiling_shaders = self._sp_shader_compiling_manager.IsCompiling()

        num_outstanding_distance_field_tasks = self._sp_distance_field_async_queue.GetNumOutstandingTasks()
        num_remaining_assets = self._sp_asset_compiling_manager.GetNumRemainingAssets()
        num_remaining_build_tasks = self._sp_navigation_system_v1.GetNumRemainingBuildTasks(NavigationSystem=self._navigation_system)
        num_wanting_streaming_resources = self._sp_streaming_manager.GetNumWantingResources()

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

        engine_idle = not is_async_loading and not is_loading_assets and num_remaining_assets == 0 and not is_compiling_shaders and num_remaining_build_tasks == 0 and num_wanting_streaming_resources == 0 and num_outstanding_distance_field_tasks == 0 and not are_streaming_levels_loading

        spear.log(f"engine_idle={engine_idle}")
        if not engine_idle:
            spear.log(f"    is_async_loading={is_async_loading}, is_loading_assets={is_loading_assets}, is_compiling_shaders={is_compiling_shaders}")
            spear.log(f"    num_outstanding_distance_field_tasks={num_outstanding_distance_field_tasks}, num_remaining_assets={num_remaining_assets}, num_remaining_build_tasks={num_remaining_build_tasks}, num_wanting_streaming_resources={num_wanting_streaming_resources}")
            spear.log(f"    are_streaming_levels_loading={are_streaming_levels_loading})")

        return engine_idle

    def wait_for_engine_idle(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for engine to be idle...")
        spear.log(f"Waiting for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")

        start_time_seconds = time.time()
        elapsed_time_seconds = time.time() - start_time_seconds

        engine_idle = False
        while not engine_idle:
            assert time.time() - start_time_seconds < max_time_seconds
            with engine_service.begin_frame():
                engine_idle = self.is_engine_idle()
            with engine_service.end_frame():
                pass
            if not engine_idle:
                time.sleep(sleep_time_seconds)
            elapsed_time_seconds = time.time() - start_time_seconds

        spear.log(f"Finished waiting for engine to be idle (waited for {elapsed_time_seconds:.2f} seconds).")

    def wait_for_engine_idle_in_editor_script(self, max_time_seconds=1000.0, sleep_time_seconds=1.0):
        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for engine to be idle for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")

        start_time_seconds = time.time()
        elapsed_time_seconds = time.time() - start_time_seconds

        engine_idle = False
        while not engine_idle:
            assert time.time() - start_time_seconds < max_time_seconds
            with engine_service.begin_frame():
                engine_idle = self.is_engine_idle()
            yield
            with engine_service.end_frame():
                pass
            yield
            if not engine_idle:
                time.sleep(sleep_time_seconds)
            elapsed_time_seconds = time.time() - start_time_seconds

        spear.log(f"Finished waiting for engine to be idle (waited for {elapsed_time_seconds:.2f} seconds).")
