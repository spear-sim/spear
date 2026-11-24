#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import time
import spear


class AsyncLoadingService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, engine_globals_service, cdo, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()
        assert engine_globals_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            cdo=cdo,
            config=config)

        self._engine_globals_service = engine_globals_service
        self._asset_registry = None

        self._landscape_subsystem = None
        self._world_partition_subsystem = None

    def initialize(self):
        asset_registry_uclass = self.unreal_service.get_static_class(uclass="IAssetRegistry")
        asset_registry_handle = self.cdo.UAssetRegistryHelpers.GetAssetRegistry(as_handle=True)
        self._asset_registry = self.get_unreal_object(uobject=asset_registry_handle, uclass=asset_registry_uclass)

        self._landscape_subsystem = self.unreal_service.get_subsystem(subsystem_provider_class_name="UWorld", subsystem_uclass="ULandscapeSubsystem")
        self._world_partition_subsystem = self.unreal_service.get_subsystem(subsystem_provider_class_name="UWorld", subsystem_uclass="UWorldPartitionSubsystem")

    #
    # Catch-all functions
    #

    def is_engine_idle(self, verbose=False, return_detailed_info=False):

        is_with_editor = self._engine_globals_service.is_with_editor()

        asset_compiling_manager_idle, asset_compiling_manager_detailed_info = self.is_asset_compiling_manager_idle(return_detailed_info=True)
        asset_registry_idle, asset_registry_detailed_info = self.is_asset_registry_idle(return_detailed_info=True)
        async_loading_idle, async_loading_detailed_info = self.is_async_loading_idle(return_detailed_info=True)
        distance_field_async_queue_idle, distance_field_async_queue_detailed_info = self.is_distance_field_async_queue_idle(return_detailed_info=True)
        pipeline_state_cache_idle, pipeline_state_cache_detailed_info = self.is_pipeline_state_cache_idle(return_detailed_info=True)
        shader_compiling_manager_idle, shader_compiling_manager_detailed_info = self.is_shader_compiling_manager_idle(return_detailed_info=True)
        streaming_manager_idle, streaming_manager_detailed_info = self.is_streaming_manager_idle(return_detailed_info=True)
        streaming_levels_idle, streaming_levels_detailed_info = self.are_streaming_levels_idle(return_detailed_info=True)
        world_partition_streaming_idle, world_partition_streaming_detailed_info = self.is_world_partition_streaming_idle(return_detailed_info=True)

        engine_idle = \
            asset_compiling_manager_idle and \
            asset_registry_idle and \
            async_loading_idle and \
            distance_field_async_queue_idle and \
            pipeline_state_cache_idle and \
            shader_compiling_manager_idle and \
            streaming_manager_idle and \
            streaming_levels_idle and \
            world_partition_streaming_idle

        detailed_info = {}
        detailed_info.update(asset_compiling_manager_detailed_info)
        detailed_info.update(asset_registry_detailed_info)
        detailed_info.update(async_loading_detailed_info)
        detailed_info.update(distance_field_async_queue_detailed_info)
        detailed_info.update(pipeline_state_cache_detailed_info)
        detailed_info.update(shader_compiling_manager_detailed_info)
        detailed_info.update(streaming_manager_detailed_info)
        detailed_info.update(streaming_levels_detailed_info)
        detailed_info.update(world_partition_streaming_detailed_info)

        if verbose:
            is_distance_field_async_queue_initialized = detailed_info["is_distance_field_async_queue_initialized"]
            is_shader_compiling_manager_initialized = detailed_info["is_shader_compiling_manager_initialized"]
            if not engine_idle or (is_with_editor and not is_distance_field_async_queue_initialized) or (is_with_editor and not is_shader_compiling_manager_initialized):
                spear.log(f"engine_idle = {engine_idle} (is_with_editor = {is_with_editor})")
                spear.log(f"    asset_compiling_manager_idle    = {asset_compiling_manager_idle} (asset_compiling_manager_num_remaining_assets = {detailed_info['asset_compiling_manager_num_remaining_assets']})")
                spear.log(f"    asset_registry_idle             = {asset_registry_idle}")
                spear.log(f"    async_loading_idle              = {async_loading_idle}")
                spear.log(f"    distance_field_async_queue_idle = {distance_field_async_queue_idle} (num_outstanding_distance_field_tasks = {detailed_info['num_outstanding_distance_field_tasks']}, is_distance_field_async_queue_initialized = {is_distance_field_async_queue_initialized})")
                spear.log(f"    pipeline_state_cache_idle       = {pipeline_state_cache_idle} (num_active_pipeline_precompile_tasks = {detailed_info['num_active_pipeline_precompile_tasks']}, is_pipeline_state_cache_precaching = {detailed_info['is_pipeline_state_cache_precaching']}, num_active_precache_requests = {detailed_info['num_active_precache_requests']})")
                spear.log(f"    shader_compiling_manager_idle   = {shader_compiling_manager_idle} (shader_compiling_manager_num_remaining_shader_jobs = {detailed_info['shader_compiling_manager_num_remaining_shader_jobs']}, is_shader_compiling_manager_initialized = {is_shader_compiling_manager_initialized})")
                spear.log(f"    streaming_manager_idle          = {streaming_manager_idle} (streaming_manager_num_pending_requests = {detailed_info['streaming_manager_num_pending_requests']}, streaming_manager_num_wanting_resources = {detailed_info['streaming_manager_num_wanting_resources']})")
                spear.log(f"    streaming_levels_idle           = {streaming_levels_idle}")
                spear.log(f"    world_partition_streaming_idle  = {world_partition_streaming_idle}")

        if return_detailed_info:
            return engine_idle, detailed_info
        else:
            return engine_idle

    def wait_for_engine_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_engine_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_engine_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_engine_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    #
    # Fine-grained functions
    #

    def is_asset_compiling_manager_idle(self, verbose=False, return_detailed_info=False):
        asset_compiling_manager_num_remaining_assets = self.cdo.USpAssetCompilingManager.GetNumRemainingAssets()
        asset_compiling_manager_idle = asset_compiling_manager_num_remaining_assets == 0
        if verbose:
            spear.log(f"asset_compiling_manager_idle = {asset_compiling_manager_idle} (asset_compiling_manager_num_remaining_assets = {asset_compiling_manager_num_remaining_assets})")
        if return_detailed_info:
            detailed_info = {"asset_compiling_manager_num_remaining_assets": asset_compiling_manager_num_remaining_assets}
            return asset_compiling_manager_idle, detailed_info
        else:
            return asset_compiling_manager_idle

    def is_asset_registry_idle(self, verbose=False, return_detailed_info=False):
        asset_registry_idle = not self._asset_registry.IsLoadingAssets()
        if verbose:
            spear.log(f"asset_registry_idle = {asset_registry_idle}")
        if return_detailed_info:
            detailed_info = {}
            return asset_registry_idle, detailed_info
        else:
            return asset_registry_idle

    def is_async_loading_idle(self, verbose=False, return_detailed_info=False):
        async_loading_idle = not self._engine_globals_service.is_async_loading()
        if verbose:
            spear.log(f"async_loading_idle = {async_loading_idle}")
        if return_detailed_info:
            detailed_info = {}
            return async_loading_idle, detailed_info
        else:
            return async_loading_idle

    def is_distance_field_async_queue_idle(self, verbose=False, return_detailed_info=False):
        return_values = self.cdo.USpDistanceFieldAsyncQueue.GetNumOutstandingTasks(as_dict=True)
        num_outstanding_distance_field_tasks = return_values["ReturnValue"]
        is_distance_field_async_queue_initialized = return_values["bIsInitialized"]
        distance_field_async_queue_idle = num_outstanding_distance_field_tasks == 0
        if verbose:
            spear.log(f"distance_field_async_queue_idle = {distance_field_async_queue_idle} (num_outstanding_distance_field_tasks = {num_outstanding_distance_field_tasks}, is_distance_field_async_queue_initialized = {is_distance_field_async_queue_initialized})")
        if return_detailed_info:
            detailed_info = {
                "num_outstanding_distance_field_tasks": num_outstanding_distance_field_tasks,
                "is_distance_field_async_queue_initialized": is_distance_field_async_queue_initialized}
            return distance_field_async_queue_idle, detailed_info
        else:
            return distance_field_async_queue_idle

    def is_pipeline_state_cache_idle(self, verbose=False, return_detailed_info=False):
        num_active_pipeline_precompile_tasks = self.cdo.USpPipelineStateCache.GetNumActivePipelinePrecompileTasks()
        is_pipeline_state_cache_precaching = self.cdo.USpPipelineStateCache.IsPrecaching()
        num_active_precache_requests = self.cdo.USpPipelineStateCache.NumActivePrecacheRequests()
        pipeline_state_cache_idle = num_active_pipeline_precompile_tasks == 0 and not is_pipeline_state_cache_precaching and num_active_precache_requests == 0
        if verbose:
            spear.log(f"pipeline_state_cache_idle = {pipeline_state_cache_idle} (num_active_pipeline_precompile_tasks = {num_active_pipeline_precompile_tasks}, is_pipeline_state_cache_precaching = {is_pipeline_state_cache_precaching}, num_active_precache_requests = {num_active_precache_requests})")
        if return_detailed_info:
            detailed_info = {
                "num_active_pipeline_precompile_tasks": num_active_pipeline_precompile_tasks,
                "is_pipeline_state_cache_precaching": is_pipeline_state_cache_precaching,
                "num_active_precache_requests": num_active_precache_requests}
            return pipeline_state_cache_idle, detailed_info
        else:
            return pipeline_state_cache_idle

    def is_shader_compiling_manager_idle(self, verbose=False, return_detailed_info=False):
        return_values = self.cdo.USpShaderCompilingManager.GetNumRemainingJobs(as_dict=True)
        shader_compiling_manager_num_remaining_shader_jobs = return_values["ReturnValue"]
        is_shader_compiling_manager_initialized = return_values["bIsInitialized"]
        shader_compiling_manager_idle = shader_compiling_manager_num_remaining_shader_jobs == 0
        if verbose:
            spear.log(f"shader_compiling_manager_idle = {shader_compiling_manager_idle} (shader_compiling_manager_num_remaining_shader_jobs = {shader_compiling_manager_num_remaining_shader_jobs}, is_shader_compiling_manager_initialized = {is_shader_compiling_manager_initialized})")
        if return_detailed_info:
            detailed_info = {
                "shader_compiling_manager_num_remaining_shader_jobs": shader_compiling_manager_num_remaining_shader_jobs,
                "is_shader_compiling_manager_initialized": is_shader_compiling_manager_initialized}
            return shader_compiling_manager_idle, detailed_info
        else:
            return shader_compiling_manager_idle

    def are_streaming_levels_idle(self, verbose=False, return_detailed_info=False):
        are_streaming_levels_loading = False
        world = self.get_world()
        streaming_levels = self.cdo.USpWorld.GetStreamingLevels(World=spear.to_handle(obj=world))
        for streaming_level in streaming_levels:
            if self.cdo.USpLevelStreaming.IsStreamingStatePending(LevelStreaming=streaming_level):
                are_streaming_levels_loading = True
        streaming_levels_idle = not are_streaming_levels_loading
        if verbose:
            spear.log(f"streaming_levels_idle = {streaming_levels_idle}")
        if return_detailed_info:
            detailed_info = {}
            return streaming_levels_idle, detailed_info
        else:
            return streaming_levels_idle

    def is_streaming_manager_idle(self, verbose=False, return_detailed_info=False):
        streaming_manager_num_pending_requests = self.cdo.USpStreamingManager.StreamAllResources(TimeLimit=0.01)
        streaming_manager_num_wanting_resources = self.cdo.USpStreamingManager.GetNumWantingResources()
        streaming_manager_idle = streaming_manager_num_pending_requests == 0 and streaming_manager_num_wanting_resources == 0
        if verbose:
            spear.log(f"streaming_manager_idle = {streaming_manager_idle} (streaming_manager_num_pending_requests = {streaming_manager_num_pending_requests}, streaming_manager_num_wanting_resources = {streaming_manager_num_wanting_resources})")
        if return_detailed_info:
            detailed_info = {
                "streaming_manager_num_pending_requests": streaming_manager_num_pending_requests,
                "streaming_manager_num_wanting_resources": streaming_manager_num_wanting_resources}
            return streaming_manager_idle, detailed_info
        else:
            return streaming_manager_idle

    def is_world_partition_streaming_idle(self, verbose=False, return_detailed_info=False):
        world_partition_streaming_idle = self._world_partition_subsystem.IsAllStreamingCompleted()
        if verbose:
            spear.log(f"world_partition_streaming_idle = {world_partition_streaming_idle}")
        if return_detailed_info:
            detailed_info = {}
            return world_partition_streaming_idle, detailed_info
        else:
            return world_partition_streaming_idle

    def wait_for_asset_compiling_manager_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_asset_compiling_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_asset_compiling_manager_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_asset_compiling_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_asset_registry_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_asset_registry_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_asset_registry_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_asset_registry_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_async_loading_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_async_loading_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_async_loading_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_async_loading_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_distance_field_async_queue_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_distance_field_async_queue_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_distance_field_async_queue_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_distance_field_async_queue_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_pipeline_state_cache_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_pipeline_state_cache_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_pipeline_state_cache_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_pipeline_state_cache_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_shader_compiling_manager_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_shader_compiling_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_shader_compiling_manager_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_shader_compiling_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_streaming_levels_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.are_streaming_levels_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_streaming_levels_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.are_streaming_levels_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_streaming_manager_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_streaming_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_streaming_manager_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_streaming_manager_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_world_partition_streaming_idle(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        self._wait_for(func=self.is_world_partition_streaming_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    def wait_for_world_partition_streaming_idle_in_editor_script(self, max_time_seconds=1000.0, log_interval_seconds=1.0):
        yield from self._wait_for_in_editor_script(func=self.is_world_partition_streaming_idle, max_time_seconds=max_time_seconds, log_interval_seconds=log_interval_seconds)

    #
    # Private helper functions
    #

    def _force_update_engine(self):
        # Force update async-loading engine systems in the same order as UMoviePipeline::FlushAsyncEngineSystems()
        world = self.get_world()
        feature_level = self.cdo.USpWorld.GetFeatureLevel(World=spear.to_handle(obj=world))
        self.cdo.USpMaterialInterface.SubmitRemainingJobsForWorld(World=spear.to_handle(obj=world), CompileMode="Background")
        self.cdo.USpStreamingManager.StreamAllResources(TimeLimit=0.1)
        self.cdo.USpLandscapeSubsystem.RegenerateGrass(LandscapeSubsystem=self._landscape_subsystem, bFlushGrass=False, bForceSync=True, bOverrideCameraLocations=True, CameraLocations=[])
        self.cdo.USpRendererModule.LoadPendingVirtualTextureTiles(FeatureLevel=feature_level)

    def _wait_for(self, func, max_time_seconds, log_interval_seconds):
        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for function to return true: {func_name}")
        spear.log(f"Waiting for up to {max_time_seconds} seconds, logging progress every {log_interval_seconds} seconds...")

        start_time_seconds = time.time()
        last_verbose_time_seconds = start_time_seconds

        # Advance a fixed number of frames so that a capture component initialized in the caller's most recent
        # begin_frame block is fully initialized and rendering before the first idle check (otherwise the first
        # check could observe a premature idle state before the component is rendering).
        num_warmup_frames = 2
        engine_service.step(num_frames=num_warmup_frames)

        # We don't sleep between iterations; each iteration already does bounded work (advancing frames, which lets
        # compilation and streaming finalize, and StreamAllResources(...) blocking briefly), so the loop is self-paced.
        # We require the idle signal to hold for a fixed number of consecutive frames before stopping: each of
        # those frames still runs _force_update_engine(), so if a streamed-in level's primitives become render-stated
        # during one of them, that frame submits their shader jobs and the idle signal resets. We only emit the verbose
        # idle breakdown at most once per log_interval_seconds to avoid spam.
        num_consecutive_idle_frames_required = 2
        num_consecutive_idle_frames = 0
        elapsed_time_seconds = time.time() - start_time_seconds
        while num_consecutive_idle_frames < num_consecutive_idle_frames_required and elapsed_time_seconds < max_time_seconds:

            verbose = time.time() - last_verbose_time_seconds >= log_interval_seconds
            if verbose:
                last_verbose_time_seconds = time.time()

            with engine_service.begin_frame():
                self._force_update_engine()
                idle = func(verbose=verbose)
            with engine_service.end_frame():
                pass

            if idle:
                num_consecutive_idle_frames = num_consecutive_idle_frames + 1
            else:
                num_consecutive_idle_frames = 0

            elapsed_time_seconds = time.time() - start_time_seconds

        success = num_consecutive_idle_frames == num_consecutive_idle_frames_required
        if not success:
            spear.log("ERROR: Function never returned true, giving up...")
            assert False

        spear.log(f"Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")

    def _wait_for_in_editor_script(self, func, max_time_seconds, log_interval_seconds):
        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        engine_service = self.entry_point_caller.engine_service

        spear.log(f"Waiting for function to return true: {func_name}")
        spear.log(f"Waiting for up to {max_time_seconds} seconds, logging progress every {log_interval_seconds} seconds...")

        start_time_seconds = time.time()
        last_verbose_time_seconds = start_time_seconds

        # see _wait_for(...) above for details
        num_warmup_frames = 2
        yield from engine_service.step_in_editor_script(num_frames=num_warmup_frames)

        # see _wait_for(...) above for details
        num_consecutive_idle_frames_required = 2
        num_consecutive_idle_frames = 0
        elapsed_time_seconds = time.time() - start_time_seconds
        while num_consecutive_idle_frames < num_consecutive_idle_frames_required and elapsed_time_seconds < max_time_seconds:

            verbose = time.time() - last_verbose_time_seconds >= log_interval_seconds
            if verbose:
                last_verbose_time_seconds = time.time()

            with engine_service.begin_frame():
                self._force_update_engine()
                idle = func(verbose=verbose)
            yield
            with engine_service.end_frame():
                pass
            yield

            if idle:
                num_consecutive_idle_frames = num_consecutive_idle_frames + 1
            else:
                num_consecutive_idle_frames = 0

            elapsed_time_seconds = time.time() - start_time_seconds

        success = num_consecutive_idle_frames == num_consecutive_idle_frames_required
        if not success:
            spear.log("ERROR: Function never returned true, giving up...")
            assert False

        spear.log(f"Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")
