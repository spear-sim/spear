#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import inspect
import os
import psutil
import spear
import subprocess
import sys
import time

if spear.__can_import_spear_ext__:
    import spear_ext


#
# Global constants
#

expected_process_status_values = ["disk-sleep", "running", "sleeping", "stopped"] # stopped can happen when attaching to a debugger


#
# Instance
#

class Instance():
    def __init__(self, config=None):

        spear.log_current_function()

        if config is None:
            config = spear.get_config()

        self._config = config
        self._client = None

        if spear.__can_import_unreal__:
            spear.log('    Running inside the Unreal Editor Python environment, setting launch mode: "none"')
            self._config.defrost()
            self._config.SPEAR.LAUNCH_MODE = "none"
            self._config.freeze()

        spear.log("    Creating instance with launch mode: ", self._config.SPEAR.LAUNCH_MODE)
        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE == "editor":
            spear.log("    Editor launch mode: ", self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE)
            spear.log("    Editor executable:  ", self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE)
            spear.log("    Editor uproject:    ", self._config.SPEAR.INSTANCE.EDITOR_UPROJECT)
        elif self._config.SPEAR.LAUNCH_MODE == "game":
            spear.log("    Game executable: ", self._config.SPEAR.INSTANCE.GAME_EXECUTABLE)
        else:
            assert False

        # Request to launch the unreal instance and initialize the client.
        self._request_launch_unreal_instance()
        self._initialize_client()

        # Initialize EngineService.
        self._engine_service = spear.EngineService(client=self._client, config=self._config)
        self._engine_service.initialize() # must be initialized immediately

        if spear.__can_import_unreal__:
            entry_point_caller_type = spear.EditorEntryPointCaller
        else:
            entry_point_caller_type = spear.CallSyncEntryPointCaller

        # Initialize services that require a reference to EngineService.

        self.debug_service = spear.DebugService(
            entry_point_caller=entry_point_caller_type(service_name="debug_service", engine_service=self._engine_service))

        self.enhanced_input_service = spear.EnhancedInputService(
            entry_point_caller=entry_point_caller_type(service_name="enhanced_input_service", engine_service=self._engine_service))

        self.input_service = spear.InputService(
            entry_point_caller=entry_point_caller_type(service_name="input_service", engine_service=self._engine_service))

        self.shared_memory_service = spear.SharedMemoryService(
            entry_point_caller=entry_point_caller_type(service_name="shared_memory_service", engine_service=self._engine_service))

        self.world_registry_service = spear.WorldRegistryService(
            entry_point_caller=entry_point_caller_type(service_name="world_registry_service", engine_service=self._engine_service))

        # Initialize services that require a reference to EngineService, SharedMemoryService.

        self.sp_func_service = spear.SpFuncService(
            entry_point_caller=entry_point_caller_type(service_name="sp_func_service", engine_service=self._engine_service),
            shared_memory_service=self.shared_memory_service)

        # Initialize services that require a reference to EngineService, SpFuncService.

        # intended for internal use only
        self._unreal_service = spear.UnrealService(
            entry_point_caller=entry_point_caller_type(service_name=f"unreal_service", engine_service=self._engine_service),
            sp_func_service=self.sp_func_service,
            config=self._config)

        # Initialize services that require a reference to EngineService, SpFuncService, UnrealService.

        self.engine_globals_service = spear.EngineGlobalsService(
            entry_point_caller=entry_point_caller_type(service_name="engine_globals_service", engine_service=self._engine_service),
            sp_func_service=self.sp_func_service,
            unreal_service=self._unreal_service,
            config=self._config)

        # Initialize world-scoped services.

        self._game = Instance.WorldScopedServices(
            engine_service=self._engine_service,
            shared_memory_service=self.shared_memory_service,
            world_registry_service=self.world_registry_service,
            sp_func_service=self.sp_func_service,
            config=self._config)

        if self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet():
            self._editor = Instance.EditorWorldScopedServices(
                engine_service=self._engine_service,
                shared_memory_service=self.shared_memory_service,
                world_registry_service=self.world_registry_service,
                sp_func_service=self.sp_func_service,
                config=self._config)

    def get_unreal_object(self, uobject=None, uclass=None, with_sp_funcs=False):
        return spear.UnrealObject(
            unreal_service=self._unreal_service,
            sp_func_service=self.sp_func_service,
            config=self._config,
            uobject=uobject,
            uclass=uclass,
            with_sp_funcs=with_sp_funcs)

    class WorldScopedServices():
        def __init__(self, engine_service, shared_memory_service, world_registry_service, sp_func_service, config):

            # needed internally in get_unreal_object(...)
            self._sp_func_service = sp_func_service
            self._config = config

            # needed internally in invalidate(...)
            self._world_registry_service = world_registry_service
            self._world = None

            if spear.__can_import_unreal__:
                entry_point_caller_type = spear.EditorEntryPointCaller
            else:
                entry_point_caller_type = spear.CallSyncEntryPointCaller

            # Initialize services that require a reference to EngineService, SpFuncService.

            self.unreal_service = spear.UnrealService(
                entry_point_caller=entry_point_caller_type(service_name=f"unreal_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                config=self._config)

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService.

            self.navigation_service = spear.NavigationService(
                entry_point_caller=entry_point_caller_type(service_name=f"navigation_service", engine_service=engine_service),
                shared_memory_service=shared_memory_service,
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                config=self._config)

        def get_unreal_object(self, uobject=None, uclass=None, with_sp_funcs=False):
            return spear.UnrealObject(
                unreal_service=self.unreal_service,
                sp_func_service=self._sp_func_service,
                config=self._config,
                uobject=uobject,
                uclass=uclass,
                with_sp_funcs=with_sp_funcs)

        def invalidate(self):
            self._world_registry_service.remove_world(world=self._world) # synchronous call on the game thread, so no need to flush()
            self._world = None
 
        def _get_world(self):
            return self._world

        def _set_world(self, world):
            self._world = world
            self._set_world_impl(world=world)

        def _set_world_impl(self, world):
            self.unreal_service.set_world(world=world)
            self.navigation_service.set_world(world=world)

    class EditorWorldScopedServices(WorldScopedServices):
        def __init__(self, engine_service, shared_memory_service, world_registry_service, sp_func_service, config):
            super().__init__(engine_service=engine_service, shared_memory_service=shared_memory_service, world_registry_service=world_registry_service, sp_func_service=sp_func_service, config=config)

            if spear.__can_import_unreal__:
                entry_point_caller_type = spear.EditorEntryPointCaller
            else:
                entry_point_caller_type = spear.CallSyncEntryPointCaller

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService.

            self.python_service = spear.PythonService(
                entry_point_caller=entry_point_caller_type(service_name=f"python_service", engine_service=engine_service),
                sp_func_service=sp_func_service,
                unreal_service=self.unreal_service,
                config=config)

        def _set_world_impl(self, world):
            super()._set_world_impl(world=world)
            self.python_service.set_world(world=world)

    #
    # Public functions that can potentially call begin_frame() and end_frame(). When inside a spear.editor.script,
    # call the in_editor_script variants below using Python's "yield from" syntax as follows:
    #
    #     editor = yield from instance.get_editor_in_editor_script()
    #

    # initialize(...)

    def _is_any_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return True
            elif world_desc.is_game_world and world_desc.is_playing:
                return True
        return False

    def initialize(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()

        self._engine_service.initialize()
        self._initialize_wait_until(func=self._is_any_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        self._initialize_warm_up(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        with self.begin_frame():
            self._unreal_service.initialize()
        with self.end_frame():
            pass

    def initialize_in_editor_script(self, wait=None, wait_max_time_seconds=None, wait_sleep_time_seconds=None, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()

        self._engine_service.initialize()
        yield from self._initialize_wait_until_in_editor_script(func=self._is_any_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        yield from self._initialize_warm_up_in_editor_script(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        with self.begin_frame():
            self._unreal_service.initialize()
        yield
        with self.end_frame():
            pass
        yield

    def _initialize_wait_until(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_SLEEP_TIME_SECONDS)
        self._wait_until(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _initialize_wait_until_in_editor_script(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_SLEEP_TIME_SECONDS)
        yield from self._wait_until_in_editor_script(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _initialize_warm_up(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.INITIALIZE_WARM_UP_NUM_FRAMES)
        self._warm_up(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)

    def _initialize_warm_up_in_editor_script(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.INITIALIZE_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.INITIALIZE_WARM_UP_NUM_FRAMES)
        yield from self._warm_up_in_editor_script(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)


    # get_editor(...)

    def _is_editor_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return True
            elif world_desc.is_game_world and world_desc.is_playing:
                return True
        return False

    def _get_editor_world(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return world_desc.world
        assert False

    def get_editor(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()
        assert self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet()

        self._engine_service.initialize()
        self._get_editor_wait_until(func=self._is_editor_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        self._get_editor_warm_up(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        world = self._get_editor_world()
        self._editor._set_world(world=world)

        with self.begin_frame():
            self._unreal_service.initialize()
            self._editor.unreal_service.initialize(unreal_service=self._unreal_service)
            self._editor.python_service.initialize()
        with self.end_frame():
            pass

        return self._editor

    def get_editor_in_editor_script(self, wait=None, wait_max_time_seconds=None, wait_sleep_time_seconds=None, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()
        assert self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet()

        self._engine_service.initialize()
        yield from self._get_editor_wait_until_in_editor_script(func=self._is_editor_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        yield from self._get_editor_warm_up_in_editor_script(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        world = self._get_editor_world()
        self._editor._set_world(world=world)

        with self.begin_frame():
            self._unreal_service.initialize()
            self._editor.unreal_service.initialize(unreal_service=self._unreal_service)
            self._editor.python_service.initialize()
        yield
        with self.end_frame():
            pass
        yield

        return self._editor

    def _get_editor_wait_until(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_SLEEP_TIME_SECONDS)
        self._wait_until(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _get_editor_wait_until_in_editor_script(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_SLEEP_TIME_SECONDS)
        yield from self._wait_until_in_editor_script(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _get_editor_warm_up(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_NUM_FRAMES)
        self._warm_up(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)

    def _get_editor_warm_up_in_editor_script(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_NUM_FRAMES)
        yield from self._warm_up_in_editor_script(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)


    # get_game(...)

    def _is_game_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return True
            elif world_desc.is_game_world and world_desc.is_playing:
                return True
        return False

    def _get_game_world(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_game_world and world_desc.is_playing:
                return world_desc.world
        assert False

    def get_game(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()

        self._engine_service.initialize()
        self._get_game_wait_until(func=self._is_game_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        self._get_game_warm_up(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        world = self._get_game_world()
        self._game._set_world(world=world)

        with self.begin_frame():
            self._unreal_service.initialize()
            self._game.unreal_service.initialize(unreal_service=self._unreal_service)
        with self.end_frame():
            pass

        return self._game

    def get_game_in_editor_script(self, wait=None, wait_max_time_seconds=None, wait_sleep_time_seconds=None, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()

        self._engine_service.initialize()
        yield from self._get_game_wait_until_in_editor_script(func=self._is_game_world_initialized, wait=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        yield from self._get_game_warm_up_in_editor_script(warm_up=warm_up, time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        world = self._get_game_world()
        self._game._set_world(world=world)

        with self.begin_frame():
            self._unreal_service.initialize()
            self._game.unreal_service.initialize(unreal_service=self._unreal_service)
        yield
        with self.end_frame():
            pass
        yield

        return self._game

    def _get_game_wait_until(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_SLEEP_TIME_SECONDS)
        self._wait_until(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _get_game_wait_until_in_editor_script(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):
        wait, max_time_seconds, sleep_time_seconds = self._get_wait_until_info(
            wait=wait,
            max_time_seconds=max_time_seconds,
            sleep_time_seconds=sleep_time_seconds,
            default_max_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_MAX_TIME_SECONDS,
            default_sleep_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_SLEEP_TIME_SECONDS)
        yield from self._wait_until_in_editor_script(func=func, wait=wait, max_time_seconds=max_time_seconds, sleep_time_seconds=sleep_time_seconds)

    def _get_game_warm_up(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_NUM_FRAMES)
        self._warm_up(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)

    def _get_game_warm_up_in_editor_script(self, warm_up=None, time_seconds=0.0, num_frames=0):
        warm_up, time_seconds, num_frames = self._get_warm_up_info(
            warm_up=warm_up,
            time_seconds=time_seconds,
            num_frames=num_frames,
            default_time_seconds=self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_TIME_SECONDS,
            default_num_frames=self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_NUM_FRAMES)
        yield from self._warm_up_in_editor_script(warm_up=warm_up, time_seconds=time_seconds, num_frames=num_frames)


    # step(...)

    def step(self, num_frames=1, single_step=False):
        for i in range(num_frames):
            with self.begin_frame():
                pass
            with self.end_frame(single_step=single_step):
                pass

    def step_in_editor_script(self, num_frames=1, single_step=False):
        for i in range(num_frames):
            with self.begin_frame():
                pass
            yield
            with self.end_frame(single_step=single_step):
                pass
            yield


    #
    # begin_frame() / end_frame()
    #

    def begin_frame(self):
        return self._engine_service.begin_frame()

    def end_frame(self, single_step=False):
        return self._engine_service.end_frame(single_step=single_step)


    #
    # Public functions that do not call begin_frame() or end_frame().
    #

    def is_running(self):
        try:
            connected = self.engine_globals_service.ping() == "ping"
            return connected
        except:
            pass # no need to log exception because this case is expected when the instance is no longer running
        return False


    def close(self, force=False):

        spear.log_current_function()

        # Note that in the constructor, we launch the Unreal instance first and then initialize the client
        # Normally, we would do things in the reverse order here. But if we terminate the client first, then
        # we can't call the "request_exit" and "terminate" entry points to close the Unreal instance. So we
        # terminate the Unreal instance first, and then terminate the client afterwards.

        if force:
            self._force_kill_unreal_instance()
        else:
            self._request_terminate_unreal_instance()
        self._terminate_client(verbose=True, log_prefix="    ")


    #
    # calling flush() inside a begin_frame() or end_frame() block ensures that there is no work pending on the game thread
    #

    def flush(self):
        self._engine_service.flush()


    #
    # Initialize helper functions
    #

    def _request_launch_unreal_instance(self):

        spear.log_current_function(prefix="    ")
        spear.log("        Requesting to launch Unreal instance...")

        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
            pass

        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # Write a temp config file. We do this before checking LAUNCH_MODE in case the user wants to
            # initialize the config system in the editor using the temp file that gets written here.
            temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.INSTANCE.TEMP_DIR))
            temp_config_file = os.path.realpath(os.path.join(temp_dir, self._config.SPEAR.INSTANCE.TEMP_CONFIG_FILE))

            spear.log("        Writing temp config file: ", temp_config_file)

            os.makedirs(temp_dir, exist_ok=True)
            with open(temp_config_file, "w") as f:
                self._config.dump(stream=f, default_flow_style=False)

            # Find launch executable to launch. The correct executable to launch depends on the platform and
            # whether we're launching a standalone game or editor. Also create a list of command-line
            # arguments.

            launch_args = []

            if self._config.SPEAR.LAUNCH_MODE == "editor":
                assert self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE in ["editor", "game"]
                assert os.path.exists(self._config.SPEAR.INSTANCE.EDITOR_UPROJECT)
                launch_executable = self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE
                launch_args.append(self._config.SPEAR.INSTANCE.EDITOR_UPROJECT)
                if self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE == "game":
                    launch_args.append("-game")
            elif self._config.SPEAR.LAUNCH_MODE == "game":
                launch_executable = self._config.SPEAR.INSTANCE.GAME_EXECUTABLE
            else:
                assert False

            assert os.path.exists(launch_executable)

            launch_executable_dir, launch_executable_name = os.path.split(launch_executable)
            launch_executable_name_no_ext, launch_executable_ext = os.path.splitext(launch_executable_name)

            if sys.platform == "win32":
                assert launch_executable_ext == ".exe"
            elif sys.platform == "darwin":
                assert launch_executable_ext == ".app"
            elif sys.platform == "linux":
                if self._config.SPEAR.LAUNCH_MODE == "editor":
                    assert launch_executable_ext == ""
                elif self._config.SPEAR.LAUNCH_MODE == "game":
                    assert launch_executable_ext == ".sh"
                else:
                    assert False
            else:
                assert False

            if self._config.SPEAR.LAUNCH_MODE == "editor":
                if sys.platform == "win32":
                    launch_executable_dir_internal = launch_executable_dir
                elif sys.platform == "darwin":
                    launch_executable_dir_internal = os.path.realpath(os.path.join(launch_executable, "Contents", "MacOS"))
                elif sys.platform == "linux":
                    launch_executable_dir_internal = launch_executable_dir
                else:
                    assert False
            elif self._config.SPEAR.LAUNCH_MODE == "game":
                if sys.platform == "win32":
                    launch_executable_dir_internal = os.path.realpath(os.path.join(launch_executable_dir, launch_executable_name_no_ext, "Binaries", "Win64"))
                elif sys.platform == "darwin":
                    launch_executable_dir_internal = os.path.realpath(os.path.join(launch_executable, "Contents", "MacOS"))
                elif sys.platform == "linux":
                    launch_executable_dir_internal = os.path.realpath(os.path.join(launch_executable_dir, launch_executable_name_no_ext, "Binaries", "Linux"))
                else:
                    assert False
            else:
                assert False

            if sys.platform == "win32":
                launch_executable_internal_template = os.path.realpath(os.path.join(launch_executable_dir_internal, f"{launch_executable_name_no_ext}*-Cmd.exe"))
                if self._config.SPEAR.LAUNCH_MODE == "editor":
                    launch_executable_internal_paths = [ p for p in glob.glob(launch_executable_internal_template) if not "-Win64-" in os.path.basename(p) ]
                elif self._config.SPEAR.LAUNCH_MODE == "game":
                    launch_executable_internal_paths = glob.glob(launch_executable_internal_template)
                else:
                    assert False
                assert len(launch_executable_internal_paths) == 1
                launch_executable_internal = launch_executable_internal_paths[0]
            elif sys.platform == "darwin":
                launch_executable_internal_template = os.path.realpath(os.path.join(launch_executable_dir_internal, f"{launch_executable_name_no_ext}*"))
                launch_executable_internal_paths = glob.glob(launch_executable_internal_template)
                assert len(launch_executable_internal_paths) == 1
                launch_executable_internal = launch_executable_internal_paths[0]
            elif sys.platform == "linux":
                launch_executable_internal_template = os.path.realpath(os.path.join(launch_executable_dir_internal, f"{launch_executable_name_no_ext}*"))
                if self._config.SPEAR.LAUNCH_MODE == "editor":
                    launch_executable_internal_paths = [ p for p in glob.glob(launch_executable_internal_template) if (not "." in os.path.basename(p)) and (not "-" in os.path.basename(p)) ]
                elif self._config.SPEAR.LAUNCH_MODE == "game":
                    launch_executable_internal_paths = [ p for p in glob.glob(launch_executable_internal_template) if not "." in os.path.basename(p) ]
                else:
                    assert False
                assert len(launch_executable_internal_paths) == 1
                launch_executable_internal = launch_executable_internal_paths[0]
            else:
                assert False

            spear.log("        Attempting to launch executable: ", launch_executable_internal)
            assert os.path.exists(launch_executable_internal)
  
            for arg, value in (self._config.SPEAR.INSTANCE.COMMAND_LINE_ARGS).items():
                if value is None:
                    launch_args.append(f"-{arg}")
                else:
                    launch_args.append(f"-{arg}={value}")

            launch_args.append(f"-sp-config-file={temp_config_file}")
            cmd = [launch_executable_internal] + launch_args

            spear.log("        Launching executable with the following command-line arguments:")
            spear.log(f"            {launch_executable_internal}")
            for launch_arg in launch_args:
                spear.log(f"                {launch_arg}")
            spear.log("        Launching executable with the following config values:")
            spear.log_no_prefix(self._config)

            popen = subprocess.Popen(cmd)
            self._process = psutil.Process(popen.pid)

            # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
            status = self._process.status()
            if status not in expected_process_status_values:
                spear.log("        ERROR: Unrecognized process status: ", status)
                spear.log(f"        ERROR: Killing process {self._process.pid}...")
                self._force_kill_unreal_instance()
                assert False

        else:
            assert False

        spear.log("        Finished requesting to launch Unreal instance.")


    def _initialize_client(self):

        spear.log_current_function(prefix="    ")        
        spear.log("        Initializing client...")

        connected = False

        # If we're connecting to a running instance, then we assume that the server is already running and
        # only try to connect once.
        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                # Once a connection has been established, the client will wait for timeout seconds before
                # throwing when calling a server function.
                spear.log("        Attempting to connect to server...")
                if spear.__can_import_spear_ext__:
                    self._client = spear_ext.Client("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT)
                    self._client.set_timeout(int(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS)*1000)
                else:
                    self._client = spear.editor.Client(
                        address="127.0.0.1",
                        port=self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT,
                        timeout=float(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS),
                        reconnect_limit=int(self._config.SPEAR.INSTANCE.EDITOR_CLIENT_INTERNAL_RECONNECT_LIMIT))
                connected = self._client.ping() == "ping"
            except Exception as e:
                spear.log("        Exception: ", e)
                self._terminate_client(verbose=True, log_prefix="        ")
                assert False

        # otherwise try to connect repeatedly, since the server might not have started yet
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while elapsed_time_seconds < self._config.SPEAR.INSTANCE.INITIALIZE_CLIENT_MAX_TIME_SECONDS:

                # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible values
                status = self._process.status()
                if status not in expected_process_status_values:
                    spear.log("        ERROR: Unrecognized process status: ", status)
                    break

                try:
                    # Once a connection has been established, the client will wait for timeout seconds before
                    # throwing when calling a server function.
                    spear.log("        Attempting to connect to server...")
                    if spear.__can_import_spear_ext__:
                        self._client = spear_ext.Client("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT)
                        self._client.set_timeout(int(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS)*1000)
                    else:
                        self._client = spear.editor.Client(
                            address="127.0.0.1",
                            port=self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT,
                            timeout=float(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS),
                            reconnect_limit=int(self._config.SPEAR.INSTANCE.EDITOR_CLIENT_INTERNAL_RECONNECT_LIMIT))
                    connected = self._client.ping() == "ping"
                    break

                except:
                    # There is no need to log the exception because this case is expected until until we can
                    # successfully connect to the server, which is why we set verbose=False when terminating
                    # the client.
                    self._terminate_client(verbose=False)

                time.sleep(self._config.SPEAR.INSTANCE.INITIALIZE_CLIENT_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        else:
            assert False

        if connected:
            spear.log("        Connected to server.")
        else:
            spear.log("        ERROR: Couldn't connect to RPC server, giving up...")
            self._terminate_client(verbose=True, log_prefix="        ")
            assert False

        self._client.initialize() # self._engine_service hasn't been initialized yet

        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            pid = self._client.call("engine_globals_service.call_sync_on_worker_thread.get_current_process_id") # self._engine_service hasn't been initialized yet
            if pid == self._process.pid:
                spear.log("        Validated engine_globals_service.call_sync_on_worker_thread.get_current_process_id: ", pid)
            else:            
                spear.log(f"        ERROR: engine_globals_service.call_sync_on_worker_thread.get_current_process_id returned {pid} but the PID of the process we just launched is {self._process.pid}. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables and try launching again.")
                self._terminate_client(verbose=True, log_prefix="        ")
                self._force_kill_unreal_instance()
                assert False
        else:
            assert False

        self._client.force_return_aligned_arrays = self._config.SPEAR.INSTANCE.CLIENT_FORCE_RETURN_ALIGNED_ARRAYS
        self._client.verbose_rpc_calls = self._config.SPEAR.INSTANCE.CLIENT_VERBOSE_RPC_CALLS
        self._client.verbose_allocations = self._config.SPEAR.INSTANCE.CLIENT_VERBOSE_ALLOCATIONS
        self._client.verbose_exceptions = self._config.SPEAR.INSTANCE.CLIENT_VERBOSE_EXCEPTIONS

        spear.log("        Finished initializing client.")


    def _get_wait_until_info(self, wait, max_time_seconds, sleep_time_seconds, default_max_time_seconds, default_sleep_time_seconds):
        if wait is None:
            wait = True
            max_time_seconds = default_max_time_seconds
            sleep_time_seconds = default_sleep_time_seconds
        return wait, max_time_seconds, sleep_time_seconds

    def _wait_until(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):

        spear.log_current_function(prefix="    ")

        if wait is None:
            retry = True
        else:
            retry = wait

        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        spear.log("        Waiting for function to return true: ", func_name)

        if retry:
            spear.log(f"        Waiting for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")
            success = func()
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not success and elapsed_time_seconds < max_time_seconds:
                success = func()
                if success:
                    break
                time.sleep(sleep_time_seconds)
                elapsed_time_seconds = time.time() - start_time_seconds
        else:
            spear.log("        Attempting to call function once...")
            success = func()

        if not success:
            spear.log("        ERROR: Function never returned true, giving up...")
            self._terminate_client(verbose=True, log_prefix="        ")
            assert False

        if retry:
            spear.log(f"        Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")
        else:
            spear.log(f"        Finished waiting for function to return true.")

    def _wait_until_in_editor_script(self, func, wait=None, max_time_seconds=0.0, sleep_time_seconds=0.0):

        spear.log_current_function(prefix="    ")

        if wait is None:
            retry = True
        else:
            retry = wait

        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        spear.log("        Waiting for function to return true: ", func_name)

        if retry:
            spear.log(f"        Waiting for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")
            success = func()
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not success and elapsed_time_seconds < max_time_seconds:
                success = func()
                if success:
                    break
                yield from self.step_in_editor_script(num_frames=1)
                elapsed_time_seconds = time.time() - start_time_seconds
        else:
            spear.log("        Attempting to call function once...")
            success = func()

        if not success:
            spear.log("        ERROR: Function never returned true, giving up...")
            self._terminate_client(verbose=True, log_prefix="        ")
            assert False

        if retry:
            spear.log(f"        Finished waiting for function to return true (waited for {elapsed_time_seconds:.2f} seconds).")
        else:
            spear.log(f"        Finished waiting for function to return true.")

    def _get_warm_up_info(self, warm_up, time_seconds, num_frames, default_time_seconds, default_num_frames):
        if warm_up is None:
            if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
                warm_up = False
            elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
                warm_up = True
                time_seconds = default_time_seconds
                num_frames = default_num_frames
            else:
                assert False
        return warm_up, time_seconds, num_frames

    def _warm_up(self, warm_up, time_seconds, num_frames):

        if not warm_up:
            return

        spear.log_current_function(prefix="    ")
        spear.log(f"        Warming up Unreal instance for {time_seconds} seconds and {num_frames} frames...")

        if time_seconds > 0.0:
            time.sleep(time_seconds)

        self.step(num_frames=num_frames)

        spear.log("        Finished warming up Unreal instance.")

    def _warm_up_in_editor_script(self, warm_up, time_seconds, num_frames):

        if not warm_up:
            return

        spear.log_current_function(prefix="    ")
        spear.log(f"        Warming up Unreal instance for {time_seconds} seconds and {num_frames} frames...")

        start_time_seconds = time.time()
        elapsed_time_seconds = time.time() - start_time_seconds
        while elapsed_time_seconds < time_seconds:
            yield from self.step_in_editor_script(num_frames=1)
            elapsed_time_seconds = time.time() - start_time_seconds

        yield from self.step_in_editor_script(num_frames=num_frames)

        spear.log("        Finished warming up Unreal instance.")


    #
    # Terminate helper functions
    #

    def _request_terminate_unreal_instance(self):

        spear.log_current_function(prefix="    ")
        spear.log("        Requesting to terminate Unreal instance...")

        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:        
            try:
                self.step()
                self.engine_globals_service.request_exit(immediate_shutdown=False)
                self._engine_service.terminate() # EngineService must be explicitly terminated, needs to be the last entry point that gets called
            except:
                pass # no need to log exception because this case is expected when the instance is no longer running
            try:
                status = self._process.status()
            except psutil.NoSuchProcess:
                status = None
            while status in expected_process_status_values:
                time.sleep(self._config.SPEAR.INSTANCE.REQUEST_EXIT_UNREAL_INSTANCE_SLEEP_TIME_SECONDS)
                try:
                    status = self._process.status()
                except psutil.NoSuchProcess:
                    break
        else:
            assert False

        spear.log("        Finished requesting to terminate Unreal instance.")


    def _force_kill_unreal_instance(self):
        spear.log_current_function(prefix="    ")
        spear.log("        Forcefully killing Unreal instance...")

        if spear.__can_import_unreal__ or self._config.SPEAR.LAUNCH_MODE == "none":
            assert False
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            try:
                self._process.terminate()
                self._process.kill()
            except psutil.NoSuchProcess:
                spear.log("        Process already terminated.")
            except Exception as e:
                spear.log("        Exception: ", e)
            spear.log("        Finished forcefully killing Unreal instance.")
        else:
            assert False


    def _terminate_client(self, verbose, log_prefix=""):
        if verbose:
            spear.log_current_function(prefix=log_prefix)
            spear.log(f"{log_prefix}    Terminating client: {self._client}")
        if self._client is not None:
            self._client.terminate()
            self._client = None
        if verbose:
            spear.log(f"{log_prefix}    Finished terminating client.")


    def __repr__(self):
        return f"Instance(_client={self._client})"
