#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import contextlib
import spear
import time


#
# Instance
#

class Instance():
    def __init__(self, config=None):

        spear.log_current_function()

        if config is None:
            config = spear.get_config()

        self._config = config

        if spear.__can_import_unreal__:
            spear.log('Running inside the Unreal Editor Python environment, setting launch mode: "none"')
            self._config.defrost()
            self._config.SPEAR.LAUNCH_MODE = "none"
            self._config.freeze()

        spear.log("Creating instance with launch mode: ", self._config.SPEAR.LAUNCH_MODE)
        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE == "editor":
            spear.log("Editor launch mode: ", self._config.SPEAR.EDITOR_LAUNCH_MODE)
            spear.log("Editor executable:  ", self._config.SPEAR.EDITOR_EXECUTABLE)
            spear.log("Editor uproject:    ", self._config.SPEAR.EDITOR_UPROJECT)
        elif self._config.SPEAR.LAUNCH_MODE == "game":
            spear.log("Game executable: ", self._config.SPEAR.GAME_EXECUTABLE)
        else:
            assert False

        # Launch the Unreal executable.
        self._launcher = spear.Launcher(config=self._config)
        self._launcher.launch()

        # Initialize the client. We do this after launching the Unreal executable because initializing the
        # client will not proceed until it is able to form an RPC connection with the Unreal executable.
        self._client = spear.Client(config=self._config)
        self._client.initialize(launcher=self._launcher)

        # Create EngineService but do not initialize yet.
        self._engine_service = spear.EngineService(client=self._client.client, config=self._config)

        # Initialize the launcher, which will cache our client, and will immediately
        # validate that the PID of the Unreal executable we launched matches the PID reported via the client.
        self._launcher.initialize(client=self._client)

        # Initialize EngineService. We do this after initializing the launcher because we want the launcher to
        # perform its PID validation as soon as possible, but we want to initialize EngineService as soon as
        # possible after that, so we can begin calling other entry points through EngineService and its
        # higher-level helper functionality instead of directly through the client.
        self._engine_service.initialize()

        if spear.__can_import_unreal__:
            entry_point_caller_type = spear.EditorCallSyncEntryPointCaller
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
            entry_point_caller=entry_point_caller_type(service_name="unreal_service", engine_service=self._engine_service),
            sp_func_service=self.sp_func_service,
            config=self._config)

        # Initialize services that require a reference to EngineService, SpFuncService, UnrealService.

        self.engine_globals_service = spear.EngineGlobalsService(
            entry_point_caller=entry_point_caller_type(service_name="engine_globals_service", engine_service=self._engine_service),
            sp_func_service=self.sp_func_service,
            unreal_service=self._unreal_service,
            config=self._config)

        # Initialize world-scoped services.

        self._game = Instance.GameWorldScopedServices(
            engine_service=self._engine_service,
            shared_memory_service=self.shared_memory_service,
            world_registry_service=self.world_registry_service,
            sp_func_service=self.sp_func_service,
            engine_globals_service=self.engine_globals_service,
            config=self._config)

        # Only create editor-scoped services if we're running the editor (regardless of whether or not we're in a PIE session)
        # and we're not running a commandlet. We check is_editor() rather than is_with_editor(), because is_with_editor() is
        # also true when running the editor executable with -game, in which case there is no editor world.
        if self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet():
            self._editor = Instance.EditorWorldScopedServices(
                engine_service=self._engine_service,
                shared_memory_service=self.shared_memory_service,
                world_registry_service=self.world_registry_service,
                sp_func_service=self.sp_func_service,
                engine_globals_service=self.engine_globals_service,
                config=self._config)

    class WorldScopedServices():
        def __init__(self, engine_service, shared_memory_service, world_registry_service, sp_func_service, engine_globals_service, config):

            # needed internally in get_unreal_object(...)
            self._sp_func_service = sp_func_service
            self._config = config

            # needed internally in invalidate(...)
            self._world_registry_service = world_registry_service
            self._world = None

            if spear.__can_import_unreal__:
                entry_point_caller_type = spear.EditorCallSyncEntryPointCaller
            else:
                entry_point_caller_type = spear.CallSyncEntryPointCaller

            # Initialize services that require a reference to EngineService, SpFuncService.

            self.unreal_service = spear.UnrealService(
                entry_point_caller=entry_point_caller_type(service_name="unreal_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                config=self._config)

            # Initialize services that require a reference to SpFuncService, UnrealService.

            self.class_default_object_service = spear.ClassDefaultObjectService(
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                config=self._config)

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService.

            self.console_service = spear.ConsoleService(
                entry_point_caller=entry_point_caller_type(service_name="console_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                config=self._config)

            self.navigation_service = spear.NavigationService(
                entry_point_caller=entry_point_caller_type(service_name="navigation_service", engine_service=engine_service),
                shared_memory_service=shared_memory_service,
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                config=self._config)

            self.segmentation_service = spear.SegmentationService(
                entry_point_caller=entry_point_caller_type(service_name="segmentation_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                config=self._config)

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService, EngineGlobalsService, and ClassDefaultObjectService.

            self.async_loading_service = spear.AsyncLoadingService(
                entry_point_caller=entry_point_caller_type(service_name="async_loading_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                engine_globals_service=engine_globals_service,
                cdo=self.class_default_object_service,
                config=self._config)

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService, EngineGlobalsService, ConsoleService, and ClassDefaultObjectService.

            self.rendering_quality_service = spear.RenderingQualityService(
                entry_point_caller=entry_point_caller_type(service_name="rendering_quality_service", engine_service=engine_service),
                sp_func_service=self._sp_func_service,
                unreal_service=self.unreal_service,
                engine_globals_service=engine_globals_service,
                console_service=self.console_service,
                cdo=self.class_default_object_service,
                config=self._config)

            # Initialize the self.cdo alias for our ClassDefaultObjectService.

            self.cdo = self.class_default_object_service

        def initialize(self, unreal_service=None):
            self.unreal_service.initialize(unreal_service=unreal_service)
            # self.segmentation_service.initialize() this has a non-trivial runtime cost, so we don't initialize by default
            self.async_loading_service.initialize()

        def terminate(self):
            pass

        def get_unreal_object(self, uobject=None, uclass=None, with_sp_funcs=False):
            return spear.get_unreal_object(
                unreal_service=self.unreal_service,
                sp_func_service=self._sp_func_service,
                config=self._config,
                uobject=uobject,
                uclass=uclass,
                with_sp_funcs=with_sp_funcs)

        #
        # invalidate() is a context manager that must wrap any call that destroys the current world, e.g.,
        # OpenLevel(...) or LoadLevel(...). On entry, we unregister the world on the server, so it is no
        # longer visible to get_world_descs(), but from UE's perspective we don't do anything to invalidate
        # the world, so from UE's perspective the world is still valid inside the block. On exit, we set the
        # world for every service to None, so any subsequent service call to get_world() will assert until the
        # user calls instance.get_game() or instance.get_editor() again. Inside the block, every service can
        # still access the world, and from UE's perspective it is still valid. So it is fine to make other
        # calls before the call that destroys the world, but the call that invalidates the world from UE's
        # perspective must come last.
        #
        #     with instance.end_frame():
        #         with game.invalidate():
        #             game.cdo.UGameplayStatics.OpenLevel(LevelName=level_name)
        #     game = instance.get_game()
        #

        @contextlib.contextmanager
        def invalidate(self):
            assert self._world is not None # otherwise unregister_world(...) would assert on the server
            self._world_registry_service.unregister_world(world=self._world) # synchronous call on the game thread, so no need to flush()
            try:
                yield
            finally:
                self.set_world(world=None) # dispatches to the derived set_world(...), so every service drops its world

        def rebuild(self, unreal_service=None):
            self.unreal_service.rebuild_cache(unreal_service=unreal_service)
            self.class_default_object_service.invalidate_cache()
            self.console_service.invalidate_cache()

        # called from Instance.get_game(), Instance.get_editor(), and invalidate(), most users don't need to call this function directly
        def set_world(self, world):
            self._world = world
            self.unreal_service.set_world(world=world)
            self.console_service.set_world(world=world)
            self.navigation_service.set_world(world=world)
            self.segmentation_service.set_world(world=world)
            self.async_loading_service.set_world(world=world)
            self.rendering_quality_service.set_world(world=world)

    class GameWorldScopedServices(WorldScopedServices):
        def __init__(self, engine_service, shared_memory_service, world_registry_service, sp_func_service, engine_globals_service, config):
            super().__init__(
                engine_service=engine_service,
                shared_memory_service=shared_memory_service,
                world_registry_service=world_registry_service,
                sp_func_service=sp_func_service,
                engine_globals_service=engine_globals_service,
                config=config)

            if spear.__can_import_unreal__:
                entry_point_caller_type = spear.EditorCallSyncEntryPointCaller
            else:
                entry_point_caller_type = spear.CallSyncEntryPointCaller

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService, EngineGlobalsService, and ClassDefaultObjectService.

            self.viewport_service = spear.GameViewportService(
                entry_point_caller=entry_point_caller_type(service_name="viewport_service", engine_service=engine_service),
                sp_func_service=sp_func_service,
                unreal_service=self.unreal_service,
                engine_globals_service=engine_globals_service,
                cdo=self.class_default_object_service,
                config=config)

        def initialize(self, unreal_service=None):
            super().initialize(unreal_service=unreal_service)
            self.viewport_service.initialize()

        def terminate(self):
            super().terminate()

        # called from Instance.get_game() and invalidate(), most users don't need to call this function directly
        def set_world(self, world):
            super().set_world(world=world)
            self.viewport_service.set_world(world=world)

    class EditorWorldScopedServices(WorldScopedServices):
        def __init__(self, engine_service, shared_memory_service, world_registry_service, sp_func_service, engine_globals_service, config):
            super().__init__(
                engine_service=engine_service,
                shared_memory_service=shared_memory_service,
                world_registry_service=world_registry_service,
                sp_func_service=sp_func_service,
                engine_globals_service=engine_globals_service,
                config=config)

            if spear.__can_import_unreal__:
                entry_point_caller_type = spear.EditorCallSyncEntryPointCaller
            else:
                entry_point_caller_type = spear.CallSyncEntryPointCaller

            # Initialize services that require a reference to EngineService, SpFuncService, UnrealService, and ClassDefaultObjectService.

            self.python_service = spear.PythonService(
                entry_point_caller=entry_point_caller_type(service_name="python_service", engine_service=engine_service),
                sp_func_service=sp_func_service,
                unreal_service=self.unreal_service,
                cdo=self.class_default_object_service,
                config=config)

            self.viewport_service = spear.EditorViewportService(
                entry_point_caller=entry_point_caller_type(service_name="viewport_service", engine_service=engine_service),
                sp_func_service=sp_func_service,
                unreal_service=self.unreal_service,
                cdo=self.class_default_object_service,
                config=config)

        def initialize(self, unreal_service=None):
            super().initialize(unreal_service=unreal_service)
            self.python_service.initialize()

        def terminate(self):
            super().terminate()

        # called from Instance.get_editor() and invalidate(), most users don't need to call this function directly
        def set_world(self, world):
            super().set_world(world=world)
            self.python_service.set_world(world=world)
            self.viewport_service.set_world(world=world)

    #
    # Public functions that can potentially call begin_frame() and end_frame(). When inside a spear.editor.script,
    # call the in_editor_script variants below using Python's "yield from" syntax as follows:
    #
    #     editor = yield from instance.get_editor_in_editor_script()
    #
    # More generally, any function that blocks the calling thread, e.g., by calling begin_frame(), end_frame(),
    # or time.sleep(), needs an in_editor_script variant that yields instead of blocking, because a
    # spear.editor.script runs on the editor's main thread, and blocking it prevents the editor from advancing.
    #

    #
    # initialize(...)
    #

    def initialize(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0):

        spear.log_current_function()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        with self.end_frame():
            self.world_registry_service.register_worlds()

        # wait for _is_any_world_initialized(...) or try _is_any_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_any_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        self._wait_for(wait_for_desc=wait_for_desc)

        # initialize services
        with self.begin_frame():
            self._unreal_service.initialize()
        with self.end_frame():
            pass

    def initialize_in_editor_script(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0):

        spear.log_current_function()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        yield
        with self.end_frame():
            self.world_registry_service.register_worlds()
        yield

        # wait for _is_any_world_initialized(...) or try _is_any_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_any_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.INITIALIZE_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        yield from self._wait_for_in_editor_script(wait_for_desc=wait_for_desc)

        # initialize services
        with self.begin_frame():
            self._unreal_service.initialize()
        yield
        with self.end_frame():
            pass
        yield

    def _is_any_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return True
            elif world_desc.is_game_world:
                return True
        return False

    #
    # get_editor(...)
    #

    def get_editor(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0,
        wait_for_engine_idle=None, wait_for_engine_idle_max_time_seconds=0.0, wait_for_engine_idle_log_interval_seconds=0.0):

        spear.log_current_function()

        # only allow getting editor-scoped services if editor-scoped services were created in __init__(...)
        assert self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        with self.end_frame():
            self.world_registry_service.register_worlds()

        # wait for _is_editor_world_initialized(...) or try _is_editor_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_editor_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        self._wait_for(wait_for_desc=wait_for_desc)

        # set world and initialize services
        with self.begin_frame():
            world = self._get_editor_world()
            self._editor.set_world(world=world)
            self._unreal_service.initialize()
            self._editor.initialize(unreal_service=self._unreal_service)
        with self.end_frame():
            pass

        # wait_for_engine_idle(...) or try is_engine_idle() once and assert
        wait_for_desc = self._get_wait_for_desc(
            wait=wait_for_engine_idle, config=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_FOR_ENGINE_IDLE,
            max_time_seconds=wait_for_engine_idle_max_time_seconds, log_interval_seconds=wait_for_engine_idle_log_interval_seconds)
        if wait_for_desc["retry"]:
            self._editor.async_loading_service.wait_for_engine_idle(max_time_seconds=wait_for_desc["max_time_seconds"], log_interval_seconds=wait_for_desc["log_interval_seconds"])
        else:
            with self.begin_frame():
                assert self._editor.async_loading_service.is_engine_idle()
            with self.end_frame():
                pass

        return self._editor

    def get_editor_in_editor_script(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0,
        wait_for_engine_idle=None, wait_for_engine_idle_max_time_seconds=0.0, wait_for_engine_idle_log_interval_seconds=0.0):

        spear.log_current_function()

        # only allow getting editor-scoped services if editor-scoped services were created in __init__(...)
        assert self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        yield
        with self.end_frame():
            self.world_registry_service.register_worlds()
        yield

        # wait for _is_editor_world_initialized(...) or try _is_editor_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_editor_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        yield from self._wait_for_in_editor_script(wait_for_desc=wait_for_desc)

        # set world and initialize services
        with self.begin_frame():
            world = self._get_editor_world()
            self._editor.set_world(world=world)
            self._unreal_service.initialize()
            self._editor.initialize(unreal_service=self._unreal_service)
        yield
        with self.end_frame():
            pass
        yield

        # wait_for_engine_idle_in_editor_script(...) or try is_engine_idle() once and assert
        wait_for_desc = self._get_wait_for_desc(
            wait=wait_for_engine_idle, config=self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_FOR_ENGINE_IDLE,
            max_time_seconds=wait_for_engine_idle_max_time_seconds, log_interval_seconds=wait_for_engine_idle_log_interval_seconds)
        if wait_for_desc["retry"]:
            yield from self._editor.async_loading_service.wait_for_engine_idle_in_editor_script(max_time_seconds=wait_for_desc["max_time_seconds"], log_interval_seconds=wait_for_desc["log_interval_seconds"])
        else:
            with self.begin_frame():
                assert self._editor.async_loading_service.is_engine_idle()
            yield
            with self.end_frame():
                pass
            yield

        return self._editor

    def _is_editor_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return True
        return False

    def _get_editor_world(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_editor_world:
                return world_desc.world
        assert False

    #
    # get_game(...)
    #

    def get_game(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0,
        wait_for_engine_idle=None, wait_for_engine_idle_max_time_seconds=0.0, wait_for_engine_idle_log_interval_seconds=0.0):

        spear.log_current_function()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        with self.end_frame():
            self.world_registry_service.register_worlds()

        # wait for _is_game_world_initialized(...) or try _is_game_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_game_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        self._wait_for(wait_for_desc=wait_for_desc)

        # set world and initialize services
        with self.begin_frame():
            world = self._get_game_world()
            self._game.set_world(world=world)
            self._unreal_service.initialize()
            self._game.initialize(unreal_service=self._unreal_service)
        with self.end_frame():
            pass

        # wait_for_engine_idle(...) or try is_engine_idle() once and assert
        wait_for_desc = self._get_wait_for_desc(
            wait=wait_for_engine_idle, config=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_FOR_ENGINE_IDLE,
            max_time_seconds=wait_for_engine_idle_max_time_seconds, log_interval_seconds=wait_for_engine_idle_log_interval_seconds)
        if wait_for_desc["retry"]:
            self._game.async_loading_service.wait_for_engine_idle(max_time_seconds=wait_for_desc["max_time_seconds"], log_interval_seconds=wait_for_desc["log_interval_seconds"])
        else:
            with self.begin_frame():
                assert self._game.async_loading_service.is_engine_idle()
            with self.end_frame():
                pass

        return self._game

    def get_game_in_editor_script(self,
        wait_for_world_initialized=None, wait_for_world_initialized_max_time_seconds=0.0, wait_for_world_initialized_sleep_time_seconds=0.0,
        wait_for_engine_idle=None, wait_for_engine_idle_max_time_seconds=0.0, wait_for_engine_idle_log_interval_seconds=0.0):

        spear.log_current_function()

        # Register all worlds. If a previous call to invalidate() raised an exception before destroying its world,
        # then the world is still unregistered on the server, so we register it again here to recover. We register
        # in an end_frame() block, which executes after UE has processed any pending level changes (e.g., from a
        # previous call to OpenLevel(...)), so we don't register a world that is about to be destroyed.
        with self.begin_frame():
            pass
        yield
        with self.end_frame():
            self.world_registry_service.register_worlds()
        yield

        # wait for _is_game_world_initialized(...) or try _is_game_world_initialized(...) once and assert
        wait_for_desc = self._get_wait_for_desc(
            func=self._is_game_world_initialized,
            wait=wait_for_world_initialized, config=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_FOR_WORLD_INITIALIZED,
            max_time_seconds=wait_for_world_initialized_max_time_seconds, sleep_time_seconds=wait_for_world_initialized_sleep_time_seconds)
        yield from self._wait_for_in_editor_script(wait_for_desc=wait_for_desc)

        # set world and initialize services
        with self.begin_frame():
            world = self._get_game_world()
            self._game.set_world(world=world)
            self._unreal_service.initialize()
            self._game.initialize(unreal_service=self._unreal_service)
        yield
        with self.end_frame():
            pass
        yield

        # wait_for_engine_idle_in_editor_script(...) or try is_engine_idle() once and assert
        wait_for_desc = self._get_wait_for_desc(
            wait=wait_for_engine_idle, config=self._config.SPEAR.INSTANCE.GET_GAME_WAIT_FOR_ENGINE_IDLE,
            max_time_seconds=wait_for_engine_idle_max_time_seconds, log_interval_seconds=wait_for_engine_idle_log_interval_seconds)
        if wait_for_desc["retry"]:
            yield from self._game.async_loading_service.wait_for_engine_idle_in_editor_script(max_time_seconds=wait_for_desc["max_time_seconds"], log_interval_seconds=wait_for_desc["log_interval_seconds"])
        else:
            with self.begin_frame():
                assert self._game.async_loading_service.is_engine_idle()
            yield
            with self.end_frame():
                pass
            yield

        return self._game

    def _is_game_world_initialized(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_game_world:
                return True
        return False

    def _get_game_world(self):
        world_descs = self.world_registry_service.get_world_descs()
        for world_desc in world_descs.values():
            if world_desc.is_game_world:
                return world_desc.world
        assert False

    #
    # step(...)
    #

    def step(self, num_frames=1, single_step=False):
        self._engine_service.step(num_frames=num_frames, single_step=single_step)

    def step_in_editor_script(self, num_frames=1, single_step=False):
        yield from self._engine_service.step_in_editor_script(num_frames=num_frames, single_step=single_step)

    #
    # warm_up(...)
    #

    # It can be useful to warm up for a minimum amount of time after getting an editor or game reference, e.g.,
    # because some rendering features (e.g., auto-exposure) leverage temporal coherence between frames, and
    # therefore need to render several frames before converging. warm_up(...) calls step() at least once,
    # and then repeatedly until time_seconds has elapsed, which also takes the engine out of single-step mode.

    def warm_up(self, time_seconds):
        start_time_seconds = time.time()
        self.step()
        while time.time() - start_time_seconds < time_seconds:
            self.step()

    def warm_up_in_editor_script(self, time_seconds):
        start_time_seconds = time.time()
        yield from self.step_in_editor_script()
        while time.time() - start_time_seconds < time_seconds:
            yield from self.step_in_editor_script()

    #
    # rebuild(...)
    #

    def rebuild(self):

        spear.log_current_function()

        with self.begin_frame():
            self._unreal_service.rebuild_cache()
            self._game.rebuild(unreal_service=self._unreal_service)
            if self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet():
                self._editor.rebuild(unreal_service=self._unreal_service)
        with self.end_frame():
            pass

    def rebuild_in_editor_script(self):

        spear.log_current_function()

        with self.begin_frame():
            self._unreal_service.rebuild_cache()
            self._game.rebuild(unreal_service=self._unreal_service)
            if self.engine_globals_service.is_editor() and not self.engine_globals_service.is_running_commandlet():
                self._editor.rebuild(unreal_service=self._unreal_service)
        yield
        with self.end_frame():
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
        except Exception:
            pass # no need to log exception because this case is expected when the Unreal executable is no longer running
        return False

    def close(self, force=False):

        spear.log_current_function()

        # Note that in the constructor, we launch the Unreal executable first and then initialize the client.
        # Normally, we would do things in the reverse order here. But if we terminate the client first, then
        # we can't call the "terminate" and "request_exit" entry points to shut down the Unreal executable. So
        # we terminate the Unreal executable first, and then terminate the client afterwards. If we're killing
        # the Unreal executable, we don't need to call any entry points, so we terminate the client first,
        # because otherwise the client's watchdog would exit as soon as the Unreal executable terminates.

        if force:
            self._client.terminate()
            self._launcher.kill()

        else:
            # terminate EngineService
            if self._config.SPEAR.LAUNCH_MODE == "none":
                pass
            elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
                self._engine_service.step()      # make sure we're not in single-step mode any more
                self._engine_service.terminate() # EngineService must be explicitly terminated before requesting to exit
            else:
                assert False

            self._launcher.terminate(client=self._client)
            self._client.terminate()

    #
    # calling flush() inside a begin_frame() or end_frame() block ensures that there is no work pending on the game thread
    #

    def flush(self):
        self._engine_service.flush()

    #
    # _wait_for
    #

    def _get_wait_for_desc(self, wait, config, func=None, max_time_seconds=0.0, sleep_time_seconds=0.0, log_interval_seconds=0.0):
        if func is None:
            func_name = None
        elif hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__
        if wait is None:
            retry = True
            max_time_seconds = config.MAX_TIME_SECONDS
            sleep_time_seconds = config.SLEEP_TIME_SECONDS
            log_interval_seconds = config.LOG_INTERVAL_SECONDS
        else:
            retry = wait
        return {"func": func, "func_name": func_name, "retry": retry, "max_time_seconds": max_time_seconds, "sleep_time_seconds": sleep_time_seconds, "log_interval_seconds": log_interval_seconds}

    def _wait_for(self, wait_for_desc):
        spear.log_current_function()

        func = wait_for_desc["func"]
        func_name = wait_for_desc["func_name"]
        assert func is not None
        retry = wait_for_desc["retry"]
        max_time_seconds = wait_for_desc["max_time_seconds"]
        sleep_time_seconds = wait_for_desc["sleep_time_seconds"]

        start_time_seconds = time.time()

        if retry:
            spear.log(f"Waiting for {func_name} to return true for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")
            success = self._call_on_game_thread(func=func)
            elapsed_time_seconds = time.time() - start_time_seconds
            while not success and elapsed_time_seconds < max_time_seconds:
                time.sleep(sleep_time_seconds)
                success = self._call_on_game_thread(func=func)
                elapsed_time_seconds = time.time() - start_time_seconds
        else:
            spear.log(f"Calling {func_name} once to make sure it returns true...")
            success = self._call_on_game_thread(func=func)
            elapsed_time_seconds = time.time() - start_time_seconds

        if not success:
            if retry:
                spear.log(f"ERROR: {func_name} never returned true, giving up...")
            else:
                spear.log(f"ERROR: {func_name} didn't return true, giving up...")
            self._client.terminate()
            if self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
                self._launcher.kill()
            assert False

        if retry:
            spear.log(f"Finished waiting for {func_name}, which returned true after {elapsed_time_seconds:.2f} seconds.")
        else:
            spear.log(f"Finished calling {func_name}, which returned true in {elapsed_time_seconds:.2f} seconds.")

    def _wait_for_in_editor_script(self, wait_for_desc):
        spear.log_current_function()

        func = wait_for_desc["func"]
        func_name = wait_for_desc["func_name"]
        assert func is not None
        retry = wait_for_desc["retry"]
        max_time_seconds = wait_for_desc["max_time_seconds"]
        sleep_time_seconds = wait_for_desc["sleep_time_seconds"]

        start_time_seconds = time.time()

        if retry:
            spear.log(f"Waiting for {func_name} to return true for up to {max_time_seconds} seconds, retrying every {sleep_time_seconds} seconds...")
            success = yield from self._call_on_game_thread_in_editor_script(func=func)
            elapsed_time_seconds = time.time() - start_time_seconds
            while not success and elapsed_time_seconds < max_time_seconds:
                sleep_start_time_seconds = time.time()
                while time.time() - sleep_start_time_seconds < sleep_time_seconds:
                    yield
                success = yield from self._call_on_game_thread_in_editor_script(func=func)
                elapsed_time_seconds = time.time() - start_time_seconds
        else:
            spear.log(f"Calling {func_name} once to make sure it returns true...")
            success = yield from self._call_on_game_thread_in_editor_script(func=func)
            elapsed_time_seconds = time.time() - start_time_seconds

        if not success:
            if retry:
                spear.log(f"ERROR: {func_name} never returned true, giving up...")
            else:
                spear.log(f"ERROR: {func_name} didn't return true, giving up...")
            self._client.terminate()
            assert False

        if retry:
            spear.log(f"Finished waiting for {func_name}, which returned true after {elapsed_time_seconds:.2f} seconds.")
        else:
            spear.log(f"Finished calling {func_name}, which returned true in {elapsed_time_seconds:.2f} seconds.")

    # _wait_for(...) always calls func inside a frame, so func can call game thread entry points (e.g., get_world_descs()), and
    # the cost of the extra frame is acceptable if func doesn't need it, because _wait_for(...) is already prepared to wait for several seconds

    def _call_on_game_thread(self, func):
        with self.begin_frame():
            return_value = func()
        with self.end_frame():
            pass
        return return_value

    def _call_on_game_thread_in_editor_script(self, func):
        with self.begin_frame():
            return_value = func()
        yield
        with self.end_frame():
            pass
        yield
        return return_value

    def __repr__(self):
        return f"Instance(_launcher={self._launcher}, _client={self._client})"
