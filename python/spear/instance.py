#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import inspect
import itertools
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
# Helper classes
#

class WorldScopedServices():
    def __init__(self, namespace, engine_service, engine_globals_service, shared_memory_service, sp_func_service, config):

        # needed internally in get_unreal_object(...)
        self._sp_func_service = sp_func_service
        self._config = config

        self.unreal_service = spear.UnrealService(
            entry_point_caller=spear.CallSyncEntryPointCaller(service_name=f"{namespace}.unreal_service", engine_service=engine_service),
            sp_func_service=self._sp_func_service,
            config=self._config)

        self.navigation_service = spear.NavigationService(
            entry_point_caller=spear.CallSyncEntryPointCaller(service_name=f"{namespace}.navigation_service", engine_service=engine_service),
            shared_memory_service=shared_memory_service,
            sp_func_service=self._sp_func_service,
            unreal_service=self.unreal_service,
            config=self._config)

        self.engine_globals_service = spear.EngineGlobalsServiceWrapper(
            service=engine_globals_service,
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

class EditorScopedServices(WorldScopedServices):
    def __init__(self, namespace, engine_service, engine_globals_service, shared_memory_service, sp_func_service, config):
        super().__init__(
            namespace=namespace,
            engine_service=engine_service,
            engine_globals_service=engine_globals_service,
            shared_memory_service=shared_memory_service,
            sp_func_service=sp_func_service,
            config=config)

        self.initialize_editor_world_service = spear.InitializeWorldService(
            entry_point_caller=spear.CallSyncEntryPointCaller(service_name=f"{namespace}.initialize_editor_world_service", engine_service=engine_service))

class GameScopedServices(WorldScopedServices):
    def __init__(self, namespace, engine_service, engine_globals_service, shared_memory_service, sp_func_service, config):
        super().__init__(
            namespace=namespace,
            engine_service=engine_service,
            engine_globals_service=engine_globals_service,
            shared_memory_service=shared_memory_service,
            sp_func_service=sp_func_service,
            config=config)

        self.initialize_game_world_service = spear.InitializeWorldService(
            entry_point_caller=spear.CallSyncEntryPointCaller(service_name=f"{namespace}.initialize_game_world_service", engine_service=engine_service))

#
# Instance
#

class Instance():
    def __init__(self, config):

        spear.log_current_function()

        self._config = config
        self._client = None

        spear.log("    Creating instance with launch mode: ", self._config.SPEAR.LAUNCH_MODE)
        if self._config.SPEAR.LAUNCH_MODE == "none":
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

        self._engine_service = spear.EngineService(client=self._client, config=self._config)
        self._engine_service.initialize()

        # Now that EngineService is initialized, we can validate our client and server entry points.
        self._validate_client_and_server_entry_points()

        # Initialize services that require a reference to EngineService.
        self.engine_globals_service = spear.EngineGlobalsService(entry_point_caller=spear.CallSyncEntryPointCaller(service_name="engine_globals_service", engine_service=self._engine_service))
        self.enhanced_input_service = spear.EnhancedInputService(entry_point_caller=spear.CallSyncEntryPointCaller(service_name="enhanced_input_service", engine_service=self._engine_service))
        self.input_service = spear.InputService(entry_point_caller=spear.CallSyncEntryPointCaller(service_name="input_service", engine_service=self._engine_service))
        self.shared_memory_service = spear.SharedMemoryService(entry_point_caller=spear.CallSyncEntryPointCaller(service_name="shared_memory_service", engine_service=self._engine_service))

        # Initialize services that require a reference to EngineService and SharedMemoryService.
        self.sp_func_service = spear.SpFuncService(entry_point_caller=spear.CallSyncEntryPointCaller(service_name="sp_func_service", engine_service=self._engine_service), shared_memory_service=self.shared_memory_service)

        # Initialize EditorScopedServices and GameScopedServices.

        self._editor = EditorScopedServices(
            namespace="editor",
            engine_service=self._engine_service,
            engine_globals_service=self.engine_globals_service,
            shared_memory_service=self.shared_memory_service,
            sp_func_service=self.sp_func_service,
            config=self._config)

        self._game = GameScopedServices(
            namespace="game",
            engine_service=self._engine_service,
            engine_globals_service=self.engine_globals_service,
            shared_memory_service=self.shared_memory_service,
            sp_func_service=self.sp_func_service,
            config=self._config)

    #
    # Public interface
    #

    def get_editor(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):

        spear.log_current_function()
        assert self.engine_globals_service.is_with_editor() and not self.engine_globals_service.is_running_commandlet() and not " -game " in self.engine_globals_service.get_command_line()

        if wait is None:
            retry = True
            wait_max_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_MAX_TIME_SECONDS
            wait_sleep_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_SLEEP_TIME_SECONDS
        else:
            retry = wait
        if warm_up is None:
            warm_up = True
            warm_up_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_TIME_SECONDS
            warm_up_num_frames = self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_NUM_FRAMES

        # even if we're not waiting, we still want to guarantee that the editor is ready, so we call _wait_until(...) unconditionally
        self._wait_until(func=self._editor.initialize_editor_world_service.is_initialized, retry=retry, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)

        # only warm up if necessary
        if warm_up:
            self._request_warm_up_unreal_instance(time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        # UnrealService must be explicitly initialized but only after we're finished warming up
        spear.log("    Initializing editor-scoped services...")
        with self.begin_frame():
            self._editor.unreal_service.initialize()
        with self.end_frame():
            pass
        spear.log("    Finished initializing editor-scoped services.")

        return self._editor


    def get_game(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):

        spear.log_current_function()

        if wait is None:
            retry = True
            wait_max_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WAIT_MAX_TIME_SECONDS
            wait_sleep_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WAIT_SLEEP_TIME_SECONDS
        else:
            retry = wait
        if warm_up is None:
            warm_up = True
            warm_up_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_TIME_SECONDS
            warm_up_num_frames = self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_NUM_FRAMES

        # even if we're not waiting, we still want to guarantee that the game is ready, so we call _wait_until(...) unconditionally
        self._wait_until(func=self._game.initialize_game_world_service.is_initialized, retry=retry, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)

        # only warm up if necessary
        if warm_up:
            self._request_warm_up_unreal_instance(time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        # UnrealService must be explicitly initialized but only after we're finished warming up
        spear.log("    Initializing game-scoped services...")
        with self.begin_frame():
            self._game.unreal_service.initialize()
        with self.end_frame():
            pass
        spear.log("    Finished initializing game-scoped services.")

        return self._game


    def begin_frame(self):
        return self._engine_service.begin_frame()


    def end_frame(self):
        return self._engine_service.end_frame()


    def is_running(self):
        try:
            connected = self.engine_globals_service.ping() == "ping"
            return connected
        except:
            pass # no need to log exception because this case is expected when the instance is no longer running
        return False


    # most users should not need to call this function because get_game(...) and get_editor(...) will warm up by default
    def warm_up(self, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):
        spear.log_current_function()
        if warm_up is None:
            warm_up = True
            warm_up_time_seconds = self._config.SPEAR.INSTANCE.INSTANCE_WARM_UP_TIME_SECONDS
            warm_up_num_frames = self._config.SPEAR.INSTANCE.INSTANCE_WARM_UP_NUM_FRAMES

        assert warm_up
        self._request_warm_up_unreal_instance(time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)


    def flush(self):
        with self.begin_frame():
            pass
        with self.end_frame():
            pass


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
    # Initialization helper functions
    #

    def _request_launch_unreal_instance(self):

        spear.log_current_function(prefix="    ")
        spear.log("        Requesting to launch Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
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
        if self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                # Once a connection has been established, the client will wait for timeout seconds before
                # throwing when calling a server function.
                spear.log("        Attempting to connect to server...")
                self._client = spear_ext.Client("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT)
                self._client.set_timeout(int(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS*1000))
                connected = self._client._call_sync_on_worker_thread_as_string("engine_globals_service.call_sync_on_worker_thread.ping") == "ping" # self._engine_service hasn't been initialized yet

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
                    self._client = spear_ext.Client("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT)
                    self._client.set_timeout(int(self._config.SPEAR.INSTANCE.CLIENT_INTERNAL_TIMEOUT_SECONDS*1000))
                    connected = self._client._call_sync_on_worker_thread_as_string("engine_globals_service.call_sync_on_worker_thread.ping") == "ping" # self._engine_service hasn't been initialized yet
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

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            pid = self._client._call_sync_on_worker_thread_as_int64("engine_globals_service.call_sync_on_worker_thread.get_current_process_id") # self._engine_service hasn't been initialized yet
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


    def _validate_client_and_server_entry_points(self):

        spear.log_current_function(prefix="    ")        
        spear.log("        Validating client and server entry points...")

        # Validate func signature type descs.

        client_signature_type_descs = spear_ext.Client.get_entry_point_signature_type_descs()
        server_signature_type_descs = self._engine_service.get_server_signature_type_descs()

        valid = True
        for c, s in itertools.zip_longest(client_signature_type_descs, server_signature_type_descs):
            if c.type_names != s.type_names or c.const_strings != s.const_strings or c.ref_strings != s.ref_strings:
                valid = False
                spear.log(f"        ERROR: Mismatch between registered data types on the client and server.")
                spear.log(f"        ERROR:     Type names:")
                spear.log(f"        ERROR:         Client: {c.type_names}")
                spear.log(f"        ERROR:         Server: {s.type_names}")
                spear.log(f"        ERROR:     Const strings:")
                spear.log(f"        ERROR:         Client: {c.const_strings}")
                spear.log(f"        ERROR:         Server: {c.const_strings}")
                spear.log(f"        ERROR:     Ref Strings:")
                spear.log(f"        ERROR:         Client: {c.ref_strings}")
                spear.log(f"        ERROR:         Server: {c.ref_strings}")

        assert valid

        def get_func_signature_string(func_signature):
            if len(func_signature) == 0:
                assert False
            elif len(func_signature) == 1:
                func_signature_string = func_signature[0].type_names["python_ext"]
            else:
                return_value_string = func_signature[0].type_names["python_ext"]
                args_string = ', '.join([ desc.const_strings['python_ext'] + desc.type_names['python_ext'] + desc.ref_strings['python_ext'] for desc in func_signature[1:] ])
                func_signature_string = return_value_string + ", " + args_string
            return func_signature_string

        def validate_entry_points(registry_name, client_signature_descs, server_signature_descs, get_client_func_name):

            valid = True

            # Search for server entry points that can't be called from the client.
            for s in server_signature_descs:
                found = False
                func_signature_string = get_func_signature_string(func_signature=s.func_signature)

                for c in client_signature_descs:
                    if c.name == get_client_func_name(s.func_signature) and c.func_signature_id == s.func_signature_id:
                        if found:
                            spear.log(f"        ERROR: Server entry point callable from multiple client functions: ", s.name)
                            spear.log(f"        ERROR:     Function signature ({len(s.func_signature) - 1} args): <{func_signature_string}>")
                            valid = False
                        found = True

                if not found:
                    valid = False
                    spear.log(f"        ERROR: Server entry point not callable from client: ", s.name)
                    spear.log(f"        ERROR:     Function signature ({len(s.func_signature) - 1} args): <{func_signature_string}>")

            # Search for client entry points that don't correspond to any server entry point.
            for c in client_signature_descs:
                found = False
                func_signature_string = get_func_signature_string(c.func_signature)

                for s in server_signature_descs:
                    if s.func_signature_id == c.func_signature_id:
                        found = True
                        break

                if not found:
                    valid = False
                    spear.log(f"        ERROR: Client entry point does not correspond to any server entry point: {c.name} (registry_name={registry_name})")
                    spear.log(f"        ERROR:     Function signature ({len(c.func_signature) - 1} args): <{func_signature_string}>")

            return valid

        client_signature_descs = spear_ext.Client.get_entry_point_signature_descs()
        server_signature_descs = { registry_name: descs.values() for registry_name, descs in self._engine_service.get_server_signature_descs().items() }

        valid = \
            validate_entry_points("call_sync_on_worker_thread",         client_signature_descs["call_sync_on_worker_thread"],         server_signature_descs["call_sync_on_worker_thread"],         lambda s: "_call_sync_on_worker_thread_as_" + s[0].type_names["entry_point"]) and \
            validate_entry_points("call_sync_on_game_thread",           client_signature_descs["call_sync_on_game_thread"],           server_signature_descs["call_sync_on_game_thread"],           lambda s: "_call_sync_on_game_thread_as_" + s[0].type_names["entry_point"]) and \
            validate_entry_points("call_async_on_game_thread",          client_signature_descs["call_async_on_game_thread"],          server_signature_descs["call_async_on_game_thread"],          lambda s: "_call_async_on_game_thread") and \
            validate_entry_points("send_async_on_game_thread",          client_signature_descs["send_async_on_game_thread"],          server_signature_descs["send_async_on_game_thread"],          lambda s: "_send_async_on_game_thread") and \
            validate_entry_points("get_future_result_from_game_thread", client_signature_descs["get_future_result_from_game_thread"], server_signature_descs["get_future_result_from_game_thread"], lambda s: "_get_future_result_from_game_thread_as_" + s[0].type_names["entry_point"])

        assert valid

        spear.log("        Finished validating client and server entry points.")


    def _wait_until(self, func, retry, max_time_seconds, sleep_time_seconds):

        spear.log_current_function(prefix="    ")

        func_name = ""
        if hasattr(func, "__qualname__"):
            func_name = func.__qualname__
        elif hasattr(func, "__name__"):
            func_name = func.__name__
        else:
            func_name = type(func).__name__

        spear.log("        Waiting for function to return true: ", func_name)

        if retry:
            spear.log("        Waiting for up to ", max_time_seconds, " seconds, retrying every ", sleep_time_seconds, " seconds...")
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

        spear.log("        Finished waiting for function to return true.")


    def _request_warm_up_unreal_instance(self, time_seconds, num_frames):

        spear.log_current_function(prefix="    ")
        spear.log("        Requesting to warm up Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            self._warm_up_unreal_instance(time_seconds, num_frames)
        else:
            self._terminate_client(verbose=True, log_prefix="        ")
            assert False

        spear.log("        Finished requesting to warm up Unreal instance.")


    def _warm_up_unreal_instance(self, time_seconds, num_frames):

        spear.log_current_function(prefix="        ")
        spear.log("            Warming up Unreal instance for ", time_seconds, " seconds and ", num_frames, " frames...")

        if time_seconds > 0.0:
            time.sleep(time_seconds)

        for i in range(num_frames):
            self.flush()

        spear.log("            Finished warming up Unreal instance.")


    #
    # Shutdown helper functions
    #

    def _request_terminate_unreal_instance(self):

        spear.log_current_function(prefix="    ")
        spear.log("        Requesting to terminate Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:        
            try:
                self.flush()
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
            self._terminate_client(verbose=True, log_prefix="        ")
            assert False

        spear.log("        Finished requesting to terminate Unreal instance.")


    def _force_kill_unreal_instance(self):
        spear.log_current_function(prefix="    ")
        spear.log("        Forcefully killing Unreal instance...")
        self._process.terminate()
        self._process.kill()
        spear.log("        Finished forcefully killing Unreal instance.")


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
