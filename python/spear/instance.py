#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import msgpackrpc
import os
import psutil
import spear
import subprocess
import sys
import time

# EngineService
import spear.engine_service

# services that require a reference to EngineService
import spear.enhanced_input_service
import spear.initialize_editor_world_service
import spear.initialize_game_world_service
import spear.input_service
import spear.shared_memory_service
import spear.unreal_service

# services that require a reference to EngineService and SharedMemoryService
import spear.navigation_service
import spear.sp_func_service


expected_process_status_values = ["disk-sleep", "running", "sleeping", "stopped"] # stopped can happen when attaching to a debugger


class Editor():
    def __init__(self, entry_point_caller, shared_memory_service):
        self.initialize_editor_world_service = spear.initialize_editor_world_service.InitializeEditorWorldService(namespace="editor", entry_point_caller=entry_point_caller)
        self.unreal_service = spear.unreal_service.UnrealService(namespace="editor", entry_point_caller=entry_point_caller)
        self.navigation_service = spear.navigation_service.NavigationService(namespace="editor", entry_point_caller=entry_point_caller, shared_memory_service=shared_memory_service)


class Game():
    def __init__(self, entry_point_caller, shared_memory_service):
        self.initialize_game_world_service = spear.initialize_game_world_service.InitializeGameWorldService(namespace="game", entry_point_caller=entry_point_caller)
        self.unreal_service = spear.unreal_service.UnrealService(namespace="game", entry_point_caller=entry_point_caller)
        self.navigation_service = spear.navigation_service.NavigationService(namespace="game", entry_point_caller=entry_point_caller, shared_memory_service=shared_memory_service)


class Instance():
    def __init__(self, config):

        spear.log_current_function()

        self._config = config
        self._rpc_client = None

        spear.log("    Creating instance with launch mode: " + self._config.SPEAR.LAUNCH_MODE)
        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE == "editor":
            spear.log("    Editor launch mode: " + self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE)
            spear.log("    Editor executable:  " + self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE)
            spear.log("    Editor uproject:    " + self._config.SPEAR.INSTANCE.EDITOR_UPROJECT)
        elif self._config.SPEAR.LAUNCH_MODE == "game":
            spear.log("    Game executable: " + self._config.SPEAR.INSTANCE.GAME_EXECUTABLE)
        else:
            assert False

        # Request to launch the unreal instance and initialize the RPC client.
        self._request_launch_unreal_instance()
        self._initialize_rpc_client()

        # EngineService needs its own custom logic for interacting directly with the RPC client, because
        # EngineService implements the context managers returned by begin_frame() and end_frame().
        # Additionally, all other services call RPC entry points through EngineService.call(), rather than
        # going through the RPC client directly. So we pass in the RPC client directly when constructing
        # EngineService, and we pass in EngineService when constructing all other services.
        self.engine_service = spear.engine_service.EngineService(rpc_client=self._rpc_client, config=self._config)

        # Initialize services that require a reference to EngineService.
        self.enhanced_input_service = spear.enhanced_input_service.EnhancedInputService(entry_point_caller=self.engine_service)
        self.input_service = spear.input_service.InputService(entry_point_caller=self.engine_service)
        self.shared_memory_service = spear.shared_memory_service.SharedMemoryService(entry_point_caller=self.engine_service)

        # Initialize services that require a reference to EngineService and SharedMemoryService.
        self.sp_func_service = spear.sp_func_service.SpFuncService(entry_point_caller=self.engine_service, shared_memory_service=self.shared_memory_service)

        # Initialize Editor and Game containers for world-specific services
        self._editor = Editor(entry_point_caller=self.engine_service, shared_memory_service=self.shared_memory_service)
        self._game = Game(entry_point_caller=self.engine_service, shared_memory_service=self.shared_memory_service)

        # We need to explicitly initialize EngineService before calling begin_frame() for the first time.
        self.engine_service.initialize()

        # Execute warm-up routine.
        self._request_warm_up_unreal_instance(
            time_seconds=self._config.SPEAR.INSTANCE.INSTANCE_WARM_UP_TIME_SECONDS,
            num_frames=self._config.SPEAR.INSTANCE.INSTANCE_WARM_UP_NUM_FRAMES)


    #
    # Public interface
    #

    def get_editor(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):

        spear.log_current_function()
        assert self.engine_service.with_editor()

        if self._config.SPEAR.LAUNCH_MODE == "none":
            if wait is None:
                wait = False
            if warm_up is None:
                warm_up = False
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            if wait is None:
                wait = True
                wait_max_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_MAX_TIME_SECONDS
                wait_sleep_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WAIT_SLEEP_TIME_SECONDS
            if warm_up is None:
                warm_up = True
                warm_up_time_seconds = self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_TIME_SECONDS
                warm_up_num_frames = self._config.SPEAR.INSTANCE.GET_EDITOR_WARM_UP_NUM_FRAMES
        else:
            self._close_rpc_client(verbose=True)
            assert False

        self._wait_until(func=self._editor.initialize_editor_world_service.is_initialized, retry=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        self._warm_up_unreal_instance(time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        return self._editor


    def get_game(self, wait=None, wait_max_time_seconds=0.0, wait_sleep_time_seconds=0.0, warm_up=None, warm_up_time_seconds=0.0, warm_up_num_frames=0):

        spear.log_current_function()

        if self._config.SPEAR.LAUNCH_MODE == "none":
            if wait is None:
                wait = False
            if warm_up is None:
                warm_up = False
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            if wait is None:
                wait = True
                wait_max_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WAIT_MAX_TIME_SECONDS
                wait_sleep_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WAIT_SLEEP_TIME_SECONDS
            if warm_up is None:
                warm_up = True
                warm_up_time_seconds = self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_TIME_SECONDS
                warm_up_num_frames = self._config.SPEAR.INSTANCE.GET_GAME_WARM_UP_NUM_FRAMES
        else:
            self._close_rpc_client(verbose=True)
            assert False

        self._wait_until(func=self._game.initialize_game_world_service.is_initialized, retry=wait, max_time_seconds=wait_max_time_seconds, sleep_time_seconds=wait_sleep_time_seconds)
        self._warm_up_unreal_instance(time_seconds=warm_up_time_seconds, num_frames=warm_up_num_frames)

        return self._game


    def begin_frame(self):
        return self.engine_service.begin_frame()


    def end_frame(self):
        return self.engine_service.end_frame()


    def is_running(self):
        try:
            connected = self.engine_service.ping() == "ping"
            return connected
        except:
            pass # no need to log exception because this case is expected when the instance is no longer running
        return False


    def close(self):
        # Note that in the constructor, we launch the Unreal instance first and then initialize the RPC
        # client. Normally, we would do things in the reverse order here. But if we close the client first,
        # then we can't send a "request_exit" command to the Unreal instance to close it. So we request_exit
        # the Unreal instance first, and then close the client afterwards.
        self._request_exit_unreal_instance()
        self._close_rpc_client(verbose=True)


    #
    # Initialization helper functions
    #

    def _request_launch_unreal_instance(self):

        spear.log_current_function()        
        spear.log("    Requesting to launch Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass

        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # Write a temp config file. We do this before checking LAUNCH_MODE in case the user wants to
            # initialize the config system in the editor using the temp file that gets written here.
            temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.INSTANCE.TEMP_DIR))
            temp_config_file = os.path.realpath(os.path.join(temp_dir, self._config.SPEAR.INSTANCE.TEMP_CONFIG_FILE))

            spear.log("    Writing temp config file: " + temp_config_file)

            os.makedirs(temp_dir, exist_ok=True)
            with open(temp_config_file, "w") as f:
                self._config.dump(stream=f, default_flow_style=False)

            # set up launch executable and command-line arguments
            launch_args = []

            if self._config.SPEAR.LAUNCH_MODE == "editor":
                assert self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE in ["editor", "game"]
                launch_executable = self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE
                launch_args.append(self._config.SPEAR.INSTANCE.EDITOR_UPROJECT)
                if self._config.SPEAR.INSTANCE.EDITOR_LAUNCH_MODE == "game":
                    launch_args.append("-game")
            elif self._config.SPEAR.LAUNCH_MODE == "game":
                launch_executable = self._config.SPEAR.INSTANCE.GAME_EXECUTABLE
            else:
                assert False

            assert os.path.exists(launch_executable)

            launch_executable_name, launch_executable_ext = os.path.splitext(launch_executable)

            if sys.platform == "win32":
                assert launch_executable_name[-4:] == "-Cmd"
                assert launch_executable_ext == ".exe"
                launch_executable_internal = launch_executable
            elif sys.platform == "darwin":
                assert launch_executable_ext == ".app"
                launch_executable_internal = os.path.realpath(os.path.join(launch_executable, "Contents", "MacOS", os.path.basename(launch_executable_name)))
            elif sys.platform == "linux":
                assert launch_executable_ext == ".sh"
                launch_executable_internal = launch_executable
            else:
                assert False

            assert os.path.exists(launch_executable_internal)

            for arg, value in (self._config.SPEAR.INSTANCE.COMMAND_LINE_ARGS).items():
                if value is None:
                    launch_args.append("-{}".format(arg))
                else:
                    launch_args.append("-{}={}".format(arg, value))

            launch_args.append("-config_file={}".format(temp_config_file))
            cmd = [launch_executable_internal] + launch_args

            spear.log("    Launching executable with the following command-line arguments:")
            spear.log_no_prefix(" ".join(cmd))
            spear.log("    Launching executable with the following config values:")
            spear.log_no_prefix(self._config)

            popen = subprocess.Popen(cmd)
            self._process = psutil.Process(popen.pid)

            # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
            status = self._process.status()
            if status not in expected_process_status_values:
                spear.log("    ERROR: Unrecognized process status: " + status)
                spear.log("    ERROR: Killing process " + str(self._process.pid) + "...")
                self._force_kill_unreal_instance()
                assert False

        else:
            assert False

        spear.log("    Finished requesting to launch Unreal instance.")


    def _initialize_rpc_client(self):

        spear.log_current_function()        
        spear.log("    Initializing RPC client...")

        connected = False

        # If we're connecting to a running instance, then we assume that the RPC server is already running
        # and only try to connect once.
        if self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                # Once a connection has been established, the RPC client will wait for timeout seconds before
                # throwing when calling a server function. The RPC client will try to connect reconnect_limit
                # times before returning from its constructor.
                self._rpc_client = msgpackrpc.Client(
                    msgpackrpc.Address("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT),
                    timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                    reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)

                # don't use self.engine_service because it hasn't been initialized yet
                connected = self._rpc_client.call("engine_service.ping") == "ping"

            except Exception as e:
                spear.log("    Exception: ", e)
                self._close_rpc_client(verbose=True)
                assert False

        # otherwise try to connect repeatedly, since the RPC server might not have started yet
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while elapsed_time_seconds < self._config.SPEAR.INSTANCE.INITIALIZE_RPC_CLIENT_MAX_TIME_SECONDS:

                # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible values
                status = self._process.status()
                if status not in expected_process_status_values:
                    spear.log("    ERROR: Unrecognized process status: " + status)
                    break

                try:
                    # Once a connection has been established, the RPC client will wait for timeout seconds
                    # before throwing when calling a server function. The RPC client will try to connect
                    # reconnect_limit times before returning from its constructor.
                    self._rpc_client = msgpackrpc.Client(
                        msgpackrpc.Address("127.0.0.1", self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT), 
                        timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)

                    # don't use self.engine_service because it hasn't been initialized yet
                    connected = self._rpc_client.call("engine_service.ping") == "ping"
                    break

                except:
                    # The client may not clean up resources correctly in this case, so we clean things up
                    # explicitly. See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more
                    # details. There is no need to log the exception because this case is expected until
                    # until we can successfully connect to the RPC server, which is why we set verbose=False
                    # when closing the RPC client.
                    self._close_rpc_client(verbose=False)

                time.sleep(self._config.SPEAR.INSTANCE.INITIALIZE_RPC_CLIENT_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        else:
            assert False

        if not connected:
            spear.log("    ERROR: Couldn't connect to RPC server, giving up...")
            assert False

        spear.log("    Finished initializing RPC client.")


    def _request_warm_up_unreal_instance(self, time_seconds, num_frames):

        spear.log_current_function()
        spear.log("    Requesting to warm up Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass

        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            self._warm_up_unreal_instance(time_seconds, num_frames)

        else:
            self._close_rpc_client(verbose=True)
            assert False

        spear.log("    Finished requesting to warm up Unreal instance.")


    def _warm_up_unreal_instance(self, time_seconds, num_frames):

        spear.log_current_function()        
        spear.log("    Warming up Unreal instance for " + str(time_seconds) + " seconds and " + str(num_frames) + " frames...")

        time.sleep(time_seconds)

        # Execute at least one complete warmup frame to guarantee that we can receive valid observations. If
        # we don't do this, it is possible that Unreal will return an initial visual observation of all
        # zeros. We generally also want to execute more than one warmup frame to warm up various caches and
        # rendering features that leverage temporal coherence between frames.
        for i in range(num_frames):
            with self.begin_frame():
                pass
            with self.end_frame():
                pass

        spear.log("    Finished warming up Unreal instance.")


    def _wait_until(self, func, retry, max_time_seconds, sleep_time_seconds):

        spear.log_current_function()
        spear.log("    Waiting for function to return true: " + str(func))

        if retry:
            spear.log("    Waiting for " + str(max_time_seconds) + " seconds, retrying every " + str(sleep_time_seconds) + " seconds...")
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
            spear.log("    Attempting to call function once...")
            success = func()

        if not success:
            spear.log("    ERROR: Function never returned true, giving up...")
            self._close_rpc_client(verbose=True)
            assert False

        spear.log("    Finished waiting for function.")

    #
    # Termination helper functions
    #

    def _request_exit_unreal_instance(self):

        spear.log_current_function()        
        spear.log("    Requesting to exit Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass

        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:        
            try:
                self.engine_service.request_exit()
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
            self._close_rpc_client(verbose=True)
            assert False

        spear.log("    Finished requesting to exit Unreal instance.")


    def _force_kill_unreal_instance(self):
        spear.log_current_function()
        spear.log("    Forcefully killing Unreal instance...")
        self._process.terminate()
        self._process.kill()
        spear.log("    Finished forcefully killing Unreal instance.")


    def _close_rpc_client(self, verbose):
        if verbose:
            spear.log_current_function()
            spear.log("    Closing RPC client " + str(self._rpc_client) + "...")
        if self._rpc_client is not None:
            self._rpc_client.close()
            self._rpc_client._loop._ioloop.close()
            self._rpc_client = None
        if verbose:
            spear.log("    Finished closing RPC client.")
