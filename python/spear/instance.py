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

expected_status_values = ["disk-sleep", "running", "sleeping", "stopped"] # stopped can happen when attaching to a debugger

class Instance():
    def __init__(self, config):

        self._config = config

        self._request_launch_unreal_instance()
        self._initialize_rpc_client()

        # Initialize services after we have a valid RPC client.

        # EngineService needs its own custom logic for interacting directly with the RPC client, because
        # EngineService implements the context managers returned by begin_frame() and end_frame().
        # Additionally, all other services call RPC entry points through EngineService.call(), rather than
        # going through the RPC client directly. So we pass in the RPC client directly when constructing
        # EngineService, and we pass in EngineService when constructing all other services.
        self.engine_service = spear.EngineService(rpc_client=self._rpc_client, config=self._config)

        # Construct all other services by passing in EngineService
        self.enhanced_input_service = spear.EnhancedInputService(entry_point_caller=self.engine_service)
        self.input_service = spear.InputService(entry_point_caller=self.engine_service)
        self.legacy_service = spear.LegacyService(entry_point_caller=self.engine_service)
        self.sp_func_service = spear.SpFuncService(entry_point_caller=self.engine_service)
        self.unreal_service = spear.UnrealService(entry_point_caller=self.engine_service)

        # We need to do this after we have a valid EngineService object because we call EngineService.begin_frame()
        # and EngineService.end_frame() here.
        self._request_initialize_unreal_instance()

    def is_running(self):
        try:
            initialized = self.engine_service.is_initialized()
            return initialized
        except:
            pass # no need to log exception because this case is expected when the instance is no longer running
        return False

    def begin_frame(self):
        return self.engine_service.begin_frame()

    def end_frame(self):
        return self.engine_service.end_frame()

    def close(self):
        # Note that in the constructor, we launch the Unreal instance first and then initialize the RPC
        # client. Normally, we would do things in the reverse order here. But if we close the client first,
        # then we can't send a command to the Unreal instance to close it. So we close the Unreal instance
        # first and then close the client.
        self._request_close_unreal_instance()
        self._close_rpc_client(verbose=True)

    def _request_launch_unreal_instance(self):

        spear.log("Requesting to launch Unreal instance...")

        # Write a temp config file. We do this before checking LAUNCH_MODE in case the user wants to
        # initialize the config system in the editor using the temp file that gets written here.
        temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.INSTANCE.TEMP_DIR))
        temp_config_file = os.path.realpath(os.path.join(temp_dir, self._config.SPEAR.INSTANCE.TEMP_CONFIG_FILE))

        spear.log("Writing temp config file: " + temp_config_file)

        os.makedirs(temp_dir, exist_ok=True)
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that an Unreal instance has been launched externally...')
            return

        # set up launch executable and command-line arguments
        launch_args = []

        if self._config.SPEAR.LAUNCH_MODE == "editor":
            launch_executable = self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE
            launch_args.append(self._config.SPEAR.INSTANCE.UPROJECT)
            launch_args.append("-game") # launch the game using uncooked content
        elif self._config.SPEAR.LAUNCH_MODE == "standalone":
            launch_executable = self._config.SPEAR.STANDALONE_EXECUTABLE
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

        spear.log("Launching executable with the following command-line arguments:")
        spear.log_no_prefix(" ".join(cmd))

        spear.log("Launching executable with the following config values:")
        spear.log_no_prefix(self._config)

        popen = subprocess.Popen(cmd)
        self._process = psutil.Process(popen.pid)

        # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in expected_status_values:
            spear.log("ERROR: Unrecognized process status: " + status)
            spear.log("ERROR: Killing process " + str(self._process.pid) + "...")
            self._force_kill_unreal_instance()
            self._close_rpc_client(verbose=True)
            assert False

        spear.log("Finished launching Unreal instance.")

    def _request_initialize_unreal_instance(self):

        spear.log("Requesting to initialize Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that the Unreal instance is already initialized...')
            return

        spear.log("Waiting for " + str(self._config.SPEAR.INSTANCE.INITIALIZE_UNREAL_INSTANCE_SLEEP_TIME_SECONDS) + " seconds before attempting to execute warmup frames...")
        time.sleep(self._config.SPEAR.INSTANCE.INITIALIZE_UNREAL_INSTANCE_SLEEP_TIME_SECONDS)

        # Execute at least one complete warmup frame to guarantee that we can receive valid observations. If
        # we don't do this, it is possible that Unreal will return an initial visual observation of all
        # zeros. We generally also want to execute more than one warmup frame to warm up various caches and
        # rendering features that leverage temporal coherence between frames.
        spear.log("Executing " + str(self._config.SPEAR.INSTANCE.INITIALIZE_UNREAL_INSTANCE_NUM_WARMUP_FRAMES) + " warmup frames...")
        for i in range(self._config.SPEAR.INSTANCE.INITIALIZE_UNREAL_INSTANCE_NUM_WARMUP_FRAMES):
            with self.begin_frame():
                pass
            with self.end_frame():
                pass

        spear.log("Finished initializing Unreal instance.")

    def _request_close_unreal_instance(self):

        spear.log("Requesting to close Unreal instance...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that the Unreal instance should remain open...')
            return

        try:
            self.engine_service.request_exit()
        except:
            pass # no need to log exception because this case is expected when the instance is no longer running

        try:
            status = self._process.status()
        except psutil.NoSuchProcess:
            status = None

        while status in expected_status_values:
            time.sleep(self._config.SPEAR.INSTANCE.REQUEST_CLOSE_UNREAL_INSTANCE_SLEEP_TIME_SECONDS)
            try:
                status = self._process.status()
            except psutil.NoSuchProcess:
                break

        spear.log("Finished closing Unreal instance.")

    def _force_kill_unreal_instance(self):

        spear.log("Forcefully killing Unreal instance...")
        self._process.terminate()
        self._process.kill()
        spear.log("Finished forcefully killing Unreal instance.")

    def _initialize_rpc_client(self):

        spear.log("Initializing RPC client...")

        connected = False
        
        # If we're connecting to a running instance, then we assume that the RPC server is already running
        # and only try to connect once.
        if self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                # Once a connection has been established, the RPC client will wait for timeout seconds before
                # throwing when calling a server function. The RPC client will try to connect reconnect_limit
                # times before returning from its constructor.
                self._rpc_client = msgpackrpc.Client(
                    msgpackrpc.Address("127.0.0.1", self._config.SP_SERVICES.RPC_SERVER_PORT),
                    timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                    reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)

                # don't use self._engine_service because it hasn't been initialized yet
                engine_service_is_initialized = self._rpc_client.call("engine_service.is_initialized")
                spear.log("engine_service_is_initialized: ", engine_service_is_initialized)
                if engine_service_is_initialized:
                    connected = True

            except Exception as e:
                spear.log("Exception: ", e)

            # The client may not clean up resources correctly in this case, so we clean things up
            # explicitly. See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more
            # details.
            if not connected:
                self._close_rpc_client(verbose=True)

        # otherwise try to connect repeatedly, since the RPC server might not have started yet
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "standalone"]:

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while elapsed_time_seconds < self._config.SPEAR.INSTANCE.INITIALIZE_RPC_CLIENT_MAX_TIME_SECONDS:

                # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible values
                status = self._process.status()
                if status not in expected_status_values:
                    spear.log("ERROR: Unrecognized process status: " + status)
                    break

                try:
                    # Once a connection has been established, the RPC client will wait for timeout seconds
                    # before throwing when calling a server function. The RPC client will try to connect
                    # reconnect_limit times before returning from its constructor.
                    self._rpc_client = msgpackrpc.Client(
                        msgpackrpc.Address("127.0.0.1", self._config.SP_SERVICES.RPC_SERVER_PORT), 
                        timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    engine_service_is_initialized = self._rpc_client.call("engine_service.is_initialized")
                    spear.log("engine_service_is_initialized: ", engine_service_is_initialized)
                    if engine_service_is_initialized:
                        connected = True
                        break

                except:
                    pass

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
            spear.log("ERROR: Couldn't connect to RPC server, giving up...")
            if self._config.SPEAR.LAUNCH_MODE in ["editor", "standalone"]:
                self._force_kill_unreal_instance()
            assert False

        spear.log("Finished initializing RPC client.")

    def _close_rpc_client(self, verbose):
        if verbose:
            spear.log("Closing RPC client...")
        self._rpc_client.close()
        self._rpc_client._loop._ioloop.close()
        if verbose:
            spear.log("Finished closing RPC client.")
