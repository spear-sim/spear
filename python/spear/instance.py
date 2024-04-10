#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import msgpackrpc
import os
import psutil
import spear
from subprocess import Popen
import sys
import time

expected_status_values = ["disk-sleep", "running", "sleeping"]

class Instance():
    def __init__(self, config):

        self._config = config

        self._request_launch_unreal_instance()
        self._initialize_rpc_client()

        # Need to do these after we have a valid rpc_client
        self.engine_service = spear.EngineService(self.rpc_client)
        self.game_world_service = spear.GameWorldService(self.rpc_client)
        self.legacy_service = spear.LegacyService(self.rpc_client)

        # Need to do this after we have a valid EngineService object because we call begin_tick(), tick(), and end_tick() here.
        self._initialize_unreal_instance()

    def close(self):
        # Note that in the constructor, we launch the Unreal instance first and then initialize the RPC client. Normally,
        # we would do things in the reverse order here. But if we close the client first, then we can't send a command to
        # the Unreal instance to close it. So we close the Unreal instance first and then close the client.
        self._request_close_unreal_instance()
        self._close_rpc_client()

    def _request_launch_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that an Unreal instance has been launched externally...')
            return

        spear.log("Launching Unreal instance...")

        # write temp file
        temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.INSTANCE.TEMP_DIR))
        temp_config_file = os.path.realpath(os.path.join(temp_dir, self._config.SPEAR.INSTANCE.TEMP_CONFIG_FILE))

        spear.log("Writing temp config file: " + temp_config_file)

        os.makedirs(temp_dir, exist_ok=True)
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        # set up launch executable and command-line arguments
        launch_args = []

        if self._config.SPEAR.LAUNCH_MODE == "uproject":
            launch_executable = self._config.SPEAR.INSTANCE.EDITOR_EXECUTABLE
            launch_args.append(self._config.SPEAR.INSTANCE.UPROJECT)
            launch_args.append("-game") # launch the game using uncooked content
        elif self._config.SPEAR.LAUNCH_MODE == "standalone":
            launch_executable = self._config.SPEAR.STANDALONE
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

        popen = Popen(cmd)
        self._process = psutil.Process(popen.pid)

        # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in expected_status_values:
            spear.log("ERROR: Unrecognized process status: " + status)
            spear.log("ERROR: Killing process " + str(self._process.pid) + "...")
            self._force_kill_unreal_instance()
            self._close_rpc_client()
            assert False

    def _initialize_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that the Unreal instance is already initialized...')
            return

        spear.log("Initializing Unreal instance, warming up for " + str(1 + self._config.SPEAR.INSTANCE.NUM_EXTRA_WARMUP_TICKS) + " ticks...")

        # Do at least one complete tick to guarantee that we can receive valid observations. If we don't
        # do this, it is possible that Unreal will return an initial visual observation of all zeros. We
        # generally also want to do more than one tick to warm up various caches and rendering features
        # that leverage temporal coherence between frames.
        for i in range(1 + self._config.SPEAR.INSTANCE.NUM_EXTRA_WARMUP_TICKS):
            self.engine_service.begin_tick()
            self.game_world_service.unpause_game()
            self.engine_service.tick()
            self.game_world_service.pause_game()
            self.engine_service.end_tick()

        spear.log("Finished initializing Unreal instance.")

    def _request_close_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "none":
            spear.log('SPEAR.LAUNCH_MODE == "none" so we assume that the Unreal instance should remain open...')
            return

        spear.log("Closing Unreal instance...")
        self.rpc_client.call("engine_service.request_close")
        status = "running"
        while status in expected_status_values:
            try:
                status = self._process.status()
            except psutil.NoSuchProcess:
                break
        time.sleep(self._config.SPEAR.INSTANCE.REQUEST_CLOSE_UNREAL_INSTANCE_SLEEP_TIME_SECONDS)

        spear.log("Finished closing Unreal instance.")

    def _force_kill_unreal_instance(self):
        spear.log("Forcefully killing Unreal instance...")
        self._process.terminate()
        self._process.kill()
        spear.log("Finished forcefully killing Unreal instance.")

    def _initialize_rpc_client(self):

        spear.log("Initializing RPC client...")

        connected = False
        
        # if we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                self.rpc_client = msgpackrpc.Client(
                    msgpackrpc.Address("127.0.0.1", self._config.SP_ENGINE.PORT),
                    timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                    reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self.rpc_client.call("engine_service.ping")
                connected = True

            except Exception as e:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more details.
                spear.log("Exception: ", e)
                self._close_rpc_client()

        # otherwise try to connect repeatedly, since the RPC server might not have started yet
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "standalone"]:

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while elapsed_time_seconds < self._config.SPEAR.INSTANCE.INITIALIZE_RPC_CLIENT_MAX_TIME_SECONDS:

                # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
                status = self._process.status()
                if status not in expected_status_values:
                    spear.log("ERROR: Unrecognized process status: " + status)
                    break

                try:
                    self.rpc_client = msgpackrpc.Client(
                        msgpackrpc.Address("127.0.0.1", self._config.SP_ENGINE.PORT), 
                        timeout=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.SPEAR.INSTANCE.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self.rpc_client.call("engine_service.ping")
                    connected = True
                    break

                except Exception as e:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14 for more details.
                    spear.log("Exception:", e)
                    self._close_rpc_client()

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

    def _close_rpc_client(self):
        spear.log("Closing RPC client...")
        self.rpc_client.close()
        self.rpc_client._loop._ioloop.close()
        spear.log("Finished closing RPC client.")
