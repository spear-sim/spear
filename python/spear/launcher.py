#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import os
import psutil
import spear
import sys
import time


#
# Launcher
#

class Launcher():

    # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
    expected_process_status_values = ["disk-sleep", "running", "sleeping", "stopped"] # stopped can happen when attaching to a debugger

    def __init__(self, config):
        self._config = config

        self.process = None

    def launch(self):

        spear.log_current_function()
        spear.log("Requesting to launch Unreal executable...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass

        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # Write a temp config file.
            temp_dir = os.path.realpath(self._config.SPEAR.LAUNCHER.TEMP_DIR)
            temp_config_file = os.path.realpath(os.path.join(temp_dir, self._config.SPEAR.LAUNCHER.TEMP_CONFIG_FILE))

            spear.log("Writing temp config file: ", temp_config_file)

            os.makedirs(temp_dir, exist_ok=True)
            with open(temp_config_file, "w") as f:
                self._config.dump(stream=f, default_flow_style=False)

            # Find launch executable to launch. The correct executable to launch depends on the platform and
            # whether we're launching a standalone game or editor. Also create a list of command-line
            # arguments.

            launch_args = []

            if self._config.SPEAR.LAUNCH_MODE == "editor":
                assert self._config.SPEAR.EDITOR_LAUNCH_MODE in ["editor", "game"]
                assert os.path.exists(self._config.SPEAR.EDITOR_UPROJECT)
                launch_executable = self._config.SPEAR.EDITOR_EXECUTABLE
                launch_args.append(spear.tools.normalize_unreal_cmd_path(path=self._config.SPEAR.EDITOR_UPROJECT))
                if self._config.SPEAR.EDITOR_LAUNCH_MODE == "game":
                    launch_args.append("-game")
            elif self._config.SPEAR.LAUNCH_MODE == "game":
                launch_executable = self._config.SPEAR.GAME_EXECUTABLE
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

            assert os.path.exists(launch_executable_internal)

            # spear.tools.popen_unreal_cmd(...) only adds quotes around values that contain spaces, which matters
            # because Unreal doesn't parse quoted numerical values correctly (e.g., -resx="512").
            for arg, value in (self._config.SPEAR.LAUNCHER.COMMAND_LINE_ARGS).items():
                if value is None:
                    launch_args.append(f"-{arg}")
                else:
                    launch_args.append(f"-{arg}={value}")

            launch_args.append(f"-sp-config-file={spear.tools.normalize_unreal_cmd_path(path=temp_config_file)}")
            cmd = [launch_executable_internal] + launch_args

            spear.log("Launching executable with the following command-line arguments:")
            spear.log(f"    {launch_executable_internal}")
            for launch_arg in launch_args:
                spear.log(f"        {launch_arg}")
            spear.log("Launching executable with the following config values:")
            spear.log_no_prefix(self._config)

            popen = spear.tools.popen_unreal_cmd(cmd=cmd)
            self.process = psutil.Process(popen.pid)

            status = self.process.status()
            if status not in Launcher.expected_process_status_values:
                spear.log("ERROR: Unrecognized process status: ", status)
                spear.log(f"ERROR: Killing process {self.process.pid}...")
                self.kill()
                assert False

        else:
            assert False

        spear.log("Finished requesting to launch Unreal executable.")

    def initialize(self, client):

        spear.log_current_function()
        spear.log("Requesting to initialize Unreal executable...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # EngineService hasn't been initialized yet, so we call get_current_process_id directly through the inner client
            pid = client.client.call("engine_globals_service.call_sync_on_worker_thread.get_current_process_id")

            if pid == self.process.pid:
                spear.log("Validated engine_globals_service.call_sync_on_worker_thread.get_current_process_id: ", pid)
            else:
                spear.log(f"ERROR: engine_globals_service.call_sync_on_worker_thread.get_current_process_id returned {pid} but the PID of the process we just launched is {self.process.pid}. The Unreal Editor might be open already, or there might be another SpearSim executable running in the background. Close the Unreal Editor and other SpearSim executables and try launching again.")
                client.terminate()
                self.kill()
                assert False
        else:
            assert False

        spear.log("Finished requesting to initialize Unreal executable.")

    def terminate(self, client):

        spear.log_current_function()
        spear.log("Requesting to terminate Unreal executable...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            pass
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # We assume we're in standalone Python, because if we were in editor Python, we would have
            # already set LAUNCH_MODE == "none".
            assert spear.__can_import_spear_ext__
            assert not spear.__can_import_unreal__

            # EngineService has already been terminated, so we call request_exit directly through the inner client
            client.client.request_exit(False)

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            try:
                status = self.process.status()
            except psutil.NoSuchProcess:
                status = None
            while status in Launcher.expected_process_status_values and elapsed_time_seconds < self._config.SPEAR.LAUNCHER.TERMINATE_MAX_TIME_SECONDS:
                time.sleep(self._config.SPEAR.LAUNCHER.TERMINATE_SLEEP_TIME_SECONDS)
                try:
                    status = self.process.status()
                except psutil.NoSuchProcess:
                    status = None
                    break
                elapsed_time_seconds = time.time() - start_time_seconds

            if status in Launcher.expected_process_status_values:
                spear.log("ERROR: Unreal process did not terminate successfully, forcefully killing...")
                self.kill()
        else:
            assert False

        spear.log("Finished requesting to terminate Unreal executable.")

    def kill(self):

        spear.log_current_function()
        spear.log("Forcefully killing Unreal executable...")

        if self._config.SPEAR.LAUNCH_MODE == "none":
            assert False
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
            try:
                self.process.kill()
            except psutil.NoSuchProcess:
                spear.log("Process already terminated.")
            except Exception as e:
                spear.log("Exception: ", e)
        else:
            assert False

        spear.log("Finished forcefully killing Unreal executable.")

    def __repr__(self):
        return f"Launcher(process={self.process})"
