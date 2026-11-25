#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear
import time

if spear.__can_import_spear_ext__:
    import spear_ext


#
# Client
#

class Client():
    def __init__(self, config):
        self._config = config
        self.client = None

    def initialize(self, launcher):

        spear.log_current_function(prefix="    ")
        spear.log("        Initializing client...")

        connected = False

        # If we're connecting to an already running Unreal executable, then we assume that the server is
        # already running and only try to connect once.
        if self._config.SPEAR.LAUNCH_MODE == "none":

            try:
                # Once a connection has been established, the client will wait for timeout seconds before
                # throwing when calling a server function.
                spear.log("        Attempting to connect to server...")
                if spear.__can_import_spear_ext__:
                    self.client = spear_ext.Client(self._config.SPEAR.CLIENT.RPC_SERVER_ADDRESS, self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT, self._config.SPEAR.CLIENT.SUPPRESS_DEFAULT_LOGGING)
                    self.client.set_timeout(int(self._config.SPEAR.CLIENT.INTERNAL_TIMEOUT_SECONDS)*1000)
                elif spear.__can_import_unreal__:
                    self.client = spear.editor.Client(
                        address=self._config.SPEAR.CLIENT.RPC_SERVER_ADDRESS,
                        port=self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT,
                        timeout=float(self._config.SPEAR.CLIENT.INTERNAL_TIMEOUT_SECONDS),
                        reconnect_limit=int(self._config.SPEAR.CLIENT.EDITOR_INTERNAL_RECONNECT_LIMIT))
                else:
                    assert False
                connected = self.client.ping() == "ping"
            except Exception as e:
                spear.log("        Exception: ", e)
                self.terminate(log_prefix="        ")
                assert False

        # Otherwise try to connect repeatedly, since the server might not have started yet.
        elif self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:

            # We assume we're in standalone Python, because if we were in editor Python, we would have
            # already set LAUNCH_MODE == "none".
            assert spear.__can_import_spear_ext__
            assert not spear.__can_import_unreal__
            assert launcher.process is not None

            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while elapsed_time_seconds < self._config.SPEAR.CLIENT.INITIALIZE_MAX_TIME_SECONDS:

                status = launcher.process.status()
                if status not in spear.Launcher.expected_process_status_values:
                    spear.log("        ERROR: Unrecognized process status: ", status)
                    break

                try:
                    # Once a connection has been established, the client will wait for timeout seconds before
                    # throwing when calling a server function.
                    spear.log("        Attempting to connect to server...")
                    self.client = spear_ext.Client(self._config.SPEAR.CLIENT.RPC_SERVER_ADDRESS, self._config.SP_SERVICES.RPC_SERVICE.RPC_SERVER_PORT, self._config.SPEAR.CLIENT.SUPPRESS_DEFAULT_LOGGING)
                    self.client.set_timeout(int(self._config.SPEAR.CLIENT.INTERNAL_TIMEOUT_SECONDS)*1000)
                    connected = self.client.ping() == "ping"
                    break

                except RuntimeError:
                    # There is no need to log the exception because this case is expected until we can
                    # successfully connect to the server, which is why we set verbose=False when terminating
                    # the client.
                    self._terminate_client(verbose=False)

                except Exception as e:
                    spear.log("        Exception: ", e)
                    self.terminate(log_prefix="        ")
                    launcher.kill(log_prefix="        ")
                    assert False

                time.sleep(self._config.SPEAR.CLIENT.INITIALIZE_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        else:
            assert False

        if connected:
            spear.log("        Connected to server.")
        else:
            spear.log("        ERROR: Couldn't connect to RPC server, giving up...")
            self.terminate(log_prefix="        ")
            if self._config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
                launcher.kill(log_prefix="        ")
            assert False

        self.client.initialize() # EngineService hasn't been initialized yet

        self.client.force_return_aligned_arrays = self._config.SPEAR.CLIENT.FORCE_RETURN_ALIGNED_ARRAYS
        self.client.suppress_default_logging = self._config.SPEAR.CLIENT.SUPPRESS_DEFAULT_LOGGING
        self.client.verbose_rpc_calls = self._config.SPEAR.CLIENT.VERBOSE_RPC_CALLS
        self.client.verbose_allocations = self._config.SPEAR.CLIENT.VERBOSE_ALLOCATIONS
        self.client.verbose_exceptions = self._config.SPEAR.CLIENT.VERBOSE_EXCEPTIONS

        spear.log("        Finished initializing client.")

    def terminate(self, log_prefix=""):
        self._terminate_client(verbose=True, log_prefix=log_prefix)

    def _terminate_client(self, verbose, log_prefix=""):
        if verbose:
            spear.log_current_function(prefix=log_prefix)
            spear.log(f"{log_prefix}    Terminating client: {self.client}")
        if self.client is not None:
            self.client.terminate()
            self.client = None
        if verbose:
            spear.log(f"{log_prefix}    Finished terminating client.")

    def __repr__(self):
        return f"Client(client={self.client})"
