from abc import ABC, abstractmethod
import numpy as np
import os
import psutil
from subprocess import Popen
import sys
import time
from typing import Any, Dict, List, Optional
from yacs.config import CfgNode
from interiorsim.adaptors import RpcEndiannessType, RpcDataType, RpcAction, RpcObservation, Box

from interiorsim.communicator import Communicator
from interiorsim.constants import PACKAGE_DEFAULT_CONFIG_FILE

class BaseEnv(ABC):
    @abstractmethod
    def step(self):
        """
        Signals the environment to move the simulation forward
        by one step.
        """

    @abstractmethod
    def reset(self):
        """
        Signals the environment to reset the simulation.
        """

    @abstractmethod
    def close(self):
        """
        Signals the environment to close.
        """


class InteriorSimEnv:

    def __init__(self, config: CfgNode):

        super(InteriorSimEnv, self).__init__()

        launch_params: List[str] = []

        # Get launch executable from config
        if config.INTERIORSIM.LAUNCH_MODE == "uproject":
            launch_executable = config.INTERIORSIM.UNREAL_EDITOR_EXECUTABLE
            launch_params.insert(0, config.INTERIORSIM.UPROJECT) # prepend uproject file to launch params
        elif config.INTERIORSIM.LAUNCH_MODE == "standalone_executable":
            launch_executable = config.INTERIORSIM.STANDALONE_EXECUTABLE

        if config.INTERIORSIM.LAUNCH_MODE != "running_instance":
        
            # Dump updated config params into a new yaml file
            temp_config_file = os.path.join(os.path.abspath(config.INTERIORSIM.TEMP_DIR), "config.yaml")
            if not os.path.exists(os.path.abspath(config.INTERIORSIM.TEMP_DIR)):
                os.makedirs(os.path.abspath(config.INTERIORSIM.TEMP_DIR))
            try:
                with open(temp_config_file, "w") as output:
                    config.dump(stream=output, default_flow_style=False)
            except EnvironmentError as e:
                raise Exception(e)

            print("Starting Unreal engine...")

            # Read params required for launching the Unreal Engine executable:
            launch_params.append("".format(config.INTERIORSIM.MAP_ID)) # Allows convenient selection of the map to be loaded by your executable. 
            launch_params.append("-game")
            launch_params.append("-windowed")
            launch_params.append("-novsync")
            launch_params.append("-NoSound")
            launch_params.append("-resx={}".format(config.INTERIORSIM.WINDOW_RESOLUTION_X))
            launch_params.append("-resy={}".format(config.INTERIORSIM.WINDOW_RESOLUTION_Y))
            launch_params.append("-graphicsadapter={}".format(config.INTERIORSIM.GPU_ID))
            
            if config.INTERIORSIM.RENDER_OFFSCREEN:
                launch_params.append("-RenderOffscreen")

            # Unreal Engine server needs to read values from this config file
            launch_params.append("-configfile={}".format(temp_config_file))
            
            # Allows avoiding the "settings.json not found" error at simulation start
            launch_params.append("-RobotSimSettingPath={}".format(config.INTERIORSIM.ROBOTSIM_SETTINGS_PATH)) 

            # Additional control over which Vulkan devices are recognized by Unreal
            if len(config.INTERIORSIM.VULKAN_DEVICE_FILES) > 0:
                os.environ["VK_ICD_FILENAMES"] = config.INTERIORSIM.VULKAN_DEVICE_FILES

            print("Launching executable with the following parameters...")
            print(" ".join([launch_executable] + launch_params))

            popen = self._start_unreal_engine(launch_executable, *launch_params)
            process = psutil.Process(popen.pid)

            print(process.status())
            # assert process.status() == "running"

        print("Connecting to Unreal engine...")

        # If we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if config.INTERIORSIM.LAUNCH_MODE == "running_instance":
            connected = False
            try:
                self._communicator = Communicator(
                    ip=config.INTERIORSIM.IP,
                    port=config.INTERIORSIM.PORT,
                    timeout=config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                    reconnect_limit=config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self._communicator.ping()
                connected = True
            except:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                self._communicator._client.close()
                self._communicator._client._loop._ioloop.close()

        # Otherwise try to connect repeatedly, since the RPC server might not have started yet
        else:
            connected = False
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not connected and elapsed_time_seconds < config.INTERIORSIM.RPC_CLIENT_INITIALIZE_CONNECTION_MAX_TIME_SECONDS:
                print(process.status())
                # assert process.status() == "running"
                try:
                    self._communicator = Communicator(
                        ip=config.INTERIORSIM.IP,
                        port=config.INTERIORSIM.PORT,
                        timeout_value=config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS,
                        reconn_limit=config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self._communicator.ping()
                    connected = True
                except:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                    self._communicator._client.close()
                    self._communicator._client._loop._ioloop.close()
                time.sleep(config.INTERIORSIM.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        assert connected

        print(f"Waiting for Unreal Environment to get ready...")

    def _get_obs(self, obs: RpcObservation) -> Dict[str, np.ndarray]:
        """Convert observations retrieved from Unreal Environment to a numpy array."""

        ret_dict: Dict[str, np.ndarray] = {}

        for obs_name, obs_vec in obs["ObservationsMap"].items():
            if len(obs_vec) == 0:
                raise Exception(f"Did not receive any observations for {obs_name}.")

            # go over each obs in the vector of observations returned by agent
            # for ind, obs in enumerate(obs_vec):
            #     oshape = tuple(x for x in self._obs_space[ind]["Shape"])

            #     data_type = utils.get_numpy_dtype(self._obs_specs[agent_name][ind]["Dtype"])

            #     if self._communicator.get_endianness() == utils.get_endianness():
            #         pass
            #     elif (self._communicator.get_endianness() == Endianness.BigEndian.value):
            #         data_type = data_type.newbyteorder(">")
            #     elif (self._communicator.get_endianness() == Endianness.LittleEndian.value):
            #         data_type = data_type.newbyteorder("<")

            #     arr = np.frombuffer(obs["Data"], dtype=data_type, count=-1).reshape(oshape)

            #     if agent_name in ret_dict:
            #         ret_dict[agent_name].append(arr)
            #     else:
            #         ret_dict[agent_name] = [arr]
        return ret_dict

    def close(self) -> None:
        self._communicator.close()
        self._communicator._client._loop._ioloop.close()
        self._communicator.close_connection()

    # This function returns a config object, obtained by loading and merging a list of config
    # files in the order they appear in the config_files input argument. This function is useful
    # for loading default values from multiple different components and systems, and then
    # overriding some of the default values with experiment-specific or user-specific overrides.
    # Before loading any of the config files specified in config_files, all the default values
    # required by the INTERIORSIM Python module are loaded into a top-level INTERIORSIM namespace, and
    # can be overridden by any of the files appearing in config_files.
    @staticmethod
    def get_config(config_files: List[str] = []) -> CfgNode:

        # create a single CfgNode that will eventually contain data from all config files
        config = CfgNode(new_allowed=True)

        config.merge_from_file(PACKAGE_DEFAULT_CONFIG_FILE)
        for c in config_files:
            config.merge_from_file(c)
        config.freeze()

        return config

    def _start_unreal_engine(self, binary_path: str = "", *args) -> Optional[Popen]:

        if not binary_path:
            raise Exception(f"Empty path entered. Please enter a valid path.")

        if not os.path.exists(binary_path):
            raise Exception(f"Entered path {binary_path} does not exist. Please enter a valid path.")

        _, ext = os.path.splitext(binary_path)

        if sys.platform == "darwin":
            if ext != ".app":
                raise Exception(f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file.")
            exe_path = binary_path + "/Contents/MacOS/"
            exe_file = os.listdir(exe_path)[0]
            try:
                p = Popen([exe_path + exe_file, *args])
            except OSError:
                raise OSError(f"Could not launch the Unreal Engine executable. Please check the executable's path.")
            except ValueError:
                raise ValueError(f"The parameters entered to run with the executable are invalid. Please check the parameters.")

        elif sys.platform == "win32":
            if ext != ".exe":
                raise Exception(f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file.")
            try:
                p = Popen([binary_path, *args])
            except OSError:
                raise OSError(f"Could not launch the Unreal Engine executable. Please check the executable's path.")
            except ValueError:
                raise ValueError(f"The parameters entered to run with the executable are invalid. Please check the parameters.")

        elif sys.platform == "linux":
            if ext != "" and ext != ".sh":
                raise Exception(f"The path entered contains an extension {ext} which is not an executable on {sys.platform}. Please enter a path to a valid Unreal Engine executable file.")
            try:
                p = Popen([binary_path, *args])
            except OSError:
                raise OSError(f"Could not launch the Unreal Engine executable. Please check the executable's path.")
            except ValueError:
                raise ValueError(f"The parameters entered to run with the executable are invalid. Please check the parameters.")
        else:
            raise Exception(f"Unknown platform detected. Make sure you are on either MacOS, Windows(64bit), or Ubuntu platform.")

        return p

    @staticmethod
    def get_numpy_dtype(x):
        return {
            RpcDataType.Boolean.value: np.dtype("?"),
            RpcDataType.UInteger8.value: np.dtype("u1"),
            RpcDataType.Integer8.value: np.dtype("i1"),
            RpcDataType.UInteger16.value: np.dtype("u2"),
            RpcDataType.Integer16.value: np.dtype("i2"),
            RpcDataType.UInteger32.value: np.dtype("u4"),
            RpcDataType.Integer32.value: np.dtype("i4"),
            RpcDataType.Float32.value: np.dtype("f4"),
            RpcDataType.Double.value: np.dtype("f8"),
        }[x]

    @staticmethod
    def getEndianness():
        if sys.byteorder == "little":
            return RpcEndiannessType.LittleEndian
        elif sys.byteorder == "big":
            return RpcEndiannessType.BigEndian

    @property
    def conn(self):
        return self._communicator
