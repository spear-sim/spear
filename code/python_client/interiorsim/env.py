from enum import Enum
import gym
from gym import spaces
import numpy as np
import os
from subprocess import Popen
import sys
import time

import msgpackrpc   # pip install -e code/third_party/msgpack-rpc-python
import psutil

class DataType(Enum):
    Boolean = 0
    UInteger8 = 1
    Integer8 = 2
    UInteger16 = 3
    Integer16 = 4
    UInteger32 = 5
    Integer32 = 6
    Float32 = 7
    Double = 8


class EndiannessType(Enum):
    LittleEndian = 0
    BigEndian = 1


class InteriorSimEnv(gym.Env):

    def __init__(self, config):

        super(InteriorSimEnv, self).__init__()

        self._config = config

        self._launchExecutable()
        self._connectToExecutable()
        self._setupEnvironment()

        self.observation_space = self._getObservationSpace()
        self.action_space = self._getActionSpace()
    
    # TODO: add reward functionality
    def step(self, action):
        
        self._beginTick()

        self._applyAction(action)

        self._tick()

        obs = self._getObservation()

        self._endTick()

        return obs

    def reset(self):
        self._beginTick()
        # TODO: implement to bring agent to reset state
        self._tick()
        obs = self._getObservation()
        self._endTick()
        return obs

    # need to override gym.Env member function
    def render(self):
        pass

    def close(self):
        self._closeUnrealInstance()
        self._closeConnection()

    def _launchExecutable(self):

        if self._config.INTERIORSIM.LAUNCH_MODE == "running_instance":
            return

        launch_params = []

        # Get launch executable from config
        if self._config.INTERIORSIM.LAUNCH_MODE == "uproject":
            launch_executable = self._config.INTERIORSIM.UNREAL_EDITOR_EXECUTABLE
            launch_params.append(self._config.INTERIORSIM.UPROJECT) # prepend uproject file to launch params
        elif self._config.INTERIORSIM.LAUNCH_MODE == "standalone_executable":
            launch_executable = self._config.INTERIORSIM.STANDALONE_EXECUTABLE
        else:
            assert False

        # Read params required for launching the Unreal Engine executable:
        launch_params.append("{}".format(self._config.INTERIORSIM.MAP_ID)) # Allows convenient selection of the map to be loaded by your executable. 
        launch_params.append("-game")
        launch_params.append("-windowed")
        launch_params.append("-novsync")
        launch_params.append("-NoSound")
        launch_params.append("-resx={}".format(self._config.INTERIORSIM.WINDOW_RESOLUTION_X))
        launch_params.append("-resy={}".format(self._config.INTERIORSIM.WINDOW_RESOLUTION_Y))
        launch_params.append("-graphicsadapter={}".format(self._config.INTERIORSIM.GPU_ID))

        if self._config.INTERIORSIM.RENDER_OFFSCREEN:
            launch_params.append("-RenderOffscreen")

        if len(self._config.INTERIORSIM.UNREAL_INTERNAL_LOG_FILE) > 0:
            launch_params.append("-log={}".format(self._config.INTERIORSIM.UNREAL_INTERNAL_LOG_FILE))
       
        # Dump updated config params into a new yaml file
        temp_config_file = os.path.join(os.path.abspath(self._config.INTERIORSIM.TEMP_DIR), "config.yaml")

        # Unreal Engine server needs to read values from this config file
        launch_params.append("-configfile={}".format(temp_config_file))

        # Append any additional command-line arguments
        for a in self._config.INTERIORSIM.CUSTOM_COMMAND_LINE_ARGUMENTS:
            launch_params.append("{}".format(a))

        # Provides additional control over which Vulkan devices are recognized by Unreal
        if len(self._config.INTERIORSIM.VULKAN_DEVICE_FILES) > 0:
            print("Setting VK_ICD_FILENAMES environment variable: " + self._config.INTERIORSIM.VULKAN_DEVICE_FILES)
            os.environ["VK_ICD_FILENAMES"] = self._config.INTERIORSIM.VULKAN_DEVICE_FILES

        print("Writing temp config file: " + temp_config_file)
        if not os.path.exists(os.path.abspath(self._config.INTERIORSIM.TEMP_DIR)):
            os.makedirs(os.path.abspath(self._config.INTERIORSIM.TEMP_DIR))
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        assert os.path.exists(launch_executable)

        _, launch_executable_ext = os.path.splitext(launch_executable)

        if sys.platform == "darwin":
            assert launch_executable_ext == ".app"
            launch_executable_internal_dir = os.path.join(launch_executable, "Contents", "MacOS")
            launch_executable_internal = os.path.join(launch_executable_internal_dir, os.listdir(launch_executable_internal_dir)[0])
        elif sys.platform == "linux":
            assert launch_executable_ext == "" or launch_executable_ext == ".sh"
            launch_executable_internal = launch_executable
        elif sys.platform == "win32":
            assert launch_executable_ext == ".exe"
            launch_executable_internal = launch_executable
        else:
            assert False

        assert os.path.exists(launch_executable_internal)

        args = [launch_executable_internal] + launch_params

        print("Launching executable with the following parameters:")
        print(" ".join(args))

        popen = Popen(args)
        self._process = psutil.Process(popen.pid)

        # See https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in ["running", "sleeping", "disk-sleep"]:
            print("ERROR: Unrecognized process status: " + status)
            print("ERROR: Killing process " + str(self._process.pid) + "...")
            self._forceKillUnrealInstance()
            assert False


    def _connectToExecutable(self):

        print(f"Connecting to Unreal application...")

        # If we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if self._config.INTERIORSIM.LAUNCH_MODE == "running_instance":
            connected = False
            try:
                self._client = msgpackrpc.Client(
                    msgpackrpc.Address(self._config.INTERIORSIM.IP, self._config.INTERIORSIM.PORT), 
                    timeout=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                    reconnect_limit=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self._ping()
                connected = True
            except:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                self._closeConnection()

        # Otherwise try to connect repeatedly, since the RPC server might not have started yet
        else:
            connected = False
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not connected and elapsed_time_seconds < self._config.INTERIORSIM.RPC_CLIENT_INITIALIZE_CONNECTION_MAX_TIME_SECONDS:
                # See https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
                status = self._process.status()
                if status not in ["running", "sleeping", "disk-sleep"]:
                    print("ERROR: Unrecognized process status: " + status)
                    print("ERROR: Killing process " + str(self._process.pid) + "...")
                    self._forceKillUnrealInstance()
                    assert False
                try:
                    self._client = msgpackrpc.Client(
                        msgpackrpc.Address(self._config.INTERIORSIM.IP, self._config.INTERIORSIM.PORT), 
                        timeout=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self._ping()
                    connected = True
                except:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                    self._closeConnection()
                time.sleep(self._config.INTERIORSIM.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        if not connected:
            if self._self._config.INTERIORSIM.LAUNCH_MODE != "running_instance":
                print("ERROR: Couldn't connect, killing process " + str(self._process.pid) + "...")
                self._forceKillUnrealInstance()
            assert False

    def _setupEnvironment(self):
        # do one tick cyle here to prep Unreal Engine so that we can receive valid observations
        self._beginTick()
        self._tick()
        self._endTick()

    def _forceKillUnrealInstance(self):
        self._process.terminate()
        self._process.kill()
        self._closeConnection()

    def _closeConnection(self):
        self._client.close()
        self._client._loop._ioloop.close()

    def _ping(self):
        return self._client.call("ping")

    def _isPaused(self):
        return self._client.call("isPaused")

    def _getUnrealInstanceEndianness(self):
        return self._client.call("getEndianness")

    def _closeUnrealInstance(self):
        self._client.call("exit")

    def _beginTick(self):
        return self._client.call("beginTick")

    def _tick(self):
        return self._client.call("tick")

    def _endTick(self):
        return self._client.call("endTick")

    def _getObservationSpace(self):
        observation_space = self._client.call("getObservationSpace")
        if len(observation_space) == 0:
            assert False
        
        # construct a dict with gym spaces
        gym_spaces_dict = {}

        for obs_name, obs_info in observation_space.items():
            low = obs_info["low"]
            high = obs_info["high"]
            shape = tuple(x for x in obs_info["shape"])
            dtype = self._getNumpyDtype(obs_info["dtype"]).type
            gym_spaces_dict[obs_name] = spaces.Box(low, high, shape, dtype)

        return spaces.Dict(gym_spaces_dict)


    # TODO: expand functionality to support discrete action spaces
    def _getActionSpace(self):
        action_space = self._client.call("getActionSpace")
        if len(action_space) == 0:
            assert False
        
        # construct a dict with gym spaces
        gym_spaces_dict = {}

        for action_name, action_info in action_space.items():
            low = action_info["low"]
            high = action_info["high"]
            shape = tuple(x for x in action_info["shape"])
            dtype = self._getNumpyDtype(action_info["dtype"]).type
            gym_spaces_dict[action_name] = spaces.Box(low, high, shape, dtype)

        return spaces.Dict(gym_spaces_dict)

    def _getObservation(self):
        obs_dict = self._client.call("getObservation")
        if len(obs_dict) == 0:
            assert False
            
        return_obs_dict = {}

        def getEndianness():
            if sys.byteorder == "little":
                return EndiannessType.LittleEndian.value
            elif sys.byteorder == "big":
                return EndiannessType.BigEndian.value

        for obs_name, obs in obs_dict.items():
            # get shape and dtype of the observation component
            obs_shape = self.observation_space[obs_name].shape
            obs_data_type = self.observation_space[obs_name].dtype

            if self._getUnrealInstanceEndianness() == getEndianness():
                pass
            elif self._getUnrealInstanceEndianness() == EndiannessType.BigEndian.value:
                obs_data_type = obs_data_type.newbyteorder(">")
            elif self._getUnrealInstanceEndianness() == EndiannessType.LittleEndian.value:
                obs_data_type = obs_data_type.newbyteorder("<")

            return_obs_dict[obs_name] = np.frombuffer(obs, dtype=obs_data_type, count=-1).reshape(obs_shape)
        
        return return_obs_dict

    def _applyAction(self, action):
        self._client.call("applyAction", action)

    def _getNumpyDtype(self, x):
        return {
            DataType.Boolean.value: np.dtype("?"),
            DataType.UInteger8.value: np.dtype("u1"),
            DataType.Integer8.value: np.dtype("i1"),
            DataType.UInteger16.value: np.dtype("u2"),
            DataType.Integer16.value: np.dtype("i2"),
            DataType.UInteger32.value: np.dtype("u4"),
            DataType.Integer32.value: np.dtype("i4"),
            DataType.Float32.value: np.dtype("f4"),
            DataType.Double.value: np.dtype("f8"),
        }[x]
