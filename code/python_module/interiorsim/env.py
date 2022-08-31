from enum import Enum
import gym
from gym import spaces
import numpy as np
import os
import psutil
from subprocess import Popen
import sys
import time

import msgpackrpc   # pip install -e code/third_party/msgpack-rpc-python


# Enum values should match Box.h in SimulationController plugin
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

DATA_TYPE_TO_NUMPY_DTYPE = {
    DataType.Boolean.value: np.dtype("?").type,
    DataType.UInteger8.value: np.dtype("u1").type,
    DataType.Integer8.value: np.dtype("i1").type,
    DataType.UInteger16.value: np.dtype("u2").type,
    DataType.Integer16.value: np.dtype("i2").type,
    DataType.UInteger32.value: np.dtype("u4").type,
    DataType.Integer32.value: np.dtype("i4").type,
    DataType.Float32.value: np.dtype("f4").type,
    DataType.Double.value: np.dtype("f8").type
}


# Enum values should match SimulationController.cpp in SimulationController plugin
class EndiannessType(Enum):
    LittleEndian = 0
    BigEndian = 1


class Env(gym.Env):

    def __init__(self, config):

        super(Env, self).__init__()

        self._config = config

        self._request_launch_unreal_instance()
        self._connect_to_unreal_instance()
        self._initialize_unreal_instance()

        self._byte_order = self._get_byte_order()

        self.action_space = self._get_action_space()
        self.observation_space = self._get_observation_space()
        self.step_info_space = self._get_step_info_space()

    def step(self, action):
        
        self._begin_tick()
        self._apply_action(action)
        self._tick()
        obs = self._get_observation()
        reward = self._get_reward()
        is_done = self._is_episode_done()
        step_info = self._get_step_info()
        self._end_tick()

        return obs, reward, is_done, step_info

    def reset(self):

        ready = False
        once = False

        while not ready:
            self._begin_tick()

            # only reset the simulation once
            if not once:
                self._reset()
                once = True

            self._tick()

            # only get the observation if ready
            ready = self._is_ready()
            if ready:
                obs = self._get_observation()

            self._end_tick()

        return obs

    # need to override gym.Env member function
    def render(self):
        pass

    def close(self):

        print("Closing Unreal instance...")
        print()

        # request to close Unreal instance
        self._close_unreal_instance()
        self._close_client_server_connection()

        # do not return until Unreal application is closed
        status = self._process.status()
        while status in ["running", "sleeping", "disk-sleep"]:
            time.sleep(1.0)
            try:
                status = self._process.status()
            except psutil.NoSuchProcess:
                break

        print("Finished closing Unreal instance.")
        print()

    def _request_launch_unreal_instance(self):

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
            self._config.defrost()
            self._config.SIMULATION_CONTROLLER.ENABLE_VISUALIZER = False
            self._config.freeze()

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
        print()

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

        print("Launching executable with the following arguments:")
        print(" ".join(args))
        print()

        print("Launching executable with the following config values:")
        print(self._config)
        print()
        
        popen = Popen(args)
        self._process = psutil.Process(popen.pid)

        # See https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in ["running", "sleeping", "disk-sleep"]:
            print("ERROR: Unrecognized process status: " + status)
            print("ERROR: Killing process " + str(self._process.pid) + "...")
            self._force_kill_unreal_instance()
            self._close_client_server_connection()
            assert False

    def _connect_to_unreal_instance(self):

        print(f"Connecting to Unreal application...")
        print()
        
        # If we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if self._config.INTERIORSIM.LAUNCH_MODE == "running_instance":
            connected = False
            try:
                self._client = msgpackrpc.Client(
                    msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT), 
                    timeout=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                    reconnect_limit=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self._ping()
                connected = True
            except:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                self._close_client_server_connection()

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
                    self._force_kill_unreal_instance()
                    self._close_client_server_connection()
                    assert False
                try:
                    self._client = msgpackrpc.Client(
                        msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT), 
                        timeout=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.INTERIORSIM.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self._ping()
                    connected = True
                except:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                    self._close_client_server_connection()
                time.sleep(self._config.INTERIORSIM.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        if not connected:
            if self._config.INTERIORSIM.LAUNCH_MODE != "running_instance":
                print("ERROR: Couldn't connect, killing process " + str(self._process.pid) + "...")
                self._force_kill_unreal_instance()
                self._close_client_server_connection()
            assert False

    def _initialize_unreal_instance(self):
        # do one tick cyle here to prep Unreal Engine so that we can receive valid observations
        self._begin_tick()
        self._tick()
        self._end_tick()

    def _force_kill_unreal_instance(self):
        self._process.terminate()
        self._process.kill()

    def _close_client_server_connection(self):
        self._client.close()
        self._client._loop._ioloop.close()
    
    def _get_byte_order(self):

        unreal_instance_endianness = self._client.call("getEndianness")

        if sys.byteorder == "little":
            client_endianess = EndiannessType.LittleEndian.value
        elif sys.byteorder == "big":
            client_endianess = EndiannessType.BigEndian.value

        if unreal_instance_endianness == client_endianess:
            return None
        elif unreal_instance_endianness == EndiannessType.BigEndian.value:
            return ">"
        elif unreal_instance_endianness == EndiannessType.LittleEndian.value:
            return "<"
        else:
            assert False

    def _get_gym_space(self, space):

        gym_spaces = {}
        for name, component in space.items():
            low = component["low"]
            high = component["high"]
            shape = tuple(component["shape"])
            dtype = DATA_TYPE_TO_NUMPY_DTYPE[component["dtype"]]
            gym_spaces[name] = spaces.Box(low, high, shape, dtype)

        return spaces.Dict(gym_spaces)

    def _deserialize(self, data, space):

        assert data.keys() == space.keys()

        return_data = {}
        for name, component in data.items():
            
            assert len(component) > 0
            # get shape and dtype of the data component
            shape = space[name].shape
            dtype = space[name].dtype

            # change byte order based on Unreal instance and client endianness
            if self._byte_order is not None:
                dtype = dtype.newbyteorder(self._byte_order)

            return_data[name] = np.frombuffer(component, dtype=dtype, count=-1).reshape(shape)

        return return_data

    def _ping(self):
        return self._client.call("ping")

    def _close_unreal_instance(self):
        self._client.call("close")

    def _begin_tick(self):
        self._client.call("beginTick")

    def _tick(self):
        self._client.call("tick")

    def _end_tick(self):
        self._client.call("endTick")

    def _get_action_space(self):
        space = self._client.call("getActionSpace")
        assert len(space) > 0
        return self._get_gym_space(space)

    def _get_observation_space(self):
        space = self._client.call("getObservationSpace")
        assert len(space) > 0
        return self._get_gym_space(space)

    def _get_step_info_space(self):
        space = self._client.call("getStepInfoSpace")
        return self._get_gym_space(space)

    def _apply_action(self, action):
        self._client.call("applyAction", action)

    def _get_observation(self):
        observation = self._client.call("getObservation")
        return self._deserialize(observation, self.observation_space)

    def _get_reward(self):
        return self._client.call("getReward")
    
    def _is_episode_done(self):
        return self._client.call("isEpisodeDone")

    def _get_step_info(self):
        step_info = self._client.call("getStepInfo")
        return self._deserialize(step_info, self.step_info_space)

    def _reset(self):
        self._client.call("reset")

    def _is_ready(self):
        return self._client.call("isReady")
