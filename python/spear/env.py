#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

from enum import Enum
import gym.spaces
import msgpackrpc 
import numpy as np
import os
import psutil
from subprocess import Popen
import spear
import sys
import time


# enum values must match cpp/unreal_plugins/SimulationController/Box.h
class DataType(Enum):
    Boolean    = 0
    UInteger8  = 1
    Integer8   = 2
    UInteger16 = 3
    Integer16  = 4
    UInteger32 = 5
    Integer32  = 6
    Float32    = 7
    Double     = 8

DATA_TYPE_TO_NUMPY_DTYPE = {
    DataType.Boolean.value:    np.dtype("?").type,
    DataType.UInteger8.value:  np.dtype("u1").type,
    DataType.Integer8.value:   np.dtype("i1").type,
    DataType.UInteger16.value: np.dtype("u2").type,
    DataType.Integer16.value:  np.dtype("i2").type,
    DataType.UInteger32.value: np.dtype("u4").type,
    DataType.Integer32.value:  np.dtype("i4").type,
    DataType.Float32.value:    np.dtype("f4").type,
    DataType.Double.value:     np.dtype("f8").type
}


# mimics the behavior of gym.spaces.Box but allows shape to have the entry -1
class Box():
    def __init__(self, low, high, shape, dtype):
        self.low = low
        self.high = high
        self.shape = shape
        self.dtype = dtype

# mimics the behavior of gym.spaces.Dict
class Dict():
    def __init__(self, spaces):
        self.spaces = spaces


class Env(gym.Env):

    def __init__(self, config):

        super(Env, self).__init__()

        self._config = config

        self._request_launch_unreal_instance()
        self._connect_to_unreal_instance()
        self._initialize_unreal_instance()

        self.action_space = self._get_action_space()
        self.observation_space = self._get_observation_space()

        self._byte_order = self._get_byte_order()
        self._task_step_info_space = self._get_task_step_info_space()
        self._agent_step_info_space = self._get_agent_step_info_space()
        
        self._ready = False

    def step(self, action):
        
        self._begin_tick()
        self._apply_action(action)
        self._tick()
        obs = self._get_observation()
        reward = self._get_reward()
        is_done = not self._ready or self._is_episode_done() # if the last call to reset() failed or the episode is done
        step_info = self._get_step_info()
        self._end_tick()

        return obs, reward, is_done, step_info

    def reset(self, reset_info=None):
        
        for i in range(self._config.SPEAR.MAX_NUM_TICKS_AFTER_RESET):
            self._begin_tick()
            if i == 0:
                self._reset() # only reset the simulation once
            self._tick()
            ready = self._is_ready()
            if ready or i == self._config.SPEAR.MAX_NUM_TICKS_AFTER_RESET - 1:
                obs = self._get_observation() # only get the observation if ready, or if we're about to give up
            self._end_tick()
            if ready:
                break
        
        self._ready = ready # store if our reset() attempt was successful or not, so step() can return done=True if we were unsuccessful

        if reset_info is not None:
            assert isinstance(reset_info, dict)
            reset_info["success"] = ready

        return obs

    # need to override gym.Env member function
    def render(self):
        pass

    def close(self):

        print("[SPEAR | env.py] Closing Unreal instance...")
        print()

        self._close_unreal_instance()
        self._close_client_server_connection()

        try:
            status = self._process.status()
        except psutil.NoSuchProcess:
            pass
        else:
            while status in ["running", "sleeping", "disk-sleep"]:
                time.sleep(1.0)
                try:
                    status = self._process.status()
                except psutil.NoSuchProcess:
                    break

        print("[SPEAR | env.py] Finished closing Unreal instance.")
        print()

    def _request_launch_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            return

        # write temp file
        temp_config_file = os.path.join(os.path.abspath(self._config.SPEAR.TEMP_DIR), "config.yaml")

        print("[SPEAR | env.py] Writing temp config file: " + temp_config_file)
        print()

        if not os.path.exists(os.path.abspath(self._config.SPEAR.TEMP_DIR)):
            os.makedirs(os.path.abspath(self._config.SPEAR.TEMP_DIR))
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        # create a symlink to SPEAR.DATA_DIR
        if self._config.SPEAR.DATA_DIR != "":

            paks_dir = os.path.join(self._config.SPEAR.CONTENT_DIR, "Paks")
            assert os.path.exists(paks_dir)
            spear_data_dir = os.path.join(paks_dir, "SpearData")

            if spear.path_exists(spear_data_dir):
                print(f"[SPEAR | env.py] File or directory or symlink exists, removing: {spear_data_dir}")
                spear.remove_path(spear_data_dir)

            print(f"[SPEAR | env.py] Creating symlink: {spear_data_dir} -> {self._config.SPEAR.DATA_DIR}")
            print()
            os.symlink(self._config.SPEAR.DATA_DIR, spear_data_dir)

        # provide additional control over which Vulkan devices are recognized by Unreal
        if len(self._config.SPEAR.VULKAN_DEVICE_FILES) > 0:
            print("[SPEAR | env.py] Setting VK_ICD_FILENAMES environment variable: " + self._config.SPEAR.VULKAN_DEVICE_FILES)
            os.environ["VK_ICD_FILENAMES"] = self._config.SPEAR.VULKAN_DEVICE_FILES

        # set up launch executable and command-line arguments
        launch_args = []

        if self._config.SPEAR.LAUNCH_MODE == "uproject":
            launch_executable = self._config.SPEAR.UNREAL_EDITOR_EXECUTABLE
            launch_args.append(self._config.SPEAR.UPROJECT)
        elif self._config.SPEAR.LAUNCH_MODE == "standalone_executable":
            launch_executable = self._config.SPEAR.STANDALONE_EXECUTABLE
        else:
            assert False

        assert os.path.exists(launch_executable)

        launch_executable_name, launch_executable_ext = os.path.splitext(launch_executable)

        if sys.platform == "win32":
            assert launch_executable_ext == ".exe"
            assert launch_executable_name[-4:] == "-Cmd"
            launch_executable_internal = launch_executable
        elif sys.platform == "darwin":
            assert launch_executable_ext == ".app"
            launch_executable_internal_dir = os.path.join(launch_executable, "Contents", "MacOS")
            launch_executable_internal = os.path.join(launch_executable_internal_dir, os.listdir(launch_executable_internal_dir)[0])
        elif sys.platform == "linux":
            assert launch_executable_ext == "" or launch_executable_ext == ".sh"
            launch_executable_internal = launch_executable
        else:
            assert False

        assert os.path.exists(launch_executable_internal)

        launch_args.append("-game")
        launch_args.append("-windowed")
        launch_args.append("-novsync")
        launch_args.append("-NoSound")
        launch_args.append("-NoTextureStreaming")
        launch_args.append("-resx={}".format(self._config.SPEAR.WINDOW_RESOLUTION_X))
        launch_args.append("-resy={}".format(self._config.SPEAR.WINDOW_RESOLUTION_Y))
        launch_args.append("-graphicsadapter={}".format(self._config.SPEAR.GPU_ID))

        launch_args.append("-config_file={}".format(temp_config_file))

        if self._config.SPEAR.RENDER_OFFSCREEN:
            launch_args.append("-RenderOffscreen")

        if len(self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE) > 0:
            launch_args.append("-log={}".format(self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE))
       
        # on Windows, we need to pass in extra command-line parameters to enable DirectX 12
        # and so that calls to UE_Log and writes to std::cout are visible on the command-line
        if sys.platform == "win32":
            launch_args.append("-dx12")            
            launch_args.append("-stdout")
            launch_args.append("-FullStdOutLogOutput")

        for a in self._config.SPEAR.CUSTOM_COMMAND_LINE_ARGUMENTS:
            launch_args.append("{}".format(a))

        cmd = [launch_executable_internal] + launch_args

        print("[SPEAR | env.py] Launching executable with the following command-line arguments:")
        print(" ".join(cmd))
        print()

        print("[SPEAR | env.py] Launching executable with the following config values:")
        print(self._config)
        print()
        
        popen = Popen(cmd)
        self._process = psutil.Process(popen.pid)

        # see https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
        status = self._process.status()
        if status not in ["running", "sleeping", "disk-sleep"]:
            print("[SPEAR | env.py] ERROR: Unrecognized process status: " + status)
            print("[SPEAR | env.py] ERROR: Killing process " + str(self._process.pid) + "...")
            self._force_kill_unreal_instance()
            self._close_client_server_connection()
            assert False

    def _connect_to_unreal_instance(self):

        print(f"[SPEAR | env.py] Connecting to Unreal application...")
        print()
        
        # if we're connecting to a running instance, then we assume that the RPC server is already running and only try to connect once
        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            connected = False
            try:
                self._client = msgpackrpc.Client(
                    msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT), 
                    timeout=self._config.SPEAR.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                    reconnect_limit=self._config.SPEAR.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                self._ping()
                connected = True
            except:
                # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                self._close_client_server_connection()

        # otherwise try to connect repeatedly, since the RPC server might not have started yet
        else:
            connected = False
            start_time_seconds = time.time()
            elapsed_time_seconds = time.time() - start_time_seconds
            while not connected and elapsed_time_seconds < self._config.SPEAR.RPC_CLIENT_INITIALIZE_CONNECTION_MAX_TIME_SECONDS:
                # See https://github.com/giampaolo/psutil/blob/master/psutil/_common.py for possible status values
                status = self._process.status()
                if status not in ["running", "sleeping", "disk-sleep"]:
                    print("[SPEAR | env.py] ERROR: Unrecognized process status: " + status)
                    print("[SPEAR | env.py] ERROR: Killing process " + str(self._process.pid) + "...")
                    self._force_kill_unreal_instance()
                    self._close_client_server_connection()
                    assert False
                try:
                    self._client = msgpackrpc.Client(
                        msgpackrpc.Address(self._config.SIMULATION_CONTROLLER.IP, self._config.SIMULATION_CONTROLLER.PORT), 
                        timeout=self._config.SPEAR.RPC_CLIENT_INTERNAL_TIMEOUT_SECONDS, 
                        reconnect_limit=self._config.SPEAR.RPC_CLIENT_INTERNAL_RECONNECT_LIMIT)
                    self._ping()
                    connected = True
                except:
                    # Client may not clean up resources correctly in this case, so we clean things up explicitly.
                    # See https://github.com/msgpack-rpc/msgpack-rpc-python/issues/14
                    self._close_client_server_connection()
                time.sleep(self._config.SPEAR.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        if not connected:
            if self._config.SPEAR.LAUNCH_MODE != "running_instance":
                print("[SPEAR | env.py] ERROR: Couldn't connect, killing process " + str(self._process.pid) + "...")
                self._force_kill_unreal_instance()
                self._close_client_server_connection()
            assert False

        if self._config.SPEAR.LAUNCH_MODE != "running_instance":
            time.sleep(self._config.SPEAR.RPC_CLIENT_AFTER_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)

    def _initialize_unreal_instance(self):
        # Do one complete tick to guarantee that we can receive valid observations. If we don't do
        # this, it is possible that Unreal will return an initial visual observation of all zeros.
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
        client_endianess = sys.byteorder

        if unreal_instance_endianness == client_endianess:
            return None
        elif unreal_instance_endianness == "little":
            return "<"
        elif unreal_instance_endianness == "big":
            return ">"
        else:
            assert False

    def _get_dict_space(self, space, box_space_type, dict_space_type):

        dict_space_components = {}
        for name, component in space.items():
            low = component["low_"]
            high = component["high_"]
            shape = tuple(component["shape_"])
            dtype = DATA_TYPE_TO_NUMPY_DTYPE[component["dtype_"]]
            dict_space_components[name] = box_space_type(low, high, shape, dtype)

        return dict_space_type(dict_space_components)

    def _deserialize(self, data, space):

        assert data.keys() == space.spaces.keys()

        return_dict = {}
        for name, component in data.items():
            
            shape = space.spaces[name].shape
            dtype = space.spaces[name].dtype

            if self._byte_order is not None:
                dtype = dtype.newbyteorder(self._byte_order)

            return_dict[name] = np.frombuffer(component, dtype=dtype, count=-1).reshape(shape)
            
            assert (return_dict[name] >= space.spaces[name].low).all()
            assert (return_dict[name] <= space.spaces[name].high).all()

        return return_dict

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
        return self._get_dict_space(space, gym.spaces.Box, gym.spaces.Dict)

    def _get_observation_space(self):
        space = self._client.call("getObservationSpace")
        assert len(space) > 0
        return self._get_dict_space(space, gym.spaces.Box, gym.spaces.Dict)

    def _get_task_step_info_space(self):
        space = self._client.call("getTaskStepInfoSpace")
        return self._get_dict_space(space, Box, Dict)

    def _get_agent_step_info_space(self):
        space = self._client.call("getAgentStepInfoSpace")
        return self._get_dict_space(space, Box, Dict)

    def _apply_action(self, action):

        assert action.keys() == self.action_space.spaces.keys()
        
        action_dict = {}
        for name, component in action.items():
            assert isinstance(component, np.ndarray)
            assert component.shape == self.action_space.spaces[name].shape
            assert component.dtype == self.action_space.spaces[name].dtype
            assert (component >= self.action_space.spaces[name].low).all()
            assert (component <= self.action_space.spaces[name].high).all()
            action_dict[name] = component.tolist()
        
        self._client.call("applyAction", action_dict)

    def _get_observation(self):
        observation = self._client.call("getObservation")
        return self._deserialize(observation, self.observation_space)

    def _get_reward(self):
        return self._client.call("getReward")
    
    def _is_episode_done(self):
        return self._client.call("isEpisodeDone")

    def _get_step_info(self):
        task_step_info = self._client.call("getTaskStepInfo")
        agent_step_info = self._client.call("getAgentStepInfo")
        return { "task_step_info": self._deserialize(task_step_info, self._task_step_info_space),
                 "agent_step_info": self._deserialize(agent_step_info, self._agent_step_info_space) }

    def _reset(self):
        # reset the task first in case it needs to set the pose of actors,
        # then reset agent so it can refine the pose of actors
        self._client.call("resetTask")
        self._client.call("resetAgent")

    def _is_ready(self):
        return self._client.call("isTaskReady") and self._client.call("isAgentReady")
