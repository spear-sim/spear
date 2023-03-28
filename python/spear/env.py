#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

from enum import Enum
import gym.spaces
import mmap
import msgpackrpc 
import multiprocessing.shared_memory
import numpy as np
import os
import psutil
from subprocess import Popen
import spear
import sys
import time


class Env(gym.Env):
    def __init__(self, config):

        super(Env, self).__init__()

        self._config = config

        self._request_launch_unreal_instance()
        self._initialize_client()
        self._initialize_unreal_instance()

        self._byte_order = self._get_byte_order()

        self._action_space_info = SpaceInfo(self._get_action_space(), dict_space_type=gym.spaces.Dict, box_space_type=gym.spaces.Box)
        self._observation_space_info = SpaceInfo(self._get_observation_space(), dict_space_type=gym.spaces.Dict, box_space_type=gym.spaces.Box)
        self._task_step_info_space_info = SpaceInfo(self._get_task_step_info_space(), dict_space_type=Dict, box_space_type=Box)
        self._agent_step_info_space_info = SpaceInfo(self._get_agent_step_info_space(), dict_space_type=Dict, box_space_type=Box)

        self.action_space = self._action_space_info.space
        self.observation_space = self._observation_space_info.space

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
        
        self._ready = ready # store if our reset() attempt was successful or not, so step(...) can return done=True if we were unsuccessful

        if reset_info is not None:
            assert isinstance(reset_info, dict)
            reset_info["success"] = ready

        return obs

    # need to override gym.Env member function
    def render(self):
        pass

    def close(self):

        print("[SPEAR | env.py] Closing Unreal instance...")

        self._action_space_info.terminate()
        self._observation_space_info.terminate()
        self._task_step_info_space_info.terminate()
        self._agent_step_info_space_info.terminate()

        # Note that in the constructor, we launch the Unreal instance first and then initialize the client. Normally, we
        # would do things in the reverse order here. But if we close the client first, then we can't send a command to
        # the Unreal instance to close it. So we close the Unreal instance first and then close the client.
        self._request_close_unreal_instance()

        print("[SPEAR | env.py] Finished closing Unreal instance.")

    def _request_launch_unreal_instance(self):

        if self._config.SPEAR.LAUNCH_MODE == "running_instance":
            return

        # write temp file
        temp_dir = os.path.realpath(os.path.join(self._config.SPEAR.TEMP_DIR))
        temp_config_file = os.path.realpath(os.path.join(temp_dir, "config.yaml"))

        print("[SPEAR | env.py] Writing temp config file: " + temp_config_file)

        if not os.path.exists(temp_dir):
            os.makedirs(temp_dir)
        with open(temp_config_file, "w") as output:
            self._config.dump(stream=output, default_flow_style=False)

        # create a symlink to SPEAR.PAKS_DIR
        if self._config.SPEAR.LAUNCH_MODE == "standalone_executable" and self._config.SPEAR.PAKS_DIR != "":

            assert os.path.exists(self._config.SPEAR.STANDALONE_EXECUTABLE)
            assert os.path.exists(self._config.SPEAR.PAKS_DIR)

            if sys.platform == "win32":
                paks_dir = \
                    os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(self._config.SPEAR.STANDALONE_EXECUTABLE)), "..", "..", "Content", "Paks"))
            elif sys.platform == "darwin":
                paks_dir = \
                    os.path.realpath(os.path.join(self._config.SPEAR.STANDALONE_EXECUTABLE, "Contents", "UE4", "SpearSim", "Content", "Paks"))
            elif sys.platform == "linux":
                paks_dir = \
                    os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(self._config.SPEAR.STANDALONE_EXECUTABLE)), "SpearSim", "Content", "Paks"))
            else:
                assert False

            assert os.path.exists(paks_dir)

            # we don't use os.path.realpath here because we don't want to resolve the symlink
            spear_paks_dir = os.path.join(paks_dir, "SpearPaks")

            if spear.path_exists(spear_paks_dir):
                print(f"[SPEAR | env.py] File or directory or symlink exists, removing: {spear_paks_dir}")
                spear.remove_path(spear_paks_dir)

            print(f"[SPEAR | env.py] Creating symlink: {spear_paks_dir} -> {self._config.SPEAR.PAKS_DIR}")
            os.symlink(self._config.SPEAR.PAKS_DIR, spear_paks_dir)

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

        launch_args.append("-game")
        launch_args.append("-windowed")
        launch_args.append("-novsync")
        launch_args.append("-nosound")
        launch_args.append("-notexturestreaming")
        launch_args.append("-resx={}".format(self._config.SPEAR.WINDOW_RESOLUTION_X))
        launch_args.append("-resy={}".format(self._config.SPEAR.WINDOW_RESOLUTION_Y))
        launch_args.append("-graphicsadapter={}".format(self._config.SPEAR.GPU_ID))

        if self._config.SPEAR.RENDER_OFFSCREEN:
            launch_args.append("-renderoffscreen")

        if len(self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE) > 0:
            launch_args.append("-log={}".format(self._config.SPEAR.UNREAL_INTERNAL_LOG_FILE))
       
        # on Windows, we need to pass in extra command-line parameters to enable DirectX 12
        # and so that calls to UE_Log and writes to std::cout are visible on the command-line
        if sys.platform == "win32":
            launch_args.append("-dx12")            
            launch_args.append("-stdout")
            launch_args.append("-fullstdoutlogoutput")

        launch_args.append("-config_file={}".format(temp_config_file))

        for a in self._config.SPEAR.CUSTOM_COMMAND_LINE_ARGUMENTS:
            launch_args.append("{}".format(a))

        cmd = [launch_executable_internal] + launch_args

        print("[SPEAR | env.py] Launching executable with the following command-line arguments:")
        print(" ".join(cmd))

        print("[SPEAR | env.py] Launching executable with the following config values:")
        print(self._config)
        
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

    def _request_close_unreal_instance(self):
        if self._config.SPEAR.LAUNCH_MODE != "running_instance":
            self._close()
            self._wait_until_unreal_instance_is_closed()

    def _initialize_client(self):

        print(f"[SPEAR | env.py] Connecting to Unreal application...")
        
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
                if status not in ["disk-sleep", "running", "sleeping", "stopped"]:
                    print("[SPEAR | env.py] ERROR: Unrecognized process status: " + status)
                    print("[SPEAR | env.py] ERROR: Killing process " + str(self._process.pid) + "...")
                    self._force_kill_unreal_instance()
                    self._close_client()
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
                    self._close_client()
                time.sleep(self._config.SPEAR.RPC_CLIENT_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)
                elapsed_time_seconds = time.time() - start_time_seconds

        if not connected:
            if self._config.SPEAR.LAUNCH_MODE != "running_instance":
                print("[SPEAR | env.py] ERROR: Couldn't connect, killing process " + str(self._process.pid) + "...")
                self._force_kill_unreal_instance()
                self._close_client()
            assert False

        if self._config.SPEAR.LAUNCH_MODE != "running_instance":
            time.sleep(self._config.SPEAR.RPC_CLIENT_AFTER_INITIALIZE_CONNECTION_SLEEP_TIME_SECONDS)

    def _initialize_unreal_instance(self):
        # Do one complete tick to guarantee that we can receive valid observations. If we don't do
        # this, it is possible that Unreal will return an initial visual observation of all zeros.
        self._begin_tick()
        self._tick()
        self._end_tick()

    def _wait_until_unreal_instance_is_closed(self):
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

    def _force_kill_unreal_instance(self):
        self._process.terminate()
        self._process.kill()

    def _close_client(self):
        self._client.close()
        self._client._loop._ioloop.close()
    
    def _get_byte_order(self):
        unreal_instance_byte_order = self._client.call("getByteOrder")
        client_byte_order = sys.byteorder
        if unreal_instance_byte_order == client_byte_order:
            return None
        elif unreal_instance_byte_order == "little":
            return "<"
        elif unreal_instance_byte_order == "big":
            return ">"
        else:
            assert False

    def _ping(self):
        return self._client.call("ping")

    def _close(self):
        self._client.call("close")

    def _begin_tick(self):
        self._client.call("beginTick")

    def _tick(self):
        self._client.call("tick")

    def _end_tick(self):
        self._client.call("endTick")

    def _get_action_space(self):
        raw_space = self._client.call("getActionSpace")
        assert len(raw_space) > 0
        return raw_space

    def _get_observation_space(self):
        raw_space = self._client.call("getObservationSpace")
        assert len(raw_space) > 0
        return raw_space

    def _get_task_step_info_space(self):
        return self._client.call("getTaskStepInfoSpace")

    def _get_agent_step_info_space(self):
        return self._client.call("getAgentStepInfoSpace")

    def _apply_action(self, action):

        assert action.keys() == self._action_space_info.space.spaces.keys()

        action_shared = { name:component for name, component in action.items() if name in self._action_space_info.space_shared.spaces.keys() }
        self._action_space_info.set_shared_memory_data(action_shared)

        action_non_shared = { name:component for name, component in action.items() if name in self._action_space_info.space_non_shared.spaces.keys() }
        action_non_shared_serialized = _serialize_arrays(
            action_non_shared, space=self._action_space_info.space_non_shared, byte_order=self._byte_order)
        self._client.call("applyAction", action_non_shared_serialized)

    def _get_observation(self):

        observation_shared = self._observation_space_info.shared_memory_arrays

        observation_non_shared_serialized = self._client.call("getObservation")
        observation_non_shared = _deserialize_arrays(
            observation_non_shared_serialized, space=self._observation_space_info.space_non_shared, byte_order=self._byte_order)

        assert len(set(observation_shared.keys()) & set(observation_non_shared.keys())) == 0

        return {**observation_shared, **observation_non_shared}

    def _get_reward(self):
        return self._client.call("getReward")
    
    def _is_episode_done(self):
        return self._client.call("isEpisodeDone")

    def _get_step_info(self):

        task_step_info_shared = self._task_step_info_space_info.shared_memory_arrays
        agent_step_info_shared = self._agent_step_info_space_info.shared_memory_arrays

        task_step_info_non_shared_serialized = self._client.call("getTaskStepInfo")
        agent_step_info_non_shared_serialized = self._client.call("getAgentStepInfo")

        task_step_info_non_shared = _deserialize_arrays(
            task_step_info_non_shared_serialized, space=self._task_step_info_space_info.space_non_shared, byte_order=self._byte_order)
        agent_step_info_non_shared = _deserialize_arrays(
            agent_step_info_non_shared_serialized, space=self._agent_step_info_space_info.space_non_shared, byte_order=self._byte_order)

        assert len(set(task_step_info_shared.keys()) & set(task_step_info_non_shared.keys())) == 0
        assert len(set(agent_step_info_shared.keys()) & set(agent_step_info_non_shared.keys())) == 0

        return {
            "task_step_info":{**task_step_info_shared, **task_step_info_non_shared},
            "agent_step_info":{**agent_step_info_shared, **agent_step_info_non_shared}}

    def _reset(self):
        # reset the task first in case it needs to set the pose of actors,
        # then reset agent so it can refine the pose of actors
        self._client.call("resetTask")
        self._client.call("resetAgent")

    def _is_ready(self):
        return self._client.call("isTaskReady") and self._client.call("isAgentReady")


# metadata describing a space including shared memory objects
class SpaceInfo():
    def __init__(self, raw_space, dict_space_type, box_space_type):

        # raw spaces
        self.raw_space = raw_space
        self.raw_space_shared = { name:component for name, component in self.raw_space.items() if component["use_shared_memory_"] }
        self.raw_space_non_shared = { name:component for name, component in self.raw_space.items() if not component["use_shared_memory_"] }

        # deserialized spaces
        self.space = _deserialize_dict_space(self.raw_space, dict_space_type=dict_space_type, box_space_type=box_space_type)
        self.space_shared = _deserialize_dict_space(self.raw_space_shared, dict_space_type=dict_space_type, box_space_type=box_space_type)
        self.space_non_shared = _deserialize_dict_space(self.raw_space_non_shared, dict_space_type=dict_space_type, box_space_type=box_space_type)

        # shared memory
        self.shared_memory_objects = {}
        self.shared_memory_arrays = {}
        for name, component in self.raw_space_shared.items():
            if sys.platform == "win32":
                self.shared_memory_objects[name] = mmap.mmap(
                    -1, np.prod(component["shape_"]) * DATATYPE_TO_DTYPE[component["datatype_"]].itemsize, component["shared_memory_name_"])
                self.shared_memory_arrays[name] = np.ndarray(
                    shape=tuple(component["shape_"]), dtype=DATATYPE_TO_DTYPE[component["datatype_"]], buffer=self.shared_memory_objects[name])
            elif sys.platform in ["darwin", "linux"]:
                self.shared_memory_objects[name] = multiprocessing.shared_memory.SharedMemory(name=component["shared_memory_name_"])
                self.shared_memory_arrays[name] = np.ndarray(
                    shape=tuple(component["shape_"]), dtype=DATATYPE_TO_DTYPE[component["datatype_"]], buffer=self.shared_memory_objects[name].buf)
            else:
                assert False

    def terminate(self):
        self.shared_memory_arrays = {}
        for name, shared_memory_object in self.shared_memory_objects.items():
            if sys.platform in ["darwin", "linux"]:
                shared_memory_object.close()
                shared_memory_object.unlink()

    def set_shared_memory_data(self, data):
        assert data.keys() == self.space_shared.spaces.keys()
        for name, component in data.items():
            assert isinstance(component, np.ndarray)
            assert component.shape == self.space_shared.spaces[name].shape
            assert component.dtype == self.space_shared.spaces[name].dtype
            assert (component >= self.space_shared.spaces[name].low).all()
            assert (component <= self.space_shared.spaces[name].high).all()
            self.shared_memory_arrays[name][:] = data[name][:]


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


# enum values must match cpp/unreal_plugins/SimulationController/Box.h
class DataType(Enum):
    Invalid    = -1,
    Boolean    = 0
    UInteger8  = 1
    Integer8   = 2
    UInteger16 = 3
    Integer16  = 4
    UInteger32 = 5
    Integer32  = 6
    Float32    = 7
    Double     = 8

DATATYPE_TO_DTYPE = {
    DataType.Boolean.value:    np.dtype("?"),
    DataType.UInteger8.value:  np.dtype("u1"),
    DataType.Integer8.value:   np.dtype("i1"),
    DataType.UInteger16.value: np.dtype("u2"),
    DataType.Integer16.value:  np.dtype("i2"),
    DataType.UInteger32.value: np.dtype("u4"),
    DataType.Integer32.value:  np.dtype("i4"),
    DataType.Float32.value:    np.dtype("f4"),
    DataType.Double.value:     np.dtype("f8"),
}


# serialize and deserialize functions for converting between Python and C++ data
def _deserialize_dict_space(raw_space, dict_space_type, box_space_type):
    return dict_space_type({ name:_deserialize_box_space(component, box_space_type=box_space_type) for name, component in raw_space.items() })

def _deserialize_box_space(data, box_space_type):
    low = data["low_"]
    high = data["high_"]
    shape = tuple(data["shape_"])
    dtype = DATATYPE_TO_DTYPE[data["datatype_"]]
    return box_space_type(low, high, shape, dtype)

def _deserialize_arrays(data, space, byte_order):
    assert data.keys() == space.spaces.keys()
    return { name:_deserialize_array(component, space=space.spaces[name], byte_order=byte_order) for (name, component) in data.items() }

def _serialize_arrays(arrays, space, byte_order):
    assert arrays.keys() == space.spaces.keys()
    return { name:_serialize_array(component, space=space.spaces[name], byte_order=byte_order) for (name, component) in arrays.items() }

def _deserialize_array(data, space, byte_order):
    dtype = space.dtype if byte_order is None else space.dtype.newbyteorder(byte_order)
    array = np.frombuffer(data, dtype=dtype, count=-1).reshape(space.shape)
    assert (array >= space.low).all()
    assert (array <= space.high).all()
    return array

def _serialize_array(array, space, byte_order):
    assert isinstance(array, np.ndarray)
    assert array.shape == space.shape
    assert array.dtype == space.dtype
    assert (array >= space.low).all()
    assert (array <= space.high).all()
    data = array.data if byte_order is None else array.newbyteorder(byte_order).data
    return data
