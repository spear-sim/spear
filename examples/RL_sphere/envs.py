import gym
import numpy as np
import os
import psutil
import sys
from yacs.config import CfgNode

from unrealai import UnrealEnv, UnrealToGymWrapper


# TODO: For environments that provide physical and visual observations, we assume that physical and
# visual observations are located indices 0 and 1 respectively. We should get these observation
# components by name rather than by integer ID.
PHYSICAL_OBSERVATION_SPACE_ID = 0
VISUAL_OBSERVATION_SPACE_ID = 1


class BaseEnv(gym.Env):
    def __init__(self, env_config):

        # We need to perform some modifications to config from within this constructor instead of
        # earlier in our code, because we need access to env_config.worker_index, and this property
        # is automatically added to env_config by RLLib just before invoking this constructor.
        self._worker_index = env_config.worker_index

        # Get the top-level YACS CfgNode from env_config
        config = env_config["config"]

        # If we're restoring from a previous checkpoint, then config will be a dict instead of a CfgNode
        if isinstance(config, dict):
            config = CfgNode(init_dict=config)

        config.defrost()

        config.UNREALAI.UNREAL_INTERNAL_LOG_FILE = os.path.join("worker_index_%02d" % self._worker_index, "log.txt")
        config.UNREALAI.TEMP_DIR = os.path.join(config.UNREALAI.TEMP_DIR, "worker_index_%02d" % self._worker_index)
        config.UNREALAI.PORT = config.UNREALAI.PORT + self._worker_index
        config.UNREALAI.RANDOM_SEED = self._worker_index

        # If there are CUDA devices, then assign gpu_id based on worker_index and available CUDA devices.
        # We don't use the CUDA_VISIBLE_DEVICES environment variable directly because RLlib overwrites
        # it. See https://docs.ray.io/en/master/ray-core/using-ray-with-gpus.html
        if len(config.TRAIN.CUDA_VISIBLE_DEVICES) > 0:
            device_ids = [int(i) for i in config.TRAIN.CUDA_VISIBLE_DEVICES.split(",")]
            config.UNREALAI.GPU_ID = device_ids[self._worker_index % len(device_ids)]

        config.freeze()

        # Verify that the desired port is not in use. Note that this test is overly conservative, because
        # it isn't a problem if a truly remote address is using our desired port. But it's hard to check
        # if an apparently remote address is truly a remote address, so we implement our overly conservative
        # test.
        for pid in psutil.pids():
            try:
                process = psutil.Process(pid)
                for connection in process.connections(kind="tcp"):
                    assert connection.laddr.port != config.UNREALAI.PORT
                    if connection.raddr != ():
                        assert connection.raddr.port != config.UNREALAI.PORT
            except psutil.AccessDenied:
                pass
            except psutil.NoSuchProcess:
                pass

        # TODO: Move this code to print the config object into the implementation of UnrealEnv.
        print("self._worker_index = " + str(self._worker_index) + ", BaseEnv.__init__()")
        print("self._worker_index = " + str(self._worker_index) + ", Creating UnrealEnv with the following configuration options...")
        print("self._worker_index = " + str(self._worker_index) + ", config = \n" + str(config))

        uenv = UnrealEnv(config=config)
        self._env = UnrealToGymWrapper(uenv)

        self._set_spaces()

    def reset(self):
        obs = self._env.reset()
        return self._transform_observation(obs)

    def step(self, action):
        print(action)
        obs, reward, done, info = self._env.step(action)
        print(reward, done, info)
        return self._transform_observation(obs), reward, done, info

    def close(self):
        print("self._worker_index = " + str(self._worker_index) + ", Closing environment...")
        self._env.close()

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False


class PhysicalObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = self._env.observation_space

    def _transform_observation(self, obs):
        return obs


class VisualObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=self._env.observation_space.shape, dtype=np.float32)

    def _transform_observation(self, obs):
        return (obs / 255.0).astype(np.float32)


class MixedObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = gym.spaces.Dict({
            "physical": self._env.observation_space[PHYSICAL_OBSERVATION_SPACE_ID],
            "visual": gym.spaces.Box(low=0.0, high=1.0, shape=self._env.observation_space[VISUAL_OBSERVATION_SPACE_ID].shape, dtype=np.float32)
        })

    def _transform_observation(self, obs):
        return {
            "physical": obs[PHYSICAL_OBSERVATION_SPACE_ID],
            "visual": (obs[VISUAL_OBSERVATION_SPACE_ID] / 255.0).astype(np.float32)
        }


class PhysicalOnlyFromMixedObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = self._env.observation_space[PHYSICAL_OBSERVATION_SPACE_ID]

    def _transform_observation(self, obs):
        return obs[PHYSICAL_OBSERVATION_SPACE_ID]


class VisualOnlyFromMixedObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=self._env.observation_space[VISUAL_OBSERVATION_SPACE_ID].shape, dtype=np.float32)

    def _transform_observation(self, obs):
        return (obs[VISUAL_OBSERVATION_SPACE_ID] / 255.0).astype(np.float32)


class DebugEnv(gym.Env):
    def __init__(self, env_config):

        # We need to perform some modifications to env_config from within this constructor
        # instead of earlier in our code, because we need access to env_config.worker_index,
        # and this property is automatically added to env_config by RLLib just before
        # invoking this constructor.
        worker_index = env_config.worker_index

        self._worker_index = worker_index

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # If there are CUDA devices, then assign gpu_id based on worker_index and available CUDA devices.
        # We don't use the CUDA_VISIBLE_DEVICES environment variable directly because RLlib overwrites
        # it. See https://docs.ray.io/en/master/ray-core/using-ray-with-gpus.html
        if len(env_config["cuda_visible_devices"]) > 0:
            device_ids = [int(i) for i in env_config["cuda_visible_devices"].split(",")]
            env_config["gpu_id"] = device_ids[worker_index % len(device_ids)]

        print("self._worker_index = " + str(self._worker_index) + ", DebugEnv.__init__()")
        print("self._worker_index = " + str(self._worker_index) + ", self.action_space = " + str(self.action_space))
        print("self._worker_index = " + str(self._worker_index) + ", self.observation_space = " + str(self.observation_space))
        print("self._worker_index = " + str(self._worker_index) + ', env_config["cuda_visible_devices"] = ' + env_config["cuda_visible_devices"])
        print("self._worker_index = " + str(self._worker_index) + ', env_config["gpu_id"] = ' + str(env_config["gpu_id"]))

    def reset(self):
        self.num_steps = 0
        obs = self.observation_space.sample()
        print("self._worker_index = " + str(self._worker_index) + ", DebugEnv.reset()")
        print("self._worker_index = " + str(self._worker_index) + ", obs = " + str(obs))
        return obs

    def step(self, action):
        self.num_steps = self.num_steps + 1
        obs = self.observation_space.sample()
        reward = action[0]
        done = self.num_steps == 100
        info = {}
        print("self._worker_index = " + str(self._worker_index) + ", DebugEnv.step(action)")
        print("self._worker_index = " + str(self._worker_index) + ", action = " + str(action))
        print("self._worker_index = " + str(self._worker_index) + ", obs = " + str(obs) + ", done = " + str(done) + ", info = " + str(info))
        return obs, reward, done, info

    def close(self):
        print("self._worker_index = " + str(self._worker_index) + ", Closing environment...")
        pass
