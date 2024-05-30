import gym
import numpy as np
import os
import psutil
import sys
from yacs.config import CfgNode

import spear

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)


class BaseEnv(gym.Env):
    def __init__(self, env_config):
        config = spear.get_config(
            user_config_files=[
                os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
                os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])
        self._config = config
        spear.configure_system(config)
        self._instance = spear.Instance(config)
        self._env = spear.Env(self._instance, config)

        spear.log("try action_space")
        self.action_space = self._env.action_space
        self.observation_space = self._env.observation_space
        print("self.action_space", self.action_space)
        print("self.observation_space", self.observation_space)
        spear.log("__init__ Done.")

    def reset(self):
        obs = self._env.reset()
        return self._transform_observation(obs)
        # return {}

    # def reset(self, *, seed=None, options=None):
    #     obs = self._env.reset()
    #     # return self._transform_observation(obs)
    #     return {}

    def step(self, action):
        print("step", action)
        obs, reward, done, info = self._env.step(action)
        truncated = False
        print(reward, done, info)
        return self._transform_observation(obs), reward, done, info

    def close(self):
        # close the environment
        self._env.close()

        # close the unreal instance and rpc connection
        self._instance.close()

        spear.log("close Done.")

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False


class PhysicalObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = self._env.observation_space
        print("self.action_space", self.action_space)
        print("self.action_space", self.observation_space)

    def _transform_observation(self, obs):
        return obs


class VisualObservationEnv(BaseEnv):
    def _set_spaces(self):
        self.action_space = self._env.action_space
        self.observation_space = gym.spaces.Box(low=0.0, high=1.0, shape=self._env.observation_space.shape, dtype=np.float32)

    def _transform_observation(self, obs):
        return (obs / 255.0).astype(np.float32)
