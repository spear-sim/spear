import json

import gym
import numpy as np
import os
import psutil
import sys

from gym import spaces
from yacs.config import CfgNode

import spear
from agent import SimpleAgent

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)


class SimpleEnv(gym.Env):
    def __init__(self, env_config):
        spear.log("__init__ Start.")
        self._config = env_config['config']
        self._instance = spear.Instance(self._config)

        self._instance.engine_service.begin_tick()

        # spawn agent
        self._agent = SimpleAgent(self._instance)
        self.action_space = self._agent.get_action_space()
        self.observation_space = self._agent.get_observation_space()

        self._instance.engine_service.tick()
        self._instance.engine_service.end_tick()

        self._goal = np.array([10, 0, 0])

        spear.log("self.action_space", self.action_space)
        spear.log("self.observation_space", self.observation_space)
        spear.log("__init__ Done.")

    def reset(self):
        self._instance.engine_service.begin_tick()

        self._agent.reset()

        self._instance.engine_service.tick()

        obs = self._agent.get_observation()

        self._instance.engine_service.end_tick()

        spear.log("reset Done.")
        return obs

    def step(self, action):

        self._instance.engine_service.begin_tick()

        current_obs = self._agent.get_observation()
        self._agent.apply_action(action)

        self._instance.engine_service.tick()

        obs = self._agent.get_observation()

        self._instance.engine_service.end_tick()

        distance = np.linalg.norm(obs['location'] - self._goal)
        reward = - distance
        done = distance < 1
        if done:
            reward += 1000.0  # normalize reward between -1 and 1
        if distance > 50:
            done = True

        info = {}
        truncated = False

        # print("obs", obs)
        # print("step", new_location, action, reward, done, info)
        if reward > 0:
            # print("SUCC!")
            spear.log("SUCC", obs['location'], action, reward, done, info)
        return obs, reward, done, info

    def close(self):
        # close the unreal instance and rpc connection
        self._instance.close()

        spear.log("close Done.")

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False
