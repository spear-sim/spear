import json
import random

import gym
import numpy as np
import os
import psutil
import sys

from gym import spaces
from yacs.config import CfgNode

import spear

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)


def random_position():
    pos = np.random.rand(3, )
    pos = pos / np.linalg.norm(pos) * 20
    return pos


class SimpleEnv2(gym.Env):
    def __init__(self, env_config):
        spear.log("__init__ Start.")
        random.seed(0)
        self._current_location = random_position()
        self._goal = np.array([0, 0, 0])
        self._step = 0

        self.action_space = gym.spaces.Dict({
            "add_to_location": gym.spaces.Box(-1, 1, (3,), np.float64),
        })
        self.observation_space = gym.spaces.Dict({
            "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
        })

        spear.log("__init__ Done.")

    def reset(self):
        self._current_location = random_position()
        obs = {
            "location": self._current_location,
        }
        self._step = 0
        print("reset Done.", self._current_location)
        return obs

    def step(self, action):
        self._current_location += action['add_to_location'] / np.linalg.norm(action['add_to_location'])
        obs = {
            "location": self._current_location,
        }

        distance = np.linalg.norm(self._current_location - self._goal)

        reward = - distance
        done = distance < 1
        if done:
            reward += 1000.0  # normalize reward between -1 and 1
        if distance > 100:
            done = True
        info = {}
        if self._step % 100 == 0 or done:
            print("step",self._step, obs, action, reward, done, info)

        self._step += 1
        return obs, reward, done, info

    def close(self):
        spear.log("close Done.")

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False
