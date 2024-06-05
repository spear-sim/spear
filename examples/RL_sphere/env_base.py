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
        self._gameplay_statics_class = self._instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        self._gameplay_statics_default_object = self._instance.unreal_service.get_default_object(uclass=self._gameplay_statics_class, create_if_needed=False)
        self._set_game_paused_func = self._instance.unreal_service.find_function_by_name(uclass=self._gameplay_statics_class, name="SetGamePaused")
        self._instance.engine_service.tick()
        self._instance.engine_service.end_tick()

        self.begin_frame()

        # spawn agent
        self._agent = SimpleAgent(self._instance)
        self.action_space = self._agent.get_action_space()
        self.observation_space = self._agent.get_observation_space()

        self._instance.engine_service.tick()

        self.end_frame()

        self._goal = np.array([10, 0, 0])
        self._step = 0

        spear.log("self.action_space", self.action_space)
        spear.log("self.observation_space", self.observation_space)
        spear.log("__init__ Done.")

    def begin_frame(self):
        self._instance.engine_service.begin_tick()
        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": False})
        pass

    def end_frame(self):
        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": True})
        self._instance.engine_service.end_tick()

    def reset(self):
        self.begin_frame()

        self._agent.reset()

        self._instance.engine_service.tick()

        obs = self._agent.get_observation()

        self.end_frame()

        self._step = 0
        spear.log("reset Done.")
        return obs

    def step(self, action):

        self.begin_frame()

        current_obs = self._agent.get_observation()
        self._agent.apply_action(action)

        self._instance.engine_service.tick()

        obs = self._agent.get_observation()

        self.end_frame()

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
        if self._step % 100 == 0 or done:
            print("step", self._step, obs, action, reward, done, info)

        self._step += 1
        return obs, reward, done, info

    def close(self):
        # close the unreal instance and rpc connection
        self._instance.close()

        spear.log("close Done.")

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False
