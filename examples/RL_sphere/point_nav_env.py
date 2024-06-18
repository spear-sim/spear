import gym
import numpy as np
import os

import spear

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)
from agent import OpenBotAgent, SimpleAgent


class SpPointNavEnv(gym.Env):
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

        self._instance.engine_service.begin_tick()

        # spawn agent TODO read from config
        self._agent = SimpleAgent(self._instance)
        # TODO add camera if config
        self.action_space = self._agent.get_action_space()
        self.observation_space = self._agent.get_observation_space()

        self._instance.engine_service.tick()

        self._instance.engine_service.end_tick()

        # TODO read from config
        position = self._agent.get_random_points(1)[0]
        self._goal = np.array([position['x'], position['y'], position['z'] + 50])
        self._step = 0

        spear.log("self.action_space", self.action_space)
        spear.log("self.observation_space", self.observation_space)
        spear.log("__init__ Done.")

    def reset(self):
        self._instance.engine_service.begin_tick()
        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": False})

        self._agent.reset()

        self._instance.engine_service.tick()

        obs = self._agent.get_observation()

        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": True})
        self._instance.engine_service.end_tick()

        spear.log("reset Done.", self._step)

        self._step = 0

        position = self._agent.get_random_points(1)[0]
        self._goal = np.array([position['x'], position['y'], position['z']])

        return obs

    def step(self, action):
        self._instance.engine_service.begin_tick()
        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": False})

        self._agent.get_observation()
        self._agent.apply_action(action)

        self._instance.engine_service.tick()

        hit_actors = self._agent.get_hit_actors()
        obs = self._agent.get_observation()

        self._instance.unreal_service.call_function(uobject=self._gameplay_statics_default_object, ufunction=self._set_game_paused_func, args={"bPaused": True})
        self._instance.engine_service.end_tick()

        offset = obs['location'] - self._goal
        distance = np.linalg.norm(offset)
        reward = - distance
        done = distance < 10
        if done:
            reward += 100000.0  # normalize reward between -1 and 1
        if distance > 1000:
            done = True
        if len(hit_actors) > 1:
            done = True
            reward += -100
            print("hit_actors!", len(hit_actors))

        info = {}

        if reward > 0:
            spear.log("SUCC", self._step, obs, action, reward, done, info)
        if self._step % 100 == 0 or done:
            spear.log("step", self._step, obs, action, reward, done, info)

        self._step += 1
        return obs, reward, done, info

    def close(self):
        # close the unreal instance and rpc connection
        self._instance.close()

        spear.log("close Done.")
