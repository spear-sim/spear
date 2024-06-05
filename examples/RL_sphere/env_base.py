import json

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


class SimpleEnv(gym.Env):
    def __init__(self, env_config):
        spear.log("__init__ Start.")
        self._config = env_config['config']
        self._instance = spear.Instance(self._config)

        self.action_space = gym.spaces.Dict({
            "add_to_location": gym.spaces.Box(-1, 1, (3,), np.float64),
            # "add_to_rotation": gym.spaces.Box(-1, 1, (3,), np.float64),
        })
        self.observation_space = gym.spaces.Dict({
            # "camera.final_color": gym.spaces.Box(0, 255, (480, 640, 3,), np.uint8),
            "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            # "rotation": gym.spaces.Box(-1000, 1000, (3,), np.float64),
        })
        # spawn agent
        self._instance.engine_service.begin_tick()

        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent"}
        )
        spear.log("agent = ", self._agent)

        # 1. get class: uclass = _agent.get_class()
        # 2. get func: ufunc = uclass.find_function_by_name(apply_action)
        # 3. call function: call(_agent, ufunc, args)

        unreal_actor_static_class = self._instance.unreal_service.get_static_class("AActor")
        self._unreal_set_actor_location_and_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")
        self._unreal_get_actor_location_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorLocation")
        self._unreal_get_actor_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorRotation")

        self._instance.engine_service.tick()
        self._instance.engine_service.end_tick()

        self._goal = np.array([10, 0, 0])
        spear.log("__init__ Done.")

    def reset(self):
        self._instance.engine_service.begin_tick()

        new_location = np.array([0, 0, 0])
        new_rotation = np.array([0, 0, 0])
        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)

        self._instance.engine_service.tick()
        self._instance.engine_service.end_tick()

        obs = {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.uint8),
            "location": np.array([0, 0, 0], dtype=np.float64),
            # "rotation": np.array([0, 0, 0], dtype=np.float64),
        }
        print("reset Done.")
        return obs

    def step(self, action):

        self._instance.engine_service.begin_tick()
        # get old observation and apply action

        current_location = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_location_func)['ReturnValue']
        current_rotation = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_rotation_func)['ReturnValue']
        current_location = json.loads(current_location)
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        current_rotation = json.loads(current_rotation)
        current_rotation = np.array([current_rotation['roll'], current_rotation['yaw'], current_rotation['pitch']])

        new_location = current_location + action['add_to_location']
        new_rotation = current_rotation
        if "add_to_rotation" in action:
            new_rotation += action['add_to_rotation']
        new_rotation = np.array([0, 0, 0])
        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)

        self._instance.engine_service.tick()

        # get observation
        current_location = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_location_func)['ReturnValue']
        current_rotation = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_rotation_func)['ReturnValue']

        current_location = json.loads(current_location)
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        current_rotation = json.loads(current_rotation)
        current_rotation = np.array([current_rotation['roll'], current_rotation['yaw'], current_rotation['pitch']])

        self._instance.engine_service.end_tick()

        obs = {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": current_location,
            # "rotation": current_rotation,
        }

        distance = np.linalg.norm(current_location - self._goal)

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
            print("SUCC", new_location, action, reward, done, info)
        return obs, reward, done, info

    def close(self):
        # close the unreal instance and rpc connection
        self._instance.close()

        spear.log("close Done.")

    def _set_spaces(self):
        assert False

    def _transform_observation(self, obs):
        assert False
