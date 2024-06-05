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


def random_position():
    pos = np.random.rand(3)
    pos = pos / np.linalg.norm(pos) * 20
    return pos.astype(np.float64)


class AgentBase():
    def __init__(self, instance):
        self._instance = instance

        unreal_actor_static_class = self._instance.unreal_service.get_static_class("AActor")
        self._unreal_set_actor_location_and_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_SetActorLocationAndRotation")
        self._unreal_get_actor_location_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorLocation")
        self._unreal_get_actor_rotation_func = self._instance.unreal_service.find_function_by_name(
            uclass=unreal_actor_static_class, name="K2_GetActorRotation")

    def get_observation_space(self):
        assert False

    def get_observation(self):
        assert False

    def get_action_space(self):
        assert False

    def apply_action(self, action):
        assert False


class SimpleAgent(AgentBase):
    def __init__(self, instance):
        super().__init__(instance)

        self._agent = self._instance.unreal_service.spawn_actor(
            class_name="/Game/Agents/BP_SimpleAgentPawn.BP_SimpleAgentPawn_C",
            location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent"}
        )

        spear.log("agent = ", self._agent)

    def get_observation_space(self):
        return gym.spaces.Dict({
            # "camera.final_color": gym.spaces.Box(0, 255, (480, 640, 3,), np.uint8),
            "location": gym.spaces.Box(-1000, 1000, (3,), np.float64),
            # "rotation": gym.spaces.Box(-1000, 1000, (3,), np.float64),
        })

    def get_action_space(self):
        return gym.spaces.Dict({
            "add_to_location": gym.spaces.Box(-1, 1, (3,), np.float64),
            # "add_to_rotation": gym.spaces.Box(-1, 1, (3,), np.float64),
        })

    def get_observation(self):
        # get observation
        current_location = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_location_func)['ReturnValue']
        current_rotation = self._instance.unreal_service.call_function(self._agent, self._unreal_get_actor_rotation_func)['ReturnValue']

        current_location = json.loads(current_location)
        current_location = np.array([current_location['x'], current_location['y'], current_location['z']])
        current_rotation = json.loads(current_rotation)
        current_rotation = np.array([current_rotation['roll'], current_rotation['yaw'], current_rotation['pitch']])
        self._obs = {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": current_location,
            # "rotation": current_rotation,
        }
        return self._obs

    def apply_action(self, action):
        new_location = self._obs['location'] + action['add_to_location']
        new_rotation = np.array([0, 0, 0])
        if 'rotation' in self._obs:
            new_rotation += self._obs['rotation']
        if "add_to_rotation" in action:
            new_rotation += action['add_to_rotation']

        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)

    def reset(self):
        new_location = random_position()
        new_rotation = np.array([0, 0, 0])
        transform_args = {
            "NewLocation": dict(zip(["X", "Y", "Z"], new_location.tolist())),
            "NewRotation": dict(zip(["Roll", "Pitch", "Yaw"], new_rotation.tolist())),
            "bSweep": False,
            "bTeleport": True}
        self._instance.unreal_service.call_function(self._agent, self._unreal_set_actor_location_and_rotation_func, transform_args)
        return {
            # "camera.final_color": np.zeros([480, 640, 3], dtype=np.float64),
            "location": new_location,
            # "rotation": current_rotation,
        }
