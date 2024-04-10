#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np

class LegacyService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_random_points(self, num_points):
        random_points = self._rpc_client.call("legacy_service.get_random_points", num_points)
        return np.asarray(random_points, dtype=np.float64).reshape(num_points, 3)

    def get_random_reachable_points_in_radius(self, reference_points, radius):
        assert reference_points.shape[1] == 3
        reachable_points = self._rpc_client.call("legacy_service.get_random_reachable_points_in_radius", reference_points.flatten().tolist(), radius)
        return np.asarray(reachable_points, dtype=np.float64).reshape(reference_points.shape)

    def get_paths(self, initial_points, goal_points):
        assert initial_points.shape[1] == 3
        assert goal_points.shape[1] == 3
        paths = self._rpc_client.call("legacy_service.get_paths", initial_points.flatten().tolist(), goal_points.flatten().tolist())
        return [ np.asarray(path, dtype=np.float64).reshape(-1, 3) for path in paths ]

    def get_action_space(self):
        return self._rpc_client.call("legacy_service.get_action_space")
    
    def get_observation_space(self):
        return self._rpc_client.call("legacy_service.get_observation_space")
    
    def get_task_step_info_space(self):
        return self._rpc_client.call("legacy_service.get_task_step_info_space")
    
    def get_agent_step_info_space(self):
        return self._rpc_client.call("legacy_service.get_agent_step_info_space")
    
    def apply_action(self, action):
        self._rpc_client.call("legacy_service.apply_action", action)
    
    def get_observation(self):
        return self._rpc_client.call("legacy_service.get_observation")
    
    def get_reward(self):
        return self._rpc_client.call("legacy_service.get_reward")

    def is_episode_done(self):
        return self._rpc_client.call("legacy_service.is_episode_done")
    
    def get_task_step_info(self):
        return self._rpc_client.call("legacy_service.get_task_step_info")

    def get_agent_step_info(self):
        return self._rpc_client.call("legacy_service.get_agent_step_info")

    def reset_task(self):
        self._rpc_client.call("legacy_service.reset_task")

    def reset_agent(self):
        self._rpc_client.call("legacy_service.reset_agent")

    def is_task_ready(self):
        return self._rpc_client.call("legacy_service.is_task_ready")

    def is_agent_ready(self):
        return self._rpc_client.call("legacy_service.is_agent_ready")
