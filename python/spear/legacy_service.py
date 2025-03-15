#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np

class LegacyService():
    def __init__(self, entry_point_caller):
        self._entry_point_caller = entry_point_caller

    def get_action_space(self):
        return self._entry_point_caller.call("legacy_service.get_action_space")
    
    def get_observation_space(self):
        return self._entry_point_caller.call("legacy_service.get_observation_space")
    
    def get_task_step_info_space(self):
        return self._entry_point_caller.call("legacy_service.get_task_step_info_space")
    
    def get_agent_step_info_space(self):
        return self._entry_point_caller.call("legacy_service.get_agent_step_info_space")
    
    def apply_action(self, action):
        self._entry_point_caller.call("legacy_service.apply_action", action)
    
    def get_observation(self):
        return self._entry_point_caller.call("legacy_service.get_observation")
    
    def get_reward(self):
        return self._entry_point_caller.call("legacy_service.get_reward")

    def is_episode_done(self):
        return self._entry_point_caller.call("legacy_service.is_episode_done")
    
    def get_task_step_info(self):
        return self._entry_point_caller.call("legacy_service.get_task_step_info")

    def get_agent_step_info(self):
        return self._entry_point_caller.call("legacy_service.get_agent_step_info")

    def reset_task(self):
        self._entry_point_caller.call("legacy_service.reset_task")

    def reset_agent(self):
        self._entry_point_caller.call("legacy_service.reset_agent")

    def is_task_ready(self):
        return self._entry_point_caller.call("legacy_service.is_task_ready")

    def is_agent_ready(self):
        return self._entry_point_caller.call("legacy_service.is_agent_ready")

    def get_world_name(self):
        return self._entry_point_caller.call("legacy_service.get_world_name")
