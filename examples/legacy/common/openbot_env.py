#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import gym
import numpy as np
import spear
from openbot_utils import get_drive_torques


# Custom Env implementation for OpenBot
class OpenBotEnv(spear.Env):

    def __init__(self, instance, config):

        assert config.SP_SERVICES.LEGACY_SERVICE.AGENT == "VehicleAgent"
        assert "set_drive_torques" in config.SP_SERVICES.LEGACY.VEHICLE_AGENT.ACTION_COMPONENTS
        assert "wheel_rotation_speeds" in config.SP_SERVICES.LEGACY.VEHICLE_AGENT.OBSERVATION_COMPONENTS

        self._config = config
        self._wheel_rotation_speeds = None

        super().__init__(instance, config)

        # In this derived class, we are expecting different actions than the base spear.Env class. So
        # we need to override self.action_space. We need to do this after calling super().__init__(...),
        # because the base spear.Env class will set self.action_space to the lower-level actions it is
        # expecting internally.
        self.action_space = gym.spaces.Dict(spaces={"set_duty_cycles": gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float64)})

    def reset(self, reset_info=None):

        obs = super().reset(reset_info=reset_info)
        assert "wheel_rotation_speeds" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_rotation_speeds"]
        return obs

    def _get_observation(self):

        obs = super()._get_observation()
        assert "wheel_rotation_speeds" in obs.keys()
        self._wheel_rotation_speeds = obs["wheel_rotation_speeds"]
        return obs

    def _apply_action(self, action):

        assert "set_duty_cycles" in action.keys()
        assert action["set_duty_cycles"].shape[0] == 2
        assert self._wheel_rotation_speeds is not None
        duty_cycles = np.array([action["set_duty_cycles"][0], action["set_duty_cycles"][1], action["set_duty_cycles"][0], action["set_duty_cycles"][1]], dtype=np.float64)
        drive_torques = get_drive_torques(duty_cycles, self._wheel_rotation_speeds, self._config)
        super()._apply_action(action={"set_drive_torques": drive_torques})
