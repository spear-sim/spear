import argparse
import csv
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import shutil
import spear
import sys

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class OpenBotEnv(spear.Env):

    def __init__(self, config, num_internal_steps):
        super(OpenBotEnv, self).__init__(config)
        self.config = config
        assert num_internal_steps > 0
        self.num_internal_steps = num_internal_steps

    def show_obs_and_wait_for_key(self):
        obs = self.get_observation()
        
        for obs_component in self.config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS:
            if obs_component == "state_data":
                print(f"State data: xyz [{obs['state_data'][0]:.2f}, {obs['state_data'][1]:.2f},{obs['state_data'][2]:.2f}]")
                print(f"State data: pitch yaw roll [{obs['state_data'][3]:.2f}, {obs['state_data'][4]:.2f},{obs['state_data'][5]:.2f}]")
            elif obs_component == "control_data":
                print(f"Control data: left right [{obs['control_data'][0]:.2f}, {obs['control_data'][1]:.2f}]")
            elif obs_component == "imu":
                print(f"IMU data: linear_acceleration [{obs['imu'][0]:.2f}, {obs['imu'][1]:.2f},{obs['imu'][2]:.2f}]"
                print(f"IMU data: angular_rate [{obs['imu'][3]:.2f}, {obs['imu'][4]:.2f}, {obs['imu'][5]:.2f}]")
            elif obs_component == "sonar":
                print(f"Sonar data: {obs['sonar']:.2f}")
            elif obs_component == "camera":
                for render_pass in self.config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES:
                    elif render_pass == "final_color":
                        cv2.imshow("rgb", obs["camera_final_color"][:, :, [2, 1, 0]]) # OpenCV expects BGR instead of RGB
                    elif render_pass == "segmentation":
                        cv2.imshow("segmentation", obs["camera_segmentation"][:, :, [2, 1, 0]]) # OpenCV expects BGR instead of RGB
                    elif render_pass == "depth_glsl":
                        cv2.imshow("depth", obs["camera_depth_glsl"][:, :, :])
                    else:
                        print(f"Error: {render_pass} is an unknown camera render pass.")
            else:
                print(f"Error: {obs_component} is an unknown observation component.")
                
        cv2.waitKey(100)

    def _transform_observation(self, observation):
        return observation
    
    def _transform_reward(self, reward):
        return reward
    
    def _transform_status(self, is_done):
        return is_done
        
    def _transform_step_info(self, step_info):
        return step_info
        
    def step(self, action):

        if self.num_internal_steps == 1:
            return self.single_step(action, get_observation=True)
        else:
            self.single_step(action)
            for _ in range(1, self.num_internal_steps - 1):
                self.single_step(action)
                # TODO: implement the remaining step logic for RL
            return self.single_step(get_observation=True)

    def single_step(self, action=None, get_observation=False):
    
        self._begin_tick()
        if action:
            self._apply_action(action)
        self._tick()
        if get_observation:
            obs = self._transform_observation(self._get_observation())
            reward = self._transform_reward(self._get_reward())
            is_done = self._transform_status(self._is_episode_done())
            step_info = self._transform_step_info(self._get_step_info())
            self._end_tick()
            return obs, reward, is_done, step_info
        else:
            self._end_tick()
            return None, None, None, None

