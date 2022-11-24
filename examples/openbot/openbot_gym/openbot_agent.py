import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import shutil
import spear
import sys

from openbot_autopilot import OpenBotAutopilot

class OpenBotAgent:

    def __init__(self, config):
        self.config = config
        self.forward = np.array([1.0,0.0], dtype=np.float32) # front axis is the x axis.
        self.autopilot = OpenBotAutopilot(config)

        for obs_component in self.config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS:
            if obs_component == "state_data":
                # state_data: [x, y, z, pitch, yaw, roll]
                # x-y-z component of agent position wrt. world in [cm]
                # pitch-yaw-roll components of agent orientation wrt. world in [rad]
                self.state_data = np.array([0.0,0.0,0.0,0.0,0.0,0.0], dtype=np.float32)
            elif obs_component == "control_data":
                # control_data: [ctrl_left, ctrl_right], where
                # ctrl_left is the control duty cycle of the left weels in [%]
                # ctrl_right is the control duty cycle of the right weels in [%]
                self.control_data = np.array([0.0,0.0], dtype=np.float32)
            elif obs_component == "imu": 
                # imu_data: [a_x, a_y, a_z, g_x, g_y, g_z], where
                # a_x, a_y, a_z are linear accelerations expressed in [m/s²] 
                # g_x, g_y, g_z are angular rates expressed in [rad/s] 
                self.imu_data = np.array([0.0,0.0,0.0,0.0,0.0,0.0], dtype=np.float32)
            elif obs_component == "sonar": # 
                # sonar_data: [sonar_dist], where
                # sonar_dist is the distance measured by the sonar in [m] 
                self.sonar_data = 0.0
            else:
                print(f"Error: {obs_component} is an unknown observation component.")

    def reset():

    def step():
        

    def show_obs_and_wait_for_key(self, obs):
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

    def get_compass_observation(self, goal_position_xy, current_pose_yaw_xy):

        # target error vector (global coordinate system):
        relative_position_to_target = goal_position_xy - np.array([current_pose_yaw_xy[1], current_pose_yaw_xy[2], dtype=np.float32)

        # compute Euclidean distance to target:
        dist = np.linalg.norm(relative_position_to_target)

        # compute robot forward axis (global coordinate system):
        yaw_vehicle = current_pose_yaw_xy[0];
        rot = np.array([[cos(yaw_vehicle), -sin(yaw_vehicle)], [sin(yaw_vehicle), cos(yaw_vehicle)]], dtype=np.float32)
        forward_rotated = np.dot(rot, self.forward)

        # compute yaw:
        delta_yaw = np.arctan2(forward_rotated[1], forward_rotated[0]) - np.arctan2(relative_position_to_target[1], relative_position_to_target[0])

        # fit to range [-pi, pi]:
        if delta_yaw > np.pi:
            delta_yaw -= 2 * np.pi
        elif delta_yaw <= -np.pi:
            delta_yaw += 2 * np.pi

        # following conventions of the actual OpenBot code:
        # https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
        sin_yaw = np.sin(delta_yaw);
        cos_yaw = np.cos(delta_yaw);

        return dist, sin_yaw, cos_yaw
