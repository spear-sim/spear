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

class OpenBotAgent:

    def __init__(self, config):
        self.config = config

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

    def get_compass_observation(goal_position_xy, current_pose_yaw_xy):

        # Target error vector (global coordinate system):
        relative_position_to_target = goal_position_xy - np.array([current_pose_xy[1], current_pose_xy[2])

        # Compute Euclidean distance to target:
        dist = np.linalg.norm(relative_position_to_target)

        # Compute robot forward axis (global coordinate system):
        yaw_vehicle = array_obs[i][7];
        rot = np.array([[cos(yaw_vehicle), -sin(yaw_vehicle)], [sin(yaw_vehicle), cos(yaw_vehicle)]])

        forward_rotated = np.dot(rot, forward)

        # Compute yaw:
        delta_yaw = np.arctan2(forward_rotated[1], forward_rotated[0]) - np.arctan2(relative_position_to_target[1], relative_position_to_target[0])

        # Fit to range [-pi, pi]:
        if delta_yaw > np.pi:
            delta_yaw -= 2 * np.pi
        elif delta_yaw <= -np.pi:
            delta_yaw += 2 * np.pi

        # Check the actual OpenBot code:
        # https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
        sin_yaw = np.sin(delta_yaw);
        cos_yaw = np.cos(delta_yaw);

        return dist, sin_yaw, cos_yaw
    
        


 


        
        
        
        
        
def clamp_axis(angle):
	# returns angle in the range (-360,360)
	angle = angle % 360.0

	if (angle < 0.0):
		# shift to [0,360) range
		angle += 360.0

	return angle


def normalize_axis( angle ):
	# returns angle in the range [0,360)
	angle = clampAxis(angle)

	if (angle > 180.0):
		# shift to (-180,180]
		angle -= 360.0

	return angle


def quaternion_to_rpy(quat):

    # Inspired by UnrealMath.cpp... 
    # quaternions are given in the format [X, Y, Z, W]
    X = quat[0]
    Y = quat[1]
    Z = quat[2]
    W = quat[3]

    singularity_test = Z*X - W*Y
    yaw_y = 2.0*(W*Z + X*Y)
    yaw_x = (1.0 - 2.0 * (np.square(Y) + np.square(Z)))

	# reference 
	# http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	# http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

	# this value was found from experience, the above websites recommend different values
	# but that isn't the case for us, so I went through different testing, and finally found the case 
	# where both of world lives happily. 
    SINGULARITY_THRESHOLD = 0.4999995
    
    if (singularity_test < -SINGULARITY_THRESHOLD):
        pitch = -90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalizeAxis(-yaw - (2.0 * np.arctan2(X, W))))

    elif (singularity_test > SINGULARITY_THRESHOLD):
        pitch = 90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalizeAxis(yaw - (2.0 * np.arctan2(X, W))))

    else:
        pitch = np.rad2deg(np.arcsin(2.0 * singularity_test))
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(np.arctan2(-2.0 * (W*X + Y*Z), (1.0 - 2.0 * (np.square(X) + np.square(Y)))))

    return roll, pitch, yaw
