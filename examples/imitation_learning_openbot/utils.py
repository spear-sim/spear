import argparse
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import shutil
import ffmpeg
import spear
import sys

def show_obs_and_wait_for_key(obs, obs_components, render_passes):
        
    for obs_component in obs_components:
        if obs_component == "state_data":
            print(f"State data: xyz [{obs['state_data'][0]:.2f}, {obs['state_data'][1]:.2f},{obs['state_data'][2]:.2f}]")
            print(f"State data: pitch yaw roll [{obs['state_data'][3]:.2f}, {obs['state_data'][4]:.2f},{obs['state_data'][5]:.2f}]")
        elif obs_component == "control_data":
            print(f"Control data: left right [{obs['control_data'][0]:.2f}, {obs['control_data'][1]:.2f}]")
        elif obs_component == "imu":
            print(f"IMU data: linear_acceleration [{obs['imu'][0]:.2f}, {obs['imu'][1]:.2f},{obs['imu'][2]:.2f}]")
            print(f"IMU data: angular_rate [{obs['imu'][3]:.2f}, {obs['imu'][4]:.2f}, {obs['imu'][5]:.2f}]")
        elif obs_component == "sonar":
            print(f"Sonar data: {obs['sonar'][0]:.2f}")
        elif obs_component == "camera":
            for render_pass in render_passes:
                if render_pass == "final_color":
                    cv2.imshow("rgb", obs["camera_final_color"][:, :, [2, 1, 0]]) # OpenCV expects BGR instead of RGB
                elif render_pass == "segmentation":
                    cv2.imshow("segmentation", obs["camera_segmentation"][:, :, [2, 1, 0]]) # OpenCV expects BGR instead of RGB
                elif render_pass == "depth_glsl":
                    cv2.imshow("depth", obs["camera_depth_glsl"][:, :, :])
                else:
                    print(f"Error: {render_pass} is an unknown camera render pass.")
                    assert False
        else:
            print(f"Error: {obs_component} is an unknown observation component.")
            assert False
                
    cv2.waitKey(0)


# computes the 2D target position relative to the agent in world frame 
# as well as the relative yaw angle between the agent forward axis and the agent-target vector
def get_relative_target_pose(position_xy_desired, position_xy_current, yaw_xy_current):

    # target error vector (global coordinate system)
    relative_agent_target_xy = position_xy_desired - position_xy_current

    # compute agent forward axis (global coordinate system)
    rot = np.array([[np.cos(yaw_xy_current), -np.sin(yaw_xy_current)], [np.sin(yaw_xy_current), np.cos(yaw_xy_current)]], dtype=np.float32)
    forward = np.array([1,0]) # front axis is the x axis.
    forward_rotated = np.dot(rot, forward)

    # compute relative yaw angle between the agent forward axis and the agent-target vector
    relative_agent_target_yaw = np.arctan2(forward_rotated[1], forward_rotated[0]) - np.arctan2(relative_agent_target_xy[1], relative_agent_target_xy[0])

    # fit to range [-pi, pi]
    if relative_agent_target_yaw > np.pi:
        relative_agent_target_yaw -= 2 * np.pi
    elif relative_agent_target_yaw <= -np.pi:
        relative_agent_target_yaw += 2 * np.pi

    return relative_agent_target_xy, relative_agent_target_yaw


# compute the compass observation following conventions of the actual OpenBot code:
# https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
def get_compass_observation(position_xy_desired, position_xy_current, yaw_current):

    # get the 2D reative pose between the agent and its target
    relative_agent_target_xy, relative_agent_target_yaw = get_relative_target_pose(position_xy_desired, position_xy_current, yaw_current)

    # compute Euclidean distance to target in [m]
    dist = np.linalg.norm(relative_agent_target_xy) * 0.01 

    # projection 
    sin_yaw = np.sin(relative_agent_target_yaw);
    cos_yaw = np.cos(relative_agent_target_yaw);

    return np.array([dist, sin_yaw, cos_yaw], dtype=np.float32)


def generate_video(config, video_name, image_dir, video_dir, compress = False):
    print("Generating video from the sequence of observations")

    name = f"{video_dir}/{video_name}.avi"
    name_compressed = f"{video_dir}/{video_name}_compressed.mp4"

    images = [img for img in os.listdir(image_dir)]
    frame = cv2.imread(os.path.join(image_dir, images[0]))
    height, width, layers = frame.shape

    rate = int(1/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS)

    video = cv2.VideoWriter(name, 0, rate, (width, height))

    # good initial sort but doesnt sort numerically very well
    images.sort(key=lambda f: int(re.sub('\D', '', f)))

    for image in images:
        video.write(cv2.imread(os.path.join(image_dir, image)))

    video.release()

    if compress:
        try:
            i = ffmpeg.input(name)
            print("Compressing video...")
            out = ffmpeg.output(i, name_compressed, **{'c:v': 'libx264', 'b:v': 8000000}).overwrite_output().run()
            # remove the uncompressed file
            os.remove(name) 
            print("Done compressing !")

        except FileNotFoundError as e:
            print("You do not have ffmpeg installed!", e)
            print("You can install ffmpeg by entering 'pip install ffmpeg-python' in a terminal.")
            assert False

