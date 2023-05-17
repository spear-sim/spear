#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import cv2
import ffmpeg
import matplotlib.pyplot as plt
import numpy as np
import os


def show_obs(obs, obs_components, render_passes):
        
    for obs_component in obs_components:
        if obs_component == "state_data":
            spear.log(f"state_data: xyz [{obs['state_data'][0]:.2f}, {obs['state_data'][1]:.2f}, {obs['state_data'][2]:.2f}]")
            spear.log(f"state_data: pitch yaw roll [{obs['state_data'][3]:.2f}, {obs['state_data'][4]:.2f},{obs['state_data'][5]:.2f}]")
        elif obs_component == "control_data":
            spear.log(f"control_data: left right [{obs['control_data'][0]:.2f}, {obs['control_data'][1]:.2f}]")
        elif obs_component == "encoder":
            spear.log(f"encoder: fl fr rl rr [{obs['encoder'][0]:.2f}, {obs['encoder'][1]:.2f}, {obs['encoder'][2]:.2f}, {obs['encoder'][3]:.2f}]")
        elif obs_component == "imu":
            spear.log(f"imu: linear_acceleration [{obs['imu'][0]:.2f}, {obs['imu'][1]:.2f},{obs['imu'][2]:.2f}]")
            spear.log(f"imu: angular_rate [{obs['imu'][3]:.2f}, {obs['imu'][4]:.2f}, {obs['imu'][5]:.2f}]")
        elif obs_component == "sonar":
            spear.log(f"sonar: {obs['sonar'][0]:.2f}")
        elif obs_component == "camera":
            for render_pass in render_passes:
                if render_pass in ["final_color", "normals", "segmentation"]:                    
                    cv2.imshow(render_pass, obs["camera." + render_pass]) # note that spear.Env returns BGRA by default
                elif render_pass == "depth":
                    depth = obs["camera.depth"]
                    depth_min = np.min(depth)
                    depth_max = np.max(depth)
                    depth_normalized = (depth-depth_min) / (depth_max-depth_min)
                    cv2.imshow(render_pass, depth_normalized)
                else:
                    spear.log(f"Error: {render_pass} is an unknown camera render pass.")
                    assert False
        else:
            spear.log(f"Error: {obs_component} is an unknown observation component.")
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
    sin_yaw = np.sin(relative_agent_target_yaw)
    cos_yaw = np.cos(relative_agent_target_yaw)

    return np.array([dist, sin_yaw, cos_yaw], dtype=np.float32)


def generate_video(image_dir, video_path, rate, compress=False):
    
    if compress:
        process = ffmpeg.input("pipe:", r=rate, f="jpeg_pipe").output(video_path, **{"c:v": "libx264", "b:v": 8000000}).overwrite_output().run_async(pipe_stdin=True)
    else:
        process = ffmpeg.input("pipe:", r=rate, f="jpeg_pipe").output(video_path, vcodec="libx264").overwrite_output().run_async(pipe_stdin=True)
    
    images = [ os.path.realpath(os.path.join(image_dir, img)) for img in sorted(os.listdir(image_dir)) ]
    for image in images:
        with open(image, "rb") as f:
            jpg_data = f.read()
            process.stdin.write(jpg_data)
    
    process.stdin.close()
    process.wait()


def plot_tracking_performance_spatial(poses_current, poses_desired, plot_path):

    fig, (ax) = plt.subplots(1, 1)

    ax.plot(poses_current[0,0], poses_current[0,1], marker="^", markersize=12.0, label="Start", color="tab:blue")
    ax.plot(poses_desired[-1,0], poses_desired[-1,1], marker="^", markersize=12.0, label="Goal", color="tab:orange")
    ax.plot(poses_desired[:,0], poses_desired[:,1], marker="o", markersize=8.0, label="Desired trajectory", color="tab:green")
    ax.plot(poses_current[:,0], poses_current[:,1], marker="x", markersize=3.0, label="Actual trajectory", color="tab:purple")

    x_0,x_1 = ax.get_xlim()
    y_0,y_1 = ax.get_ylim()
    x_half_diff = (x_1 - x_0)/2
    y_half_diff = (y_1 - y_0)/2
    x_center = (x_0 + x_1)/2
    y_center = (y_0 + y_1)/2

    if x_half_diff > y_half_diff:
        ax.set_ylim(y_center-x_half_diff, y_center+x_half_diff)
    else:
        ax.set_xlim(x_center-y_half_diff, x_center+y_half_diff)

    legend = ax.legend(bbox_to_anchor=(0.5, -0.2), loc="center", ncol=4)
    ax.set_aspect("equal")
    ax.invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
    ax.set_xlabel("x[cm]")
    ax.set_ylabel("y[cm]")
    ax.grid()

    fig.tight_layout()

    plt.savefig(plot_path, bbox_extra_artists=[legend], bbox_inches="tight")


def plot_tracking_performance_temporal(poses_current, poses_desired, plot_path):

    fig, (ax0,ax1,ax2) = plt.subplots(3, 1)

    ax0.plot(poses_desired[:,0], label="Desired x", color="tab:blue")
    ax0.plot(poses_current[:,0], label="Actual x", color="tab:orange")
    ax0.legend(loc="upper left")
    ax0.set_xlabel("iterations")
    ax0.set_ylabel("x[cm]")
    ax0.grid()
    
    ax1.plot(poses_desired[:,1], label="Desired y", color="tab:blue")
    ax1.plot(poses_current[:,1], label="Actual y", color="tab:orange")
    ax1.legend(loc="upper left")
    ax1.set_xlabel("iterations")
    ax1.set_ylabel("y[cm]")
    ax1.grid()
    
    ax2.plot(np.arctan2(poses_desired[:,1] - poses_current[:,1], poses_desired[:,0] - poses_current[:,0]), label="Desired yaw", color="tab:blue")
    ax2.plot(poses_current[:,4], label='Actual yaw', color="tab:orange")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("iterations")
    ax2.set_ylabel('yaw[rad]')
    ax2.grid()
    
    fig.tight_layout()

    plt.savefig(plot_path, bbox_inches="tight")
