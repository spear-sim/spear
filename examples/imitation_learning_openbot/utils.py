import cv2
import ffmpeg
import matplotlib.pyplot as plt
import numpy as np
import os
import re

def show_obs(obs, obs_components, render_passes):
        
    for obs_component in obs_components:
        if obs_component == "state_data":
            print(f"State data: xyz [{obs['state_data'][0]:.2f}, {obs['state_data'][1]:.2f},{obs['state_data'][2]:.2f}]")
            print(f"State data: pitch yaw roll [{obs['state_data'][3]:.2f}, {obs['state_data'][4]:.2f},{obs['state_data'][5]:.2f}]")
        elif obs_component == "control_data":
            print(f"Control data: left right [{obs['control_data'][0]:.2f}, {obs['control_data'][1]:.2f}]")
        elif obs_component == "encoder":
            print(f"FL wheel velocity: [{obs['encoder'][0]:.2f}]")
            print(f"FR wheel velocity: [{obs['encoder'][1]:.2f}]")
            print(f"RL wheel velocity: [{obs['encoder'][2]:.2f}]")
            print(f"RR wheel velocity: [{obs['encoder'][3]:.2f}]")
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
                    cv2.imshow("depth", obs["camera_depth_glsl"])
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


def generate_video(image_dir, video_path, rate, compress=False):
    
    print("Generating video from the sequence of observations")

    if compress:
        process = ffmpeg.input('pipe:', r=rate, f='jpeg_pipe').output(video_path, **{'c:v': 'libx264', 'b:v': 8000000}).overwrite_output().run_async(pipe_stdin=True)
    else:
        process = ffmpeg.input('pipe:', r=rate, f='jpeg_pipe').output(video_path, vcodec='libx264').overwrite_output().run_async(pipe_stdin=True)
    
    images = [os.path.join(image_dir, img) for img in os.listdir(image_dir)]
    images.sort(key=lambda f: int(re.sub('\D', '', f)))
    for image in images:
        with open(image, 'rb') as f:
            jpeg_data = f.read()
            process.stdin.write(jpeg_data)
    
    process.stdin.close()
    process.wait()


def plot_tracking_performance(poses_current, poses_desired, plot_path):

    fig0, ax0 = plt.subplots(1, 1)

    current, = ax0.plot(poses_current[:,0], poses_current[:,1], marker='x', markersize=5.0, label='Actual Trajectory', color='tab:blue')
    desired, = ax0.plot(poses_desired[:,0], poses_desired[:,1], marker='o', markersize=8.0, label='Desired Trajectory', color='tab:orange')
    goal, = ax0.plot(poses_desired[-1,0], poses_desired[-1,1], marker='^', markersize=12.0, label='Goal', color='tab:green')
    start, = ax0.plot(poses_current[0,0], poses_current[0,1], marker='^', markersize=12.0, label='Start', color='tab:purple')
    ax0.set_xlabel('x[cm]')
    ax0.set_ylabel('y[cm]')
    ax0.set_title('XY Position tracking')
    ax0.grid()
    ax0.legend((current, desired, goal, start), ('Actual Trajectory', 'Desired Trajectory', 'Goal', 'Start'), loc='upper left')
    
    fig0.tight_layout()
    fig0.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
    
    plt.savefig(plot_path, dpi=fig0.dpi)


def plot_control_performance(poses_current, poses_desired, plot_path):

    fig0, (ax0,ax1,ax2) = plt.subplots(3, 1)

    x, = ax0.plot(poses_current[:,0], label='Actual X', color='tab:blue')
    x_d, = ax0.plot(poses_desired[:,0], label='Desired X', color='tab:orange')
    ax0.set_xlabel('iterations')
    ax0.set_ylabel('x[cm]')
    ax0.set_title('X tracking')
    ax0.grid()
    ax0.legend((x, x_d), ('Actual X', 'Desired X'), loc='upper left')
    
    y, = ax1.plot(poses_current[:,1], label='Actual Y', color='tab:blue')
    y_d, = ax1.plot(poses_desired[:,1], label='Desired Y', color='tab:orange')
    ax1.set_xlabel('iterations')
    ax1.set_ylabel('y[cm]')
    ax1.set_title('Y tracking')
    ax1.grid()
    ax1.legend((y, y_d), ('Actual Y', 'Desired Y'), loc='upper left')
    
    yaw, = ax2.plot(poses_current[:,4], label='Actual Yaw', color='tab:blue')
    yaw_d, = ax2.plot(np.arctan2(poses_desired[:,1] - poses_current[:,1], poses_desired[:,0] - poses_current[:,0]), label='Desired Yaw', color='tab:orange')
    ax2.set_xlabel('iterations')
    ax2.set_ylabel('yaw[rad]')
    ax2.set_title('Yaw tracking')
    ax2.grid()
    ax2.legend((yaw, yaw_d), ('Actual Yaw', 'Desired Yaw'), loc='upper left')
    
    fig0.tight_layout()
    fig0.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
    
    plt.savefig(plot_path, dpi=fig0.dpi)

