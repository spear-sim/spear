#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import cv2
import ffmpeg
import matplotlib.pyplot as plt
import numpy as np
import os
import spear


def show_obs(obs):

    for obs_component in obs.keys():
        if obs_component == "location":
            spear.log(f"location_data: xyz [{obs['location'][0]:.2f}, {obs['location'][1]:.2f}, {obs['location'][2]:.2f}]")
        elif obs_component == "rotation":
            spear.log(f"rotation_data: xyz [{obs['rotation'][0]:.2f}, {obs['rotation'][1]:.2f}, {obs['rotation'][2]:.2f}]")
        elif obs_component == "wheel_rotation_speeds":
            spear.log(f"wheel_rotation_speeds: fl fr rl rr [{obs['wheel_rotation_speeds'][0]:.2f}, {obs['wheel_rotation_speeds'][1]:.2f}, {obs['wheel_rotation_speeds'][2]:.2f}, {obs['wheel_rotation_speeds'][3]:.2f}]")
        elif obs_component == "imu":
            spear.log(f"imu: linear_acceleration [{obs['imu'][0]:.2f}, {obs['imu'][1]:.2f},{obs['imu'][2]:.2f}]")
            spear.log(f"imu: angular_rate [{obs['imu'][3]:.2f}, {obs['imu'][4]:.2f}, {obs['imu'][5]:.2f}]")
        elif obs_component in ["camera.final_color", "camera.normal", "camera.segmentation"]:
            cv2.imshow(render_pass, obs[obs_component]) # note that spear.Env returns BGRA by default
        elif obs_component in ["camera.depth"]:
            depth = obs[obs_component]
            depth_min = np.min(depth)
            depth_max = np.max(depth)
            depth_normalized = (depth-depth_min) / (depth_max-depth_min)
            cv2.imshow(render_pass, depth_normalized)
        else:
            assert False

    if "camera.depth" in obs.keys() or "camera.final_color" in obs.keys() or "camera.normal" in obs.keys() or "camera.segmentation" in obs.keys():
        cv2.waitKey(0)


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


def plot_tracking_performance_spatial(actual_locations, desired_locations, plot_path):

    fig, ax = plt.subplots(1, 1)

    ax.plot(actual_locations[0,0], actual_locations[0,1], marker="^", markersize=12.0, label="Start", color="tab:blue")
    ax.plot(desired_locations[-1,0], desired_locations[-1,1], marker="^", markersize=12.0, label="Goal", color="tab:orange")
    ax.plot(desired_locations[:,0], desired_locations[:,1], marker="o", markersize=8.0, label="Desired trajectory", color="tab:green")
    ax.plot(actual_locations[:,0], actual_locations[:,1], marker="x", markersize=3.0, label="Actual trajectory", color="tab:purple")

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


def plot_tracking_performance_temporal(actual_locations, desired_locations, tracked_yaw, plot_path):

    fig, (ax0,ax1,ax2) = plt.subplots(3, 1)

    ax0.plot(desired_locations[:,0], label="Desired x", color="tab:blue")
    ax0.plot(actual_locations[:,0], label="Actual x", color="tab:orange")
    ax0.legend(loc="upper left")
    ax0.set_xlabel("iterations")
    ax0.set_ylabel("x[cm]")
    ax0.grid()
    
    ax1.plot(desired_locations[:,1], label="Desired y", color="tab:blue")
    ax1.plot(actual_locations[:,1], label="Actual y", color="tab:orange")
    ax1.legend(loc="upper left")
    ax1.set_xlabel("iterations")
    ax1.set_ylabel("y[cm]")
    ax1.grid()
    
    ax2.plot(np.arctan2(desired_locations[:,1] - actual_locations[:,1], desired_locations[:,0] - actual_locations[:,0]), label="Desired yaw", color="tab:blue")
    ax2.plot(np.deg2rad(tracked_yaw[:]), label='Actual yaw', color="tab:orange")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("iterations")
    ax2.set_ylabel('yaw[rad]')
    ax2.grid()
    
    fig.tight_layout()

    plt.savefig(plot_path, bbox_inches="tight")


def plot_paths(scene_id, paths, plot_path):

    fig, ax = plt.subplots(1, 1)

    for path in paths:
        ax.plot(path[0,0], path[0,1], "^", markersize=12.0, label="Initial", color="tab:blue", alpha=0.3)
        ax.plot(path[-1,0], path[-1,1], "^", markersize=12.0, label="Goal", color="tab:orange", alpha=0.3)
        ax.plot(path[:,0], path[:,1], "-o", markersize=8.0, label="Desired path", color="tab:green", alpha=0.3)

    handles, labels = fig.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    legend = ax.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0.5, -0.2), loc="center", ncol=3)
    fig.gca().set_aspect("equal")
    fig.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
    ax.set_xlabel("x[cm]")
    ax.set_ylabel("y[cm]")
    ax.grid()
    ax.set_title(f"scene_id: {scene_id}")
    
    plt.savefig(plot_path, bbox_extra_artists=[legend], bbox_inches="tight")
