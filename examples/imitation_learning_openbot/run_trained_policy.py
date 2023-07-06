#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import time

from policies import *
import spear
from utils import *
  
# hack to import OpenBotEnv
import sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from getting_started.OpenBotEnv import OpenBotEnv


if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_iterations_per_episode", type=int, default=500)
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "test_episodes.csv")))
    parser.add_argument("--policy_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "trained_policy.tflite")))
    parser.add_argument("--eval_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "eval")))
    parser.add_argument("--create_videos", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    
    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # make sure that we are loading trajectories from a file
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.LOAD_TRAJECTORY_FROM_FILE = True
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_LOCATIONS_FILE = os.path.abspath(args.episodes_file)
    config.freeze()

    # handle debug configuration (markers are only produed in Developent configuration; NOT in Shipping configuration)
    if args.debug:
        config.defrost()
        config.SIMULATION_CONTROLLER.NAVMESH.TRAJECTORY_SAMPLING_DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT = 1080
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH = 1920
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.RENDER_PASSES = ["depth", "final_color", "segmentation"]
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS = ["camera", "imu", "location", "rotation", "wheel_encoder"]
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Y = -50.0
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Z = 45.0
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.PITCH = -35.0
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.YAW = 45.0
        config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROLL = 0.0
        config.freeze()
    else:
        config.defrost()
        config.SIMULATION_CONTROLLER.NAVMESH.TRAJECTORY_SAMPLING_DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT = 120
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH = 160
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.RENDER_PASSES = ["final_color"]
        config.SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS = ["camera", "location", "rotation", "wheel_encoder"]
        config.freeze()

    # load driving policy
    assert os.path.exists(args.policy_file)
    config.defrost()
    config.IMITATION_LEARNING_OPENBOT.PILOT_NET.PATH = args.policy_file
    config.freeze()
    policy = OpenBotPilotNetPolicy(config)

    # load the episodes to be executed
    assert os.path.exists(args.episodes_file)
    df = pd.read_csv(args.episodes_file)

    # clean the episode data folder 
    if not args.benchmark:
        shutil.rmtree(args.eval_dir, ignore_errors=True) # remove the previous dataset to prevent data corruption
    
    # iterate over all episodes
    prev_scene_id = ""
    for episode in df.to_records():

        spear.log("----------------------")
        spear.log(f"Episode {episode['index']} of {df.shape[0]}")
        spear.log("----------------------")

        # if the scene_id of our current episode has changed, then create a new Env
        if episode["scene_id"] != prev_scene_id:

            # close the previous Env
            if prev_scene_id != "":
                env.close()              

            # change config based on current scene
            config.defrost()
            config.SIMULATION_CONTROLLER.SCENE_ID = episode["scene_id"]
            config.freeze()

            # create Env object
            env = OpenBotEnv(config=config)

        # reset the simulation
        env_reset_info = {}
        obs = env.reset(reset_info=env_reset_info)
        assert "success" in env_reset_info

        # get a trajectory for this episode based on start and end point
        episode_start_location = [[episode["start_location_x"], episode["start_location_y"], episode["start_location_z"]]]
        episode_goal_location  = [[episode["goal_location_x"], episode["goal_location_y"], episode["goal_location_z"]]]
        trajectory = env.get_trajectories(episode_start_location, episode_goal_location)

        if args.benchmark:
            start_time_seconds = time.time()
        else:
            # create dirs for storing data
            scene_dir   = os.path.realpath(os.path.join(args.eval_dir, episode["scene_id"]))
            episode_dir = os.path.realpath(os.path.join(scene_dir, "%04d" % episode["index"]))
            image_dir   = os.path.realpath(os.path.join(episode_dir, "images"))
            result_dir  = os.path.realpath(os.path.join(episode_dir, "results"))
            plots_dir   = os.path.realpath(os.path.join(episode_dir, "plots"))
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(result_dir, exist_ok=True)
            os.makedirs(plots_dir, exist_ok=True)

            # buffer containing the state_data observations made by the agent during an episode
            state_data = np.empty([args.num_iterations_per_episode, 6], dtype=np.float64)

            # save the optimal goal trajectory in a dedicated file
            df_trajectory = pd.DataFrame({"x_d[cm]" : trajectory[0][:,0],
                                          "y_d[cm]" : trajectory[0][:,1],
                                          "z_d[cm]" : trajectory[0][:,2]})
            df_trajectory.to_csv(os.path.realpath(os.path.join(result_dir,"trajectoryLog.txt")), mode="w", index=False, header=True)

        # execute the desired number of iterations in a given episode
        num_iterations = 0
        goal = np.array([episode["goal_location_x"], episode["goal_location_y"], episode["goal_location_z"]], dtype=np.float64) # goal position
        for i in range(args.num_iterations_per_episode):

            spear.log(f"Iteration {i} of {args.num_iterations_per_episode}")

            # update control action 
            action, policy_step_info = policy.step(obs, goal[0:2])

            # send control action to the agent and collect observations
            obs, _, _, env_step_info = env.step(action={"set_duty_cycles": action})

            num_iterations = num_iterations + 1
            
            if not args.benchmark:

                obs_final_color = obs["camera.final_color"]
                assert len(obs_final_color.shape) == 3
                assert obs_final_color.shape[2] == 4
                obs_final_color = obs_final_color[:,:,[2,1,0,3]].copy() # note that spear.Env returns BGRA by default

                # save the collected rgb observations
                plt.imsave(os.path.realpath(os.path.join(image_dir, "%04d.jpg"%i)), obs_final_color)

                # populate buffer and result data file
                state_data[i] = np.concatenate((obs["location"], np.deg2rad(obs["rotation"])), axis=None)
                df_result = pd.DataFrame({"left_ctrl"    : action[0],
                                          "right_ctrl"   : action[1],
                                          "x[cm]"        : state_data[i][0],
                                          "y[cm]"        : state_data[i][1],
                                          "z[cm]"        : state_data[i][2],
                                          "pitch[rad]"   : state_data[i][3],
                                          "yaw[rad]"     : state_data[i][4],
                                          "roll[rad]"    : state_data[i][5],
                                          "goal_x[cm]"   : goal[0],
                                          "goal_y[cm]"   : goal[1],
                                          "goal_z[cm]"   : goal[2],
                                          "goal_reached" : policy_step_info["goal_reached"],
                                          "hit_obstacle" : env_step_info["task_step_info"]["hit_obstacle"]})
                df_result.to_csv(os.path.realpath(os.path.join(result_dir,"resultLog.txt")), mode="a", index=False, header=i==0)
            
            # debug
            if args.debug:
                show_obs(
                    obs,
                    config.SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS,
                    config.SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.RENDER_PASSES)
            
            # termination conditions
            if env_step_info["task_step_info"]["hit_obstacle"]: 
                spear.log("Collision detected.") # let the agent collide with the environment for evaluation purposes 
            elif env_step_info["task_step_info"]["hit_goal"] or policy_step_info["goal_reached"]: 
                spear.log("Goal reached.")
                break

        # update scene reference
        prev_scene_id = episode["scene_id"]

        # print statistics
        if args.benchmark:
            end_time_seconds = time.time()
            elapsed_time_seconds = end_time_seconds - start_time_seconds
            spear.log("Average frame time: %0.4f ms (%0.4f fps)" %
                ((elapsed_time_seconds / num_iterations)*1000.0, num_iterations / elapsed_time_seconds))
            continue

        # create plots
        plot_tracking_performance_spatial(
            state_data[:num_iterations][:],
            trajectory[0],
            os.path.realpath(os.path.join(plots_dir, "tracking_performance_spatial.png")))

        if args.create_videos: # if desired, generate a video from the collected rgb observations 
            video_dir = os.path.realpath(os.path.join(args.eval_dir, "videos"))
            os.makedirs(video_dir, exist_ok=True)
            generate_video(
                image_dir,
                os.path.realpath(os.path.join(video_dir, "%04d.mp4" % episode["index"])),
                rate=int(1.0/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS), compress=True)
        
    # close the current scene
    env.close()

    spear.log("Done.")
