#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

from policies import *
from utils import *


if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_iterations_per_episode", type=int, default=500)
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "train_episodes.csv")))
    parser.add_argument("--dataset_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "dataset")))
    parser.add_argument("--split", default="train")
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--create_videos", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # make sure that we are not in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = False
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.POSITIONS_FILE = os.path.abspath(args.episodes_file)
    config.freeze()
 
    # handle debug configuration (markers are only produed in Developent configuration; NOT in Shipping configuration)
    if args.debug:
        config.defrost()
        config.SPEAR.RENDER_OFFSCREEN = True
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 512
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 512
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["depth", "final_color", "segmentation"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera", "encoder", "imu", "sonar"]

        # aim camera in a third-person vieww facing backwards at an angle
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Y = -50.0
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Z = 45.0
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.PITCH = -35.0
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.YAW = 45.0
        # config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.ROLL = 0.0

        config.freeze()
    else:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 128
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 128
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera"]

        # aim camera in a third-person view facing forward
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Y = 0.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Z = 5.0

        config.freeze()

    # load driving policy
    policy = OpenBotPIDPolicy(config)

    # load the episodes to be executed
    assert os.path.exists(args.episodes_file)
    df = pd.read_csv(args.episodes_file)
    
    # do some config modifications based on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 1.0
        config.freeze()
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 0.0
        config.freeze()
    else:
        assert False
        
    # clean the episode data folder 
    if not args.benchmark:
        shutil.rmtree(args.dataset_dir, ignore_errors=True) # remove the previous dataset to prevent data corruption
        split_dir = os.path.realpath(os.path.join(args.dataset_dir, args.split + "_data"))

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

            if episode["scene_id"] == "kujiale_0000":
                config.SIMULATION_CONTROLLER.SCENE_ID = episode["scene_id"]
                config.SIMULATION_CONTROLLER.MAP_ID   = episode["scene_id"] + rendering_mode_map_str

                # kujiale_0000 has scene-specific config values
                scene_config_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scene_config.kujiale_0000.yaml"))

            elif episode["scene_id"] == "warehouse_0000":
                config.SIMULATION_CONTROLLER.SCENE_ID = episode["scene_id"]
                config.SIMULATION_CONTROLLER.MAP_ID   = episode["scene_id"]

                # warehouse_0000 has scene-specific config values
                scene_config_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scene_config.warehouse_0000.yaml"))

            else:
                assert False

            config.merge_from_file(scene_config_file)
            config.freeze()

            # create Env object
            env = spear.Env(config=config)
                
        # reset the simulation
        env_reset_info = {}
        _ = env.reset(reset_info=env_reset_info)
        assert "success" in env_reset_info

        # if it took too long to reset the simulation, then continue
        if not env_reset_info["success"]:
            spear.log("Call to env.reset(...) was not successful. Simulation took too long to return to a ready state. Skipping...")
            prev_scene_id = episode["scene_id"]
            continue

        # send zero action to the agent and collect initial trajectory observations:
        obs, _, _, env_step_info = env.step(action={"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

        # initialize the driving policy with the desired trajectory 
        policy.reset(obs, env_step_info)

        if args.benchmark:
            start_time_seconds = time.time()
        else:     
            # build the episode data folder and its subfolders following the guidelines of the OpenBot public repository 
            # https://github.com/isl-org/OpenBot/tree/master/policy#data-collection
            scene_dir = os.path.realpath(os.path.join(split_dir, episode["scene_id"]))
            episode_dir = os.path.realpath(os.path.join(scene_dir, "%04d" % episode["index"]))
            image_dir = os.path.realpath(os.path.join(episode_dir, "images"))
            sensor_dir = os.path.realpath(os.path.join(episode_dir, "sensor_data"))
            plots_dir = os.path.realpath(os.path.join(episode_dir, "plots"))
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(sensor_dir, exist_ok=True)
            os.makedirs(plots_dir, exist_ok=True)

            control_data  = np.empty([args.num_iterations_per_episode, 2], dtype=np.float32) # control_data observations made by the agent during a episode
            state_data    = np.empty([args.num_iterations_per_episode, 6], dtype=np.float32) # state_data observations made by the agent during an episode
            waypoint_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float32) # waypoints being tracked by the agent during an episode
            compass_data  = np.empty([args.num_iterations_per_episode, 3], dtype=np.float32) # compass observations made by the agent during a episode
            time_data     = np.empty([args.num_iterations_per_episode], dtype=np.int64)      # time stamps of observations made by the agent during an episode
            frame_data    = np.empty([args.num_iterations_per_episode], dtype=np.int32)      # frame ids

        # execute the desired number of iterations in a given episode
        num_iterations = 0
        hit_obstacle = False
        for i in range(args.num_iterations_per_episode):

            spear.log(f"Iteration {i} of {args.num_iterations_per_episode}")

            time_stamp = int(10000*datetime.datetime.now().timestamp())

            # update control action 
            action, policy_step_info = policy.step(obs)

            # send control action to the agent and collect observations
            obs, _, _, env_step_info = env.step(action={"apply_voltage": action})

            num_iterations = num_iterations + 1

            if not args.benchmark:

                obs_final_color = obs["camera.final_color"]
                assert len(obs_final_color.shape) == 3
                assert obs_final_color.shape[2] == 4
                obs_final_color = obs_final_color[:,:,[2,1,0,3]].copy() # note that spear.Env returns BGRA by default

                # save the collected rgb observations
                plt.imsave(os.path.realpath(os.path.join(image_dir, "%04d.jpg"%i)), obs_final_color)

                # During an episode, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training
                # purposes. In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole
                # trajectory. In this case, rather than considering the whole episode as a fail, one can consider the last position reached by the agent as
                # the new goal position. Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather
                # than directly writing all the observations in a file iteration by iteration, we append these observations in a buffer, named "observation"
                # to later process them once the episode is completed. 
                control_data[i]     = obs["control_data"]                  # control_data: [ctrl_left, ctrl_right]
                state_data[i]       = obs["state_data"]                    # state_data: [x, y, z, pitch, yaw, roll]
                waypoint_data[i]    = policy_step_info["current_waypoint"] # current waypoint being tracked by the agent
                time_data[i]        = time_stamp                           # current time stamp
                frame_data[i]       = i                                    # current frame

            # debug
            if args.debug:
                show_obs(
                    obs, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

            # termination conditions
            if env_step_info["task_step_info"]["hit_obstacle"]: 
                spear.log("Collision detected.")
                hit_obstacle = True 
                break
            elif env_step_info["task_step_info"]["hit_goal"] or policy_step_info["goal_reached"]: 
                spear.log("Goal reached.")
                break

        # episode loop executed: update scene reference
        prev_scene_id = episode["scene_id"]
        
        # print statistics
        if args.benchmark:
            end_time_seconds = time.time()
            elapsed_time_seconds = end_time_seconds - start_time_seconds
            spear.log("Average frame time: %0.4f ms (%0.4f fps)" %
                ((elapsed_time_seconds / num_iterations)*1000, num_iterations / elapsed_time_seconds))
            continue
        
        # check the termination flags
        if hit_obstacle and not args.benchmark: # if the collision flag is raised during the episode
            shutil.rmtree(episode_dir, ignore_errors=True) # remove the collected data as it is improper for training purposes
            continue
        
        spear.log("Filling CSV files...")
        
        # get the updated compass observation (with the last recorded position set as goal)
        goal_position_xy = state_data[num_iterations-1][0:2]
        for i in range(num_iterations):
            position_xy_current = np.array([state_data[i][0], state_data[i][1]], dtype=np.float32)
            yaw_current = state_data[i][4]
            compass_data[i] = get_compass_observation(goal_position_xy, position_xy_current, yaw_current)

        # low-level commands sent to the motors
        df_ctrl = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "left_ctrl"     : control_data[:num_iterations, 0],
                                "right_ctrl"    : control_data[:num_iterations, 1]})
        df_ctrl.to_csv(os.path.realpath(os.path.join(sensor_dir, "ctrlLog.txt")), mode="w", index=False, header=True)

        # reference of the images correespoinding to each control input
        df_rgb = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                               "frame"         : frame_data[:num_iterations]})
        df_rgb.to_csv(os.path.realpath(os.path.join(sensor_dir, "rgbFrames.txt")), mode="w", index=False, header=True)

        # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
        df_pose = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "x[cm]"         : state_data[:num_iterations, 0],
                                "y[cm]"         : state_data[:num_iterations, 1],
                                "z[cm]"         : state_data[:num_iterations, 2],
                                "pitch[rad]"    : state_data[:num_iterations, 3],
                                "yaw[rad]"      : state_data[:num_iterations, 4],
                                "roll[rad]"     : state_data[:num_iterations, 5]})
        df_pose.to_csv(os.path.realpath(os.path.join(sensor_dir, "poseData.txt")), mode="w", index=False, header=True)

        # waypoint data (for debug purposes)
        df_waypoint = pd.DataFrame({"timestamp[ns]"  : time_data[:num_iterations],
                                    "waypoint_x[cm]" : waypoint_data[:num_iterations, 0],
                                    "waypoint_y[cm]" : waypoint_data[:num_iterations, 1],
                                    "waypoint_z[cm]" : waypoint_data[:num_iterations, 2]})
        df_waypoint.to_csv(os.path.realpath(os.path.join(sensor_dir, "waypointData.txt")), mode="w", index=False, header=True)

        # high level commands
        df_goal = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "dist[m]"       : compass_data[:num_iterations, 0],
                                "sinYaw"        : compass_data[:num_iterations, 1],
                                "cosYaw"        : compass_data[:num_iterations, 2]})
        df_goal.to_csv(os.path.realpath(os.path.join(sensor_dir, "goalLog.txt")), mode="w", index=False, header=True)

        # Create plots. Note that creating these plots will resize our cv2 windows in an
        # unpleasant way, so we only generate these plots if we're not in debug mode.
        if not args.debug:
            plot_tracking_performance_spatial(
                state_data[:num_iterations][:], waypoint_data[:num_iterations][:], os.path.realpath(os.path.join(plots_dir, "tracking_performance_spatial.png")))
            plot_tracking_performance_temporal(
                state_data[:num_iterations][:], waypoint_data[:num_iterations][:], os.path.realpath(os.path.join(plots_dir, "tracking_performance_temporal.png")))

        if args.create_videos: # if desired, generate a video from the collected RGB observations 
            video_dir = os.path.realpath(os.path.join(args.dataset_dir, "videos"))
            video_split_dir = os.path.realpath(os.path.join(video_dir, args.split + "_data"))
            os.makedirs(video_split_dir, exist_ok=True)
            generate_video(
                image_dir,
                os.path.realpath(os.path.join(video_split_dir, "%04d.mp4" % episode["index"])),
                rate=int(1.0/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS), compress=True)

    # close the current scene
    env.close()
    
    spear.log("Done.")
