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
    parser.add_argument("--episodes_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "train_episodes.csv"))
    parser.add_argument("--dataset_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "dataset"))
    parser.add_argument("--split", default="train")
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--create_plot", action="store_true")
    parser.add_argument("--create_video", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # make sure that we are not in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = False
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.POSITIONS_FILE = args.episodes_file
    config.freeze()
 
    # handle debug configuration (markers are only produed in Developent configuration; NOT in Shipping configuration)
    if args.debug:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 1080
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 1920
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color", "segmentation", "depth_glsl"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera", "encoder", "imu", "sonar"]
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Y = -50.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Z = 45.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.PITCH = -35.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.YAW = 45.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.ROLL = 0.0
        config.freeze()
    else:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 120
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 160
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera"]
        config.freeze()

    # load driving policy
    policy = OpenBotPIDPolicy(config)

    # load the episodes to be executed
    assert os.path.exists(args.episodes_file)
    df = pd.read_csv(args.episodes_file, dtype={"scene_id":str})
    
    # string to load a different map depending on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
    else:
        assert False
        
    # build the episode data folder and its subfolders following the guidelines of the OpenBot public repository 
    # https://github.com/isl-org/OpenBot/tree/master/policy#data-collection
    if not args.benchmark:
        split_dir = os.path.join(args.dataset_dir, args.split + "_data")

    # iterate over all episodes
    prev_scene_id = ""
    for episode in df.to_records():

        print("----------------------")
        print(f"episode {episode['index']} over {df.shape[0]}")
        print("----------------------")
        
        # if the scene_id of our current episode has changed, then create a new Env
        if episode["scene_id"] != prev_scene_id:

            # close the previous Env
            if prev_scene_id != "":
                env.close()              

            # change config based on current scene
            config.defrost()
            config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + episode["scene_id"] + rendering_mode_map_str + "." + "Map_" + episode["scene_id"] + rendering_mode_map_str
            config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + episode["scene_id"] + rendering_mode_map_str
            config.SIMULATION_CONTROLLER.SCENE_ID = episode["scene_id"]
            config.freeze()

            # create Env object
            env = spear.Env(config=config)

            # update scene reference
            prev_scene_id = episode["scene_id"]

        # create dir for storing data
        if not args.benchmark:
            scene_dir = os.path.join(split_dir, episode["scene_id"])
            episode_dir = os.path.join(scene_dir, "%04d" % episode["index"])
            image_dir = os.path.join(episode_dir, "images")
            sensor_dir = os.path.join(episode_dir, "sensor_data")
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(sensor_dir, exist_ok=True)

        # reset the simulation
        _ = env.reset()
        
        # send zero action to the agent and collect initial trajectory observations:
        obs, _, _, env_info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

        # initialize the driving policy with the desired trajectory 
        policy.reset(obs, env_info)

        hit_obstacle        = False # flag raised when the vehicle collides with the environment. 
        num_iterations = 0

        if args.benchmark:
            start_time_seconds = time.time()
        else:                            
            control_data  = np.empty([args.num_iterations_per_episode, 2], dtype=np.float32) # control_data observations made by the agent during a episode
            state_data    = np.empty([args.num_iterations_per_episode, 6], dtype=np.float32) # state_data observations made by the agent during an episode
            waypoint_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float32) # waypoint coordinates being tracked by the agent during an episode
            compass_data  = np.empty([args.num_iterations_per_episode, 3], dtype=np.float32) # compass observations made by the agent during a episode
            time_data     = np.empty([args.num_iterations_per_episode], dtype=np.int32)      # time stamps of the observations made by the agent during an episode
            frame_data    = np.empty([args.num_iterations_per_episode], dtype=np.int32)      # frame ids
            
        # execute the desired number of iterations in a given episode
        for i in range(args.num_iterations_per_episode):

            print(f"iteration {i} of {args.num_iterations_per_episode}")

            time_stamp = int(10000*datetime.datetime.now().timestamp())

            # update control action 
            action, policy_info = policy.step(obs)

            # send control action to the agent and collect observations
            obs, _, _, env_info = env.step({"apply_voltage": action})

            num_iterations = num_iterations + 1

            if not args.benchmark:

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # During an episode, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training purposes. 
                # In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole trajectory. 
                # In this case, rather than considering the whole episode as a fail, one can consider the last position reached by the agent as the new goal position. 
                # Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather than directly writing all the 
                # observations in a file iteration by iteration, we append these observations in a buffer, named "observation" to later process them once the 
                # episode is completed. 
                control_data[i]     = obs["control_data"]               # control_data: [ctrl_left, ctrl_right]
                state_data[i]       = obs["state_data"]                 # state_data: [x, y, z, pitch, yaw, roll]
                waypoint_data[i]    = policy_info["current_waypoint"]   # current waypoint being tracked by the agent
                time_data[i]        = time_stamp                        # current time stamp
                frame_data[i]       = i                                 # current frame

            # debug
            if args.debug:
                show_obs(obs, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

            # termination conditions
            if env_info["task_step_info"]["hit_obstacle"]: 
                print("Collision detected !")
                hit_obstacle = True 
                break
            elif env_info["task_step_info"]["hit_goal"] or policy_info["goal_reached"]: 
                print("Goal reached !")
                break
        
        # episode loop executed: check the termination flags
        
        if hit_obstacle: # if the collision flag is raised during the episode
            if not args.benchmark:
                shutil.rmtree(episode_dir) # remove the collected data as it is improper for training purposes
            continue

        if args.benchmark:
            end_time_seconds = time.time()
            elapsed_time_seconds = end_time_seconds - start_time_seconds
            print("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / num_iterations)*1000, num_iterations / elapsed_time_seconds))
            continue
        
        print("Filling database...")

        # low-level commands sent to the motors
        df_ctrl = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "left_ctrl"     : control_data[:num_iterations, 0],
                                "right_ctrl"    : control_data[:num_iterations, 1]})
        df_ctrl.to_csv(os.path.join(sensor_dir,"ctrlLog.txt"), mode="w", index=False, header=True)

        # reference of the images correespoinding to each control input
        df_rgb = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                            "frame"            : frame_data[:num_iterations]})
        df_rgb.to_csv(os.path.join(sensor_dir,"rgbFrames.txt"), mode="w", index=False, header=True)

        # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
        df_pose = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "x[cm]"         : state_data[:num_iterations, 0],
                                "y[cm]"         : state_data[:num_iterations, 1],
                                "z[cm]"         : state_data[:num_iterations, 2],
                                "pitch[rad]"    : state_data[:num_iterations, 3],
                                "yaw[rad]"      : state_data[:num_iterations, 4],
                                "roll[rad]"     : state_data[:num_iterations, 5]})
        df_pose.to_csv(os.path.join(sensor_dir,"poseData.txt"), mode="w", index=False, header=True)

        # waypoint data (for debug purposes)
        df_waypoint = pd.DataFrame({"timestamp[ns]"  : time_data[:num_iterations],
                                    "waypoint_x[cm]" : waypoint_data[:num_iterations, 0],
                                    "waypoint_y[cm]" : waypoint_data[:num_iterations, 1],
                                    "waypoint_z[cm]" : waypoint_data[:num_iterations, 2]})
        df_waypoint.to_csv(os.path.join(sensor_dir,"waypointData.txt"), mode="w", index=False, header=True)

        # set the goal position as the last position reached by the agent
        goal_position_xy = state_data[num_iterations-1][0:2] # use the vehicle last x-y location as goal for the episode

        for i in range(num_iterations):

            # get the updated compass observation (with the last recorded position set as goal)
            position_xy_current = np.array([state_data[i][0], state_data[i][1]], dtype=np.float32)
            yaw_current = state_data[i][4]
            compass_data[i] = get_compass_observation(goal_position_xy, position_xy_current, yaw_current)

        # high level commands
        df_goal = pd.DataFrame({"timestamp[ns]" : time_data[:num_iterations],
                                "dist[m]"       : compass_data[:num_iterations, 0],
                                "sinYaw"        : compass_data[:num_iterations, 1],
                                "cosYaw"        : compass_data[:num_iterations, 2]})
        df_goal.to_csv(os.path.join(sensor_dir,"goalLog.txt"), mode="a", index=False, header=i==0)

        if args.create_plot: # if desired, generate a plot of the control performance
            plot_tracking_performance(state_data[:num_iterations][:], waypoint_data[:num_iterations][:], os.path.join(sensor_dir, 'tracking_performance.png'))
            plot_control_performance(state_data[:num_iterations][:], waypoint_data[:num_iterations][:], os.path.join(sensor_dir, 'control_performance.png'))

        if args.create_video: # if desired, generate a video from the collected rgb observations 
            video_dir = os.path.join(args.dataset_dir, "videos")
            video_split_dir = os.path.join(video_dir, args.split + "_data")
            os.makedirs(video_split_dir, exist_ok=True)
            generate_video(image_dir, os.path.join(video_split_dir, "%04d.mp4" % episode["index"]), rate=int(1/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS), compress=True)

    # close the current scene
    env.close()
    
    print("Done.")
