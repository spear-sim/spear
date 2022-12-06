# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear
import time

from policies import OpenBotPilotNetPolicy
from utils import *
  
  
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--control_policy", required=True)
    parser.add_argument("--iterations", default=500)
    parser.add_argument("--episodes_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "train_episodes.csv"))
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--create_plot", action="store_true")
    parser.add_argument("--create_video", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # build the required folders
    base_dir = os.path.dirname(os.path.dirname(__file__))
    model_dir = os.path.join(base_dir, "models")
    eval_dir = os.path.join(base_dir, "eval")
    video_dir = os.path.join(eval_dir, "videos")
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # make sure that we are not in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = False
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.POSITIONS_FILE = args.episodes_file
    config.freeze()

    # string to load a different map depending on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
    else:
        assert False

    # handle debug configuration (markers are only produed in Developent configuration; NOT in Shipping configuration)
    if args.debug:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 1080
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 1920
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color", "segmentation", "depth_glsl"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera", "imu", "sonar"]
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
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH= 160
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera"]
        config.freeze()

    # load driving policy
    policy_tflite_file = os.path.join(model_dir, args.control_policy + ".tflite")
    assert os.path.exists(policy_tflite_file)
    config.defrost()
    config.DRIVING_POLICY.PILOT_NET.PATH = "./models/" + args.control_policy + ".tflite"
    config.freeze()
    driving_policy = OpenBotPilotNetPolicy(config)

    # sanity checks (without these observation modes, the code will not behave properly)
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)
    
    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = [args.scene_id]
    
    # loop through the desired set of scenes
    for i, scene_id in enumerate(scene_ids): 
        
        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + rendering_mode_map_str + "." + "Map_" + scene_id + rendering_mode_map_str
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id + rendering_mode_map_str
        config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
        config.freeze()

        # load the episodes to be executed
        assert os.path.exists(episodes_csv_file)
        episodes_csv_file = pd.read_csv(args.episodes_file, dtype={"scene_id":str})
        number_of_episodes = len(episodes_csv_file)

        # create Env object
        env = spear.Env(config=config)
        
        # execute the desired number of episodes in a given scene
        for episode in range(number_of_episodes):

            # build the evauation episode data folder and its subfolders
            exp_dir = f"episode_{episode}_{scene_id}"
            episode_dir = os.path.join(eval_dir, exp_dir)
            data_dir = os.path.join(episode_dir,"data")
            image_dir = os.path.join(data_dir, "images")
            result_dir = os.path.join(data_dir, "results")
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(result_dir, exist_ok=True)

            print("----------------------")
            print(f"episode {episode} over {args.runs}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # send zero action to the agent and collect initial trajectory observations:
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            # initialize the driving policy with the desired trajectory 
            driving_policy.set_trajectory(info["agent_step_info"]["trajectory_data"])

            state_data_buffer = np.empty([args.iterations, 6], dtype=np.float32) # buffer containing the state_data observations made by the agent during an episode
            waypoint_data_buffer = np.empty([len(info["agent_step_info"]["trajectory_data"]), 3], dtype=np.float32) # buffer containing the waypoint coordinates of the optimal goal trajectory

            # save the optimal goal trajectory in a dedicated file
            for j in range(len(info["agent_step_info"]["trajectory_data"])):
                waypoint_data_buffer[j] = info["agent_step_info"]["trajectory_data"][j]
                df_trajectory = pd.DataFrame({"x_d[cm]" : [info["agent_step_info"]["trajectory_data"][j][0]],
                            "y_d[cm]" : [info["agent_step_info"]["trajectory_data"][j][1]],
                            "z_d[cm]" : [info["agent_step_info"]["trajectory_data"][j][2]]})
                df_trajectory.to_csv(os.path.join(result_dir,"trajectoryLog.txt"), mode="a", index=False, header=j==0)

            # execute the desired number of iterations in a given episode
            for i in range(args.iterations):

                print(f"iteration {i} of {args.iterations}")

                # update control action 
                action, policy_step_info = driving_policy.step(obs)

                # send control action to the agent and collect observations
                obs, _, _, info = env.step({"apply_voltage": action})

                # debug
                if args.debug:
                    show_obs(obs, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

                # check the stop conditions of the episode
                if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                    print("Collision detected !")
                    break
                if info["task_step_info"]["hit_goal"] or policy_step_info["goal_reached"]: # if the vehicle reached the goal
                    print("Goal reached !")
                    break 

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # populate buffer and result data file
                state_data_buffer[i] = obs["state_data"]
                df_result = pd.DataFrame({"left_ctrl" : action[0],
                           "right_ctrl" : action[1],
                           "x[cm]" : obs["state_data"][0],
                           "y[cm]" : obs["state_data"][1],
                           "z[cm]" : obs["state_data"][2],
                           "pitch[rad]" : obs["state_data"][3],
                           "yaw[rad]" : obs["state_data"][4],
                           "roll[rad]" : obs["state_data"][5],
                           "goal_x[cm]" : info["agent_step_info"]["trajectory_data"][-1][0],
                           "goal_y[cm]" : info["agent_step_info"]["trajectory_data"][-1][1],
                           "goal_z[cm]" : info["agent_step_info"]["trajectory_data"][-1][2],
                           "goal_reached": policy_step_info["goal_reached"],
                           "collision" : info["task_step_info"]["hit_obstacle"]})
                df_result.to_csv(os.path.join(result_dir,"resultLog.txt"), mode="a", index=False, header=i==0)

            if args.create_video: # if desired, generate a video from the collected rgb observations 
                os.makedirs(video_dir, exist_ok=True)
                video_name = str(episode) + "_" + scene_id
                generate_video(config, video_name, image_dir, video_dir, True)
            
            if args.create_plot: # if desired, generate a plot of the control performance
                plot_tracking_performance(state_data_buffer, waypoint_data_buffer, result_dir)

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()

    print("Done.")
