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
    parser.add_argument("-c", "--control_policy", type=str, help="name of the control policy to be executed", required=True)
    parser.add_argument("-d", "--debug", action="store_true", help="debug flag to display the raw observations.")
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-p", "--create_plot", action="store_true", help="generate a set of plots to assess the performance of the control policy.")
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--scene_id", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    args = parser.parse_args()

    # build the required folders
    base_dir = os.path.dirname(os.path.dirname(__file__))
    model_dir = os.path.join(base_dir, "models")
    eval_dir = os.path.join(base_dir, "eval")
    video_dir = os.path.join(eval_dir, "videos")
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

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
    if args.scene_id == [""]:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = args.scene_id
    
    # loop through the desired set of scenes
    for scene_id in scene_ids: 
        
        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + "." + "Map_" + scene_id
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id
        config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
        config.freeze()

        # create Env object
        env = spear.Env(config=config)
        
        run = 0
        # execute the desired number of runs in a given sceen
        for run in range(args.runs):

            # build the evauation run data folder and its subfolders
            exp_dir = f"run_{run}_{scene_id}"
            run_dir = os.path.join(eval_dir, exp_dir)
            data_dir = os.path.join(run_dir,"data")
            image_dir = os.path.join(data_dir, "images")
            result_dir = os.path.join(data_dir, "results")
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(result_dir, exist_ok=True)

            print("----------------------")
            print(f"run {run} over {args.runs}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # send zero action to the agent and collect initial trajectory observations:
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            # initialize the driving policy with the desired trajectory 
            driving_policy.set_trajectory(info["agent_step_info"]["trajectory_data"])

            # execute the desired number of iterations in a given run
            for i in range(args.iterations):

                print(f"iteration {i} of {args.iterations}")

                time_stamp = int(10000*datetime.datetime.now().timestamp())

                # update control action 
                action, policy_step_info = driving_policy.update(obs)

                # send control action to the agent and collect observations
                obs, reward, done, info = env.step({"apply_voltage": action})

                # debug
                if args.debug:
                    show_obs_and_wait_for_key(obs, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

                # check the stop conditions of the run
                if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                    print("Collision detected !")
                    break
                if info["task_step_info"]["hit_goal"] or policy_step_info["goal_reached"]: # if the vehicle reached the goal
                    print("Goal reached !")
                    break 

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # populate result data file
                df_result = pd.DataFrame({"timestamp[ns]"  : time_stamp,
                           "left_ctrl" : action[0],
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
                video_name = str(run) + "_" + scene_id
                generate_video(config, video_name, image_dir, video_dir, True)
            
            if args.create_plot: # if desired, generate a plot of the control performance
                plot_tracking_performance(state_data_buffer, waypoint_data_buffer, run_dir)

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()

    print("Done.")
