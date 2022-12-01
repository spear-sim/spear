# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear
import time

from openbot_spear.policies import OpenBotPilotNet
from openbot_spear.utils import *
  
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-p", "--policy", type=str, help="name of the control policy to be executed", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--scene_id", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    args = parser.parse_args()
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # sanity checks (without these observation modes, the code will not behave properly)
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

    # load the control policy
    base_dir = os.path.dirname(os.path.dirname(__file__))
    video_dir = os.path.join(base_dir, "videos")
    eval_dir = os.path.join(base_dir, "evaluation")
    model_dir = os.path.join(base_dir, "models")
    policy_tflite_file = os.path.join(model_dir, args.policy + ".tflite")
    assert os.path.exists(policy_tflite_file)
    config.defrost()
    config.DRIVING_POLICY.PILOT_NET.PATH = "./models/" + args.policy + ".tflite"
    config.freeze()
    
    # load driving policy
    driving_policy = OpenBotPilotNet(config)
    
    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is [""]:
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
        while run < args.runs:

            collision_flag = False # flag raised when the vehicle collides with the environment. It restarts the run without iterating the run count
            goal_reached_flag = False # flag raised when the vehicle get close enouth to the goal (i.e. the last waypoint).
            index_waypoint = 1 # initialized to 1 as waypoint with index 0 refers to the agent initial position
            result = np.empty((0, 2), dtype=float)

            # build the evauation run data folder and its subfolders
            exp_dir = f"run_{scene_id}_{run}"
            run_dir = os.path.join(eval_dir, exp_dir)
            data_dir = os.path.join(run_dir,"data")
            image_dir = os.path.join(data_dir, "images")
            result_dir = os.path.join(data_dir, "results")
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(result_dir, exist_ok=True)

            # result file
            f_result = open(os.path.join(result_dir,"resultLog.txt"), 'w')  
            writer_result = csv.writer(f_result, delimiter=",")
            writer_result.writerow( ('timestamp[ns]','left_ctrl','right_ctrl','x[cm]','y[cm]','z[cm]','pitch[rad]','yaw[rad]','roll[rad]','goal_x[cm]','goal_y[cm]','goal_z[cm]', 'goal_reached', 'collision') )

            print("----------------------")
            print(f"run {run} over {args.runs}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # send zero action to the agent and collect initial trajectory observations:
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            executed_iterations = 0
            # execute the desired number of iterations in a given run
            for i in range(args.iterations):

                print(f"iteration {i} over {args.iterations}")

                executed_iterations = i+1
                ct = datetime.datetime.now()
                time_stamp = int(10000*ct.timestamp())

                # xy position of the goal in world frame (not the next waypoint):
                desired_position_xy = np.array([info["agent_step_info"]["trajectory_data"][-1][0], info["agent_step_info"]["trajectory_data"][-1][1]], dtype=np.float32) # [x_des, y_des]

                # update control action 
                action, goal_reached_flag = driving_policy.update(desired_position_xy, obs)

                # send control action to the agent and collect observations
                obs, reward, done, info = env.step({"apply_voltage": action})

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # check the stop conditions of the run
                if done: # if the done flag is raised
                    if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                        print("Collision detected !")
                        collision_flag = True # raise collision_flag to interrupt and restart the current run
                    if info["task_step_info"]["hit_goal"]: # if the vehicle collided with the goal
                        goal_reached_flag = True # raise goal_reached_flag to interrupt the current run and move to the next run

                # populate result data file
                writer_result.writerow( (time_stamp, action[0], action[1], obs["state_data"][0], obs["state_data"][1], obs["state_data"][2], obs["state_data"][3], obs["state_data"][4], obs["state_data"][5], info["agent_step_info"]["trajectory_data"][-1][0], info["agent_step_info"]["trajectory_data"][-1][1], info["agent_step_info"]["trajectory_data"][-1][2], goal_reached_flag, collision_flag) )  

                # termination condition
                if goal_reached_flag:
                    print("Goal reached !")
                    break # only break when the goal is reached 

            f_result.close()

            if args.create_video: # if desired, generate a video from the collected rgb observations 
                video_name = scene_id + str(run)
                generate_video(config, video_name, image_dir, video_dir)

            run = run + 1 # update the run count and move to the next run 

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()
        time.sleep(3)

    print("Done.")
