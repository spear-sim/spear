# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import spear
import tensorflow as tf
import tflite_runtime.interpreter as tflite
import time

from ..openbot_interface.openbot_env import OpenBotEnv
from ..openbot_interface.openbot_agent import OpenBotAgent
from ..openbot_interface import openbot_utils
  
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-p", "--policy", type=int, help="name of the control policy to be executed", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--scenes", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    args = parser.parse_args()
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])
    
    # sanity checks (without these observation modes, the code will not behave properly)
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

    # main OpenBot class, encapsulating the agent functionalities
    agent = OpenBotAgent(config)

    # load the control policy
    policy_name = "./models/" + args.policy + ".tflite"
    assert os.path.exists(policy_name)
    interpreter = tflite.Interpreter(policy_name)
    interpreter.allocate_tensors()

    # get input and output tensor details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    print(f"Input details of the control policy: {input_details}")
    print(f"Output details of the control policy: {output_details}")

    # the control policy takes two inputs: normalized rgb image and a 1D high-level comand (-1=left, 0=forward, +1=right).
    rgb_input = np.zeros(input_details[0]["shape"], dtype=np.float32)
    cmd_input = np.zeros(input_details[1]["shape"], dtype=np.float32)

    # loop through the desired set of scenes
    for scene_id in args.scenes: 
        
        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + "." + "Map_" + scene_id
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id
        config.freeze()

        # create Env object
        env = OpenBotEnv(config=config)
        
        run = 0
        # execute the desired number of runs in a given sceen
        while run < args.runs:

            collision_flag = False # flag raised when the vehicle collides with the environment. It restarts the run without iterating the run count
            goal_reached_flag = False # flag raised when the vehicle get close enouth to the goal (i.e. the last waypoint).
            index_waypoint = 1 # initialized to 1 as waypoint with index 0 refers to the agent initial position

            # build the evauation run data folder and its subfolders
            run_dir = f"evaluation/{args.policy}/run_{scene_id}_{run}"
            data_dir = run_dir+"/data/"
            image_dir = data_dir+"images"
            result_dir = data_dir+"run_data"
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
                time_stamp = 10000*ct.timestamp()

                # preprocess (normalize + crop) visual observations:
                rgb_input = np.float32(obs["visual_observation"])/255.0 # normalization in the [0.0, 1.0] range
                rgb_input = tf.image.crop_to_bounding_box(rgb_input, tf.shape(rgb_input)[0] - 90, tf.shape(rgb_input)[1] - 160, 90, 160)

                # xy position of the goal in world frame (not the next waypoint):
                desired_position_xy = np.array([info["agent_step_info"]["trajectory_data"][-1][0], info["agent_step_info"]["trajectory_data"][-1][1]], dtype=np.float32) # [x_des, y_des]

                # current position and heading of the vehicle in world frame:
                current_pose_yaw_xy = np.array([obs["state_data"][4], obs["state_data"][0], obs["state_data"][1]], dtype=np.float32) # [yaw, x, y]

                # control inference
                action, goal_reached_flag = agent.update_policy(desired_position_xy, current_pose_yaw_xy)

                interpreter.set_tensor(input_details[0]["index"], np.expand_dims(img_input, axis=0))
                interpreter.set_tensor(input_details[1]["index"], cmd_input)
                start_time = time.time()
                interpreter.invoke()
                stop_time = time.time()
                executionTime = (stop_time - start_time) * 1000
                print('Infererence time: {:.3f}ms'.format(executionTime))

                output = interpreter.get_tensor(output_details[0]["index"])
                act = np.clip(np.concatenate((result, output.astype(float))), -1.0, 1.0)
                action = np.array([act[0][0],act[0][1]])

                # send control action to the agent and collect observations
                obs, reward, done, info = env.step({"apply_voltage": action})

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # check the stop conditions of the run
                if done: # if the done flag is raised
                    if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                        print("Collision detected ! Killing simulation and restarting run...")
                        collision_flag = True # raise collision_flag to interrupt and restart the current run
                    if info["task_step_info"]["hit_goal"]: # if the vehicle collided with the goal
                        print("Goal reached (collision check)  !")
                        goal_reached_flag = True # raise goal_reached_flag to interrupt the current run and move to the next run
                        break # only break when the goal is reached 
            
                # populate result data file
                writer_result.writerow( (int(time_stamp), action[0], action[1], obs["state_data"][0], obs["state_data"][1], obs["state_data"][2], obs["state_data"][3], obs["state_data"][4], obs["state_data"][5], info["agent_step_info"]["trajectory_data"][-1][0], info["agent_step_info"]["trajectory_data"][-1][1], info["agent_step_info"]["trajectory_data"][-1][2], goal_reached_flag, collision_flag) )  

            f_result.close()

            if args.create_video: # if desired, generate a video from the collected rgb observations 
                openbot_utils.generate_video(config, scene_id, run)

            run = run + 1 # update the run count and move to the next run 

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()
        time.sleep(3)

    print("Done.")
