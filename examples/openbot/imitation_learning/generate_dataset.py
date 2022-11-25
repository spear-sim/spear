# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import spear
import time

from openbot_spear.env import OpenBotEnv
from openbot_spear.policies import OpenBotPID
from openbot_spear.utils import *
  
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--scenes", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    args = parser.parse_args()
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # load driving policy
    driving_policy = OpenBotPID(config)
    
    # sanity checks (without these observation modes, the code will not behave properly)
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

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
            control_data_buffer = np.empty([args.iterations, 2]) # buffer containing the control_data observations made by the agent during a run
            state_data_buffer = np.empty([args.iterations, 6]) # buffer containing the state_data observations made by the agent during a run
            waypoint_data_buffer = np.empty([args.iterations, 3]) # buffer containing the waypoint coordinates being tracked by the agent during a run
            time_data_buffer = np.empty([args.iterations, 1]) # buffer containing the time stamps of the observations made by the agent during a run
            index_waypoint = 1 # initialized to 1 as waypoint with index 0 refers to the agent initial position

            # build the run data folder and its subfolders following the guidelines of the OpenBot public repository 
            # https://github.com/isl-org/OpenBot/tree/master/policy#data-collection

            base_dir = os.path.dirname(os.path.dirname(__file__))
            dataset_dir = os.path.join(base_dir, "dataset")
            upload_dir = os.path.join(dataset_dir, "uploaded")
            train_data_dir = os.path.join(dataset_dir, "train_data")
            test_data_dir = os.path.join(dataset_dir, "test_data")
            exp_dir = f"run_{scene_id}_{run}"
            
            # split data between training and evaluation sets in a 80%-20% ratio as suggested in the OpenBot public repo
            if 100 * run < 80 * args.runs:
                run_dir = os.path.join(train_data_dir, exp_dir) 
            else:
                run_dir = os.path.join(test_data_dir, exp_dir) 
                
            data_dir = os.path.join(run_dir,"data")
            image_dir = os.path.join(data_dir, "images")
            sensor_dir = os.path.join(data_dir, "sensor_data")
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(sensor_dir, exist_ok=True)

            print("----------------------")
            print(f"run {run} over {args.runs}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # send zero action to the agent and collect initial trajectory observations:
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})
        
            num_waypoints = len(info["agent_step_info"]["trajectory_data"]) - 1

            executed_iterations = 0
            # execute the desired number of iterations in a given run
            for i in range(args.iterations):

                print(f"iteration {i} over {args.iterations}")

                executed_iterations = i+1
                ct = datetime.datetime.now()
                time_stamp = 10000*ct.timestamp()

                # xy position of the next waypoint in world frame:
                desired_position_xy = np.array([info["agent_step_info"]["trajectory_data"][index_waypoint][0], info["agent_step_info"]["trajectory_data"][index_waypoint][1]], dtype=np.float32) # [x_des, y_des]

                # update control action 
                action, waypoint_reached = agent.driving_policy.update(desired_position_xy, obs)

                # send control action to the agent and collect observations
                obs, reward, done, info = env.step({"apply_voltage": action})

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # During a run, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training purposes. 
                # In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole trajectory. 
                # In this case, rather than considering the whole run as a fail, one can consider the last position reached by the agent as the new goal position. 
                # Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather than directly writing all the 
                # observations in a file iteration by iteration, we append these observations in a buffer, named "observation_buffer" to later process them once the run is completed. 
                control_data_buffer[i] = obs["control_data"]    # control_data: [ctrl_left, ctrl_right]
                state_data_buffer[i] = obs["state_data"]        # state_data: [x, y, z, pitch, yaw, roll]
                waypoint_data_buffer[i] = info["agent_step_info"]["trajectory_data"][index_waypoint] # current waypoint being tracked by the agent
                time_data_buffer[i] = time_stamp                # current time stamp

                pos_x_des = info["agent_step_info"]["trajectory_data"][index_waypoint][0]   # x component of desired agent position wrt. world in [cm]
                pos_y_des = info["agent_step_info"]["trajectory_data"][index_waypoint][1]   # y component of desired agent position wrt. world in [cm]

                # check the stop conditions of the run
                if waypoint_reached: # if a waypoint of the trajectory is reached, based on the autopilot's acceptance_radius condition 
                    if index_waypoint < num_waypoints: # if the considered waypoint is not the final goal
                        print(f"Waypoint {index_waypoint} over {num_waypoints} reached !")
                        index_waypoint = index_waypoint + 1 # set the next way point as the current target to be tracked by the agent
                    else: # if the considered waypoint is the final goal
                        print("Goal reached (distance check) !")
                        goal_reached_flag = True # raise goal_reached_flag to interrupt the current run and move to the next run
                        break

                if done: # if the done flag is raised
                    if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                        print("Collision detected ! Killing simulation and restarting run...")
                        collision_flag = True # raise collision_flag to interrupt and restart the current run
                    if info["task_step_info"]["hit_goal"]: # if the vehicle collided with the goal
                        print("Goal reached (collision check)  !")
                        goal_reached_flag = True # raise goal_reached_flag to interrupt the current run and move to the next run
                    break
            
            # run loop executed: check the termination flags
            
            if collision_flag == True: # if the collision flag is raised during the run
                print("Restarting run...")
                shutil.rmtree(run_dir) # remove the data collected so far as it is improper for training purposes
                # do not update the run count as the run must be restarted

            else: # as no collision occured during the run, the collected data can be used for training purposes

                # populate the observation data files with the observation_buffer buffer content
                print("Filling database...")

                # low-level commands sent to the motors
                f_ctrl = open(os.path.join(sensor_dir,"ctrlLog.txt"), 'w')  
                writer_ctrl = csv.writer(f_ctrl, delimiter=",")
                writer_ctrl.writerow( ('timestamp[ns]','left_ctrl','right_ctrl') )

                # high level commands
                f_goal = open(os.path.join(sensor_dir,"goalLog.txt"), 'w')  
                writer_goal = csv.writer(f_goal, delimiter=",")
                writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )

                # reference of the images correespoinding to each control input
                f_rgb = open(os.path.join(sensor_dir,"rgbFrames.txt"), 'w') 
                writer_rgb = csv.writer(f_rgb, delimiter=",")
                writer_rgb.writerow( ('timestamp[ns]','frame') )

                # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
                f_pose = open(os.path.join(sensor_dir,"poseData.txt"), 'w') 
                writer_pose = csv.writer(f_pose , delimiter=",")
                writer_pose.writerow( ('timestamp[ns]','x[cm]','y[cm]','z[cm]','pitch[rad]','yaw[rad]','roll[rad]') )

                # waypoint data (for debug purposes)
                f_waypoint = open(os.path.join(sensor_dir,"waypointData.txt"), 'w') 
                writer_waypoint = csv.writer(f_waypoint , delimiter=",")
                writer_waypoint.writerow( ('timestamp[ns]', 'waypoint_x[cm]','waypoint_y[cm]','waypoint_z[cm]') )

                # set the goal position as the last position reached by the agent
                goal_position_xy = np.array([state_data_buffer[executed_iterations-1][0],state_data_buffer[executed_iterations-1][1]], dtype=np.float32) # use the vehicle last x-y location as goal for the run

                for i in range(executed_iterations):

                    # get the updated compass observation (with the last recorded position set as goal)
                    current_pose_yaw_xy = np.array([state_data_buffer[i][4], state_data_buffer[i][0], state_data_buffer[i][1]], dtype=np.float32)
                    compass_observation = get_compass_observation(goal_position_xy, current_pose_yaw_xy)

                    # wite the low-level control observation into a file:
                    writer_ctrl.writerow( (int(time_data_buffer[i]), control_data_buffer[i][0], control_data_buffer[i][1]) )

                    # write the corresponding high level command into a file:
                    writer_goal.writerow( (int(time_data_buffer[i]), compass_observation[0], compass_observation[1], compass_observation[2]) )

                    # write the corresponding image index into a file:
                    writer_rgb.writerow( (int(time_data_buffer[i]), i) )

                    # write pose data
                    writer_pose.writerow( (int(time_data_buffer[i]), state_data_buffer[i][0], state_data_buffer[i][1], state_data_buffer[i][2], state_data_buffer[i][3], state_data_buffer[i][4], state_data_buffer[i][5]) )  

                    # write waypoint data
                    writer_waypoint.writerow( (int(time_data_buffer[i]), waypoint_data_buffer[i][0], waypoint_data_buffer[i][1], waypoint_data_buffer[i][2]) ) 

                f_ctrl.close()
                f_pose.close()
                f_goal.close()
                f_rgb.close()

                if args.create_video: # if desired, generate a video from the collected rgb observations 
                    video_name = scene_id + run
                    generate_video(config, video_name, image_dir, video_dir)

                run = run + 1 # update the run count and move to the next run 

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()
        time.sleep(3)

    print("Done.")
