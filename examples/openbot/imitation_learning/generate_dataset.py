# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import numpy as np
import csv
import datetime
import time
from openbot_gym.openbot_env import OpenBotEnv
from openbot_gym.openbot_agent import OpenBotAgent


  
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--scenes", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    args = parser.parse_args()
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])
    
    # sanity checks
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

    # main OpenBot class
    agent = OpenBotAgent(config)

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
        while run < args.runs:

            collision_flag = False
            goal_reached_flag = False
            array_obs = np.empty([args.iterations, 18])
            executed_iterations = 0
            index_waypoint = 1

            folder_name = f"dataset/uploaded/run_{mapName}_{run}"
            data_folder_name = folder_name+"/data/"
            os.makedirs(data_folder_name, exist_ok=True)
            os.makedirs(data_folder_name+"sensor_data", exist_ok=True)
            os.makedirs(data_folder_name+"images", exist_ok=True)

            print("----------------------")
            print(f"run {run} over {args.runs}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # Send Zero action to the agent and collect initial trajectory observations:
            # obs["state_data"] = X, Y, Z, Pitch, Yaw, Roll
            # obs["control_data"] = ctrl left, ctrl right
        
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})
        
            ctrl_left = obs["control_data"][0]
            ctrl_right = obs["control_data"][1]
            pos_x = obs["state_data"][0]
            pos_y = obs["state_data"][1]
            pos_z = obs["state_data"][2]
            pitch = obs["state_data"][3]
            yaw = obs["state_data"][4]
            roll = obs["state_data"][5]
            sonar = obs["sonar"]
            imu_ax = obs["imu"][0]
            imu_ay = obs["imu"][1]
            imu_az = obs["imu"][2]
            imu_gx = obs["imu"][3]
            imu_gy = obs["imu"][4]
            imu_gz = obs["imu"][5]
        
            num_waypoints = len(info["agent_step_info"]["trajectory_data"]) - 1
            print(info["agent_step_info"]["trajectory_data"])
            desired_position_xy = np.array([info["agent_step_info"]["trajectory_data"][index_waypoint][0], info["agent_step_info"]["trajectory_data"][index_waypoint][1]]) # [Xdes, Ydes]
            actual_pose_yaw_xy = np.array([yaw, pos_x, pos_y]) 

            # execute the desired number of iterations
            for i in range(args.iterations):

                print(f"iteration {i} over {args.iterations}")

                executed_iterations = i+1
                ct = datetime.datetime.now()
                time_stamp = 10000*ct.timestamp()

                # xy position of the next waypoint in world frame:
                desired_position_xy = np.array([info["agent_step_info"]["trajectory_data"][index_waypoint][0], info["agent_step_info"]["trajectory_data"][index_waypoint][1]]) # [x_des, y_des]

                # current position and heading of the vehicle in world frame:
                current_pose_yaw_xy = np.array([yaw, pos_x, pos_y]) # [yaw, x, y]

                # update control action 
                action, waypoint_reached = agent.update_autopilot(desired_position_xy, current_pose_yaw_xy)

                # send control action to the agent and collect observations
                obs, reward, done, info = env.step({"apply_voltage": action})
            
                ctrl_left = obs["control_data"][0]                                          # control duty cycle of the left weels in [%]
                ctrl_right = obs["control_data"][1]                                         # control duty cycle of the right weels in [%]
                pos_x = obs["state_data"][0]                                                # x component of agent position wrt. world in [cm]
                pos_y = obs["state_data"][1]                                                # y component of agent position wrt. world in [cm]
                pos_z = obs["state_data"][2]                                                # z component of agent position wrt. world in [cm]
                pitch = obs["state_data"][3]                                                # pitch component of agent orientation wrt. world in [rad]
                yaw = obs["state_data"][4]                                                  # yaw component of agent orientation wrt. world in [rad]
                roll = obs["state_data"][5]                                                 # roll component of agent orientation wrt. world in [rad]
                sonar = obs["sonar"]                                                        # sonar measured distance in [m]
                imu_ax = obs["imu"][0]                                                      # x component of linear acceleration in [m/s²] 
                imu_ay = obs["imu"][1]                                                      # y component of linear acceleration in [m/s²] 
                imu_az = obs["imu"][2]                                                      # z component of linear acceleration in [m/s²] 
                imu_gx = obs["imu"][3]                                                      # x component of angular rate in [rad/s] 
                imu_gy = obs["imu"][4]                                                      # y component of angular rate in [rad/s] 
                imu_gz = obs["imu"][5]                                                      # z component of angular rate in [rad/s] 
                pos_x_des = info["agent_step_info"]["trajectory_data"][index_waypoint][0]   # x component of desired agent position wrt. world in [cm]
                pos_y_des = info["agent_step_info"]["trajectory_data"][index_waypoint][1]   # y component of desired agent position wrt. world in [cm]
                
                if pos_z < 0: # For now we don't consider underground operation ! 
                    collision_flag = True
                    break

                # During a run, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training purposes. 
                # In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole trajectory. 
                # In this case, rather than considering the whole run as a fail, one can consider the last position reached by the agent as the new goal position. 
                # Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather than directly writing all the 
                # observations in a file iteration by iteration, we append these observations in a buffer, named "array_obs" to later process them once the run is completed. 
                array_obs[i][0] = ctrl_left
                array_obs[i][1] = ctrl_right
                array_obs[i][2] = pos_x 
                array_obs[i][3] = pos_y 
                array_obs[i][4] = pos_z 
                array_obs[i][5] = roll 
                array_obs[i][6] = pitch 
                array_obs[i][7] = yaw 
                array_obs[i][8] = pos_x_des
                array_obs[i][9] = pos_y_des
                array_obs[i][10] = time_stamp 
                array_obs[i][11] = sonar
                array_obs[i][12] = imu_ax
                array_obs[i][13] = imu_ay
                array_obs[i][14] = imu_az
                array_obs[i][15] = imu_gx
                array_obs[i][16] = imu_gy
                array_obs[i][17] = imu_gz
            
                # save the collected rgb observations:
                im = Image.fromarray(obs["camera_final_color"])
                im.save(data_folder_name+"images/%d.jpeg" % i)
                
                # Check the stop conditions of the run
                
                if waypoint_reached: # if a waypoint of the trajectory is reached, based on the autopilot's acceptance_radius condition 
                    if index_waypoint < num_waypoints: # if the considered waypoint is not the final goal
                        print(f"Waypoint {index_waypoint} over {num_waypoints} reached !")
                        index_waypoint = index_waypoint + 1 # set the next way point as the current target to be tracked by the agent
                    else: # if the considered waypoint is the final goal
                        print("Goal reached (distance check) !")
                        goal_reached_flag = True # raise goal_reached_flag to interrupt the current run and move to the next run
                        break

                if done: # if the done flag is raised:
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
                shutil.rmtree(folder_name) # remove the data collected so far as it is improper for training purposes
                # do not update the run count as the run must be restarted

            else: # as no collision occured during the run, the collected data can be used for training purposes
                if args.create_video: # if desired, generate a video from the collected rgb observations 
                    GenerateVideo(config, mapName, run)
                run = run + 1 # update the run count and move to the next run 

                # Populate the observation data files with the array_obs buffer content
                print("Filling database...")
                f_ctrl = open(data_folder_name+"sensor_data/ctrlLog.txt", 'w')  # low-level commands sent to the motors
                f_goal = open(data_folder_name+"sensor_data/goalLog.txt", 'w')  # high level commands
                f_rgb = open(data_folder_name+"sensor_data/rgbFrames.txt", 'w') # reference of the images correespoinding to each control input
                f_pose = open(data_folder_name+"sensor_data/poseData.txt", 'w') # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)

                writer_ctrl = csv.writer(f_ctrl, delimiter=",")
                writer_ctrl.writerow( ('timestamp[ns]','leftCtrl','right_ctrl') )
                writer_pose = csv.writer(f_pose , delimiter=",")
                writer_pose.writerow( ('timestamp[ns]','posX','posY','posZ','rollAngle','pitchAngle','yawAngle') )
                writer_goal = csv.writer(f_goal, delimiter=",")
                writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )
                writer_rgb = csv.writer(f_rgb, delimiter=",")
                writer_rgb.writerow( ('timestamp[ns]','frame') )

                goal_position_xy = np.array([array_obs[executed_iterations-1][2],array_obs[executed_iterations-1][3]]) # use the vehicle last location as goal
                forward = np.array([1,0]) # Front axis is the X axis.
                forward_rotated = np.array([0,0])

                for i in range(executed_iterations):

                    # get the updated compass observation (with the last recorded position set as goal)
                    current_pose_yaw_xy = np.array([array_obs[i][2], array_obs[i][2], array_obs[i][3]])
                    dist, sin_yaw, cos_yaw = agent.get_compass_observation(goal_position_xy, current_pose_yaw_xy)

                    # write pose data
                    writer_pose.writerow( (int(array_obs[i][10]), array_obs[i][2], array_obs[i][3], array_obs[i][4], array_obs[i][5], array_obs[i][6], array_obs[i][7]) )

                    # wite the low-level control observation into a file:
                    writer_ctrl.writerow( (int(array_obs[i][10]), array_obs[i][0], array_obs[i][1]) )

                    # write the corresponding image index into a file:
                    writer_rgb.writerow( (int(array_obs[i][10]), i) )

                    # write the corresponding high level command into a file:
                    writer_goal.writerow( (int(array_obs[i][10]), dist, sin_yaw, cos_yaw) )

                f_ctrl.close()
                f_pose.close()
                f_goal.close()
                f_rgb.close()

        # close the current scene and give the system a bit of time before switching to the next scene.
        env.close()
        time.sleep(3)

    print("Done.")
