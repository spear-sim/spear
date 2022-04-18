#############################################################################

# Quick and dirty script to issue random commands to an OpenBot agent and get
# egocentric visual obsevations.

#############################################################################
import numpy as np
import tensorflow as tf
import tflite_runtime.interpreter as tflite
import unrealai as uai
import matplotlib.pyplot as plt
from PIL import Image
import datetime;
import random
import time
import csv
import cv2
import argparse
import os
from unrealai.constants import PACKAGE_DEFAULT_CONFIG_FILE
from unrealai.constants import PACKAGE_ROOT_DIR
from unrealai.exceptions import UnrealAIException
from unrealai.config import get_config
import math
from math import cos, sin, atan2, pi, isclose

def client(args, config, mapNames):
    """
        Client script for the InteriorSim project. Calls for the creation of a virtual environment containing an openbot agent.
        Then executes a set of actions with the agent while gathering suitable observations.
        """
        
    numIter = args.iterations
    learningMode = args.setup
    runs = args.runs
    
    print(numIter)
    print(learningMode)
    
    if learningMode == "Data":
        
        # Main loop, executing a set of random actions, getting observations and storing everything in a set of files: 
        
        # The agent makes the following observations:
        
        # ---> left wheel commands in the range [-1, 1]
        # ---> right wheel commands in the range [-1, 1]
        # ---> Yaw in world frame.
        # ---> X position in world frame.
        # ---> Y position in world frame.
        
        config.defrost()
        config.INTERIOR_SIM_BRIDGE.OBSERVATION_VECTOR = "yaw-x-y"
        config.freeze()
        
        speedMultiplier = 1
        
        for mapName in mapNames: # For each map
        
            # Load the correct map:
            config.defrost()
            config.UNREALAI.MAP_ID = "/Game/Maps/Map_" + mapName
            config.freeze()
        
            # Create Uneal Environment: 
            # uenv = uai.UnrealEnv(config)
    
            # Gym wrapper:
            # env = UnrealToGymWrapper(uenv)
            # env.reset()

            # get current observation and reward
            # agent_name = uenv.agents[0]
        
            run = 0
            while run < runs:
                print("----------------------")
                print(f"run {run} over {runs}")
                print("----------------------")
            
                collisionFlag = False
                goalReachedFlag = False
            
                uenv = uai.UnrealEnv(config)
                
                agent_name = uenv.agents[0]
            
                folderName = f"dataset/uploaded/run_{mapName}_{run}/data/"
                os.makedirs(folderName, exist_ok=True)
                os.makedirs(folderName+"sensor_data", exist_ok=True)
                os.makedirs(folderName+"images", exist_ok=True)
                
                failed = []
    
                array_obs = np.empty([numIter, 6])
                
                reward = 0.0
                
                executedIterations = 0
                
        
                for i in range(numIter):
                    executedIterations = i+1
                    ct = datetime.datetime.now() 
                    ts = 10000*ct.timestamp()
                    command_x = 0#random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. 
                    command_y = 0#random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. 
            
                    # Send action to the agent:
                    uenv.step(action={agent_name: [ np.array([command_x,command_y]) ] })
            
                    # Collect observation from the agent:
                    observation = uenv.get_obs_for_agent(agent_name=agent_name)
            
                    # Fill an array with the different observations:
                    array_obs[i][0] = speedMultiplier*observation[0][0] # ctrl left
                    array_obs[i][1] = speedMultiplier*observation[0][1] # ctrl right
                    array_obs[i][2] = observation[0][2] # agent yaw wrt. world
                    array_obs[i][3] = observation[0][3] # agent pos X wrt. world
                    array_obs[i][4] = observation[0][4] # agent pos Y wrt. world
                    array_obs[i][5] = ts # time stamp

                    # Save the images:
                    im = Image.fromarray(observation[1])
                    im.save(folderName+"images/%d.jpeg" % i)
  
                    print(f"iteration {i} over {numIter}")
                    
                    deltaReward = reward - uenv.current_reward['SimModeRobotSim_0']
                    print(f"delta reward: {deltaReward}")
                                        
                    if isclose(deltaReward, config.INTERIOR_SIM_BRIDGE.REWARD_COLLISION, abs_tol=5e1):
                        print("Collision detected ! Killing simulation and restarting run...")  
                        collisionFlag = True
                        break
                    
                    if isclose(deltaReward, -config.INTERIOR_SIM_BRIDGE.REWARD_GOAL_REACHED, abs_tol=5e1):
                        print("Goal reached !")  
                        goalReachedFlag = True
                        break
                    
                    reward = uenv.current_reward['SimModeRobotSim_0']
            
                    # An event triggered premature ending of the simulation
                    #if uenv.current_done: 
                    #    break
                    
                if collisionFlag == True:
                    print("Restarting run...") 
                    #os.removedirs(folderName+"sensor_data")
                    #os.removedirs(folderName+"images")
                     
                else:
                    run = run + 1
                    
                    print("Filling database...")    
                    f_ctrl = open(folderName+"sensor_data/ctrlLog.txt", 'w') # Low-level commands sent to the motors
                    f_goal = open(folderName+"sensor_data/goalLog.txt", 'w') # High level commands 
                    f_rgb = open(folderName+"sensor_data/rgbFrames.txt", 'w') # Reference of the images correespoinding to each control input
                    f_pose = open(folderName+"sensor_data/poseData.txt", 'w') # Raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
        
                    writer_ctrl = csv.writer(f_ctrl, delimiter=",")
                    writer_ctrl.writerow( ('timestamp[ns]','leftCtrl','rightCtrl') )
                    writer_pose = csv.writer(f_pose , delimiter=",")
                    writer_pose.writerow( ('timestamp[ns]','yawAngle','posX','posY') )
                    writer_goal = csv.writer(f_goal, delimiter=",")
                    writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )
                    writer_rgb = csv.writer(f_rgb, delimiter=",")
                    writer_rgb.writerow( ('timestamp[ns]','frame') )

                    goalLocation = np.array([array_obs[numIter-1][3],array_obs[numIter-1][4]]) # use the vehicle last location as goal
                    forward = np.array([1,0]) # Front axis is the X axis.
                    forwardRotated = np.array([0,0])
            
                    for i in range(executedIterations):
            
                        # Target error vector (global coordinate system):
                        relativePositionToTarget = goalLocation - np.array([array_obs[i][3],array_obs[i][4]])
            
                        # Compute Euclidean distance to target:
                        dist = np.linalg.norm(relativePositionToTarget)
            
                        # Compute robot forward axis (global coordinate system)
                        yawVehicle = array_obs[i][2];
                        rot = np.array([[cos(yawVehicle), -sin(yawVehicle)], [sin(yawVehicle), cos(yawVehicle)]])
            
                        forwardRotated = np.dot(rot, forward)
            
                        # Compute yaw:
                        deltaYaw = atan2(forwardRotated[1], forwardRotated[0]) - atan2(relativePositionToTarget[1], relativePositionToTarget[0])
            
                        # Fit to range [-pi, pi]:
                        if deltaYaw > math.pi:
                            deltaYaw -= 2 * math.pi
                        elif deltaYaw <= -math.pi:
                            deltaYaw += 2 * math.pi
            
                        # Check the actual OpenBot code:
                        # https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
                        sinYaw = sin(deltaYaw);
                        cosYaw = cos(deltaYaw);
            
                        # Write pose data
                        writer_pose.writerow( (int(array_obs[i][5]), array_obs[i][2], array_obs[i][3], array_obs[i][4]) )
            
                        # Write the low-level control observation into a file: 
                        writer_ctrl.writerow( (int(array_obs[i][5]), array_obs[i][0], array_obs[i][1]) )
            
                        # Write the corresponding image index into a file: 
                        writer_rgb.writerow( (int(array_obs[i][5]), i) )
            
                        # Write the corresponding high level command into a file:
                        # For imitation learning, use the latest position as a goal 
                        writer_goal.writerow( (int(array_obs[i][5]), dist/100, sinYaw, cosYaw) )
                 
                    f_ctrl.close()
                    f_pose.close()
                    f_goal.close()
                    f_rgb.close()
                
                    # Reset the environment after each run:
                    #uenv.reset()      
                uenv.close()        
                time.sleep(5) 
                
        # Close the environment:
        uenv.close()
        time.sleep(3) 
        
    elif learningMode == "Infer":
    
        # Main loop, testing a trained neural policy: 
    
        # The agent makes the following observations:
        
        # ---> left wheel commands in the range [-1, 1]
        # ---> right wheel commands in the range [-1, 1]
        # ---> Euclidean distance between current x-y position and target x-y position.
        # ---> Sinus of the relative yaw between current pose and target pose.
        # ---> Cosinus of the relative yaw between current pose and target pose.
        
        config.defrost()
        config.INTERIOR_SIM_BRIDGE.OBSERVATION_VECTOR = "dist-sin-cos"
        config.freeze()
    
        # Create Uneal Environment: 
        uenv = uai.UnrealEnv(config)
        agent_name = uenv.agents[0]
    
        # Load TFLite model and allocate tensors.
        interpreter = tflite.Interpreter("/home/quentin/Desktop/interiorsim/code/experiments/rl_openbot/imitation_learning/models/TestSession_1_pilot_net_lr0.0001_bz64_bn/checkpoints/best-val.tflite")
        interpreter.allocate_tensors()

        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        print(input_details)
        print(output_details)
        
        img_input = np.array(np.random.random_sample(input_details[0]["shape"]), dtype=np.float32)
        cmd_input = np.array(np.random.random_sample(input_details[1]["shape"]), dtype=np.float32)
        
        print(cmd_input)
        
        result = np.empty((0, 2), dtype=float)
        
        
        for i in range(numIter):
            
            # Collect observation from the agent:
            observation = uenv.get_obs_for_agent(agent_name=agent_name)
            img_input = np.float32(observation[1])/255
            img_input = tf.image.crop_to_bounding_box(img_input, tf.shape(img_input)[0] - 90, tf.shape(img_input)[1] - 160, 90, 160)
            cmd_input[0][0] = np.float32(observation[0][2])/100
            cmd_input[0][1] = np.float32(observation[0][3])
            cmd_input[0][2] = np.float32(observation[0][4])
          
            #print(cmd_input)
            
            # Inference:
            interpreter.set_tensor(input_details[0]["index"], np.expand_dims(img_input, axis=0))
            interpreter.set_tensor(input_details[1]["index"], cmd_input)    
            
            #print(np.expand_dims(img_input, axis=0))
            #print(cmd_input)
        
            start_time = time.time()
            interpreter.invoke()
            stop_time = time.time()
            #print('time: {:.3f}ms'.format((stop_time - start_time) * 1000))
            
            output = interpreter.get_tensor(output_details[0]["index"])
            

            action = np.clip(np.concatenate((result, output.astype(float))), -1.0, 1.0)
            
            print(cmd_input)
            print(output)
            #print(action)
            
            # Send action to the agent:
            uenv.step(action={agent_name: action })
            
            print(f"iteration {i} over {numIter}")
            
            # An event triggered premature ending of the simulation
            #if uenv.current_done: 
            #    break
            
        # Close the environment:
        uenv.close()
            
    elif learningMode == "Keyboard": # Just play with the keyboard while checking the observations
                
        # Create Uneal Environment: 
        uenv = uai.UnrealEnv(config)
        agent_name = uenv.agents[0]
    
        for i in range(numIter):
            
            command_x = 0#random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. 
            command_y = 0#random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. 
            
            # Send action to the agent:
            uenv.step(action={agent_name: [ np.array([command_x,command_y]) ] })
            
            # Collect observation from the agent:
            observation = uenv.get_obs_for_agent(agent_name=agent_name)
            #print(f"Reward {uenv.current_reward}")
            print(f"iteration {i} over {numIter}")
            
            # An event triggered premature ending of the simulation
            #if uenv.current_done: 
                #break
                #print("uenv.current_done")

        # Close the environment:
        uenv.close()
    
    else:
        
        print("No mode selected...")
        
    return 0


if __name__ == "__main__":

    # List of config files to be used 
    config_files = []

    # Add default config files first and then user config files
    config_files.append(os.path.join(os.getcwd(), "default_config.yaml"))
    config_files.append("../../../unreal_plugins/InteriorSimBridge/default_config.yaml")
    config_files.append("../../../unreal_plugins/RobotSim/default_config.yaml")
    config_files.append("../../../unreal_projects/RobotProject/default_config.yaml")

    # Load configs
    config = get_config(config_files)
    
    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--setup", type=str, help="Data (for data collection), Infer (for ANN inference), Keyboard (for testing purposes only)", required=True)
    parser.add_argument("-m", "--map", nargs="+", default=[""], help="Array of map references. A number of s distinct runs will be executed in each map. This argument overwrites the MAP_ID argument and allows collecting data in multiple environments programatically", required=False)
    args = parser.parse_args()
    
    if args.map == [""]: # Use the default MAP_ID argument from parameter file 
        if config.UNREALAI.MAP_ID == "":
            mapNames = ["simpleMap"]
        elif config.UNREALAI.MAP_ID[0:15] == "/Game/Maps/Map_":
            mapNames = [config.UNREALAI.MAP_ID[15:]]
        else:
            mapNames = ["errorMap"]
    else: # Overwrite the map value
        mapNames = args.map

    client(args, config, mapNames) 
    
    # python run.py -i 10 -r 3 -s "Data" -m "235114801" "235114775" 


