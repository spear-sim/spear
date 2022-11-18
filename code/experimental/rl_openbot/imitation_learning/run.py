#############################################################################

# Quick and dirty script to issue random commands to an OpenBot agent and get
# egocentric visual obsevations.

#############################################################################

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
#
#
# python run.py -i 10 -r 3 -s "Data" -m "235114801" "235114775"
# python run.py -i 10 -r 3 -s "Infer" -m "235114801" "235114775"
# python run.py -i 10 -r 3 -s "Debug" -m "235114801" "235114775"

import argparse
import csv
import cv2
import datetime;
import math
from math import cos, sin, atan2, pi, isclose
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os
import random
import shutil
#import tensorflow as tf
#import tflite_runtime.interpreter as tflite
import time
import re

#from interiorsim import Env
#from interiorsim.config import get_config
#from interiorsim.constants import INTERIORSIM_ROOT_DIR

import spear


def GenerateVideo(config, mapName, run):
    print("Generating video from the sequence of observations")
    image_folder = f"dataset/uploaded/run_{mapName}_{run}/data/images"
    video_name = f"videos/run_{mapName}_{run}.avi"
    
    if not (os.path.exists("videos")):
        os.makedirs("videos")

    images = [img for img in os.listdir(image_folder)]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape
    
    rate = int(1/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS)

    video = cv2.VideoWriter(video_name, 0, 100, (width,height))

    images.sort(key=lambda f: int(re.sub('\D', '', f))) #good initial sort but doesnt sort numerically very well

    for image in images:
        #print(image)
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

def Clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def computeNeurNetInput(goalPositionXY, actualPoseYawXY, acceptanceRadius):

    targetLocationReached = False
    obsNet = np.array([0.0,0.0,1.0])
    deltaYaw = 0.0
    forward = np.array([1,0]) # Front axis is the X axis.
    forwardRotated = np.array([0,0])

    # Target error vector (global coordinate system):
    relativePositionToTarget = desiredPositionXY - np.array([actualPoseYawXY[1],actualPoseYawXY[2]])
    
    # Compute Euclidean distance to target in [m]:
    dist = np.linalg.norm(relativePositionToTarget)*0.01

    if dist < acceptanceRadius :
        targetLocationReached = True;

    # Otherwise compute the PID command:
    else:
    
        # Compute robot forward axis (global coordinate system):
        yawVehicle = actualPoseYawXY[0];
        rot = np.array([[cos(yawVehicle), -sin(yawVehicle)], [sin(yawVehicle), cos(yawVehicle)]])

        forwardRotated = np.dot(rot, forward)

        # Compute yaw:
        deltaYaw = atan2(forwardRotated[1], forwardRotated[0]) - atan2(relativePositionToTarget[1], relativePositionToTarget[0])

        # Fit to range [-pi, pi]:
        if deltaYaw > math.pi:
            deltaYaw -= 2 * math.pi
        elif deltaYaw <= -math.pi:
            deltaYaw += 2 * math.pi
        
        sinYaw = sin(deltaYaw)
        cosYaw = cos(deltaYaw)
        
        obsNet = np.array([dist,sinYaw,cosYaw])

    return obsNet, targetLocationReached
    

def clampAxis(angle):
	# returns angle in the range (-360,360)
	angle = angle % 360.0

	if (angle < 0.0):
		# shift to [0,360) range
		angle += 360.0

	return angle


def normalizeAxis( angle ):
	# returns angle in the range [0,360)
	angle = clampAxis(angle)

	if (angle > 180.0):
		# shift to (-180,180]
		angle -= 360.0

	return angle


def quatToRPY(quat):

    # Inspired by UnrealMath.cpp... 

    X = quat[0]
    Y = quat[1]
    Z = quat[2]
    W = quat[3]

    singularity_test = Z*X - W*Y
    yaw_y = 2.0*(W*Z + X*Y)
    yaw_x = (1.0 - 2.0 * (np.square(Y) + np.square(Z)))

	# reference 
	# http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	# http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/

	# this value was found from experience, the above websites recommend different values
	# but that isn't the case for us, so I went through different testing, and finally found the case 
	# where both of world lives happily. 
    SINGULARITY_THRESHOLD = 0.4999995
    
    if (singularity_test < -SINGULARITY_THRESHOLD):
        pitch = -90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalizeAxis(-yaw - (2.0 * np.arctan2(X, W))))

    elif (singularity_test > SINGULARITY_THRESHOLD):
        pitch = 90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalizeAxis(yaw - (2.0 * np.arctan2(X, W))))

    else:
        pitch = np.rad2deg(np.arcsin(2.0 * singularity_test))
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(np.arctan2(-2.0 * (W*X + Y*Z), (1.0 - 2.0 * (np.square(X) + np.square(Y)))))

    return roll, pitch, yaw

def iterationAutopilot(desiredPositionXY, actualPoseYawXY, linVelNorm, yawVel, Kp_lin, Kd_lin, Kp_ang, Kd_ang, acceptanceRadius, forwardMinAngle, controlSaturation):

    targetLocationReached = False
    forwardCtrl = 0.0
    rightCtrl = 0.0
    action = np.array([0.0,0.0])
    deltaYaw = 0.0
    forward = np.array([1,0]) # Front axis is the X axis.
    forwardRotated = np.array([0,0])

    # Target error vector (global coordinate system):
    relativePositionToTarget = desiredPositionXY - np.array([actualPoseYawXY[1],actualPoseYawXY[2]])
        
    # Compute Euclidean distance to target in [m]:
    dist = np.linalg.norm(relativePositionToTarget)*0.01

    if dist < acceptanceRadius :
        targetLocationReached = True;

    # Otherwise compute the PID command:
    else:
    
        # Compute robot forward axis (global coordinate system):
        yawVehicle = actualPoseYawXY[0];
        rot = np.array([[cos(yawVehicle), -sin(yawVehicle)], [sin(yawVehicle), cos(yawVehicle)]])

        forwardRotated = np.dot(rot, forward)

        # Compute yaw:
        deltaYaw = atan2(forwardRotated[1], forwardRotated[0]) - atan2(relativePositionToTarget[1], relativePositionToTarget[0])

        # Fit to range [-pi, pi]:
        if deltaYaw > math.pi:
            deltaYaw -= 2 * math.pi
        elif deltaYaw <= -math.pi:
            deltaYaw += 2 * math.pi

        linVel = linVelNorm * 0.036; # In [m/s]

        rightCtrl = -Kp_ang * deltaYaw - Kd_ang * yawVel;
        Clamp(rightCtrl, -controlSaturation, controlSaturation)

        if abs(deltaYaw) < forwardMinAngle :
            forwardCtrl = Kp_lin * dist - Kd_lin * linVel
            forwardCtrl = Clamp(forwardCtrl, -controlSaturation, controlSaturation)
            forwardCtrl *= abs(cos(deltaYaw)); # Full throttle if the vehicle facing the objective. Otherwise give more priority to the yaw command.

        # Compute action:
        leftWheelCommand = forwardCtrl + rightCtrl
        leftWheelCommand = Clamp(leftWheelCommand, -controlSaturation, controlSaturation)
        
        rightWheelCommand = forwardCtrl - rightCtrl
        rightWheelCommand = Clamp(rightWheelCommand, -controlSaturation, controlSaturation)
        
        action = np.array([leftWheelCommand,rightWheelCommand], dtype=np.float32)

    #print(f"dist: {dist} m")
    #print(f"deltaYaw: {deltaYaw}")
    #print(f"rightCtrl: {rightCtrl}")
    #print(f"forwardCtrl: {forwardCtrl}")
    #print(f"action: {action[0]}, {action[1]}")

    return action, targetLocationReached
    
    

if __name__ == "__main__":

    # List of config files to be used
    config_files = []

    # Add default config files first and then user config files
    config_files.append(os.path.join(os.getcwd(), "user_config.yaml"))
    
    # Load config (this function will load the parameter values specified in user_config.yaml, as well as sensible defaults for all other parameters)
    config = spear.get_config(user_config_files=config_files)

    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--create_video", action="store_true", help="create a video out of the observations.")
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment", required=True)
    parser.add_argument("-r", "--runs", type=int, help="number of distinct runs in the considered environment", required=True)
    parser.add_argument("-s", "--setup", type=str, help="Data (for data collection), Infer (for ANN inference), Debug (for debug purposes only)", required=True)
    parser.add_argument("-p", "--policy", type=str, help="1_env, 5_envs,  25_envs, 50_envs or real_envs", required=False)
    parser.add_argument("-m", "--map", nargs="+", default=[""], help="Array of map references. A number of s distinct runs will be executed in each map. This argument overwrites the LEVEL_ID argument and allows collecting data in multiple environments programatically", required=False)
    parser.add_argument("-c", "--connect", type=int, help="id of the connection port", required=True)
    args = parser.parse_args()

    config.defrost()
    config.SIMULATION_CONTROLLER.PORT = 30000 + args.connect
    config.freeze()

    if args.map == [""]: # Use the default LEVEL_ID argument from parameter file
        if config.SIMULATION_CONTROLLER.LEVEL_ID == "":
            mapNames = ["simpleMap"]
        else:
            mapNames = [config.SIMULATION_CONTROLLER.LEVEL_ID]

    else: # Overwrite the map value
        mapNames = args.map

    numIter = args.iterations
    learningMode = args.setup
    runs = args.runs

    if learningMode == "Data":

        # Main loop, executing a set of random actions, getting observations and storing everything in a set of files:

        # The agent makes the following observations:

        # ---> left wheel commands in the range [-1, 1]
        # ---> right wheel commands in the range [-1, 1]
        # ---> X position in world frame.
        # ---> Y position in world frame.
        # ---> Z position in world frame.
        # ---> Roll in world frame.
        # ---> Pitch in world frame.
        # ---> Yaw in world frame.
        # ---> X position of the next waypoint in world frame.
        # ---> Y position of the next waypoint in world frame.

        speedMultiplier = 1

        for mapName in mapNames: # For each map

            # Load the correct map:
            config.defrost()
            config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_"+ mapName + ".Map_" + mapName # full path to the map including the repeated name, e.g., "/Game/NewMap.NewMap"
            config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_"+ mapName
           

            config.freeze()

            # Create Env object:
            env = spear.Env(config=config)
            
            run = 0
            while run < runs:

                collisionFlag = False
                goalReachedFlag = False
                array_obs = np.empty([numIter, 11])
                executedIterations = 0
                index_waypoint = 1

                folderName = f"dataset/uploaded/run_{mapName}_{run}"
                dataFolderName = folderName+"/data/"
                os.makedirs(dataFolderName, exist_ok=True)
                os.makedirs(dataFolderName+"sensor_data", exist_ok=True)
                os.makedirs(dataFolderName+"images", exist_ok=True)

                print("----------------------")
                print(f"run {run} over {runs}")
                print("----------------------")

                # Reset the simulation to get the first observation
                obs = env.reset()
                
                # Send Zero action to the agent and collect initial trajectory observations:
                obs, reward, done, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})
                Q = obs["telemetry_data"][3:7]
                roll, pitch, yaw = quatToRPY(Q)
                actualPoseYawXY = np.array([yaw, obs["telemetry_data"][0], obs["telemetry_data"][1]]) # [Yaw, X, Y], for velocity initialization
                dYaw = yaw - actualPoseYawXY[0]
                desiredPositionXY = np.array([info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint], info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint + 1]]) # [Xdes, Ydes]

                numWaypoints = len(info["agent_controller_step_info"]["trajectory_data"])/3 - 1

                # Take a few steps:
                for i in range(numIter):

                    print(f"iteration {i} over {numIter}")

                    executedIterations = i+1
                    ct = datetime.datetime.now()
                    ts = 10000*ct.timestamp()

                    # Run autopilot:
                    Kp_lin = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.PROPORTIONAL_GAIN_DIST
                    Kd_lin = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.DERIVATIVE_GAIN_DIST
                    Kp_ang = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.PROPORTIONAL_GAIN_HEADING
                    Kd_ang = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.DERIVATIVE_GAIN_HEADING
                    acceptanceRadius = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.ACCEPTANCE_RADIUS
                    forwardMinAngle = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.FORWARD_MIN_ANGLE
                    controlSaturation = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.CONTROL_SATURATION
                    dt = config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS

                    # XY position of the next waypoint in world frame:
                    dXY = np.array([obs["telemetry_data"][0], obs["telemetry_data"][1]]) - desiredPositionXY
                    desiredPositionXY = np.array([info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint], info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint + 1]]) # [Xdes, Ydes]

                    # Current position and heading of the vehicle in world frame:
                    Q = obs["telemetry_data"][3:7]
                    roll, pitch, yaw = quatToRPY(Q)
                    dYaw = yaw - actualPoseYawXY[0]
                    actualPoseYawXY = np.array([yaw, obs["telemetry_data"][0], obs["telemetry_data"][1]]) # [Yaw, X, Y]

                    # Numerical diff:
                    linVelNorm = np.linalg.norm(dXY/dt)
                    yawVel = dYaw/dt

                    action, waypointReached = iterationAutopilot(desiredPositionXY, actualPoseYawXY, linVelNorm, yawVel, Kp_lin, Kd_lin, Kp_ang, Kd_ang, acceptanceRadius, forwardMinAngle, controlSaturation)

                    # Send action to the agent and collect observations:
                    obs, reward, done, info = env.step({"apply_voltage": action})
                    
                    print(obs["sonar_data"])

                    # Fill an array with the different observations:
                    array_obs[i][0] = speedMultiplier*obs["control_data"][0] # ctrl left
                    array_obs[i][1] = speedMultiplier*obs["control_data"][1] # ctrl right
                    array_obs[i][2] = obs["telemetry_data"][0] # agent pos X wrt. world
                    array_obs[i][3] = obs["telemetry_data"][1] # agent pos Y wrt. world
                    array_obs[i][4] = obs["telemetry_data"][2] # agent pos Z wrt. world
                    array_obs[i][5] = roll # agent Roll wrt. world
                    array_obs[i][6] = pitch # agent Pitch wrt. world
                    array_obs[i][7] = yaw # agent Yaw wrt. world
                    array_obs[i][8] = info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint] # desired (waypoint) agent pos X wrt. world
                    array_obs[i][9] = info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint + 1] # desired (waypoint) agent pos Y wrt. world
                    array_obs[i][10] = ts # time stamp
                    
                    if array_obs[i][4] < 0: # For now we don't consider underground operation ! 
                        collisionFlag = True
                        break
                    
                    #print(f"Action: {array_obs[i][0]}, {array_obs[i][1]}")
                    #print(f"Pose: {array_obs[i][2], array_obs[i][3], array_obs[i][4], array_obs[i][5], array_obs[i][6], array_obs[i][7]}")

                    # Save the images:
                    im = Image.fromarray(obs["rgb_data"])
                    im.save(dataFolderName+"images/%d.jpeg" % i)

                    if waypointReached:
                        if index_waypoint < numWaypoints: # if the waypoint is not the goal
                            print(f"Waypoint {index_waypoint} over {numWaypoints} reached !")
                            index_waypoint = index_waypoint + 1
                        else: # Goal reached !
                            print("Goal reached !")
                            goalReachedFlag = True
                            break

                    # Interrupt the step loop if the done flag is raised:
                    if done:
                        if info["task_step_info"]["hit_obstacle"]:
                            print("Collision detected ! Killing simulation and restarting run...")
                            collisionFlag = True

                        if info["task_step_info"]["hit_goal"]:
                            print("Goal reached !")
                            goalReachedFlag = True

                        break

                if collisionFlag == True:
                    print("Restarting run...")
                    shutil.rmtree(folderName)

                else:
                    if args.create_video:
                        GenerateVideo(config, mapName, run)
                    run = run + 1

                    print("Filling database...")
                    f_ctrl = open(dataFolderName+"sensor_data/ctrlLog.txt", 'w') # Low-level commands sent to the motors
                    f_goal = open(dataFolderName+"sensor_data/goalLog.txt", 'w') # High level commands
                    f_rgb = open(dataFolderName+"sensor_data/rgbFrames.txt", 'w') # Reference of the images correespoinding to each control input
                    f_pose = open(dataFolderName+"sensor_data/poseData.txt", 'w') # Raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)

                    writer_ctrl = csv.writer(f_ctrl, delimiter=",")
                    writer_ctrl.writerow( ('timestamp[ns]','leftCtrl','rightCtrl') )
                    writer_pose = csv.writer(f_pose , delimiter=",")
                    writer_pose.writerow( ('timestamp[ns]','posX','posY','posZ','rollAngle','pitchAngle','yawAngle') )
                    writer_goal = csv.writer(f_goal, delimiter=",")
                    writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )
                    writer_rgb = csv.writer(f_rgb, delimiter=",")
                    writer_rgb.writerow( ('timestamp[ns]','frame') )

                    goalLocation = np.array([array_obs[executedIterations-1][2],array_obs[executedIterations-1][3]]) # use the vehicle last location as goal
                    forward = np.array([1,0]) # Front axis is the X axis.
                    forwardRotated = np.array([0,0])

                    for i in range(executedIterations):

                        # Target error vector (global coordinate system):
                        relativePositionToTarget = goalLocation - np.array([array_obs[i][2],array_obs[i][3]])

                        # Compute Euclidean distance to target:
                        dist = np.linalg.norm(relativePositionToTarget)

                        # Compute robot forward axis (global coordinate system):
                        yawVehicle = array_obs[i][7];
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
                        writer_pose.writerow( (int(array_obs[i][10]), array_obs[i][2], array_obs[i][3], array_obs[i][4], array_obs[i][5], array_obs[i][6], array_obs[i][7]) )

                        # Write the low-level control observation into a file:
                        writer_ctrl.writerow( (int(array_obs[i][10]), array_obs[i][0], array_obs[i][1]) )

                        # Write the corresponding image index into a file:
                        writer_rgb.writerow( (int(array_obs[i][10]), i) )

                        # Write the corresponding high level command into a file:
                        # For imitation learning, use the latest position as a goal
                        writer_goal.writerow( (int(array_obs[i][10]), dist/100, sinYaw, cosYaw) )

                    f_ctrl.close()
                    f_pose.close()
                    f_goal.close()
                    f_rgb.close()

            # Close the environment:
            env.close()
            time.sleep(3)

    elif learningMode == "Infer":

        # Main loop, testing a trained neural policy:

        # The agent makes the following observations:

        # ---> left wheel commands in the range [-1, 1]
        # ---> right wheel commands in the range [-1, 1]
        # ---> X position in world frame.
        # ---> Y position in world frame.
        # ---> Z position in world frame.
        # ---> Roll in world frame.
        # ---> Pitch in world frame.
        # ---> Yaw in world frame.
        # ---> X position of the goal in world frame.
        # ---> Y position of the goal in world frame.

        # Load TFLite model and allocate tensors.
        policyName = ""
        if args.policy == "1_env":
            policyName = "./models/1_env.tflite"
        elif args.policy == "5_envs":
            policyName = "./models/5_envs.tflite"
        elif args.policy == "25_envs":
            policyName = "./models/25_envs.tflite"
        elif args.policy == "50_envs":
            policyName = "./models/50_envs.tflite"
        elif args.policy == "real":
            policyName = "./models/real.tflite"
        elif args.policy == "":
            print("Warning: no policy name was provided. Trying Navigation.tflite as default...")
            policyName = "./models/navigation.tflite"
        else:
            print("Warning: no policy name was provided. Trying navigation.tflite as default...")
            policyName = "./models/navigation.tflite"
        
        interpreter = tflite.Interpreter(policyName)
        interpreter.allocate_tensors()

        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        print(input_details)
        print(output_details)

        img_input = np.array(np.random.random_sample(input_details[0]["shape"]), dtype=np.float32)
        cmd_input = np.array(np.random.random_sample(input_details[1]["shape"]), dtype=np.float32)

        for mapName in mapNames: # For each map

            # Load the correct map:
            config.defrost()
            print("/Game/Maps/Map_"+ mapName + ".Map_" + mapName)
            config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_"+ mapName + ".Map_" + mapName # full path to the map including the repeated name, e.g., "/Game/NewMap.NewMap"
            config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_"+ mapName
            config.freeze()

            # Create Env object:
            env = Env(config)

            run = 0
            while run < runs:

                collisionFlag = False
                goalReachedFlag = False
                result = np.empty((0, 2), dtype=float)
                executedIterations = 0
                
                folderName = f"dataset/uploaded/run_{mapName}_{run}"
                dataFolderName = folderName+"/data/"
                os.makedirs(dataFolderName, exist_ok=True)
                os.makedirs(dataFolderName+"sensor_data", exist_ok=True)
                os.makedirs(dataFolderName+"images", exist_ok=True)

                print("----------------------")
                print(f"run {run} over {runs}")
                print("----------------------")

                # Reset the simulation to get the first observation
                obs = env.reset()
                reward = 0.0
                #actualPoseYawXY = np.array([obs["physical_observation"][7], obs["physical_observation"][2], obs["physical_observation"][3]]) # [Yaw, X, Y], for velocity initialization
                #desiredPositionXY = np.array([obs["physical_observation"][8], obs["physical_observation"][9]]) # [Xdes, Ydes]

                
                f_infer = open(dataFolderName+"sensor_data/Inference.txt", 'w') # Low-level commands sent to the motors
                writer_infer = csv.writer(f_infer , delimiter=",")
                writer_infer.writerow( ('Iteration','posX','posY','posZ','rollAngle','pitchAngle','yawAngle','goalX','goalY','dist','sinYaw','cosYaw', 'ctrl_left', 'ctrl_right', 'inference_time', 'reward', 'distTrajToGoal') )


                # Take a few steps:
                for i in range(numIter):

                    print(f"iteration {i} over {numIter}")
                    
                    #executedIterations = i+1
                    ct = datetime.datetime.now()
                    ts = 10000*ct.timestamp()
                    acceptanceRadius = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.ACCEPTANCE_RADIUS

                    # Process (crop) visual observations:
                    
                    if not obs["rgb_data"].any():
                        print(obs["rgb_data"])
                        break; # Something went wrong and the robot was spawed out of the map... 
                        
                    img_input = np.float32(obs["rgb_data"])/255
                    img_input = tf.image.crop_to_bounding_box(img_input, tf.shape(img_input)[0] - 90, tf.shape(img_input)[1] - 160, 90, 160)

                    # Physical observations
                    desiredPositionXY = np.array([obs["physical_observation"][8], obs["physical_observation"][9]]) # [Xdes, Ydes]
                    actualPoseYawXY = np.array([obs["physical_observation"][7], obs["physical_observation"][2], obs["physical_observation"][3]]) # [Yaw, X, Y]
                    cmd, targetLocationReached = computeNeurNetInput(desiredPositionXY, actualPoseYawXY, acceptanceRadius) # here desiredPositionXY directly refers to the goal position rather than any waypoint...
                    #cmd_input[0][0] = np.float32(obs["physical_observation"][2])/100
                    #cmd_input[0][1] = np.float32(obs["physical_observation"][3])
                    #cmd_input[0][2] = np.float32(obs["physical_observation"][4])
                    cmd_input[0][0] = np.float32(cmd[0])
                    cmd_input[0][1] = np.float32(cmd[1])
                    cmd_input[0][2] = np.float32(cmd[2])
                    
                    print(f"cmd_input: {cmd}")

                    # Inference:
                    interpreter.set_tensor(input_details[0]["index"], np.expand_dims(img_input, axis=0))
                    interpreter.set_tensor(input_details[1]["index"], cmd_input)
                    start_time = time.time()
                    interpreter.invoke()
                    stop_time = time.time()
                    executionTime = (stop_time - start_time) * 1000
                    #print('Infererence time: {:.3f}ms'.format(executionTime))

                    # Output of the Artificial Neural Network
                    output = interpreter.get_tensor(output_details[0]["index"])

                    # Command
                    act = np.clip(np.concatenate((result, output.astype(float))), -1.0, 1.0)
                    action = np.array([act[0][0],act[0][1]])
                    #print(action)
                    
                    writer_infer.writerow( (i, obs["physical_observation"][2], obs["physical_observation"][3], obs["physical_observation"][4], obs["physical_observation"][5], obs["physical_observation"][6], obs["physical_observation"][7], obs["physical_observation"][8], obs["physical_observation"][9], cmd[0], cmd[1], cmd[2], action[0], action[1], executionTime, reward, obs["physical_observation"][10]) )

                    # Send action to the agent:
                    obs, reward, done, info = env.step({"apply_voltage": [action[0], action[1]]})
                    
                    # Save the images:
                    im = Image.fromarray(obs["rgb_data"])
                    im.save(dataFolderName+"images/%d.jpeg" % i)
                    
                    if run >= numIter-1:
                        f_status = open(dataFolderName+"sensor_data/Status.txt", 'w') 
                        writer_status = csv.writer(f_status , delimiter=",")          
                        writer_status.writerow( ('Status','Iterations') )
                        writer_status.writerow( ('Iteration Limit',i) )
                        f_status.close()
                        break
                    
                    if targetLocationReached:
                        
                        f_status = open(dataFolderName+"sensor_data/Status.txt", 'w') 
                        writer_status = csv.writer(f_status , delimiter=",")          
                        writer_status.writerow( ('Status','Iterations') )
                        goalReachedFlag = True
                        writer_status.writerow( ('Goal',i) )
                        f_status.close()
                        
                        if i < 50: # There is some issue related to initial collision
                            break # re-execute the run
                        else:
                            run = run + 1
                            break

                    # Interrupt the step loop if the done flag is raised:
                    if done:
                     
                        f_status = open(dataFolderName+"sensor_data/Status.txt", 'w') 
                        writer_status = csv.writer(f_status , delimiter=",")          
                        writer_status.writerow( ('Status','Iterations') )
                        
                        if info["task_step_info"]["hit_obstacle"]:
                            print("Collision detected !")
                            collisionFlag = True
                            writer_status.writerow( ('Collision',i) )
                            f_status.close()

                        elif info["task_step_info"]["hit_goal"]:
                            print("Goal reached !")
                            goalReachedFlag = True
                            writer_status.writerow( ('Goal',i) )
                            f_status.close()
                            
                        else: 
                            print("limit numer of iterations reached !")
                            writer_status.writerow( ('Iteration Limit',i) )
                            f_status.close()

                        if i < 50: # There is some issue related to initial collision
                            break # re-execute the run
                        else:
                            run = run + 1
                            break
                
                f_infer.close()
                if args.create_video:
                    GenerateVideo(config, mapName, run-1)
                

        # Close the environment:
        env.close()

    elif learningMode == "Debug": # Just play with the keyboard while checking the observations

        # Load the correct map:
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_"+ mapName + ".Map_" + mapName # full path to the map including the repeated name, e.g., "/Game/NewMap.NewMap"
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_"+ mapName
        config.freeze()
        
        index_waypoint = 1

        # Create Env object:
        env = Env(config)

        # Reset the simulation to get the first observation
        obs = env.reset()
                
        # Send Zero action to the agent and collect initial trajectory observations:
        obs, reward, done, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})
        Q = obs["telemetry_data"][3:7]
        roll, pitch, yaw = quatToRPY(Q)
        actualPoseYawXY = np.array([yaw, obs["telemetry_data"][0], obs["telemetry_data"][1]]) # [Yaw, X, Y], for velocity initialization
        dYaw = yaw - actualPoseYawXY[0]
        desiredPositionXY = np.array([info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint], info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint + 1]]) # [Xdes, Ydes]

        numWaypoints = len(info["agent_controller_step_info"]["trajectory_data"])/3 - 1

        # cv2.imshow("rgb_data", obs["rgb_data"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)

        # Take a few steps:
        for i in range(numIter):

            print(f"iteration {i} over {numIter}")

            executedIterations = i+1
            ct = datetime.datetime.now()
            ts = 10000*ct.timestamp()

            # Run autopilot:
            Kp_lin = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.PROPORTIONAL_GAIN_DIST
            Kd_lin = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.DERIVATIVE_GAIN_DIST
            Kp_ang = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.PROPORTIONAL_GAIN_HEADING
            Kd_ang = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.DERIVATIVE_GAIN_HEADING
            acceptanceRadius = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.ACCEPTANCE_RADIUS
            forwardMinAngle = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.FORWARD_MIN_ANGLE
            controlSaturation = config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.AUTOPILOT.CONTROL_SATURATION
            dt = config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS

            # XY position of the next waypoint in world frame:
            dXY = np.array([obs["telemetry_data"][0], obs["telemetry_data"][1]]) - desiredPositionXY
            desiredPositionXY = np.array([info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint], info["agent_controller_step_info"]["trajectory_data"][3*index_waypoint + 1]]) # [Xdes, Ydes]

            # Current position and heading of the vehicle in world frame:
            Q = obs["telemetry_data"][3:7]
            roll, pitch, yaw = quatToRPY(Q)
            actualPoseYawXY = np.array([yaw, obs["telemetry_data"][0], obs["telemetry_data"][1]]) # [Yaw, X, Y]
            dYaw = yaw - actualPoseYawXY[0]
            

            # Numerical diff:
            linVelNorm = np.linalg.norm(dXY/dt)
            yawVel = dYaw/dt

            action, targetLocationReached = iterationAutopilot(desiredPositionXY, actualPoseYawXY, linVelNorm, yawVel, Kp_lin, Kd_lin, Kp_ang, Kd_ang, acceptanceRadius, forwardMinAngle, controlSaturation)

            # Send action to the agent and collect observations:

            obs, reward, done, info = env.step({"apply_voltage": action})
            # cv2.imshow("rgb_data", obs["rgb_data"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.imshow("depth_data", obs["depth_data"][:,:]) # OpenCV expects BGR instead of RGB
            # cv2.imshow("segmentation_data", obs["segmentation_data"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.imshow("surface_normals_data", obs["surface_normals_data"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.waitKey(0)

            #if done:
            #    print("Reset run...")
            #    env.reset()

        cv2.destroyAllWindows()

        # Close the environment:
        env.close()

    else:

        print("No mode selected...")

