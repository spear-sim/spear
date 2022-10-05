import numpy as np
import csv
import os
import math
from math import cos, sin, atan2, pi
import zipfile
import shutil

# This script allows to reparse the raw yaw-x-y measurements, invert the yaw component and then regenerate 
# the corresponding dist-sin-cos measurements. This is useful for debug purposes. 


unzippedFolderName = "dataset/uploaded/"

for folder in os.listdir(f"dataset/zip"):
    
    if folder.endswith(".zip"):
    
        path = f"dataset/zip/{folder}" 
        unzippedFileName = os.path.join(unzippedFolderName, folder[:-4])
        with zipfile.ZipFile(path, "r") as zip_ref:
            
            zip_ref.extractall(unzippedFileName)
            
        for dataFolder in os.listdir(unzippedFileName):
            
            dataFolderName = f"{unzippedFileName}/{dataFolder}/data/"
            folderName = f"{unzippedFileName}/{dataFolder}"
            print(dataFolderName)
            
            numIterMax = 1000
            array_obs = np.empty([numIterMax, 7])
            
            f_pose_valid = False

            try:
                f_pose = open(dataFolderName+"sensor_data/poseData.txt", 'r')
                f_pose_valid = True

            except IOError:
            
                print(f"Error: File {dataFolderName}sensor_data/poseData.txt appears to be corrupted.")
                print(f"Removing {folderName}...")
                shutil.rmtree(folderName)
                
                
            if f_pose_valid:
            
                csvreader = csv.reader(f_pose)
                header = []
                header = next(csvreader)
                numIter = 0
                for row in csvreader:
        
                    #print(i)
                    #print(row)
                    ts = row[0]

                    # Fill an array with the different observations:
                    array_obs[numIter][0] = row[1] # agent pos X wrt. world
                    array_obs[numIter][1] = row[2] # agent pos Y wrt. world
                    array_obs[numIter][2] = row[3] # agent pos Z wrt. world
                    array_obs[numIter][3] = row[4] # agent roll wrt. world
                    array_obs[numIter][4] = row[5] # agent pitch wrt. world
                    array_obs[numIter][5] = row[6] # agent yaw wrt. world
                    array_obs[numIter][6] = ts # time stamp
                    numIter = numIter+1

                f_pose.close()

                print("Filling database...") 
   
                f_goal = open(dataFolderName+"sensor_data/goalLog.txt", 'w') # High level commands 
                writer_goal = csv.writer(f_goal, delimiter=",")
                writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )

                goalLocation = np.array([array_obs[numIter-1][0],array_obs[numIter-1][1]]) # use the vehicle last location as goal
                forward = np.array([1,0]) # Front axis is the X axis.
                forwardRotated = np.array([0,0])

                for i in range(numIter):

                    # Target error vector (global coordinate system):
                    relativePositionToTarget = goalLocation - np.array([array_obs[i][0],array_obs[i][1]])

                    # Compute Euclidean distance to target:
                    dist = np.linalg.norm(relativePositionToTarget)

                    # Compute robot forward axis (global coordinate system)
                    yawVehicle = array_obs[i][5];
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

                    # Write the corresponding high level command into a file:
                    # For imitation learning, use the latest position as a goal 
                    writer_goal.writerow( (int(array_obs[i][6]), dist/100, sinYaw, cosYaw) )
 

                f_goal.close()
