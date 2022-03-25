import numpy as np
import csv
import os
import math
from math import cos, sin, atan2, pi

# This script allows to reparse the raw yaw-x-y measurements, invert the yaw component and then regenerate 
# the corresponding dist-sin-cos measurements. This is useful for debug purposes. 

for folder in os.listdir(f"dataset/uploaded"):
    folderName = f"dataset/uploaded/{folder}/data/"


    numIter = 1000
    array_obs = np.empty([numIter, 6])


    f_pose = open(folderName+"sensor_data/poseData.txt", 'r')
    csvreader = csv.reader(f_pose)
    header = []
    header = next(csvreader)
    i = 0
    for row in csvreader:
        
        #print(i)
        #print(row)
        ts = row[0]

        # Fill an array with the different observations:
        array_obs[i][0] = row[1] # agent yaw wrt. world
        array_obs[i][1] = row[2] # agent pos X wrt. world
        array_obs[i][2] = row[3] # agent pos Y wrt. world
        array_obs[i][3] = ts # time stamp
        i = i+1

    f_pose.close()

    print("Filling database...") 
   
    f_goal = open(folderName+"sensor_data/goalLog.txt", 'w') # High level commands 
    writer_goal = csv.writer(f_goal, delimiter=",")
    writer_goal.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )

    goalLocation = np.array([array_obs[numIter-1][1],array_obs[numIter-1][2]]) # use the vehicle last location as goal
    forward = np.array([1,0]) # Front axis is the X axis.
    forwardRotated = np.array([0,0])

    for i in range(numIter):

        # Target error vector (global coordinate system):
        relativePositionToTarget = goalLocation - np.array([array_obs[i][1],array_obs[i][2]])

        # Compute Euclidean distance to target:
        dist = np.linalg.norm(relativePositionToTarget)

        # Compute robot forward axis (global coordinate system)
        yawVehicle = array_obs[i][0];
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
        writer_goal.writerow( (int(array_obs[i][3]), dist/100, cosYaw, sinYaw) )
 

    f_goal.close()
