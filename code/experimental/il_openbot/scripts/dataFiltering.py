import numpy as np
import csv
import os
import math
from math import cos, sin, atan2, pi
import shutil

DIR_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")


# This script allows to reparse the raw yaw-x-y measurements, invert the yaw component and then regenerate
# the corresponding dist-sin-cos measurements. This is useful for debug purposes.


# for each environment folders
for folder in os.listdir(f"{DIR_PATH}/dataset/uploaded"):

    path = f"{DIR_PATH}/dataset/uploaded/{folder}"

    for dataFolder in os.listdir(path):  # for each of the 100 data folders

        folderName = f"{path}/{dataFolder}"

        numIterMax = 1000
        numIter = 0
        numIterCtrl = 0
        array_goal = np.empty([numIterMax, 4])
        array_ctrl = np.empty([numIterMax, 3])

        f_goal_valid = False

        try:
            # Filter files based on the content of the goalLog
            f_goal = open(folderName+"/sensor_data/goalLog.txt", 'r')

            csvreader = csv.reader(f_goal)
            header = []
            header = next(csvreader)
            numIter = 0

            for row in csvreader:
                array_goal[numIter][0] = row[0]
                array_goal[numIter][1] = row[1]
                array_goal[numIter][2] = row[2]
                array_goal[numIter][3] = row[3]
                numIter = numIter+1

            f_goal.close()

            # Filter files based on the content of the ctrlLog
            f_ctrl = open(folderName+"/sensor_data/ctrlLog.txt", 'r')

            csvreaderCtrl = csv.reader(f_ctrl)
            headerCtrl = []
            headerCtrl = next(csvreaderCtrl)

            # Put the control array on a matrix so that a subset (buffer) can be used to compute a criterion
            for row in csvreaderCtrl:
                array_ctrl[numIterCtrl][0] = row[0]
                array_ctrl[numIterCtrl][1] = row[1]
                array_ctrl[numIterCtrl][2] = row[2]
                numIterCtrl = numIterCtrl+1

            f_ctrl.close()

            # Is the vehicle facing the goal at the end of the simulation ?
            vehicleFacingGoal = math.isclose(
                array_goal[numIter - 10][3], 1.0, abs_tol=0.2)

            # Are the last 10 control iterations close to zero ?
            finalControlIsZero = math.isclose(np.mean(
                array_ctrl[numIter - 11:, 1]) + np.mean(array_ctrl[numIter - 11:, 2]), 0.0, abs_tol=0.01)

            # Filtering
            if vehicleFacingGoal and not finalControlIsZero and numIter - 1 > 500:
                f_goal_valid = True
            else:
                f_goal_valid = False

        except IOError:  # Something went wrong opening one of the files

            print(
                f"Error: One of the files in {folderName}/sensor_data appears to be corrupted.")
            print(f"Removing {folderName}...")
            shutil.rmtree(folderName)

        if not f_goal_valid:
            print(
                f"File {folderName}/sensor_data/goalLog.txt does not appear to contain relevant data.")
            print(f"Removing {folderName}...")
            shutil.rmtree(folderName)
