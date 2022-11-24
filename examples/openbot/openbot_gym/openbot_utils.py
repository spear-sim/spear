import argparse
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import shutil
import ffmpeg
import spear
import sys

def generate_video(config, scene_id, run, compress = False):
    print("Generating video from the sequence of observations")
    image_folder = f"dataset/uploaded/run_{scene_id}_{run}/data/images/rgb"
    video_name = f"videos/run_{scene_id}_{run}.avi"
    video_name_compressed = f"videos/run_{scene_id}_{run}_compressed.mp4"

    if not (os.path.exists("videos")):
        os.makedirs("videos")

    images = [img for img in os.listdir(image_folder)]
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    rate = int(1/config.SIMULATION_CONTROLLER.SIMULATION_STEP_TIME_SECONDS)

    video = cv2.VideoWriter(video_name, 0, rate, (width, height))

    # good initial sort but doesnt sort numerically very well
    images.sort(key=lambda f: int(re.sub('\D', '', f)))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

    if compress:
        try:
            i = ffmpeg.input(video_name)
            print("Compressing video...")
            out = ffmpeg.output(i, video_name_compressed, **{'c:v': 'libx264', 'b:v': 800000}).overwrite_output().run()
            print("Done compressing !")

        except FileNotFoundError as e:
            print("You do not have ffmpeg installed!", e)
            print("You can install ffmpeg by entering 'pip install ffmpeg-python' in a terminal.")

def clamp_axis(angle):
	# returns angle in the range (-360,360)
	angle = angle % 360.0

	if (angle < 0.0):
		# shift to [0,360) range
		angle += 360.0

	return angle


def normalize_axis( angle ):
	# returns angle in the range [0,360)
	angle = clamp_axis(angle)

	if (angle > 180.0):
		# shift to (-180,180]
		angle -= 360.0

	return angle


def quaternion_to_pyr(quat):

    # inspired by UnrealMath.cpp... 
    # quaternions are given in the format [X, Y, Z, W]
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
    singularity_threshold = 0.4999995
    
    if (singularity_test < -singularity_threshold):
        pitch = -90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalize_axis(-yaw - (2.0 * np.arctan2(X, W))))

    elif (singularity_test > singularity_threshold):
        pitch = 90.0
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(normalize_axis(yaw - (2.0 * np.arctan2(X, W))))

    else:
        pitch = np.rad2deg(np.arcsin(2.0 * singularity_test))
        yaw = np.rad2deg(np.arctan2(yaw_y, yaw_x))
        roll = np.rad2deg(np.arctan2(-2.0 * (W*X + Y*Z), (1.0 - 2.0 * (np.square(X) + np.square(Y)))))

    return pitch, yaw, roll

def pyr_to_quaternion(pitch, yaw, roll):

    # inspired by UnrealMath.cpp... 
    quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    pitch_no_winding = np.fmod(pitch, 360.0)
    yaw_no_winding = np.fmod(yaw, 360.0)
    roll_no_winding = np.fmod(roll, 360.0)

    sin_pitch = np.sin(0.5*np.deg2rad(pitch_no_winding))
    sin_yaw = np.sin(0.5*np.deg2rad(yaw_no_winding))
    sin_roll = np.sin(0.5*np.deg2rad(roll_no_winding))
    cos_pitch = np.cos(0.5*np.deg2rad(pitch_no_winding))
    cos_yaw = np.cos(0.5*np.deg2rad(yaw_no_winding))
    cos_roll = np.cos(0.5*np.deg2rad(roll_no_winding))

    # quaternions are given in the format [X, Y, Z, W]
    quat[0] =  cos_roll*sin_pitch*sin_yaw - sin_roll*cos_pitch*cos_yaw;
    quat[1] = -cos_roll*sin_pitch*cos_yaw - sin_roll*cos_pitch*sin_yaw;
    quat[2] =  cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw;
    quat[3] =  cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw;

    return quat
