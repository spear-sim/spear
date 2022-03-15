#############################################################################

# Quick and dirty script to issue random commands to an OpenBot agent and get 
# egocentric visual obsevations. 

#############################################################################
import numpy as np
import unrealai as uai
import matplotlib.pyplot as plt
from PIL import Image
import datetime; 
import random
import csv
import cv2
import argparse
import os
from yacs.config import CfgNode, load_cfg
from unrealai.constants import PACKAGE_DEFAULT_CONFIG_FILE
from unrealai.exceptions import UnrealAIException



# c.f. https://stackoverflow.com/questions/16702966/rotate-image-and-crop-out-black-borders
def rotate_image(image, angle):
    """
    Rotates an OpenCV 2 / NumPy image about it's centre by the given angle
    (in degrees). The returned image will be large enough to hold the entire
    new image, with a black background
    """

    # Get the image size
    # No that's not an error - NumPy stores image matricies backwards
    image_size = (image.shape[1], image.shape[0])
    image_center = tuple(np.array(image_size) / 2)

    # Convert the OpenCV 3x2 rotation matrix to 3x3
    rot_mat = np.vstack(
        [cv2.getRotationMatrix2D(image_center, angle, 1.0), [0, 0, 1]]
    )

    rot_mat_notranslate = np.matrix(rot_mat[0:2, 0:2])

    # Shorthand for below calcs
    image_w2 = image_size[0] * 0.5
    image_h2 = image_size[1] * 0.5

    # Obtain the rotated coordinates of the image corners
    rotated_coords = [
        (np.array([-image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([-image_w2, -image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2, -image_h2]) * rot_mat_notranslate).A[0]
    ]

    # Find the size of the new image
    x_coords = [pt[0] for pt in rotated_coords]
    x_pos = [x for x in x_coords if x > 0]
    x_neg = [x for x in x_coords if x < 0]

    y_coords = [pt[1] for pt in rotated_coords]
    y_pos = [y for y in y_coords if y > 0]
    y_neg = [y for y in y_coords if y < 0]

    right_bound = max(x_pos)
    left_bound = min(x_neg)
    top_bound = max(y_pos)
    bot_bound = min(y_neg)

    new_w = int(abs(right_bound - left_bound))
    new_h = int(abs(top_bound - bot_bound))

    # We require a translation matrix to keep the image centred
    trans_mat = np.matrix([
        [1, 0, int(new_w * 0.5 - image_w2)],
        [0, 1, int(new_h * 0.5 - image_h2)],
        [0, 0, 1]
    ])

    # Compute the tranform for the combined rotation and translation
    affine_mat = (np.matrix(trans_mat) * np.matrix(rot_mat))[0:2, :]

    # Apply the transform
    result = cv2.warpAffine(
        image,
        affine_mat,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR
    )

    return result

# c.f. https://stackoverflow.com/questions/16702966/rotate-image-and-crop-out-black-borders
def crop_around_center(image, width, height):
    """
    Given a NumPy / OpenCV 2 image, crops it to the given width and height,
    around it's centre point
    """

    image_size = (image.shape[1], image.shape[0])
    image_center = (int(image_size[0] * 0.5), int(image_size[1] * 0.5))

    if(width > image_size[0]):
        width = image_size[0]

    if(height > image_size[1]):
        height = image_size[1]

    x1 = int(image_center[0] - width * 0.5)
    x2 = int(image_center[0] + width * 0.5)
    y1 = int(image_center[1] - height * 0.5)
    y2 = int(image_center[1] + height * 0.5)

    return image[y1:y2, x1:x2]


def client(numIter):
	"""
    	Client script for the InteriorSim project. Calls for the creation of a virtual environment containing an openbot agent. 
    	Then executes a set of actions with the agent while gathering suitable observations. 
    	"""
	parser = argparse.ArgumentParser()
	
	parser.add_argument(
		"--user_config_file",
		help="Points to a user-defined .yaml file. This is used to override default config values.",
	)
	args = parser.parse_args()

	# create a single CfgNode that contains data from all config files
	config_node = CfgNode()

	# read UNREALAI config values
	with open(PACKAGE_DEFAULT_CONFIG_FILE, "r") as file_stream:
		config_node.UNREALAI = load_cfg(file_stream)

	# merge config from user-defined config values
	if args.user_config_file:
		config_node.merge_from_file(args.user_config_file)

	# this line will trigger many "WARNING:tornado.general:Connect error on fd 26: ECONNREFUSED" warnings, which can be ignored
	uenv = uai.UnrealEnv(config_node)

	# get current observation and reward
	agent_name = uenv.agents[0]
	print(uenv.get_obs_for_agent(agent_name=agent_name))
	print(uenv.current_reward)
	
	# Main loop, executing a set of random actions, getting observations and storing everything in a set of files: 
	
	# The agent makes the following observations:
	#
    # ---> left wheel commands in the range [-1, 1]
    # ---> right wheel commands in the range [-1, 1]
    # ---> Euclidean distance between current x-y position and target x-y position.
    # ---> Sinus of the relative yaw between current pose and target pose.
    # ---> Cosinus of the relative yaw between current pose and target pose.
	
	f_ctrl = open("sensor_data/ctrlLog.txt", 'w') # Low-level commands sent to the motors
	f_rgb = open("sensor_data/rgbFrames.txt", 'w') # Reference of the images correespoinding to each control input
	f_cmd = open("sensor_data/indicatorLog.txt", 'w') # High level commands 

	writer_ctrl = csv.writer(f_ctrl, delimiter=",")
	writer_ctrl.writerow( ('timestamp[ns]','leftCtrl','rightCtrl') )
	writer_rgb = csv.writer(f_rgb, delimiter=",")
	writer_rgb.writerow( ('timestamp[ns]','frame') )
	writer_cmd = csv.writer(f_cmd, delimiter=",")
	writer_cmd.writerow( ('timestamp[ns]','dist','sinYaw','cosYaw') )
	
	speedMultiplier = 255
	
	for i in range(numIter):
		ct = datetime.datetime.now() 
		ts = int(10000*ct.timestamp())
		command_x = random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. So far this is random bullshit... 
		command_y = random.uniform(-1.0, 1.0) # Should be in the [-1 1] range. So far this is random bullshit... 
 	
		# Send action to the agent:
		uenv.step(action={agent_name: [ np.array([command_x,command_y]) ] })
 	
		# Collect observation from the agent:
		Observation = uenv.get_obs_for_agent(agent_name=agent_name)
		
		# Write the low-level control observation into a file: 
		writer_ctrl.writerow( (ts, int(speedMultiplier*Observation[0][0]), int(speedMultiplier*Observation[0][1])) )
		
		# Write the corresponding image index into a file: 
		writer_rgb.writerow( (ts, i) )
		
		# Write the corresponding high level command into a file: 
		writer_cmd.writerow( (ts, Observation[0][2]/100, Observation[0][3], Observation[0][4]) )
		
		# Acquire picture, then rotate it:
		rot = rotate_image(Observation[1], -90)
		
		# Crop the rotated image so that it matches the Openbot CNN input:
		crop = crop_around_center(rot, 256, 96)
		
		# Save the image:
		im = Image.fromarray(crop)
		im.save("images/%d_crop.jpeg" % i)
		
	
	f_ctrl.close()
	f_rgb.close()


	# get current observation and reward
	print(uenv.get_obs_for_agent(agent_name=agent_name))
	print(uenv.current_reward)


	# close the environment
	uenv.close()
	
	return 0


if __name__ == "__main__":
    client(100)


