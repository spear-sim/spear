# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import csv
import cv2
import datetime
import numpy as np
import os
import subprocess
import time

from interiorsim import Env
from interiorsim.config import get_config

def read_recorded_data(scene):
    pose_data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/pose.txt")
    image_data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/images/SEG")

    with open(pose_data_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        poses = {}
        images = {}
        for index, row in enumerate(list(csv_reader)[1:]):
            poses[index] = [float(i) for i in row[2:]]
            images[index] = cv2.imread(os.path.join(image_data_path, f"{index}.png"))
            
    return poses, images
    
if __name__ == "__main__":

    # read scenes list
    scene_list_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scenes.txt")
    with open(scene_list_path, 'r') as f:
        csv_reader = csv.reader(f, delimiter=',')
        scenes = next(csv_reader)
        scenes = scenes[:-1]   

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    for scene in scenes:
        
        print()
        print(f"running through scene {scene}...")

        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set first scene in the list as starting scene
        config.freeze()

        # check if path exists
        assert os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}"))
        assert config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.ACTION_MODE == "replay_sampled_images"

        poses, images = read_recorded_data(scene)

        # create Env object
        env = Env(config)

        # reset the simulation
        _ = env.reset()

        # iterate over recorded poses
        for index, data in poses.items():

            obs, _, _, _ = env.step({"set_position_xyz_centimeters": [data[0], data[1], data[2]], "set_orientation_pyr_degrees": [data[4], data[5], data[3]]}) # set_orientation_pyr_degrees: [pitch, yaw, roll]

            assert obs["pose"].all() == np.array(data).all()

            cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            cv2.imshow(f"recorded image", images[index]) # OpenCV expects BGR instead of RGB
            cv2.waitKey(0)

            # write data
            # ts = datetime.datetime.now().timestamp() * 1e9
            # return_status = cv2.imwrite(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scenes[count-1]}/images/{i%100}.png"), obs["visual_observation"])
            # assert return_status == True

        env.close()
        time.sleep(5)
    
    # cv2.destroyAllWindows()
