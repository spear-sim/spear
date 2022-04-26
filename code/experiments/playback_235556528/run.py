# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import csv
import cv2
import os
import random

from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import INTERIORSIM_ROOT_DIR

def read_data():
    pose_data_path = os.path.join(INTERIORSIM_ROOT_DIR, "../../experiments/run_235556528/data/sensor_data/pose_data.txt")
    image_data_path = os.path.join(INTERIORSIM_ROOT_DIR, "../../experiments/run_235556528/data/images")
    with open(pose_data_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        poses = {}
        images = {}
        nums = [23, 29, 31, 37]
        rand = nums[random.randint(a=0, b=len(nums)-1)]
        for index, row in enumerate(csv_reader):
            if index == 0:
                # print(f'Column names are {", ".join(row)}')
                continue
            if index % rand == 0:
                poses[index] = [float(i) for i in row[1:]]
                images[index] = cv2.imread(os.path.join(image_data_path, f"{index-1}.jpeg"))
                # print(f"Poses -> {poses[index]}")
                # cv2.imshow(f"index {index}", images[index])
                # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    return poses, images

if __name__ == "__main__":

    # load config
    config_files = [ os.path.join(INTERIORSIM_ROOT_DIR, "../../experiments/run_235556528/user_config.yaml") ]
    config = get_config(config_files)
    config.defrost()
    config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_235556528"
    config.freeze()

    # read data from a recorded session in 235556528
    poses, images = read_data()

    # create Env object
    env = Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()
    print(obs["visual_observation"].shape, obs["visual_observation"].dtype)

    cv2.imshow("reset observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # iterate over recorded poses
    for index, data in poses.items():
        if config.SIMULATION_CONTROLLER.OPENBOT_AGENT_CONTROLLER.ACTION_MODE == "teleport":
            obs, _, _, _ = env.step({"set_location_xyz": [data[0], data[1], data[2]], "set_rotation_pyr": [data[4], data[5], data[3]]}) # set_rotation_pyr: [pitch, yaw, roll]
        else:
            assert False

        cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.imshow(f"recorded image", images[index]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    # close the environment
    env.close()
