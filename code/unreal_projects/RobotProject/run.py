"""
This is an example code to show how to launch an Unreal application using our configuration system.
Before running this file, please modify user_config.yaml.example -> user_config.yaml and update it with appropriate paths.
"""

import cv2
import numpy as np
import os

from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import INTERIORSIM_ROOT_DIR


if __name__ == "__main__":

    # list of config files to be used 
    config_files = []

    # add default config files first and then user config files
    config_files.append(os.path.join(INTERIORSIM_ROOT_DIR, "../../unreal_projects/RobotProject/user_config.yaml"))

    # load configs
    config = get_config(config_files)

    # create unreal environment object
    env = Env(config)

    env.reset()

    # run few iterations
    for i in range(100):
        obs, reward, done, step_info = env.step({"apply_voltage": [1, 1]})
        print(obs["visual_observation"].shape, obs["visual_observation"].dtype, reward, done, step_info)

        cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

        if done or i%23==0:
            print('resetting ....')
            env.reset()

    cv2.destroyAllWindows()

    # close your unreal executable environment gracefully
    env.close()
