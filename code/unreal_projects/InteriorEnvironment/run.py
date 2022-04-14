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
    config_files.append(os.path.join(INTERIORSIM_ROOT_DIR, "../../unreal_projects/InteriorEnvironment/user_config.yaml"))

    # load configs
    config = get_config(config_files)

    # create unreal environment object
    env = Env(config)

    env.reset()

    # run few iterations
    for _ in range(100):
        obs, reward, done, _ = env.step({"apply_force": [1, 1]})
        cv2.imshow("rgb image", obs["visual_observation"].astype(np.uint8))
        cv2.waitKey(0)
        print(reward, done)
        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close your unreal executable environment gracefully
    env.close()
