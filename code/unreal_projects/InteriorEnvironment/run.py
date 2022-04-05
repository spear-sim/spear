"""
This is an example code to show how to launch an Unreal application using our configuration system.
Before running this file, please modify user_config.yaml.example -> user_config.yaml and update it with appropriate paths.
"""

import numpy as np
import os

from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import PACKAGE_ROOT_DIR


if __name__ == "__main__":

    # list of config files to be used 
    config_files = []

    # add default config files first and then user config files
    config_files.append(os.path.join(PACKAGE_ROOT_DIR, "../../examples/InteriorEnvironment/user_config.yaml"))

    # load configs
    config = get_config(config_files)

    # create unreal environment object
    env = Env(config)

    env.reset()

    print("Printing action space and observation space...")
    print(env.action_space)
    print(env.observation_space)

    # run few iterations
    for _ in range(10):
        _, _, done = env.step({"apply_force": [1, 1]})
        if done:
            env.reset()

    # close your unreal executable environment gracefully
    env.close()
