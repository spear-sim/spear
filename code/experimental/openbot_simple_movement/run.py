#############################################################################

# Use python script to control an OpenBot agent and get egocentric visual observations.

#############################################################################

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
#
# python run.py -i 10

import argparse
import cv2
import numpy as np
import os

from interiorsim import Env
from interiorsim.config import get_config

if __name__ == "__main__":

    # List of config files to be used
    config_files = []

    # Add default config files first and then user config files
    config_files.append(os.path.join(os.getcwd(), "user_config.yaml"))

    # Load configs
    config = get_config(config_files)
    # print(config)

    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--iterations", type=int, help="number of iterations through the environment",
                        required=True)
    parser.add_argument("-c", "--connect", type=int, help="id of the connection port", required=True)
    args = parser.parse_args()

    config.defrost()
    config.SIMULATION_CONTROLLER.PORT = args.connect
    config.freeze()

    numIter = args.iterations

    # Load the observation mode:
    config.defrost()
    config.SIMULATION_CONTROLLER.OPENBOT_AGENT_CONTROLLER.PHYSICAL_OBSERVATION_MODE = "full-pose"
    config.freeze()

    if not os.path.exists("img"):
        os.mkdir("img")

    # Create Env object:
    env = Env(config)

    # Reset the simulation to get the first observation
    obs = env.reset()
    print('hello obs', obs)

    cv2.imshow("visual_observation",
               obs["visual_observation"][:, :, [2, 1, 0]])  # OpenCV expects BGR instead of RGB
    # cv2.waitKey(0)

    # Take a few steps:
    for i in range(numIter):
        print(f"iteration {i} over {numIter}")

        # Send action to the agent and collect observations:
        action = np.array([1.0, 1.0], dtype=np.float32)
        obs, reward, done, info = env.step({"apply_voltage": action})
        print(obs["visual_observation"].shape, obs["visual_observation"].dtype, reward, done, info)

        cv2.imshow("visual_observation",
                   obs["visual_observation"][:, :, [2, 1, 0]])  # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)
        if i % 10 == 0:
            cv2.imwrite(f"img/{i}.jpg", obs["visual_observation"][:, :, [2, 1, 0]])

    cv2.destroyAllWindows()

    # Close the environment:
    env.close()
