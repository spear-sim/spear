# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import os

from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import INTERIORSIM_ROOT_DIR


if __name__ == "__main__":

    # load config
    config_files = [ os.path.join(INTERIORSIM_ROOT_DIR, "../../unreal_projects/RobotProject/user_config.yaml") ]
    config = get_config(config_files)

    # create Env object
    env = Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()
    print(obs["visual_observation"].shape, obs["visual_observation"].dtype)

    cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)

    # take a few steps
    for i in range(100):
        obs, reward, done, info = env.step({"apply_voltage": [1, 1]})
        print(obs["visual_observation"].shape, obs["visual_observation"].dtype, reward, done, info)

        cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()
