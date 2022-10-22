# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os

from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import INTERIORSIM_ROOT_DIR


if __name__ == "__main__":

    # load config
    config_files = [ os.path.join(INTERIORSIM_ROOT_DIR, "../../unreal_projects/PlayEnvironment/user_config.yaml") ]
    config = get_config(config_files)

    # create Env object
    env = Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()
    print(obs["visual_observation_final_color"].shape, obs["visual_observation_final_color"].dtype)

    cv2.imshow("visual_observation_final_color", obs["visual_observation_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)

    # take a few steps
    for i in range(10):
        obs, reward, done, info = env.step({"apply_force": np.array([1, 1], dtype=np.float32)})
        print(obs["visual_observation_final_color"].shape, obs["visual_observation_final_color"].dtype, reward, done, info)

        cv2.imshow("visual_observation_final_color", obs["visual_observation_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()
