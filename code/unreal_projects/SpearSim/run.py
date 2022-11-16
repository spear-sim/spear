# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os

from spear import Env
from spear.config import get_config
from spear.constants import SPEAR_ROOT_DIR


if __name__ == "__main__":

    # load config
    config_files = [ os.path.join(SPEAR_ROOT_DIR, "../../unreal_projects/SpearSim/user_config.yaml") ]
    config = get_config(config_files)

    # create Env object
    env = Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()
    if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
        print(obs["camera_final_color"].shape, obs["camera_final_color"].dtype)

    elif config.SIMULATION_CONTROLLER.AGENT == "OpenBotAgent":
        print(obs["state_data"], obs["control_data"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype)

    else:
        assert False

    cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)

    # take a few steps
    for i in range(100):
        if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
            obs, reward, done, info = env.step({"apply_force": np.array([1, 1], dtype=np.float32)})
            print(obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)

        elif config.SIMULATION_CONTROLLER.AGENT == "OpenBotAgent":
            obs, reward, done, info = env.step({"apply_voltage": np.array([1, 1], dtype=np.float32)})
            print(obs["state_data"], obs["control_data"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)

        else:
            assert False

        cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()
