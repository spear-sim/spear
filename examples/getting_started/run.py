# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os
import spear

if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()

    cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)

    # take a few steps
    for i in range(100):
        if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
            obs, reward, done, info = env.step({"apply_force": np.array([1, 1], dtype=np.float32)})
            print("SphereAgent: ", obs["compass"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)
        elif config.SIMULATION_CONTROLLER.AGENT == "OpenBotAgent":
            obs, reward, done, info = env.step({"apply_voltage": np.array([1, 1], dtype=np.float32)})
            print("OpenBotAgent: ", obs["state_data"], obs["control_data"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)
        else:
            assert False

        cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()
