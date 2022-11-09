# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import cv2
import numpy as np
import os
import spear

if __name__ == "__main__":

    # load config (this function will load the parameter values specified in user_config.yaml, as well as sensible defaults for all other parameters)
    config = spear.get_config(user_config_files=[ os.path.join(spear.SPEAR_ROOT_DIR, "../../unreal_projects/PlayEnvironment/user_config.yaml") ])

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()
    print(obs["compass"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype)

    cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
    cv2.waitKey(0)

    # take a few steps (you should see the sphere move in the Unreal game window)
    for i in range(10):
        obs, reward, done, info = env.step({"apply_force": np.array([1, 1], dtype=np.float32)})
        print(obs["compass"], obs["camera_final_color"].shape, obs["camera_final_color"].dtype, reward, done, info)

        cv2.imshow("camera_final_color", obs["camera_final_color"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    # close the environment
    env.close()
