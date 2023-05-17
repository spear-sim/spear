#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear
import time


NUM_STEPS = 100


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        cv2.imshow("camera.final_color", obs["camera.final_color"]) # note that spear.Env returns BGRA by default
        cv2.waitKey(0)

    # take a few steps
    for i in range(NUM_STEPS):
        if config.SIMULATION_CONTROLLER.AGENT == "SphereAgent":
            obs, reward, done, info = env.step(action={"apply_force": np.array([1.0, 1.0], dtype=np.float32)})
            if not args.benchmark:
                spear.log("SphereAgent: ")
                spear.log(obs["compass"])
                spear.log(obs["camera.final_color"].shape, obs["camera.final_color"].dtype)
                spear.log(reward, done, info)
        elif config.SIMULATION_CONTROLLER.AGENT == "OpenBotAgent":
            obs, reward, done, info = env.step(action={"apply_voltage": np.array([1.0, 0.715], dtype=np.float32)})
            if not args.benchmark:
                spear.log("OpenBotAgent: ")
                spear.log(obs["state_data"])
                spear.log(obs["control_data"])
                spear.log(obs["camera.final_color"].shape, obs["camera.final_color"].dtype)
                spear.log(reward, done, info)
        else:
            assert False

        if not args.benchmark:
            cv2.imshow("camera.final_color", obs["camera.final_color"]) # note that spear.Env returns BGRA by default
            cv2.waitKey(0)

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / NUM_STEPS)*1000.0, NUM_STEPS / elapsed_time_seconds))
    else:
        cv2.destroyAllWindows()

    # close the environment
    env.close()

    spear.log("Done.")
