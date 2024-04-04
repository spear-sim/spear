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

# import OpenBotEnv from common folder
common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
import sys
sys.path.append(common_dir)
from common.openbot_env import OpenBotEnv

num_steps = 100


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # create spear.Instance object
    sp_instance = spear.Instance(config)

    # create or get gym object
    if config.SP_ENGINE.LEGACY_SERVICE.AGENT == "SphereAgent":
        env = spear.Env(config, sp_instance.engine_service)
    elif config.SP_ENGINE.LEGACY_SERVICE.AGENT == "VehicleAgent":
        env = OpenBotEnv(config, sp_instance.engine_service)

    # reset the simulation to get the first observation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        cv2.imshow("camera.final_color", obs["camera.final_color"]) # note that spear.Env returns BGRA by default
        cv2.waitKey(0)

    # take a few steps
    for i in range(num_steps):
        if config.SP_ENGINE.LEGACY_SERVICE.AGENT == "SphereAgent":
            obs, reward, done, info = env.step(action={
                "add_force": np.array([10000.0, 0.0, 0.0], dtype=np.float64),
                "add_to_rotation": np.array([0.0, 1.0, 0.0])
            })
            if not args.benchmark:
                spear.log("SphereAgent:")
                spear.log("    camera.final_color: ", obs["camera.final_color"].shape, " ", obs["camera.final_color"].dtype)
                spear.log("    location:           ", obs["location"])
                spear.log("    rotation:           ", obs["rotation"])
                spear.log("    reward:             ", reward)
                spear.log("    done:               ", done)
                spear.log("    info:               ", info.keys())
        elif config.SP_ENGINE.LEGACY_SERVICE.AGENT == "VehicleAgent":
            obs, reward, done, info = env.step(action={"set_duty_cycles": np.array([1.0, 0.715], dtype=np.float64)})
            if not args.benchmark:
                spear.log("VehicleAgent:")
                spear.log("    camera.final_color:    ", obs["camera.final_color"].shape, " ", obs["camera.final_color"].dtype)
                spear.log("    location:              ", obs["location"])
                spear.log("    rotation:              ", obs["rotation"])
                spear.log("    wheel_rotation_speeds: ", obs["wheel_rotation_speeds"])
                spear.log("    reward:                ", reward)
                spear.log("    done:                  ", done)
                spear.log("    info:                  ", info.keys())
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

    # close the unreal instance and rpc connection
    sp_instance.close()

    spear.log("Done.")
