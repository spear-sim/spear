#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import spear

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--actions_file",
                        default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "actions.csv"))
    parser.add_argument("--observation_file",
                        default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "observations.csv"))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml")])

    df = pd.read_csv(args.actions_file)

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()

    cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
    cv2.waitKey(1)
    # take a few steps
    for i in range(df.shape[0]):
        if config.SIMULATION_CONTROLLER.AGENT == "UrdfBotAgent":
            action = {}
            data = df.to_records()[i]
            for action_key in data.dtype.names:
                if action_key.startswith("joint"):
                    action[action_key] = np.array([data[action_key]], dtype=np.float32)

            obs, reward, done, info = env.step(action=action)

            print("[SPEAR | run.py] UrdfBotAgent: ", reward, done, info)
        else:
            assert False

        cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
        cv2.waitKey(1)

        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()

    print("[SPEAR | run.py] Done.")
