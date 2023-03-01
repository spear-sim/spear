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
import time

NUM_STEPS = 100

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--actions_file",default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "fetch_actions.csv"))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml")])

    df = pd.read_csv(args.actions_file)

    # create Env object
    env = spear.Env(config)
    print("env.action_space", env.action_space)
    print("env.observation_space", env.observation_space)

    # reset the simulation to get the first observation    
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
        cv2.waitKey(1)
        pass
    # take a few steps
    for i in range(df.shape[0]):
        if config.SIMULATION_CONTROLLER.AGENT == "UrdfBotAgent":
            action = {}
            data = df.to_records()[i]
            for dname in data.dtype.names:
                if dname.startswith("joint"):
                    action[dname] = np.array([data[dname]], dtype=np.float32)
            print(action)
            obs, reward, done, info = env.step(action=action)
            if not args.benchmark:
                print("[SPEAR | run.py] UrdfBotAgent: ")
                print("    ", obs["link_state.base_link"])
                print("    ", obs["camera.final_color"].shape, obs["camera.final_color"].dtype)
                print("    ", reward, done, info)
        else:
            assert False

        if not args.benchmark:
            cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
            cv2.waitKey(1)

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        print("[SPEAR | run.py] Average frame time: %0.4f ms (%0.4f fps)" % (
            (elapsed_time_seconds / df.shape[0]) * 1000.0, df.shape[0] / elapsed_time_seconds))
    else:
        cv2.destroyAllWindows()

    # close the environment
    env.close()

    print("[SPEAR | run.py] Done.")
