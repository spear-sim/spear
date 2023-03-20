#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import shutil

import cv2
import numpy as np
import os
import pandas as pd
import spear

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "actions.csv"))
    parser.add_argument("--save_images", default=True)
    parser.add_argument("--image_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "images"))
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml")])

    # get pregenerated actions for fetch urdf agent
    df = pd.read_csv(args.actions_file)

    if args.save_images:
        if os.path.exists(args.image_dir):
            shutil.rmtree(args.image_dir)
        os.makedirs(args.image_dir)

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation    
    obs = env.reset()

    cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
    cv2.waitKey(1)

    # take a few steps
    if config.SIMULATION_CONTROLLER.AGENT != "UrdfBotAgent":
        assert False

    for index ,row in df.iterrows():
        action = {}
        for k in row.to_dict():
            action[k] = np.array([row[k]], dtype=np.float32)
        obs, reward, done, info = env.step(action=action)

        print("[SPEAR | run.py] UrdfBotAgent: ", reward, done, info)

        cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
        cv2.waitKey(1)

        if args.save_images:
            cv2.imwrite(os.path.join(args.image_dir, f"{index:04d}.jpeg"), obs["camera.final_color"])
        if done:
            env.reset()

    cv2.destroyAllWindows()

    # close the environment
    env.close()

    print("[SPEAR | run.py] Done.")
