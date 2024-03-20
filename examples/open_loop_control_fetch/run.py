#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

# import observation_utils from common folder
common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
import sys
sys.path.append(common_dir)
import common.observation_utils as observation_utils


def get_action(row):
    names = [ name[:-2] for name in row.dtype.names ][::3] # strip .x .y .z from each name, select every third entry
    data = np.array([ row[name] for name in row.dtype.names ], dtype=np.float64).reshape(-1,3) # get data as Nx3 array
    return dict(zip(names, data))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    parser.add_argument("--images_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--scene_id", default="apartment_0000")
    parser.add_argument("--save_images", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # read pre-recorded actions from the actions file
    df = pd.read_csv(args.actions_file)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # update scene_id
    config.defrost()
    config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
    config.freeze()
    
    # create Env object
    env = spear.Env(config)

    # reset the simulation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        if args.save_images:
            for render_pass in config.SIMULATION_CONTROLLER.URDF_ROBOT_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
                shutil.rmtree(render_pass_dir, ignore_errors=True)
                os.makedirs(render_pass_dir)

    spear.log("Executing sequence of actions as provided in the actions file...")

    if not args.benchmark and args.save_images:
        index = 0

    for row in df.to_records(index=False):
        action = get_action(row)
        obs, reward, done, info = env.step(action=action)

        # save images for each render pass
        if not args.benchmark and args.save_images:
            observation_components_to_modify = { render_pass: ["camera." + render_pass] for render_pass in config.SIMULATION_CONTROLLER.URDF_ROBOT_AGENT.CAMERA.RENDER_PASSES }
            modified_obs = observation_utils.get_observation_components_modified_for_visualization(obs, observation_components_to_modify)
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
                assert os.path.exists(render_pass_dir)

                obs_render_pass_vis = modified_obs["camera." + render_pass]
                plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png"%index)), obs_render_pass_vis)
            index += 1

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / df.shape[0])*1000.0, df.shape[0] / elapsed_time_seconds))

    # close the environment
    env.close()

    spear.log("Done.")
