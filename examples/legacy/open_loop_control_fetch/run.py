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

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys
sys.path.append(common_dir)
import instance_utils
import visualization_utils


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

    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    instance_utils.open_level(instance, args.scene_id)
    env = spear.Env(instance, config)

    # reset the simulation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        if args.save_images:
            for render_pass in config.SP_SERVICES.LEGACY.URDF_ROBOT_AGENT.CAMERA.RENDER_PASSES:
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
            for render_pass in config.SP_SERVICES.LEGACY.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
                assert os.path.exists(render_pass_dir)
                if render_pass == "depth":
                    obs_render_pass_vis = visualization_utils.get_depth_image_for_visualization(obs["camera.depth"])
                elif render_pass == "final_color":
                    obs_render_pass_vis = visualization_utils.get_final_color_image_for_visualization(obs["camera.final_color"])
                elif render_pass == "normal":
                    obs_render_pass_vis = visualization_utils.get_normal_image_for_visualization(obs["camera.normal"])
                elif render_pass == "segmentation":
                    obs_render_pass_vis = visualization_utils.get_segmentation_image_for_visualization(obs["camera.segmentation"])
                else:
                    assert False
                plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png" % index)), obs_render_pass_vis)
            index += 1

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / df.shape[0])*1000.0, df.shape[0] / elapsed_time_seconds))

    # close the environment
    env.close()

    # close the instance
    instance.close()

    spear.log("Done.")
