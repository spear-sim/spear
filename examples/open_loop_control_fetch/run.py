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

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.kujiale_0000.csv")))
    parser.add_argument("--image_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--scene_id", default="kujiale_0000")
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--save_images", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # get pregenerated actions for fetch urdf agent
    df = pd.read_csv(args.actions_file)

    # do some config modifications based on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 1.0
        config.freeze()
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 0.0
        config.freeze()
    else:
        assert False

    # change config based on current scene
    config.defrost()

    if args.scene_id == "starter_content_0000":
        # starter_content_0000 doesn't need a rendering mode when referring to its map
        config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
        config.SIMULATION_CONTROLLER.MAP_ID   = args.scene_id

        # starter_content_0000 has scene-specific config values
        scene_config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scene_config.starter_content_0000.yaml")

    elif args.scene_id == "kujiale_0000":
        config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
        config.SIMULATION_CONTROLLER.MAP_ID   = args.scene_id + rendering_mode_map_str

        # kujiale_0000 has scene-specific config values
        scene_config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scene_config.kujiale_0000.yaml")

    config.merge_from_file(scene_config_file)
    config.freeze()

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        if args.save_images:
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.image_dir, render_pass))
                shutil.rmtree(render_pass_dir, ignore_errors=True)
                os.makedirs(render_pass_dir)

    for i, row in df.iterrows():
        action = {k: np.array([v], dtype=np.float32) for k, v in row.to_dict().items()}
        obs, reward, done, info = env.step(action=action)

        # save images for each render pass
        if not args.benchmark and args.save_images:
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.image_dir, render_pass))
                assert os.path.exists(render_pass_dir)

                obs_render_pass = obs["camera." + render_pass].squeeze()
                if render_pass in ["final_color", "normals", "segmentation"]:
                    assert len(obs_render_pass.shape) == 3
                    assert obs_render_pass.shape[2] == 4
                    obs_render_pass = obs_render_pass[:, :, [2, 1, 0, 3]].copy()  # note that spear.Env returns BGRA by default

                plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png" % i)), obs_render_pass)

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / df.shape[0])*1000.0, df.shape[0] / elapsed_time_seconds))

    # close the environment
    env.close()

    spear.log("Done.")
