#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
import spear
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--images_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--scene_id", default="apartment_0000")
    parser.add_argument("--save_images", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # update scene_id
    config.defrost()
    config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
    config.freeze()
    
    # create Env object
    env = spear.Env(config)
    
    spear.log(env.action_space)

    # reset the simulation to get the first observation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        if args.save_images:
            for render_pass in config.SIMULATION_CONTROLLER.URDF_ROBOT_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
                shutil.rmtree(render_pass_dir, ignore_errors=True)
                os.makedirs(render_pass_dir)

    # build action space
    action = {}
    for action_name, desc in env.action_space.items():
        action[action_name] = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    for _ in range(100):
        action["arm_joint_0.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        obs, reward, done, info = env.step(action=action)
        if done:
            env.reset()

    # for _ in range(100):
    #     action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.1, 0.0, 0.0], dtype=np.float64)
    #     action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.1, 0.0, 0.0], dtype=np.float64)
    #     obs, reward, done, info = env.step(action=action)
    #     if done:
    #         env.reset()

        # # save images for each render pass
        # if not args.benchmark and args.save_images:
        #     for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
        #         render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
        #         assert os.path.exists(render_pass_dir)

        #         obs_render_pass_vis = spear.get_image_data(render_pass, obs)
        #         # plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png"%pose["index"])), obs_render_pass_vis)


    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        # spear.log("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / df.shape[0])*1000.0, df.shape[0] / elapsed_time_seconds))

    # close the environment
    env.close()

    spear.log("Done.")
