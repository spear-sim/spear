# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear
import sys


# Unreal Engine's rendering system assumes coherence between frames to achieve maximum image quality. 
# However, in this example, we are teleporting the camera in an incoherent way. Hence, we implement a 
# CustomEnv that can render multiple frames per step, so that Unreal Engine's rendering system is 
# warmed up by the time we get observations. Doing this improves overall image quality due to Unreal's
# use of temporal anti-aliasing. These extra frames are not necessary in typical embodied AI scenarios, 
# but are necessary when teleporting a camera.
class CustomEnv(spear.Env):

    def __init__(self, config, num_internal_steps):
        super(CustomEnv, self).__init__(config)
        assert num_internal_steps > 0
        self.num_internal_steps = num_internal_steps

    def step(self, action):

        if self.num_internal_steps == 1:
            return self.single_step(action, get_observation=True)
        else:
            self.single_step(action)
            for _ in range(1, self.num_internal_steps - 1):
                self.single_step()
            return self.single_step(get_observation=True)

    def single_step(self, action=None, get_observation=False):
    
        self._begin_tick()
        if action:
            self._apply_action(action)
        self._tick()
        if get_observation:
            obs = self._get_observation()
            reward = self._get_reward()
            is_done = self._is_episode_done()
            step_info = self._get_step_info()
            self._end_tick()
            return obs, reward, is_done, step_info
        else:
            self._end_tick()
            return None, None, None, None


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "poses.csv"))
    parser.add_argument("--images_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "images"))
    parser.add_argument("--num_internal_steps", type=int, default=1)
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # read data from csv
    df = pd.read_csv(args.poses_file, dtype={"scene_id":str})

    # iterate over all poses
    scene_id = ""
    for pose in df.to_records():

        # if the scene_id of our current pose has changed, then create a new Env
        if pose["scene_id"] != scene_id:

            # close the current Env
            if scene_id != "":
                env.close()

            scene_id = pose["scene_id"]

            # create dir for storing images
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                image_dir = os.path.join(args.images_dir, scene_id, render_pass)
                os.makedirs(image_dir, exist_ok=True)

            # change config based on current scene
            config.defrost()
            config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + "." + "Map_" + scene_id
            config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id
            config.freeze()

            # create Env object
            env = CustomEnv(config, num_internal_steps=args.num_internal_steps)

            # reset the simulation
            _ = env.reset()

        # set the pose and obtain corresponding images
        obs, _, _, _ = env.step(
            action={
                "set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32),
                "set_num_random_points": np.array([0], np.uint32)})

        # save images for each render pass
        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
            image_dir = os.path.join(args.images_dir, scene_id, render_pass)
            assert os.path.exists(image_dir)
            plt.imsave(os.path.join(image_dir, "%04d.png"%pose["index"]), obs["camera_" + render_pass])

    # close the current Env
    env.close()

    print("Done.")
