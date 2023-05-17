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


# Unreal Engine's rendering system assumes coherence between frames to achieve maximum image quality. 
# However, in this example, we are teleporting the camera in an incoherent way. Hence, we implement a 
# CustomEnv that can render multiple internal frames per step(), so that Unreal Engine's rendering
# system is warmed up by the time we get observations. Doing this can improve overall image quality
# due to Unreal's strategy of accumulating rendering information across multiple frames. These extra
# frames are not necessary in typical embodied AI scenarios, but are useful when teleporting a camera.
class CustomEnv(spear.Env):

    def __init__(self, config, num_internal_steps):
        super(CustomEnv, self).__init__(config)
        assert num_internal_steps > 0
        self._num_internal_steps = num_internal_steps

    def step(self, action):

        if self._num_internal_steps == 1:
            return self.single_step(action, get_observation=True)
        else:
            self.single_step(action)
            for _ in range(1, self._num_internal_steps - 1):
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
    parser.add_argument("--poses_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "poses.csv")))
    parser.add_argument("--images_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--num_internal_steps", type=int, default=1)
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--wait_for_key_press", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # read data from csv
    df = pd.read_csv(args.poses_file)

    # create dir for storing images
    if not args.benchmark:
        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
            render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
            shutil.rmtree(render_pass_dir, ignore_errors=True)
            os.makedirs(render_pass_dir)

    # iterate over all poses
    prev_scene_id = ""
    for pose in df.to_records():

        # if the scene_id of our current pose has changed, then create a new Env
        if pose["scene_id"] != prev_scene_id:

            # close the previous Env
            if prev_scene_id != "":
                env.close()

            # create Env object
            env = CustomEnv(config, num_internal_steps=args.num_internal_steps)

            # reset the simulation
            _ = env.reset()

            if args.benchmark and prev_scene_id == "":
                start_time_seconds = time.time()

        obs, _, _, _ = env.step(
            action={
                "set_position": np.array([pose["position_x"], pose["position_y"], pose["position_z"]], np.float64),
                "set_rotation": np.array([pose["rotation_pitch"], pose["rotation_yaw"], pose["rotation_roll"]], np.float64)})

        # save images for each render pass
        if not args.benchmark:
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, render_pass))
                assert os.path.exists(render_pass_dir)

                obs_render_pass = obs["camera." + render_pass]
                assert len(obs_render_pass.shape) == 3
                assert obs_render_pass.shape[2] == 4

                if render_pass == "depth":
                    obs_render_pass_vis = obs_render_pass[:,:,[0,1,2]].copy() # depth is returned as RGBA

                    # discard very large depth values
                    max_depth_meters = 20.0
                    obs_render_pass_vis = obs_render_pass_vis[:,:,0]
                    obs_render_pass_vis = np.clip(0.0, max_depth_meters, obs_render_pass_vis)

                elif render_pass == "final_color":
                    obs_render_pass_vis = obs_render_pass[:,:,[2,1,0]].copy() # final_color is returned as BGRA

                elif render_pass == "normal":
                    obs_render_pass_vis = obs_render_pass[:,:,[0,1,2]].copy() # normal is returned as RGBA

                    # discard normals that aren't properly normalized, i.e., length of 1.0
                    discard_mask = np.logical_not(np.isclose(np.linalg.norm(obs_render_pass_vis, axis=2), 1.0, rtol=0.001, atol=0.001))
                    obs_render_pass_vis = np.clip(0.0, 1.0, (obs_render_pass_vis + 1.0) / 2.0)
                    obs_render_pass_vis[discard_mask] = np.nan

                elif render_pass == "segmentation":
                    obs_render_pass_vis = obs_render_pass[:,:,[2,1,0]].copy() # final_color is returned as BGRA

                else:
                    assert False

                plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png"%pose["index"])), obs_render_pass_vis)

        # useful for comparing the game window to the image that has been saved to disk
        if args.wait_for_key_press:
            spear.log("Press any key to continue...")
            input()

        prev_scene_id = pose["scene_id"]

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(
            "Average frame time: %0.4f ms (%0.4f fps)" %
            ((elapsed_time_seconds / (df.shape[0]*args.num_internal_steps))*1000, (df.shape[0]*args.num_internal_steps) / elapsed_time_seconds))

    # close the current Env
    env.close()

    spear.log("Done.")
