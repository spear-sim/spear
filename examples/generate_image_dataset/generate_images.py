# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear
import sys
import time


# Unreal Engine's rendering system assumes coherence between frames to achieve maximum image quality. 
# However, in this example, we are teleporting the camera in an incoherent way. Hence, we implement a 
# CustomEnv that can render multiple internal frames per step(), so that Unreal Engine's rendering
# system is  warmed up by the time we get observations. Doing this can improve overall image quality
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
    parser.add_argument("--poses_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "poses.csv"))
    parser.add_argument("--images_dir", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "images"))
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--num_internal_steps", type=int)
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--wait_for_key_press", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # read data from csv
    df = pd.read_csv(args.poses_file, dtype={"scene_id":str})

    # do some config modifications based on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
        if args.num_internal_steps is None:
            num_internal_steps = 1
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 1.0
        config.freeze()
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
        if args.num_internal_steps is None:
            num_internal_steps = 5
        config.defrost()
        config.SIMULATION_CONTROLLER.CAMERA_SENSOR.FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY = 0.0
        config.freeze()
    else:
        assert False

    # if the user specified num_internal_steps, then use it
    if args.num_internal_steps is not None:
        num_internal_steps = args.num_internal_steps

    # iterate over all poses
    prev_scene_id = ""
    for pose in df.to_records():

        # if the scene_id of our current pose has changed, then create a new Env
        if pose["scene_id"] != prev_scene_id:

            # close the previous Env
            if prev_scene_id != "":
                env.close()

            # create dir for storing images
            if not args.benchmark:
                for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                    render_pass_dir = os.path.join(args.images_dir, render_pass)
                    os.makedirs(render_pass_dir, exist_ok=True)

            # change config based on current scene
            config.defrost()
            config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + pose["scene_id"] + rendering_mode_map_str + "." + "Map_" + pose["scene_id"] + rendering_mode_map_str
            config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + pose["scene_id"] + rendering_mode_map_str
            config.SIMULATION_CONTROLLER.SCENE_ID = pose["scene_id"]
            config.freeze()

            # create Env object
            env = CustomEnv(config, num_internal_steps=num_internal_steps)

            # reset the simulation
            _ = env.reset()

            if args.benchmark and prev_scene_id == "":
                start_time_seconds = time.time()

        # set the pose and obtain corresponding images
        obs, _, _, _ = env.step(
            action={
                "set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32),
                "set_num_random_points": np.array([0], np.uint32)})

        # save images for each render pass
        if not args.benchmark:
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.join(args.images_dir, render_pass)
                assert os.path.exists(render_pass_dir)
                plt.imsave(os.path.join(render_pass_dir, "%04d.png"%pose["index"]), obs["camera_" + render_pass].squeeze())

        # useful for comparing the game window to the image that has been saved to disk
        if args.wait_for_key_press:
            input()

        prev_scene_id = pose["scene_id"]

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        print("Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / (df.shape[0]*args.num_internal_steps))*1000, (df.shape[0]*args.num_internal_steps) / elapsed_time_seconds))

    # close the current Env
    env.close()

    print("Done.")
