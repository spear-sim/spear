# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import shutil
import sys

from interiorsim import Env
from interiorsim.config import get_config


if sys.platform == "linux":
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"


# Unreal Engine's rendering system assumes coherence between frames to achieve maximum image quality. 
# However, in this example, we are teleporting the camera in an incoherent way. Hence, we implement 
# a CustomEnv that takes extra steps (i.e., renders extra frames) so that Unreal Engine's rendering system 
# is warmed up by the time we get observations. Doing this avoids, texture pop-in, auto-exposure artifacts,
# and improves overall image quality due to temporal anti-aliasing. These extra steps are not necessary in 
# typical embodied AI scenarios, but are necessary when teleporting a camera.
class CustomEnv(Env):

    def __init__(self, config, num_extra_single_steps):
        super(CustomEnv, self).__init__(config)
        self.num_extra_single_steps = num_extra_single_steps

    def step(self, action):

        self.single_step(action)
        for _ in range(self.num_extra_single_steps):
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
    parser.add_argument("--paks_dir", type=str, required=True)
    parser.add_argument("--executable_content_dir", type=str, required=True)
    parser.add_argument("--poses_file", type=str, required=True)
    parser.add_argument("--output_dir", "-o", type=str, required=True)
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    # read data from csv
    df = pd.read_csv(args.poses_file)

    # iterate over all scenes in poses_file
    for scene in df["map_id"].unique():

        print(f"processing scene {scene}")

        # change config based on current scene
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene)
        config.freeze()

        # copy pak to the executable dir as this is required for launching the appropriate pak file
        assert os.path.exists(f"{args.executable_content_dir}/Paks")
        if not os.path.exists(f"{args.executable_content_dir}/Paks/{scene}_{PLATFORM}.pak"):
            shutil.copy(os.path.join(args.paks_dir, f"{scene}/paks/{PLATFORM}/{scene}/{scene}_{PLATFORM}.pak"), f"{args.executable_content_dir}/Paks")

        # create dir for storing images
        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
            if not os.path.exists(os.path.join(args.output_dir, f"{scene}/{render_pass}")):
                os.makedirs(os.path.join(args.output_dir, f"{scene}/{render_pass}"))

        # create Env object
        env = CustomEnv(config, num_extra_single_steps=0)

        # reset the simulation
        _ = env.reset()

        # iterate over all poses for this scene once before getting observations to warm-up the Unreal Engine rendering system
        for pose in df.loc[df["map_id"] == scene].to_records():
            env.single_step(action={"set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32), "set_num_random_points": np.array([0], np.uint32)}, get_observation=False)

        # iterate over all poses to capture images
        for pose in df.loc[df["map_id"] == scene].to_records():

            # set the pose and obtain corresponding images
            obs, _, _, _ = env.single_step(action={"set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32), "set_num_random_points": np.array([0], np.uint32)}, get_observation=True)

            # view image
            # cv2.imshow(f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]", obs[f"visual_observation_{config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES[0]"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.waitKey(0)
            
            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                output_path = os.path.join(args.output_dir, f"{scene}/{render_pass}")
                assert os.path.exists(output_path)
                return_status = cv2.imwrite(output_path +f"/{pose['index']}.png", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                assert return_status == True

        #cv2.destroyAllWindows()

        # close the current scene
        env.close()

        # remove copied pak file from exectuable dir's Content folder as it is no longer required
        os.remove(f"{args.executable_content_dir}/Paks/{scene}_{PLATFORM}.pak")
    