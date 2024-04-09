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

from policies import *
from utils import *

# import OpenBotEnv, observation_utils from common folder
common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
import sys
sys.path.append(common_dir)
from common.openbot_utils import OpenBotEnv
import common.observation_utils as observation_utils
from common.instance_utils import open_level


if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_iterations_per_episode", type=int, default=500)
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "test_episodes.csv")))
    parser.add_argument("--policy_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "trained_policy.tflite")))
    parser.add_argument("--eval_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "eval")))
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--create_videos", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    config.defrost()
    config.IMITATION_LEARNING_OPENBOT.PILOT_NET_POLICY_PATH = os.path.realpath(args.policy_file)
    config.SP_ENGINE.LEGACY_SERVICE.TASK = "ImitationLearningTask"
    config.SP_ENGINE.LEGACY.IMITATION_LEARNING_TASK.EPISODES_FILE = os.path.realpath(args.episodes_file)
    config.freeze()

    if args.debug:
        config.defrost()
        config.SP_ENGINE.LEGACY.IMU_SENSOR.DEBUG_RENDER = True # only has an effect in Development mode, not shipping mode
        config.SP_ENGINE.LEGACY.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT = 512
        config.SP_ENGINE.LEGACY.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH = 512
        config.SP_ENGINE.LEGACY.VEHICLE_AGENT.CAMERA.RENDER_PASSES = ["depth", "final_color", "segmentation"]
        config.SP_ENGINE.LEGACY.VEHICLE_AGENT.OBSERVATION_COMPONENTS = ["camera", "imu", "location", "rotation", "wheel_rotation_speeds"]
        # aim camera in a third-person view facing backwards at an angle
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Y = -50.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.POSITION_Z = 45.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.PITCH = -35.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.YAW = 45.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROLL = 0.0
        config.freeze()

    if not args.benchmark:
        shutil.rmtree(args.eval_dir, ignore_errors=True)

    # load driving policy
    policy = OpenBotPilotNetPolicy(config)

    # load the episodes to be executed
    df = pd.read_csv(args.episodes_file)

    spear.configure_system(config)
    instance = spear.Instance(config)
    env = OpenBotEnv(config=config, instance=instance)

    # iterate over all episodes
    prev_scene_id = ""
    for episode in df.to_records():

        spear.log(f"Running trained policy on episode {episode['index']} of {df.shape[0]}...")

        # if the scene_id of our current episode has changed, then create a new Env
        if episode["scene_id"] != prev_scene_id:

            # close the previous Env
            env.close()

            # open the desired level
            open_level(instance, episode["scene_id"])

            # create Env object
            env = OpenBotEnv(config=config, instance=instance)

        # now that we have checked if we need to create a new Env, we can update prev_scene_id
        prev_scene_id = episode["scene_id"]

        # reset the simulation
        env_reset_info = {}
        obs = env.reset(reset_info=env_reset_info)
        assert "success" in env_reset_info
        assert env_reset_info["success"]

        # per-episode state
        episode_initial_location = np.array([episode["initial_location_x"], episode["initial_location_y"], episode["initial_location_z"]], dtype=np.float64).reshape(1,3)
        episode_goal_location = np.array([episode["goal_location_x"], episode["goal_location_y"], episode["goal_location_z"]], dtype=np.float64).reshape(1,3)
        episode_skip = False

        # check conditions for skipping the episode
        cm_to_m = 0.01
        goal_reached = np.linalg.norm(episode_goal_location[0, 0:2] - obs["location"][0:2]) * cm_to_m <= config.IMITATION_LEARNING_OPENBOT.GOAL_REACHED_RADIUS
        if goal_reached:
            spear.log("    Goal reached before calling env.step(), skipping episode...")
            episode_skip = True

        # if we aren't skipping the episode
        if not episode_skip:

            path = instance.legacy_service.get_paths(episode_initial_location, episode_goal_location)[0]

            # initialize the policy with the desired goal location
            policy.reset(episode_goal_location)

            if args.benchmark:
                start_time_seconds = time.time()
            else:
                # create dirs for storing data
                episode_dir = os.path.realpath(os.path.join(args.eval_dir, episode["scene_id"], "%04d" % episode["index"]))
                images_dir  = os.path.realpath(os.path.join(episode_dir, "images"))
                logs_dir    = os.path.realpath(os.path.join(episode_dir, "logs"))
                plots_dir   = os.path.realpath(os.path.join(episode_dir, "plots"))
                os.makedirs(images_dir, exist_ok=True)
                os.makedirs(logs_dir, exist_ok=True)
                os.makedirs(plots_dir, exist_ok=True)

                # Our convention in this example is to store all data that comes directly from Unreal in the native format
                # exported by Unreal, i.e., centimeters and degrees. We eventually need to convert some of this data to
                # meters and radians, but we only do so in local temporary variables.
                episode_control_data               = np.empty([args.num_iterations_per_episode, 2], dtype=np.float64)
                episode_location_data              = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64)
                episode_rotation_data              = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64)
                episode_env_step_info_hit_goal     = np.empty(args.num_iterations_per_episode, dtype=np.uint8)
                episode_env_step_info_hit_obstacle = np.empty(args.num_iterations_per_episode, dtype=np.uint8)
                episode_goal_reached               = np.empty(args.num_iterations_per_episode, dtype=np.uint8)

            # execute episode
            num_iterations_executed = 0
            for i in range(args.num_iterations_per_episode):

                spear.log(f"    Executing iteration {i} of {args.num_iterations_per_episode}...")

                # update control action 
                action = policy.step(obs)

                # send control action to the agent and collect observations
                obs, _, _, env_step_info = env.step(action={"set_duty_cycles": action})

                num_iterations_executed += 1

                # check if we've reached the goal
                cm_to_m = 0.01
                goal_reached = np.linalg.norm(episode_goal_location[0, 0:2] - obs["location"][0:2]) * cm_to_m <= config.IMITATION_LEARNING_OPENBOT.GOAL_REACHED_RADIUS

                if args.debug:
                    observation_components_to_modify = {"final_color": ["camera.final_color"]}
                    modified_obs = observation_utils.get_observation_components_modified_for_visualization(obs, observation_components_to_modify)
                    show_obs(modified_obs)

                if not args.benchmark:
                    observation_components_to_modify = {"final_color": ["camera.final_color"]}
                    modified_obs = observation_utils.get_observation_components_modified_for_visualization(obs, observation_components_to_modify)
                    obs_final_color = modified_obs["camera.final_color"]

                    # save the collected rgb observations
                    plt.imsave(os.path.realpath(os.path.join(images_dir, "%04d.jpg"%i)), obs_final_color)

                    # During an episode, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training
                    # purposes. In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole
                    # path. In this case, rather than considering the whole episode as a fail, one can consider the last position reached by the agent as
                    # the new goal position. Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather
                    # than directly writing all the observations in a file iteration by iteration, we append these observations in a buffer, named "observation"
                    # to later process them once the episode is completed.
                    episode_control_data[i]               = action                               # [ctrl_left, ctrl_right]
                    episode_location_data[i]              = obs["location"]                      # [x, y, z] in cms
                    episode_rotation_data[i]              = obs["rotation"]                      # [pitch, yaw, roll] in degs
                    episode_env_step_info_hit_goal[i]     = env_step_info["task_step_info"]["hit_goal"]
                    episode_env_step_info_hit_obstacle[i] = env_step_info["task_step_info"]["hit_obstacle"]
                    episode_goal_reached[i]               = goal_reached

                # check conditions for ending an episode
                if env_step_info["task_step_info"]["hit_obstacle"][0]:
                    spear.log("    Collision detected according to env.step(), ending episode...")
                    break
                elif env_step_info["task_step_info"]["hit_goal"][0]:
                    spear.log("    Goal reached according to env.step(), ending episode...")
                    break
                elif goal_reached:
                    spear.log("    Goal reached, ending episode...")
                    break

            # If we're in benchmarking mode, then print the framerate. Otherwise, save the collected data to log files.

            if args.benchmark:
                end_time_seconds = time.time()
                elapsed_time_seconds = end_time_seconds - start_time_seconds
                spear.log("    Average frame time: %0.4f ms (%0.4f fps)" %
                    ((elapsed_time_seconds / num_iterations_executed)*1000, num_iterations_executed / elapsed_time_seconds))

            else:
                spear.log(f"    Writing log files...")

                # save the reference path to a dedicated file
                df_episode_reference_path = pd.DataFrame({"waypoint_x" : path[:,0],
                                                          "waypoint_y" : path[:,1],
                                                          "waypoint_z" : path[:,2]})
                df_episode_reference_path.to_csv(os.path.realpath(os.path.join(logs_dir, "episode_reference_path.csv")), mode="w", index=False, header=True)

                # save the per-iteration data to a file
                df_episode_rollout = pd.DataFrame({
                    "left_ctrl"                  : episode_control_data[:num_iterations_executed, 0],
                    "right_ctrl"                 : episode_control_data[:num_iterations_executed, 1],
                    "location_x"                 : episode_location_data[:num_iterations_executed, 0],
                    "location_y"                 : episode_location_data[:num_iterations_executed, 1],
                    "location_z"                 : episode_location_data[:num_iterations_executed, 2],
                    "rotation_pitch"             : episode_rotation_data[:num_iterations_executed, 0],
                    "rotation_yaw"               : episode_rotation_data[:num_iterations_executed, 1],
                    "rotation_roll"              : episode_rotation_data[:num_iterations_executed, 2],
                    "env_step_info_hit_goal"     : episode_env_step_info_hit_goal[:num_iterations_executed],
                    "env_step_info_hit_obstacle" : episode_env_step_info_hit_obstacle[:num_iterations_executed],
                    "goal_reached"               : episode_goal_reached[:num_iterations_executed]})
                df_episode_rollout.to_csv(os.path.realpath(os.path.join(logs_dir, "episode_rollout.csv")), mode="w", index=False, header=True)

                # Create plots. Note that creating these plots will resize our cv2 windows in an
                # unpleasant way, so we only generate these plots if we're not in debug mode.
                if not args.debug:
                    spear.log(f"    Generating plots...")
                    plot_tracking_performance_spatial(
                        episode_location_data[:num_iterations_executed][:],
                        path,
                        os.path.realpath(os.path.join(plots_dir, "tracking_performance_spatial.png")))

                if args.create_videos:
                    spear.log(f"    Generating video...")
                    video_file = os.path.realpath(os.path.join(args.eval_dir, "videos", "%04d.mp4" % episode["index"]))
                    generate_video(images_dir, video_file, rate=int(1.0/config.SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME), compress=True)

    # at this point, we're finished executing all episodes, so close the Env
    env.close()

    # close unreal instance
    instance.close()

    spear.log("Done.")
