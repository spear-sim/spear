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

import policies
import utils

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys
sys.path.append(common_dir)
import instance_utils
import openbot_env
import visualization_utils


if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_iterations_per_episode", type=int, default=500)
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "train_episodes.csv")))
    parser.add_argument("--dataset_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "dataset")))
    parser.add_argument("--split", default="train")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--create_videos", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml")),
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    config.defrost()
    config.SP_SERVICES.LEGACY_SERVICE.TASK = "ImitationLearningTask"
    config.SP_SERVICES.LEGACY.IMITATION_LEARNING_TASK.EPISODES_FILE = os.path.abspath(args.episodes_file)
    config.freeze()
 
    if args.debug:
        config.defrost()
        config.SP_SERVICES.LEGACY.IMU_SENSOR.DEBUG_RENDER = True # only has an effect in Development mode, not shipping mode
        config.SP_SERVICES.LEGACY.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT = 512
        config.SP_SERVICES.LEGACY.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH = 512
        config.SP_SERVICES.LEGACY.VEHICLE_AGENT.CAMERA.RENDER_PASSES = ["depth", "final_color", "segmentation"]
        config.SP_SERVICES.LEGACY.VEHICLE_AGENT.OBSERVATION_COMPONENTS = ["camera", "imu", "location", "rotation", "wheel_rotation_speeds"]
        # aim camera in a third-person view facing backwards at an angle
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_X = -50.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Y = -50.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.LOCATION_Z = 45.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_PITCH = -35.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_YAW = 45.0
        # config.VEHICLE.VEHICLE_PAWN.CAMERA_COMPONENT.ROTATION_ROLL = 0.0
        config.freeze()

    if not args.benchmark:
        split_dir = os.path.realpath(os.path.join(args.dataset_dir, args.split + "_data"))
        shutil.rmtree(split_dir, ignore_errors=True)

    # load driving policy
    policy = policies.OpenBotPathFollowingPolicy(config)

    # load the episodes to be executed
    df = pd.read_csv(args.episodes_file)

    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    env = openbot_env.OpenBotEnv(instance, config)

    # get navigation system
    navigation_system_v1_static_class = instance.unreal_service.get_static_class(class_name="UNavigationSystemV1")
    get_navigation_system_func = instance.unreal_service.find_function_by_name(uclass=navigation_system_v1_static_class, function_name="GetNavigationSystem")
    navigation_system_v1_default_object = instance.unreal_service.get_default_object(uclass=navigation_system_v1_static_class, create_if_needed=False)
    return_values = instance.unreal_service.call_function(uobject=navigation_system_v1_default_object, ufunction=get_navigation_system_func)
    navigation_system = spear.to_handle(string=return_values["ReturnValue"])

    # iterate over all episodes
    prev_scene_id = ""
    for episode in df.to_records():

        spear.log(f"Generating data for episode {episode['index']} of {df.shape[0]}...")

        # if the scene_id of our current episode has changed, then create a new Env
        if episode["scene_id"] != prev_scene_id:

            # close the previous OpenBotEnv
            env.close()

            # open the desired level
            instance_utils.open_level(instance, episode["scene_id"])

            # open a new OpenBotEnv
            env = openbot_env.OpenBotEnv(instance, config)

            # get navigation data
            navigation_data = instance.navigation_service.get_nav_data_for_agent_name(navigation_system=navigation_system, agent_name="Default")

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

            # initialize the driving policy with the desired path
            path = instance.navigation_service.find_paths(
                navigation_system=navigation_system,
                navigation_data=navigation_data,
                num_paths=1,
                start_points=episode_initial_location,
                end_points=episode_goal_location)[0]
            policy.reset(obs, path)

            if args.benchmark:
                start_time_seconds = time.time()
            else:     
                # build the episode data folder and its subfolders following the guidelines of the OpenBot public repository 
                # https://github.com/isl-org/OpenBot/tree/master/policy#data-collection
                episode_dir     = os.path.realpath(os.path.join(args.dataset_dir, args.split + "_data", episode["scene_id"], "%04d" % episode["index"]))
                images_dir      = os.path.realpath(os.path.join(episode_dir, "images"))
                sensor_data_dir = os.path.realpath(os.path.join(episode_dir, "sensor_data"))
                plots_dir       = os.path.realpath(os.path.join(episode_dir, "plots"))
                os.makedirs(images_dir, exist_ok=True)
                os.makedirs(sensor_data_dir, exist_ok=True)
                os.makedirs(plots_dir, exist_ok=True)

                episode_timestamp_data = np.empty([args.num_iterations_per_episode], dtype=np.int64)
                episode_frame_id_data  = np.empty([args.num_iterations_per_episode], dtype=np.int32)

                # Our convention in this example is to store all data that comes directly from Unreal in the native format
                # exported by Unreal, i.e., centimeters and degrees. We eventually need to convert some of this data to
                # meters and radians, but we only do so in local temporary variables.
                episode_location_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64) 
                episode_rotation_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64)
                episode_waypoint_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64)

                # The desired yaw is derived data, but we also store it in degrees for consistency with our other rotation data.
                episode_rotation_yaw_desired_data = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64)
    
                # We store distance_to_goal in meters for compatibility with the OpenBot framework.
                episode_control_data = np.empty([args.num_iterations_per_episode, 2], dtype=np.float64) # [control_left, control_right]
                episode_goal_data    = np.empty([args.num_iterations_per_episode, 3], dtype=np.float64) # [distance_to_goal, sin_yaw, cos_yaw]

                # If our episode ends unsuccessfully (e.g., hitting an obstacle), we won't try to use the data we collected,
                # but for now, we will assume that the episode will end successfully until proven otherwise.
                episode_successful = True

            # execute episode
            num_iterations_executed = 0
            for i in range(args.num_iterations_per_episode):

                spear.log(f"    Executing iteration {i} of {args.num_iterations_per_episode}...")

                timestamp = time.time_ns()

                # update control action 
                action, policy_step_info = policy.step(obs)

                # send control action to the agent and collect observations
                obs, _, _, env_step_info = env.step(action={"set_duty_cycles": action})

                num_iterations_executed += 1

                # check if we've reached the goal
                cm_to_m = 0.01
                goal_reached = np.linalg.norm(episode_goal_location[0, 0:2] - obs["location"][0:2]) * cm_to_m <= config.IMITATION_LEARNING_OPENBOT.GOAL_REACHED_RADIUS

                if args.debug:
                    obs_final_color = visualization_utils.get_final_color_image_for_visualization(obs["camera.final_color"])
                    utils.show_obs(obs_final_color)

                if not args.benchmark:
                    obs_final_color = visualization_utils.get_final_color_image_for_visualization(obs["camera.final_color"])

                    # save the collected rgb observations
                    plt.imsave(os.path.realpath(os.path.join(images_dir, "%04d.jpeg" % i)), obs_final_color)

                    # During an episode, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training
                    # purposes. In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole
                    # path. In this case, rather than considering the whole episode as a fail, one can consider the last position reached by the agent as
                    # the new goal position. Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather
                    # than directly writing all the observations in a file iteration by iteration, we append these observations in a buffer, named "observation"
                    # to later process them once the episode is completed. 
                    episode_frame_id_data[i]             = i                                        # current frame
                    episode_timestamp_data[i]            = timestamp                                # current time stamp
                    episode_control_data[i]              = action                                   # [ctrl_left, ctrl_right]
                    episode_location_data[i]             = obs["location"]                          # [x, y, z] in cms
                    episode_rotation_data[i]             = obs["rotation"]                          # [pitch, yaw, roll] in degs
                    episode_waypoint_data[i]             = policy_step_info["waypoint"]             # current waypoint being tracked by the policy
                    episode_rotation_yaw_desired_data[i] = policy_step_info["rotation_yaw_desired"] # desired yaw computed by the policy

                # check conditions for ending an episode
                if env_step_info["task_step_info"]["hit_obstacle"][0]:
                    spear.log("    Collision detected according to env.step(), ending episode...")
                    episode_successful = False
                    break
                if env_step_info["task_step_info"]["hit_goal"][0]:
                    spear.log("    Goal reached according to env.step(), ending episode...")
                    break
                if goal_reached:
                    spear.log("    Goal reached, ending episode...")
                    break

            # If we're in benchmarking mode, then print the framerate. Otherwise, we check if our episode ended successfully.
            # If our episode did not end successfully, then remove the episode dir and proceed to the next episode, because we
            # don't want to collect any data from an unsuccessful episode. Otherwise, we are not in benchmarking mode and our
            # episode did end successfully, so save the collected data to log files.

            if args.benchmark:
                end_time_seconds = time.time()
                elapsed_time_seconds = end_time_seconds - start_time_seconds
                spear.log("    Average frame time: %0.4f ms (%0.4f fps)" %
                    ((elapsed_time_seconds / num_iterations_executed)*1000, num_iterations_executed / elapsed_time_seconds))

            elif not episode_successful:
                shutil.rmtree(episode_dir, ignore_errors=True)

            else:
                spear.log(f"    Writing log files...")

                # compute goal observations using last recorded position as the goal position
                # https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
                cm_to_m = 0.01
                location_xy_desired = episode_location_data[num_iterations_executed-1][0:2] * cm_to_m
                for i in range(num_iterations_executed):
                    location_xy_current = episode_location_data[i, 0:2] * cm_to_m
                    location_xy_error   = np.linalg.norm(location_xy_desired - location_xy_current)

                    rotation_yaw_current = np.deg2rad(episode_rotation_data[i, 1])
                    heading_xy_current   = np.array([np.cos(rotation_yaw_current), np.sin(rotation_yaw_current)])
                    heading_xy_desired   = (location_xy_desired - location_xy_current) / (np.linalg.norm(location_xy_desired - location_xy_current) + 1e-10) # adding 1e-10 to present div by zero
                    rotation_yaw_error   = np.arctan2(heading_xy_desired[1], heading_xy_desired[0]) - np.arctan2(heading_xy_current[1], heading_xy_current[0])

                    if rotation_yaw_error < -np.pi:
                        rotation_yaw_error += 2*np.pi
                    if rotation_yaw_error > np.pi:
                        rotation_yaw_error -= 2*np.pi

                    episode_goal_data[i] = np.array([location_xy_error, np.sin(rotation_yaw_error), np.cos(rotation_yaw_error)])

                # low-level commands sent to the motors
                df_ctrl = pd.DataFrame({
                    "timestamp":     episode_timestamp_data[:num_iterations_executed],
                    "control_left":  episode_control_data[:num_iterations_executed, 0],
                    "control_right": episode_control_data[:num_iterations_executed, 1]})
                df_ctrl.to_csv(os.path.realpath(os.path.join(sensor_data_dir, "ctrlLog.txt")), mode="w", index=False, header=True)

                # reference of the images correespoinding to each control input
                df_rgb = pd.DataFrame({
                    "timestamp": episode_timestamp_data[:num_iterations_executed],
                    "frame_id":  episode_frame_id_data[:num_iterations_executed]})
                df_rgb.to_csv(os.path.realpath(os.path.join(sensor_data_dir, "rgbFrames.txt")), mode="w", index=False, header=True)

                # high level commands
                df_goal = pd.DataFrame({
                    "timestamp":        episode_timestamp_data[:num_iterations_executed],
                    "distance_to_goal": episode_goal_data[:num_iterations_executed, 0],
                    "sin_yaw":          episode_goal_data[:num_iterations_executed, 1],
                    "cos_yaw":          episode_goal_data[:num_iterations_executed, 2]})
                df_goal.to_csv(os.path.realpath(os.path.join(sensor_data_dir, "goalLog.txt")), mode="w", index=False, header=True)

                # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
                df_pose = pd.DataFrame({
                    "timestamp":       episode_timestamp_data[:num_iterations_executed],
                    "frame_id":        episode_frame_id_data[:num_iterations_executed],
                    "location_x":      episode_location_data[:num_iterations_executed, 0],
                    "location_y":      episode_location_data[:num_iterations_executed, 1],
                    "location_z":      episode_location_data[:num_iterations_executed, 2],
                    "rotation_pitch":  episode_rotation_data[:num_iterations_executed, 0],
                    "rotation_yaw":    episode_rotation_data[:num_iterations_executed, 1],
                    "rotation_roll":   episode_rotation_data[:num_iterations_executed, 2],
                    "waypoint_x":      episode_waypoint_data[:num_iterations_executed, 0],
                    "waypoint_y":      episode_waypoint_data[:num_iterations_executed, 1],
                    "waypoint_z":      episode_waypoint_data[:num_iterations_executed, 2]})
                df_pose.to_csv(os.path.realpath(os.path.join(sensor_data_dir, "debugLog.txt")), mode="w", index=False, header=True)

                # Create plots. Note that creating these plots will resize our cv2 windows in an
                # unpleasant way, so we only generate these plots if we're not in debug mode.
                if not args.debug:
                    spear.log(f"    Generating plots...")
                    utils.plot_tracking_performance_spatial(
                        episode_location_data[:num_iterations_executed],
                        episode_waypoint_data[:num_iterations_executed],
                        os.path.realpath(os.path.join(plots_dir, "tracking_performance_spatial.png")))
                    utils.plot_tracking_performance_temporal(
                        episode_location_data[:num_iterations_executed],
                        episode_waypoint_data[:num_iterations_executed],
                        episode_rotation_data[:num_iterations_executed, 1],
                        episode_rotation_yaw_desired_data[:num_iterations_executed],
                        os.path.realpath(os.path.join(plots_dir, "tracking_performance_temporal.png")))

                if args.create_videos:
                    spear.log(f"    Generating video...")
                    video_file = os.path.realpath(os.path.join(args.dataset_dir, "videos", args.split + "_data", episode["scene_id"], "%04d.mp4" % episode["index"]))
                    utils.generate_video(images_dir, video_file, rate=int(1.0/config.SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME), compress=True)

    # at this point, we're finished executing all episodes, so close the Env
    env.close()

    # close the unreal instance
    instance.close()

    spear.log("Done.")
