#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import pandas as pd
import spear

import utils

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys
sys.path.append(common_dir)
import instance_utils


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--num_episodes_per_scene", type=int, default=10)
    parser.add_argument("--num_candidates_per_episode", type=int, default=10)
    parser.add_argument("--episodes_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "episodes")))
    parser.add_argument("--split", default="train")
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # create directory for storing episode information
    os.makedirs(args.episodes_dir, exist_ok=True)
    episodes_file = os.path.realpath(os.path.join(args.episodes_dir, f"{args.split}_episodes.csv"))

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scenes.csv"))
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file)["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # create dataframe
    df_columns = ["scene_id", "initial_location_x", "initial_location_y", "initial_location_z", "goal_location_x", "goal_location_y", "goal_location_z"]
    df = pd.DataFrame(columns=df_columns)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # set certain default parameters required by this script
    config.defrost()
    config.SP_SERVICES.LEGACY_SERVICE.TASK = "NullTask"
    config.freeze()

    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    # get navigation system
    navigation_system_v1_static_class = instance.unreal_service.get_static_class(class_name="UNavigationSystemV1")
    get_navigation_system_func = instance.unreal_service.find_function_by_name(uclass=navigation_system_v1_static_class, function_name="GetNavigationSystem")
    navigation_system_v1_default_object = instance.unreal_service.get_default_object(uclass=navigation_system_v1_static_class, create_if_needed=False)
    return_values = instance.unreal_service.call_function(uobject=navigation_system_v1_default_object, ufunction=get_navigation_system_func)
    navigation_system = spear.to_handle(string=return_values["ReturnValue"])

    # iterate over all scenes
    for scene_id in scene_ids:

        spear.log("Processing scene: " + scene_id)

        # reset the simulation
        instance_utils.open_level(instance, scene_id)

        # sample paths

        num_candidates_per_scene = args.num_episodes_per_scene*args.num_candidates_per_episode
        candidate_path_radius = 10000.0

        # get navigation data
        navigation_data = instance.navigation_service.get_nav_data_for_agent_name(navigation_system=navigation_system, agent_name="Default")

        # generate candidate points
        candidate_initial_points = instance.navigation_service.get_random_points(
            navigation_data=navigation_data,
            num_points=num_candidates_per_scene)

        # obtain a reachable goal point for every candidate point
        candidate_goal_points = instance.navigation_service.get_random_reachable_points_in_radius(
            navigation_data=navigation_data,
            num_points=num_candidates_per_scene,
            origin_points=candidate_initial_points,
            radius=candidate_path_radius)

        # obtain candidate paths for the candidate initial and goal points
        candidate_paths = instance.navigation_service.find_paths(
            navigation_system=navigation_system,
            navigation_data=navigation_data,
            num_paths=num_candidates_per_scene,
            start_points=candidate_initial_points,
            end_points=candidate_goal_points)

        # score each path and obtain best paths
        candidate_num_waypoints = np.array([ path.shape[0] for path in candidate_paths ])
        candidate_path_lengths  = np.array([ np.sum(np.linalg.norm(path[1:] - path[:-1], axis=1)) for path in candidate_paths ])
        candidate_scores        = candidate_num_waypoints * candidate_path_lengths
        candidate_best_indices  = np.argsort(candidate_scores)[::-1][:args.num_episodes_per_scene] # argsort in descending order, select the best indices

        # store initial and goal locations of the best paths
        df_ = pd.DataFrame(
            columns=df_columns, 
            data={
                "scene_id":           [scene_id] * args.num_episodes_per_scene,
                "initial_location_x": candidate_initial_points[candidate_best_indices, 0],
                "initial_location_y": candidate_initial_points[candidate_best_indices, 1],
                "initial_location_z": candidate_initial_points[candidate_best_indices, 2],
                "goal_location_x":    candidate_goal_points[candidate_best_indices, 0],
                "goal_location_y":    candidate_goal_points[candidate_best_indices, 1],
                "goal_location_z":    candidate_goal_points[candidate_best_indices, 2]})

        df = pd.concat([df, df_])

        plots_filename = os.path.realpath(os.path.join(args.episodes_dir, scene_id + "_" + args.split + "_paths.png"))
        utils.plot_paths(scene_id, [candidate_paths[i] for i in candidate_best_indices], plots_filename)

    # write initial and goal locations of all episodes to a csv file
    df.to_csv(episodes_file, index=False)

    # close the unreal instance
    instance.close()

    spear.log("Done.")
