#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import pandas as pd
import spear

from utils import plot_paths


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

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # set certain default parameters required by this script
    config.defrost()
    config.SIMULATION_CONTROLLER.TASK = "NullTask"
    config.freeze()

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

    # iterate over all scenes
    for scene_id in scene_ids:

        spear.log("Processing scene: " + scene_id)

        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
        config.freeze()

        # create Env object
        env = spear.Env(config)

        # reset the simulation
        env_reset_info = {}
        _ = env.reset(reset_info=env_reset_info)
        assert "success" in env_reset_info
            
        # generate candidate points based out of args.num_episodes_per_scene
        candidate_initial_points = env.get_random_points(args.num_episodes_per_scene * args.num_candidates_per_episode)

        # obtain a reachable goal point for every candidate point
        candidate_goal_points = env.get_random_reachable_points_in_radius(candidate_initial_points, 10000.0)

        # obtain candidate paths for the candidate initial and goal points
        candidate_paths = env.get_paths(candidate_initial_points, candidate_goal_points)

        # score each path and obtain best paths
        candidate_num_waypoints = np.array([ path.shape[0] for path in candidate_paths ])
        candidate_path_lengths  = np.array([ np.sum(np.linalg.norm(path[1:] - path[:-1], axis=1)) for path in candidate_paths ])
        candidate_scores        = candidate_num_waypoints * candidate_path_lengths
        candidate_best_indices  = np.argsort(candidate_scores)[::-1][:args.num_episodes_per_scene] # argsort in descending order, select the best indices

        # store initial and goal locations of the best paths
        df_ = pd.DataFrame(
            columns=df_columns, 
            data={
                "scene_id"           : [scene_id] * args.num_episodes_per_scene,
                "initial_location_x" : candidate_initial_points[candidate_best_indices, 0],
                "initial_location_y" : candidate_initial_points[candidate_best_indices, 1],
                "initial_location_z" : candidate_initial_points[candidate_best_indices, 2],
                "goal_location_x"    : candidate_goal_points[candidate_best_indices, 0],
                "goal_location_y"    : candidate_goal_points[candidate_best_indices, 1],
                "goal_location_z"    : candidate_goal_points[candidate_best_indices, 2]})

        df = pd.concat([df, df_])

        plots_filename = os.path.realpath(os.path.join(args.episodes_dir, scene_id + "_" + args.split + "_paths.png"))
        plot_paths(scene_id, [candidate_paths[i] for i in candidate_best_indices], plots_filename)

        # close the current scene
        env.close()

    # write initial and goal locations of all episodes to a csv file
    df.to_csv(episodes_file, index=False)

    spear.log("Done.")
