#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--num_episodes_per_scene", type=int, default=10)
    parser.add_argument("--num_candidate_points_per_episode", type=int, default=10)
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "train_episodes.csv")))
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # create directory for storing episodes information
    file_name = os.path.splitext(os.path.basename(args.episodes_file))[0] # isolate filename and remove extension
    episodes_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "episodes"))
    output_dir = os.path.realpath(os.path.join(episodes_dir, file_name))
    os.makedirs(output_dir, exist_ok=True)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # set certain default parameters required by this script
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.LOAD_TRAJECTORY_FROM_FILE = False
    config.freeze()

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scenes.csv"))
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file)["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # create dataframe
    df_columns = ["scene_id", "start_location_x", "start_location_y", "start_location_z", "goal_location_x", "goal_location_y", "goal_location_z"]
    df = pd.DataFrame(columns=df_columns)

    # iterate over all scenes
    for i, scene_id in enumerate(scene_ids):

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
        candidate_points = env.get_random_points(args.num_episodes_per_scene * args.num_candidate_points_per_episode)

        # obtain a reachable goal point for every candidate point
        reachable_points = env.get_reachable_points(candidate_points.tolist())

        # get trajectories for every candidate_points and corresponding reachable_points
        trajectories = env.get_trajectories(candidate_points.tolist(), reachable_points.tolist())

        # score trajectories based on a custom sort function
        def score_trajectory(trajectory):
            num_waypoints = trajectory.shape[0]
            trajectory_length = np.sqrt(np.sum((trajectory[-1] - trajectory[0])[:2] ** 2))
            return num_waypoints * trajectory_length

        trajectory_scores = np.vectorize(lambda trajectory : score_trajectory(trajectory=trajectory))(trajectories)
        sorted_indicies = np.argsort(trajectory_scores)

        # choose only the top args.num_episodes_per_scene trajectories
        start_points_sorted = candidate_points[sorted_indicies]        
        top_start_points = start_points_sorted[-args.num_episodes_per_scene:]
        
        end_points_sorted = reachable_points[sorted_indicies]        
        top_end_points = end_points_sorted[-args.num_episodes_per_scene:]

        # concat arrays for easier pd dataframe creation
        merged_array = np.hstack((top_start_points, top_end_points))

        # store start and end location of the trajectories
        df_ = pd.DataFrame(
            columns=df_columns,
            data={"scene_id"         : [scene_id] * merged_array.shape[0],
                  "start_location_x" : merged_array[:,0],
                  "start_location_y" : merged_array[:,1],
                  "start_location_z" : merged_array[:,2],
                  "goal_location_x"  : merged_array[:,3],
                  "goal_location_y"  : merged_array[:,4],
                  "goal_location_z"  : merged_array[:,5]})
        
        df = pd.concat([df, df_])

        plt.plot(merged_array[:,0], merged_array[:,1], "^", markersize=12.0, label="Start", color="tab:blue", alpha=0.3)
        plt.plot(merged_array[:,3], merged_array[:,4], "^", markersize=12.0, label="Goal", color="tab:orange", alpha=0.3)
        for trajectory in trajectories[sorted_indicies]:
            plt.plot(trajectory[:,0], trajectory[:,1], "-o", markersize=8.0, label="Desired trajectory", color="tab:green", alpha=0.3)
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        legend = plt.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0.5, -0.2), loc="center", ncol=3)
        plt.gca().set_aspect("equal")
        plt.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
        plt.xlabel("x[cm]")
        plt.ylabel("y[cm]")
        plt.grid()
        plt.title(f"scene_id: {scene_id}")
        
        plt.savefig(os.path.realpath(os.path.join(output_dir, scene_id)), bbox_extra_artists=[legend], bbox_inches="tight")

        # close the current scene
        env.close()

    # write start and goal locations of all episodes to a csv file
    df.to_csv(args.episodes_file, index=False)

    spear.log("Done.")
