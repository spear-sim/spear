# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import spear
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--num_poses_per_scene", type=int, default=10)
    parser.add_argument("-p", "--poses_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "start_goal_coordinates.csv"))
    parser.add_argument("-s", "--scene_id", nargs="+", default=[""], help="Array of scene ID references, to support data collection in multiple environments.", required=False)
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # make sure that we are in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = True
    config.freeze()

    # remove existing poses file
    if os.path.exists(args.poses_file):
        os.remove(args.poses_file)

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id == [""]:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = args.scene_id

    # iterate over all scenes
    for i, scene_id in enumerate(scene_ids):

        print("Processing scene: " + scene_id)

        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + "." + "Map_" + scene_id
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id
        config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
        config.freeze()

        # create Env object
        env = spear.Env(config)

        for j in range(args.num_poses_per_scene):
        
            # reset the simulation
            _ = env.reset()

            # get random start-goal pairs
            _, _, _, step_info = env.step(action={"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            positions = step_info["agent_step_info"]["trajectory_data"] 

            # store poses in a csv file
            df = pd.DataFrame({"scene_id"  : [scene_id],
                            "init_pos_x_cms" : positions[0,0],
                            "init_pos_y_cms" : positions[0,1],
                            "init_pos_z_cms" : positions[0,2],
                            "goal_pos_x_cms" : positions[-1,0],
                            "goal_pos_y_cms" : positions[-1,1],
                            "goal_pos_z_cms" : positions[-1,2]})
            df.to_csv(args.poses_file, mode="a", index=False, header=(i==0 and j==0))
        
            plt.plot(positions[0,0], positions[0,1], 'r^', markersize=12.0, label='Start')
            plt.plot(positions[-1,0], positions[-1,1], 'g^', markersize=12.0, label='Goal')
            plt.plot(positions[:,0], positions[:,1], '-o', markersize=5.0, label='Desired Trajectory')
            
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), bbox_to_anchor = (0.75, 1.15), ncol = 3)
        plt.gca().set_aspect("equal")
        plt.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
        plt.xlabel('x[cm]')
        plt.ylabel('y[cm]')
        plt.grid()
        plt.title(f"OpenBot trajectories (scene {scene_id})")
        plt.show()

        # close the current scene
        env.close()

    print("Done.")
