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
    parser.add_argument("--episodes_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "train_episodes.csv")))
    parser.add_argument("--scene_id")
    args = parser.parse_args()

    # directory for trajectory image storage
    split_name = os.path.splitext(os.path.basename(args.episodes_file))[0] # isolate filename and remove extension
    episodes_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "episodes"))
    split_dir = os.path.realpath(os.path.join(episodes_dir, split_name))
    os.makedirs(split_dir, exist_ok=True)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # make sure that we are in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = True
    config.freeze()

    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scenes.csv"))
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file)["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # iterate over all scenes
    for i, scene_id in enumerate(scene_ids):

        spear.log("Processing scene: " + scene_id)

        # change config based on current scene
        config.defrost()

        if scene_id == "kujiale_0000":
            config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
            config.SIMULATION_CONTROLLER.MAP_ID   = scene_id + "_bake"

            # kujiale_0000 has scene-specific config values
            scene_config_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scene_config.kujiale_0000.yaml"))

        elif scene_id == "warehouse_0000":
            config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
            config.SIMULATION_CONTROLLER.MAP_ID   = scene_id

            # warehouse_0000 has scene-specific config values
            scene_config_file = os.path.realpath(os.path.join(os.path.dirname(__file__), "scene_config.warehouse_0000.yaml"))

        else:
            assert False

        config.merge_from_file(scene_config_file)
        config.freeze()

        # create Env object
        env = spear.Env(config)

        j = 0
        while j < args.num_episodes_per_scene:

            # reset the simulation
            env_reset_info = {}
            _ = env.reset(reset_info=env_reset_info)
            assert "success" in env_reset_info
            
            # if it took too long to reset the simulation, then continue
            if not env_reset_info["success"]:
                spear.log("Call to env.reset(...) was not successful. Simulation took too long to return to a ready state. Retrying...")
                continue
                
            # get random start-goal pairs
            _, _, _, env_step_info = env.step(action={"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            positions = env_step_info["agent_step_info"]["trajectory_data"] 

            # store poses in a csv file
            df = pd.DataFrame({"scene_id"       : [scene_id],
                               "init_pos_x_cms" : positions[0,0],
                               "init_pos_y_cms" : positions[0,1],
                               "init_pos_z_cms" : positions[0,2],
                               "goal_pos_x_cms" : positions[-1,0],
                               "goal_pos_y_cms" : positions[-1,1],
                               "goal_pos_z_cms" : positions[-1,2]})
            df.to_csv(args.episodes_file, mode="w" if i==0 and j==0 else "a", index=False, header=i==0 and j==0)
        
            plt.plot(positions[0,0], positions[0,1], "^", markersize=12.0, label="Start", color="tab:blue", alpha=0.3)
            plt.plot(positions[-1,0], positions[-1,1], "^", markersize=12.0, label="Goal", color="tab:orange", alpha=0.3)
            plt.plot(positions[:,0], positions[:,1], "-o", markersize=8.0, label="Desired trajectory", color="tab:green", alpha=0.3)

            j = j+1

        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        legend = plt.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0.5, -0.2), loc="center", ncol=3)
        plt.gca().set_aspect("equal")
        plt.gca().invert_yaxis() # we invert the y-axis so our plot matches a top-down view of the scene in Unreal
        plt.xlabel("x[cm]")
        plt.ylabel("y[cm]")
        plt.grid()
        plt.title(f"scene_id {scene_id}")
        
        plt.savefig(os.path.realpath(os.path.join(split_dir, scene_id)), bbox_extra_artists=[legend], bbox_inches="tight")

        # close the current scene
        env.close()

    spear.log("Done.")
