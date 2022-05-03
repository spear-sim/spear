# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import csv
import cv2
import json
import numpy as np
import os
import subprocess

from interiorsim import Env
from interiorsim.config import get_config

if __name__ == "__main__":

    # read scene list
    scene_list_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../tools/scene_manager/data/virtualworld-ids.json")
    with open(scene_list_path, 'r') as f:
        scenes = json.load(f)
    
    # random generator
    rng = np.random.default_rng(19)

    # choose 10 random scenes
    print(f"number of scenes: {len(scenes)}")
    ri = rng.integers(low=0, high=len(scenes), size=(10))
    chosen_scenes = [scenes[x] for x in ri]
    print(f"chosen scenes {chosen_scenes}")

    # download chosen scenes
    # script_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../tools/scene_manager/scene_manager.py")
    # for scene in chosen_scenes[2:]:
    #     args = ["python", script_path, "-i", scene, "-v", "v4"]
    #     print(f"{' '.join(args)}")
    #     run_cmd = subprocess.run(args)
    #     assert run_cmd.returncode == 0
    # quit()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    # add these scenes to config
    config.defrost()
    config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.SCENES = chosen_scenes
    config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(chosen_scenes[0])
    config.freeze()

    # create Env object
    env = Env(config)

    # reset the simulation
    obs = env.reset()

    # take a few steps
    count = 0
    
    # check if path exists
    if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}".format(scenes[count]))):
        os.makedirs(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images".format(scenes[count])))
    
    pose_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/pose.txt".format(scenes[count])), "w")
    csv_writer = csv.writer(pose_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    csv_writer.writerow(["pos_x_cm", "pos_y_cm", "pos_z_cm", "roll_deg", "pitch_deg", "yaw_deg"])
    
    for i in range(200):

        obs, _, _, _ = env.step({"set_orientation_pyr_degrees": [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_HIGH),
                                                                            rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.YAW_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.YAW_HIGH),
                                                                            rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_HIGH)]})
                                                                           
        # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)
        
        # check if path exists
        if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}".format(scenes[count]))):
            os.makedirs(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images".format(scenes[count])))
        
        # write data
        cv2.imwrite(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images/{}.png".format(scenes[count], i%100)), obs["visual_observation"])
        csv_writer.writerow(obs["pose"])

        # reset condition
        if i%100 == 0 and i > 1:
            count += 1
            # obs, reward, done, info = env.step({"change_scene": [count]})
            env.close()
            
            # reset the simulation
            obs = env.reset()
            obs, _, _, _ = env.step({"set_orientation_pyr_degrees": [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_HIGH),
                                                                     rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.YAW_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.YAW_HIGH),
                                                                     rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_LOW, high=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_HIGH)]})

            # reset csv writer
            pose_output_file.close()
            pose_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/pose.txt".format(scenes[count])), "w")
            csv_writer = csv.writer(pose_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            csv_writer.writerow(["pos_x_cm", "pos_y_cm", "pos_z_cm", "roll_deg", "pitch_deg", "yaw_deg"])

            # check if path exists
            if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}".format(scenes[count]))):
                os.makedirs(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images".format(scenes[count])))

            # write data
            cv2.imwrite(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images/{}.png".format(scenes[count], i%100)), obs["visual_observation"])
            csv_writer.writerow(obs["pose"])

            continue

    # cv2.destroyAllWindows()

    # close the environment
    env.close()
