# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import csv
import cv2
import datetime
import json
import numpy as np
import os
import subprocess
import time

from interiorsim import Env
from interiorsim.config import get_config

if __name__ == "__main__":

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
    
    assert config.SIMULATION_CONTROLLER.IMAGE_SAMPLING_AGENT_CONTROLLER.ACTION_MODE == "sample_images"

    # read scene list
    scene_list_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../tools/scene_manager/data/virtualworld-ids.json")
    with open(scene_list_path, 'r') as f:
        scenes = json.load(f)
    
    # random generator
    rng = np.random.default_rng(config.IMAGE_SAMPLING_EXPERIMENT.SEED)

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


    # keep track of scene indices in scenes list
    count = 0
    
    # check if path exists
    if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")):
        os.mkdir(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data"))

    scenes_sampled = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scenes.txt".format(scenes[count])), "w")
    
    for i in range(1000):

        # reset condition
        if i%100 == 0:
            try:
                env.close()
                time.sleep(5)
                pose_output_file.close()
                frame_output_file.close()
            except:
                pass

            # choose map to load
            config.defrost()
            config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(chosen_scenes[count]) # set first scene in the list as starting scene
            config.freeze()

            # check if path exists
            if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}".format(scenes[count]))):
                os.makedirs(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images".format(scenes[count])))
            
            # write headers
            scenes_sampled.write(scenes[count])
            scenes_sampled.write(",")
            pose_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/pose.txt".format(scenes[count])), "w")
            pose_csv_writer = csv.writer(pose_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            pose_csv_writer.writerow(["scene_id", "timestamp (ns)", "pos_x_cm", "pos_y_cm", "pos_z_cm", "roll_deg", "pitch_deg", "yaw_deg"])

            frame_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/frames.txt".format(scenes[count])), "w")
            frame_csv_writer = csv.writer(frame_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            frame_csv_writer.writerow(["scene_id", "timestamp (ns)", "frame_number"])

            # create Env object
            env = Env(config)

            # reset the simulation
            obs = env.reset()

            count += 1

        obs, _, _, _ = env.step({"set_random_orientation_pyr_deg": [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.PITCH_HIGH_DEG),
                                                            rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.YAW_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.YAW_HIGH_DEG),
                                                            rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_LOW_DEG, high=config.IMAGE_SAMPLING_EXPERIMENT.ROLL_HIGH_DEG)],
                        "set_random_agent_height_cms": [rng.uniform(low=config.IMAGE_SAMPLING_EXPERIMENT.AGENT_HEIGHT_LOW_CMS, high=config.IMAGE_SAMPLING_EXPERIMENT.AGENT_HEIGHT_HIGH_CMS)]})
                                                                           
        # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
        # cv2.waitKey(0)
        
        # write data
        ts = datetime.datetime.now().timestamp() * 1e9
        return_status = cv2.imwrite(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scenes[count-1]}/images/{i%100}.png"), obs["visual_observation"])
        assert return_status == True
        pose_csv_writer.writerow([scenes[count-1], int(ts), obs["pose"][0], obs["pose"][1], obs["pose"][2], obs["pose"][3], obs["pose"][4], obs["pose"][5]])
        frame_csv_writer.writerow([scenes[count-1], int(ts), i%100])

    env.close()
    pose_output_file.close()
    frame_output_file.close()
    scenes_sampled.close()

    cv2.destroyAllWindows()
