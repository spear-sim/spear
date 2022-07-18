# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import cv2
import datetime
import numpy as np
import os
import time
import shutil

from interiorsim import Env
from interiorsim.config import get_config

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_path", type=str, default="/media/rachithp/Extreme SSD/interiorsim_all_scene_paks")
    args = parser.parse_args()

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)
        
    # check if path exists
    if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")):
        os.mkdir(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data"))

    # choose scenes from input dir
    scenes_on_disk = os.listdir(args.scenes_path)
    chosen_scenes = []
    for scene in scenes_on_disk:
        split_string_list = scene.split('_')
        if len(split_string_list) > 1 and split_string_list[1] == "Linux.pak":
            chosen_scenes.append(split_string_list[0])

    for scene in chosen_scenes:
        
        if scene != "235554690":
            continue

        print(f"processing scene {scene}")

        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to disk
        if not os.path.exists(f"/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development/LinuxNoEditor/RobotProject/Content/Paks/{scene}_Linux.pak"):
            shutil.copy(os.path.join(args.scenes_path, f"{scene}_Linux.pak"), "/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development/LinuxNoEditor/RobotProject/Content/Paks")

        # check if data path for storing images exists
        if not os.path.exists(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/images/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE}")):
            os.makedirs(os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/images/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE}"))
            print(f"collecting images for scene {scene}")

        # write headers
        frame_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/images/{}/{}.frames.txt".format(scene, config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE, config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE)), "w")
        frame_csv_writer = csv.writer(frame_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        frame_csv_writer.writerow(["scene_id", "timestamp (ns)", "frame_number", "pos_x_cms", "pos_y_cms", "pos_z_cms", "rot_r_deg", "rot_p_deg", "rot_y_deg", "camera_horizontal_fov", "camera_aspect_ratio"])
        
        # create Env object
        try:
            env = Env(config)
        except:
            continue

        # reset the simulation
        _ = env.reset()

        if config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE == "get_ceiling_images":
            pose_info_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "surveillance_camera_poses.txt")
            with open(pose_info_file, 'r') as f:
                csv_reader = csv.reader(f, delimiter=',')
                count = 0
                for row in list(csv_reader)[1:]:
                    obs, _, _, _ = env.step({"set_position_cms" : [float(row[1]), float(row[2]), float(row[3])], "set_orientation_pyr_deg":  [float(row[4]), float(row[5]), float(row[6])]})
                                                                                    
                    # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                    # cv2.waitKey(0)
                    
                    # write data
                    ts = datetime.datetime.now().timestamp() * 1e9
                    output_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/images/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE}")

                    assert os.path.exists(output_path) == True
                    return_status = cv2.imwrite(output_path +f"/{count}.png", obs["visual_observation"])
                    assert return_status == True

                    frame_csv_writer.writerow([scene, int(ts), count, obs["pose"][0], obs["pose"][1], obs["pose"][2], obs["pose"][3], obs["pose"][4], obs["pose"][5], obs["camera_horizontal_fov"], obs["camera_aspect_ratio"]])
                    count+=1
        elif config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE == "follow_trajectory":
            pose_info_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "simple_trajectory.txt")
            with open(pose_info_file, 'r') as f:
                csv_reader = csv.reader(f, delimiter=',')
                count = 0
                for row in list(csv_reader)[1:]:
                    obs, _, _, _ = env.step({"set_position_cms" : [float(row[0]), float(row[1]), float(row[2])], "set_orientation_pyr_deg":  [float(row[3]), float(row[4]), float(row[5])]})
                                                                                    
                    # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                    # cv2.waitKey(0)
                    
                    # write data
                    ts = datetime.datetime.now().timestamp() * 1e9
                    output_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/images/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE}")

                    assert os.path.exists(output_path) == True
                    return_status = cv2.imwrite(output_path +f"/{count}.png", obs["visual_observation"][:,:,[2,1,0]])
                    assert return_status == True

                    frame_csv_writer.writerow([scene, int(ts), count, obs["pose"][0], obs["pose"][1], obs["pose"][2], obs["pose"][3], obs["pose"][4], obs["pose"][5], obs["camera_horizontal_fov"], obs["camera_aspect_ratio"]])
                    count+=1
            
        # close the current environment after collecting required number of images
        env.close()
        time.sleep(10)
        frame_output_file.close()
        os.remove(f"/home/rachithp/code/github/interiorsim/code/unreal_projects/RobotProject/Standalone-Development/LinuxNoEditor/RobotProject/Content/Paks/{scene}_Linux.pak")

    cv2.destroyAllWindows()
