# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import csv
import cv2
import datetime
import numpy as np
import os
import shutil
import sys
import time
import h5py

from interiorsim import Env
from interiorsim.config import get_config


if sys.platform == "linux":
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"


if __name__ == "__main__":

    #parser = argparse.ArgumentParser()
    #parser.add_argument("--scenes_path", type=str, default="C:/Users/ADAS/")
    #parser.add_argument("--path_to_executable_dir", type=str)
    #args = parser.parse_args()

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
        if len(split_string_list) > 1 and split_string_list[1] == f"{PLATFORM}.pak":
            chosen_scenes.append(split_string_list[0])

    #for scene in chosen_scenes:
        
        #if scene != "235554690":
            #continue
    scene="235554690"
        #print(f"processing scene {scene}")

    # choose map to load
    config.defrost()
    config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set scene in the list as starting scene
    config.freeze()

    # copy pak from ssd to disk
    # if not os.path.exists(f"{args.path_to_executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_Linux.pak"):
        # shutil.copy(os.path.join(args.scenes_path, f"{scene}_Linux.pak"), f"{args.path_to_executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # copy pak from ssd to disk
        # if not os.path.exists(f"{args.path_to_executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.path_to_executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

    # write headers
    frame_output_file = open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "data/scene_{}/{}/pose_fov_data.txt".format(scene, config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE)), "w")
    frame_csv_writer = csv.writer(frame_output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    frame_csv_writer.writerow(["scene_id", "timestamp (ns)", "frame_number", "pos_x_cms", "pos_y_cms", "pos_z_cms", "rot_r_deg", "rot_p_deg", "rot_y_deg", "camera_horizontal_fov", "camera_vertical_fov", "image_width", "image_height"])
    
    # create Env object
    try:
        env = Env(config)
    except:
        pass

    # reset the simulation
    _ = env.reset()

    # load different files based on different action modes
    if config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE == "get_ceiling_images":
        pose_info_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "surveillance_camera_poses.txt")
    elif config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE == "follow_trajectory":
        pose_info_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "new_trajectory.txt")
    
    with open(pose_info_file, 'r') as f:
        csv_reader = csv.reader(f, delimiter=',')
        count = 0

        for row in list(csv_reader)[1:]:
            obs, _, _, _ = env.step({"set_position_cms" : [float(row[0]), float(row[1]), float(row[2])], "set_orientation_pyr_deg":  [float(row[3]), float(row[4]), float(row[5])]})
                                                                            
            # cv2.imshow("visual_observation", obs["visual_observation"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
            # cv2.waitKey(0)
            
            # write data
            ts = datetime.datetime.now().timestamp() * 1e9
            output_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), f"data/scene_{scene}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.ACTION_MODE}/{config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_TYPE}")

            assert os.path.exists(output_path) == True
            return_status = cv2.imwrite(output_path +f"/{count}.jpg", obs["visual_observation"][:,:,[2,1,0]], [cv2.IMWRITE_JPEG_QUALITY, 90])
            assert return_status == True

            #hdf5 file save
            #with h5py.File(output_path +f"/{count}.h5", 'w') as hf:
            #    hf.create_dataset(f"depth_{count}",  data=obs["visual_observation_depth"][:,:],compression="gzip", compression_opts=9)

            #csv file save
            a = np.asarray(obs["visual_observation_depth"][:,:])
            np.savetxt(output_path +f"{count}.csv", a, delimiter=",")

            frame_csv_writer.writerow([scene, int(ts), count, obs["pose"][0], obs["pose"][1], obs["pose"][2], obs["pose"][3], obs["pose"][4], obs["pose"][5], obs["camera_horizontal_fov"][0], obs["camera_vertical_fov"][0], config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_WIDTH, config.SIMULATION_CONTROLLER.SLAM_DATASET_AGENT_CONTROLLER.IMAGE_HEIGHT])
            count+=1
    
    # close the current environment after collecting required number of images
    env.close()
    time.sleep(10)
    frame_output_file.close()
    # os.remove(f"{args.path_to_executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_Linux.pak")

    cv2.destroyAllWindows()
