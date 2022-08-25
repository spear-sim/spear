import argparse
import os
import glob
from zipfile import ZipFile
import pandas as pd
import numpy as np
import shutil


def get_directed_angle(from_vector_x, from_vector_z, to_vector_x, to_vector_z):
    yaw = np.arctan2(from_vector_z, from_vector_x) - np.arctan2(
        to_vector_z, to_vector_x
    )

    # fit to range (-pi, pi]
    yaw[yaw > np.pi] = yaw[yaw > np.pi] - 2 * np.pi
    yaw[yaw <= -np.pi] = yaw[yaw <= -np.pi] + 2 * np.pi

    return yaw


def get_dist(x, z, target_x, target_z):
    dist_x = target_x - x
    dist_z = target_z - z

    return dist_x, dist_z


def get_robot_heading(yaw):
    heading_x = np.cos(yaw + np.pi / 2)
    heading_z = -np.sin(yaw + np.pi / 2)

    return heading_x, heading_z


def get_delta_yaw(dist_x, dist_z, robot_yaw):
    robot_heading_x, robot_heading_z = get_robot_heading(robot_yaw)

    delta_yaw = get_directed_angle(robot_heading_x, robot_heading_z, dist_x, dist_z)

    return delta_yaw


def calc_goal_vector(df):
    # Last robot position defines the target position.
    target_x = df.iloc[-1].x
    target_z = df.iloc[-1].z

    # Distance vector in "recording" coordinate system.
    dist_x = target_x - df.x
    dist_z = target_z - df.z

    # Rotate distance vector to local robot coordinate system. robotYaw defines the heading of the robot, where robotYaw=0 means looking forward.
    # Below matrix is correct since x-axis points to the right and z-axis downwards; i.e. we do not have to use -robotYaw here.
    vec_x = np.cos(df.robotYaw) * dist_x - np.sin(df.robotYaw) * dist_z
    vec_z = np.sin(df.robotYaw) * dist_x + np.cos(df.robotYaw) * dist_z

    return vec_x, vec_z


def read_arcore(f, frames):
    # INFO: Selecting the last `n` frames is required since the app sometimes logs frames before the arcore first timestamp; see https://github.com/isl-org/OpenBot-Distributed/issues/66
    df = pd.read_csv(f)

    n = len(df)

    # rgb
    df_rgb = pd.DataFrame()
    df_rgb["timestamp[ns]"] = df["timestamp"]
    df_rgb["frames"] = [int(os.path.splitext(frame)[0]) for frame in frames[-n:]]

    # depth
    df_depth = pd.DataFrame()
    df_depth["timestamp[ns]"] = df["timestamp"]
    df_depth["frames"] = [int(os.path.splitext(frame)[0]) for frame in frames[-n:]]

    # goal
    df_goal = pd.DataFrame()
    df_goal["timestamp[ns]"] = df["timestamp"]

    # INFO: According to ARCore coordinate system, x points right and -z points into the scene (forward).
    # Last robot position defines the target position.
    dist_x, dist_z = get_dist(
        df.x.to_numpy(), df.z.to_numpy(), df.iloc[-1].x, df.iloc[-1].z
    )
    delta_yaw = get_delta_yaw(dist_x, dist_z, df.robotYaw.to_numpy())

    df_goal["dist"] = np.sqrt(dist_x * dist_x + dist_z * dist_z)
    df_goal["sinYaw"] = np.sin(delta_yaw)
    df_goal["cosYaw"] = np.cos(delta_yaw)

    return df_rgb, df_depth, df_goal


def read_manual(f):
    df = pd.read_csv(f)

    df = df.rename(
        columns={
            "timestamp": "timestamp[ns]",
            "actionLeft": "leftCtrl",
            "actionRight": "rightCtlr",
        }
    )

    return df


def prepare_recording(infile, outfolder):
    image_folder = os.path.join(outfolder, "images")
    os.makedirs(image_folder, exist_ok=True)

    depth_folder = os.path.join(outfolder, "depth")
    os.makedirs(depth_folder, exist_ok=True)

    sensor_folder = os.path.join(outfolder, "sensor_data")
    os.makedirs(sensor_folder, exist_ok=True)

    with ZipFile(infile, "r") as zip_file:
        frames = []

        for filename in zip_file.namelist():
            base = os.path.basename(filename)
            if "ManualDriveState/frames" in filename and "jpeg" in filename:
                with open(os.path.join(image_folder, base), "wb") as f:
                    f.write(zip_file.read(filename))
                frames.append(base)
            elif "ManualDriveState/depth" in filename and "txt" in filename:
                with open(os.path.join(depth_folder, base), "wb") as f:
                    f.write(zip_file.read(filename))
            elif "ManualDriveState/depth" in filename and "png" in filename:
                with open(os.path.join(depth_folder, base), "wb") as f:
                    f.write(zip_file.read(filename))

        frames.sort(key=lambda x: int(os.path.splitext(x)[0]))

        df_rgb, df_depth, df_goal = read_arcore(
            zip_file.open("ManualDriveState/arcore.csv"), frames
        )

        df_rgb.to_csv(os.path.join(sensor_folder, "rgbFrames.txt"), index=False)
        df_depth.to_csv(os.path.join(sensor_folder, "depthFrames.txt"), index=False)
        df_goal.to_csv(os.path.join(sensor_folder, "goalLog.txt"), index=False)

        df = read_manual(zip_file.open("ManualDriveState/manual.csv"))
        df.to_csv(os.path.join(sensor_folder, "ctrlLog.txt"), index=False)


if __name__ == "__main__":
    # params
    parser = argparse.ArgumentParser(description="Prepare dataset.")
    parser.add_argument(
        "-i", "--infolder", type=str, help="input folder (session)", required=True
    )
    parser.add_argument(
        "-o", "--outfolder", type=str, help="output folder", default="dataset"
    )
    args = parser.parse_args()

    # init
    os.makedirs(args.outfolder, exist_ok=True)

    failed = []

    if len(glob.glob(f"{args.infolder}/*/*.zip")) == 0:
        print(
            "Error: No recordings found. "
            "Please make sure to specify a session folder that contains different user folders (userID) "
            "that then contain the recordings as zip files; see download.py"
        )
        exit()

    # process
    for user_infolder in glob.glob(f"{args.infolder}/*"):
        user_id = os.path.basename(user_infolder)
        user_outfolder = f"{args.outfolder}/{user_id}"

        for rec_file in glob.glob(f"{user_infolder}/*.zip"):
            rec_id = os.path.splitext(os.path.basename(rec_file))[0]
            rec_folder = os.path.join(user_outfolder, rec_id)

            print(f"preparing {rec_file} -> {rec_folder}...")
            try:
                prepare_recording(rec_file, rec_folder)
                print(f"   ...success.")
            except:
                failed.append(rec_file)
                shutil.rmtree(rec_folder)
                print(f"   ...failed.")

    # verbose
    print("The conversion of the following recordings failed:")
    print("\n".join(failed))
    print("Please check them manually.")
