import argparse
import glob
import os
import shutil

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--download_dir", type=str, required=True)
    parser.add_argument("--output_dir", type=str, required=True)
    args = parser.parse_args()

    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    scene_pak_files = glob.glob(f"{args.download_dir}/**/*.pak", recursive=True)

    scenes_already_present = os.listdir(args.output_dir)

    for scene in scene_pak_files:
        output_file_name = scene.split("\\")[-1]
        
        if output_file_name not in scenes_already_present:
            print(f"copying scene {scene} ...")
            shutil.copyfile(scene, os.path.join(args.output_dir, output_file_name))
        else:
            print(f"scene {output_file_name} already in destination dir, so skipping it")
