import argparse
import os
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud_path_prefix", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "build")))
    args = parser.parse_args()

    assert os.path.exists(args.input_dir)

    if sys.platform == "win32":
        platform_name     = "Win64"
        platform_dir_name = "Windows"
    elif sys.platform == "darwin":
        platform_name     = "Mac"
        platform_dir_name = "Mac"
    elif sys.platform == "linux":
        platform_name     = "Linux"
        platform_dir_name = "Linux"
    else:
        assert False

    cloud_folder = os.path.join(args.cloud_path_prefix, args.version_tag, "")
    # TODO: update this to process all built pak files
    archive_file_name = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{args.version_tag}-{platform_name}-Shipping.zip"))

    spear.log("cloud_folder", cloud_folder)

    cmd = ["aws", "s3", "cp", archive_file_name, cloud_folder]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log("Done.")
