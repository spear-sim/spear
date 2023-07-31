#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "build")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "build")))
    args = parser.parse_args()

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

    executable_dir    = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{platform_name}-Shipping"))
    archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.version_tag}-{platform_name}-Shipping"))
    if sys.platform in ["win32", "linux"]:
        shutil.make_archive(base_name=archive_file_name, format="zip", root_dir=os.path.realpath(os.path.join(executable_dir, platform_dir_name)), verbose=1)
    elif sys.platform == "darwin":
        cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", os.path.realpath(os.path.join(executable_dir, platform_dir_name, f"SpearSim-Mac-Shipping.app")), archive_file_name + ".zip"]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
    else:
        assert False
    assert os.path.exists(archive_file_name + ".zip")

    spear.log(f"Successfully archived executable to {archive_file_name}.zip")
    spear.log("Done.")
