#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--build-config", required=True)
parser.add_argument("--version-tag", required=True)
parser.add_argument("--build-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
args = parser.parse_args()


if __name__ == "__main__":

    if sys.platform == "win32":
        platform_name_archive   = "Win64"
        platform_name_outer_dir = "Win64"
        platform_name_inner_dir = "Windows"
    elif sys.platform == "darwin":
        platform_name_archive   = "Mac"
        platform_name_outer_dir = "Mac"
        platform_name_inner_dir = "Mac"
    elif sys.platform == "linux":
        platform_name_archive   = "Linux"
        platform_name_outer_dir = "Linux"
        platform_name_inner_dir = "Linux"
    else:
        assert False

    archive_contents_root_dir = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{platform_name_outer_dir}-{args.build_config}", platform_name_inner_dir))
    archive_dir  = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{platform_name_outer_dir}-{args.build_config}-Archive"))
    archive_file = os.path.realpath(os.path.join(archive_dir, f"SpearSim-{args.version_tag}-{platform_name_archive}-{args.build_config}"))

    # create the temp directory
    spear.log("Creating directory if it does not already exist: ", archive_dir)
    os.makedirs(archive_dir, exist_ok=True)

    spear.log("Creating archive: ", archive_file)
    shutil.make_archive(base_name=archive_file, format="zip", root_dir=archive_contents_root_dir, verbose=1)
    assert os.path.exists(f"{archive_file}.zip")

    spear.log(f"Successfully archived executable to {archive_file}.zip")
    spear.log("Done.")
