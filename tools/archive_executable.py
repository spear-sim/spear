#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import subprocess
import sys

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    args = parser.parse_args()

    if sys.platform == "win32":
        platform_name     = "Win64"
        platform_dir_name = "WindowsNoEditor"
    elif sys.platform == "darwin":
        platform_name     = "Mac"
        platform_dir_name = "MacNoEditor"
    elif sys.platform == "linux":
        platform_name     = "Linux"
        platform_dir_name = "LinuxNoEditor"
    else:
        assert False

    executable_dir    = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{platform_name}-Shipping"))
    archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.tag}-{platform_name}-Shipping"))
    shutil.make_archive(base_name=archive_file_name, format='zip', root_dir=os.path.join(executable_dir, f"{platform_dir_name}"), verbose=1)
    assert os.path.exists(archive_file_name + ".zip")

    print(f"[SPEAR | archive_executable.py] Successfully archived executable to {archive_file_name}.zip")
    print(f"[SPEAR | archive_executable.py] Done.")
