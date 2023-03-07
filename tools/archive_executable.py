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
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, "SpearSim-Win64-Shipping"))
        archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.tag}-Win64-Shipping"))
        shutil.make_archive(base_name=archive_file_name, format='zip', root_dir=os.path.join(executable_dir, "WindowsNoEditor"), verbose=1)
    elif sys.platform == "darwin":
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, "SpearSim-Mac-Shipping"))
        executable        = os.path.realpath(os.path.join(args.input_dir, "SpearSim-Mac-Shipping", "MacNoEditor", "SpearSim-Mac-Shipping.app"))
        archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.tag}-Mac-Shipping"))
        cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", os.path.dirname(executable), archive_file_name + ".zip"]
        print(f"[SPEAR | archive_executable.py] Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
    elif sys.platform == "linux":
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, "SpearSim-Linux-Shipping"))
        archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.tag}-Linux-Shipping"))
        shutil.make_archive(base_name=archive_file_name, format='zip', root_dir=os.path.join(executable_dir, "LinuxNoEditor"), verbose=1)
    else:
        assert False

    assert os.path.exists(archive_file_name + ".zip")

    print(f"[SPEAR | archive_executable.py] Successfully archived executable to {archive_file_name}.zip")
    print(f"[SPEAR | archive_executable.py] Done.")
