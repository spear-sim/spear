#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import shutil
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--paks_dir", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    args = parser.parse_args()

    if sys.platform == "win32":
        platform_name     = "Win64"
        platform_dir_name = "WindowsNoEditor"
        pak_platform_name = "Windows"
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{platform_name}-Shipping"))
        paks_dir_dest     = os.path.realpath(os.path.join(executable_dir, "WindowsNoEditor", "SpearSim", "Content", "Paks"))
    elif sys.platform == "darwin":
        platform_name     = "Mac"
        platform_dir_name = "MacNoEditor"
        pak_platform_name = "Mac"
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{platform_name}-Shipping"))
        paks_dir_dest     = os.path.realpath(os.path.join(executable_dir, "MacNoEditor", "SpearSim-Mac-Shipping.app", "Contents", "UE4", "SpearSim", "Content", "Paks"))
    elif sys.platform == "linux":
        platform_name     = "Linux"
        platform_dir_name = "LinuxNoEditor"
        pak_platform_name = "Linux"
        executable_dir    = os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{platform_name}-Shipping"))
        paks_dir_dest     = os.path.realpath(os.path.join(executable_dir, "LinuxNoEditor", "SpearSim", "Content", "Paks"))
    else:
        assert False

    # once we know pak_platform_name, set our default pak file src and dest
    pak_file_src  = os.path.realpath(os.path.join(args.paks_dir, "kujiale_0000-" + args.version_tag + "-" + pak_platform_name + ".pak"))
    pak_file_dest = os.path.realpath(os.path.join(paks_dir_dest, "kujiale_0000-" + args.version_tag + "-" + pak_platform_name + ".pak"))
    
    # copy our custom pak file
    shutil.copyfile(pak_file_src, pak_file_dest)
    print(f"[SPEAR | build_executable.py] Copied {pak_file_src} to {pak_file_dest}")

    archive_file_name = os.path.realpath(os.path.join(args.output_dir, f"SpearSim-{args.version_tag}-{platform_name}-Shipping"))
    shutil.make_archive(base_name=archive_file_name, format="zip", root_dir=os.path.realpath(os.path.join(executable_dir, platform_dir_name)), verbose=1)
    assert os.path.exists(archive_file_name + ".zip")

    print(f"[SPEAR | archive_executable.py] Successfully archived executable to {archive_file_name}.zip")
    print("[SPEAR | archive_executable.py] Done.")
