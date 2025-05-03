#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import posixpath
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--aws_path_prefix", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
    parser.add_argument("--upload_executable", action="store_true")
    parser.add_argument("--upload_paks", action="store_true")
    parser.add_argument("--paks_dir")
    args = parser.parse_args()

    if args.upload_executable:
        assert os.path.exists(args.build_dir)

    if args.upload_paks:
        assert os.path.exists(args.paks_dir)

    if sys.platform == "win32":
        platform_name = "Win64"
        paks_filter_string = args.version_tag + "-" + "Windows.pak"
    elif sys.platform == "darwin":
        platform_name = "Mac"
        paks_filter_string = args.version_tag + "-" + "Mac.pak"
    elif sys.platform == "linux":
        platform_name = "Linux"
        paks_filter_string = args.version_tag + "-" + "Linux.pak"
    else:
        assert False

    aws_dir = posixpath.join(args.aws_path_prefix, args.version_tag, "") # joining with a trailing empty string forces the path to end in "/"
    files_to_upload = []

    if args.upload_executable:
        files_to_upload.append(os.path.realpath(os.path.join(args.build_dir, f"SpearSim-{args.version_tag}-{platform_name}-Shipping.zip")))

    if args.upload_paks:
        paks_version_dir = os.path.realpath(os.path.join(args.paks_dir, args.version_tag))
        files_to_upload.extend([ os.path.realpath(os.path.join(paks_version_dir, x)) for x in sorted(os.listdir(paks_version_dir)) if x.endswith(paks_filter_string) ])

    for file in files_to_upload:
        assert os.path.exists(file)
        cmd = ["aws", "s3", "cp", file, aws_dir]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    spear.log("Done.")
