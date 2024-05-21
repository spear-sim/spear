import argparse
import os
import posixpath
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud_path_prefix", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--paks", action="store_true")
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "build")))
    args = parser.parse_args()

    assert os.path.exists(args.input_dir)

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

    cloud_dir = posixpath.join(args.cloud_path_prefix, args.version_tag, "")

    if args.paks:
        files_to_upload = [ os.path.realpath(os.path.join(args.input_dir, x)) for x in os.listdir(args.input_dir) if x.endswith(paks_filter_string) ]
    else:
        files_to_upload = [ os.path.realpath(os.path.join(args.input_dir, f"SpearSim-{args.version_tag}-{platform_name}-Shipping.zip")) ]

    for file in files_to_upload:
        assert os.path.exists(file)
        cmd = ["aws", "s3", "cp", file, cloud_dir]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    spear.log("Done.")
