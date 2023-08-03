#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Please refer to https://medium.com/@parthnaik92/codesign-and-notarize-unreal-engine-mac-builds-for-distribution-outside-of-the-mac-store-d2f6e444f3a7
# for pre-requisites and setup your system before trying to run this file.

import argparse
import os
import shutil
import spear
import subprocess
import sys


if __name__ == "__main__":

    assert sys.platform == "darwin"

    parser = argparse.ArgumentParser()
    parser.add_argument("--developer_id", required=True)
    parser.add_argument("--apple_id", required=True)
    parser.add_argument("--apple_teamid", required=True)
    parser.add_argument("--apple_password", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "build", "SpearSim-Mac-Shipping-Unsigned")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "build", "SpearSim-Mac-Shipping")))
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "tmp")))
    parser.add_argument("--entitlements_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "entitlements.plist")))
    args = parser.parse_args()

    # make sure output_dir is empty
    shutil.rmtree(args.output_dir, ignore_errors=True)

    # create the temp directory
    os.makedirs(args.temp_dir, exist_ok=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(args.input_dir, args.output_dir)

    executable = os.path.realpath(os.path.join(args.output_dir, "Mac", "SpearSim-Mac-Shipping.app"))
    assert os.path.exists(executable)

    executable_name = os.path.basename(executable)

    # files that need to be code-signed
    sign_files = [
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbb.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbbmalloc.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "MacOS", os.path.splitext(executable_name)[0]))
    ]

    assert os.path.exists(args.entitlements_file)

    for file in sign_files:
        cmd = [
            "sudo", "codesign", "-f", "-s", "-v", "--options", "runtime", "--timestamp", "--entitlements", 
            args.entitlements_file, "--sign", f"Developer ID Application: {args.developer_id}", file]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    # create a zip file for notarization
    notarization_zip = os.path.realpath(os.path.join(args.temp_dir, f"{os.path.splitext(executable_name)[0]}.zip"))
    cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", executable, notarization_zip]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    cmd = [
        "xcrun", "notarytool", "submit", notarization_zip, "--apple-id", args.apple_id,
        "--team-id", args.apple_teamid, "--password", args.apple_password, "--wait"
    ]
    spear.log(f"Executing: {' '.join(cmd)}")
    ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    status = ""
    for line in ps.stdout:
        spear.log(f"{line}")
        if "  status: " in line:
            status = line.split("  status: ")[1].strip()
    ps.wait()
    ps.stdout.close()
    assert status == "Accepted"

    # staple the executable
    cmd = ["xcrun", "stapler", "staple", executable]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    
    spear.log(f"{executable} has been successfully signed.")
    spear.log("Done.")
