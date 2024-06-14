#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# See the following documentation for the prerequisites you need to run this file:
#     Signing your Apps for Gatekeeper
#         https://developer.apple.com/developer-id
#     Creating Distribution-Signed Code for Mac
#         https://developer.apple.com/forums/thread/701514#701514021
#     Packaging Mac Software for Distribution
#         https://developer.apple.com/forums/thread/701581#701581021
#     Customizing the Notarization Workflow
#         https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
#    Fetching the Notary Log
#         https://developer.apple.com/forums/thread/705839

import argparse
import json
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
    parser.add_argument("--apple_team_id", required=True)
    parser.add_argument("--apple_password", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "BUILD", "SpearSim-Mac-Shipping-Unsigned")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "BUILD", "SpearSim-Mac-Shipping")))
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "tmp")))
    parser.add_argument("--entitlements_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "entitlements.plist")))
    args = parser.parse_args()

    assert os.path.exists(args.entitlements_file)

    # make sure output_dir is empty
    shutil.rmtree(args.output_dir, ignore_errors=True)

    # create the temp directory
    os.makedirs(args.temp_dir, exist_ok=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(args.input_dir, args.output_dir)

    executable = os.path.realpath(os.path.join(args.output_dir, "Mac", "SpearSim-Mac-Shipping.app"))
    assert os.path.exists(executable)

    executable_name = os.path.basename(executable)

    # files that need to be codesigned
    sign_files = [
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbb.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbbmalloc.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib")),
        os.path.realpath(os.path.join(executable, "Contents", "MacOS", os.path.splitext(executable_name)[0]))]

    # Creating Distribution-Signed Code for Mac
    #     https://developer.apple.com/forums/thread/701514#701514021
    for file in sign_files:
        cmd = [
            "sudo",
            "codesign",
            "--force",
            "--timestamp",
            "--verbose",
            "--options",
            "runtime",
            "--entitlements", args.entitlements_file,
            "--sign",
            args.developer_id,
            file]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

    # Customizing the Notarization Workflow - create an archive (-c) in pkzip format (-k) and embed the parent directory name in the archive (-keepParent)
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    notarization_zip = os.path.realpath(os.path.join(args.temp_dir, f"{os.path.splitext(executable_name)[0]}.zip"))
    cmd = [
        "ditto",
        "-c",
        "-k",
        "--sequesterRsrc",
        "--keepParent",
        executable,
        notarization_zip]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    # Customizing the Notarization Workflow - upload the archive for notarization
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    cmd = [
        "xcrun",
        "notarytool",
        "submit",
        notarization_zip,
        "--apple-id", args.apple_id,
        "--team-id", args.apple_team_id,
        "--password", args.apple_password,
        "--wait"]
    spear.log(f"Executing: {' '.join(cmd)}")
    ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    status = ""
    submission_id = None
    for line in ps.stdout:
        spear.log(f"{line}")
        if submission_id is None and "  id: " in line:
            submission_id = line.split("  id: ")[1].strip()
        if "  status: " in line:
            status = line.split("  status: ")[1].strip()
    ps.wait()
    ps.stdout.close()
    assert submission_id is not None

    # Fetching the Notary Log
    #     https://developer.apple.com/forums/thread/705839
    log_file = os.path.realpath(os.path.join(args.temp_dir, "notarization_log.json"))
    cmd = [
        "xcrun",
        "notarytool",
        "log",
        submission_id,
        "--apple-id", args.apple_id,
        "--team-id", args.apple_team_id,
        "--password", args.apple_password,
        log_file]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    spear.log(f"Log file associated with the notarization process has been successfully written to {log_file}.")
    spear.log("Printing the contents of this log file...")
    with open(log_file) as f:
        spear.log_no_prefix(json.dumps(json.load(f), indent=4))

    assert status == "Accepted"

    # Customizing the Notarization Workflow - staple the executable
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    cmd = [
        "xcrun",
        "stapler",
        "staple",
        executable]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

    spear.log(f"{executable} has been successfully signed.")
    spear.log("Done.")
