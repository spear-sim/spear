#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
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


assert sys.platform == "darwin"

parser = argparse.ArgumentParser()
parser.add_argument("--apple-developer-id", required=True)
parser.add_argument("--apple-id", required=True)
parser.add_argument("--apple-password", required=True)
parser.add_argument("--apple-team-id", required=True)
parser.add_argument("--build-config", required=True)
parser.add_argument("--build-dir", default=spear.path.realpath(os.path.dirname(__file__), "BUILD"))
parser.add_argument("--entitlements-file", default=spear.path.realpath(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim", "Build", "Mac", "Resources", "entitlements.plist"))
args = parser.parse_args()

assert os.path.exists(args.build_dir)
assert os.path.exists(args.entitlements_file)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    input_dir = spear.path.realpath(args.build_dir, f"SpearSim-Mac-{args.build_config}-Unsigned")
    notarize_dir = spear.path.realpath(args.build_dir, f"SpearSim-Mac-{args.build_config}-Notarize")
    output_dir = spear.path.realpath(args.build_dir, f"SpearSim-Mac-{args.build_config}")

    assert os.path.exists(input_dir)

    # make sure output_dir and notarize_dir are empty
    shutil.rmtree(notarize_dir, ignore_errors=True)
    shutil.rmtree(output_dir, ignore_errors=True)

    # create the temp directory
    spear.log("Creating directory if it does not already exist: ", notarize_dir)
    os.makedirs(notarize_dir, exist_ok=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(input_dir, output_dir)

    # TODO: Debug and Test builds only work when the engine is compiled from source, so I don't know the default name of the executable for these build configs
    if args.build_config == "DebugGame":
        executable = "SpearSim-Mac-DebugGame"
    elif args.build_config == "Development":
        executable = "SpearSim"
    elif args.build_config == "Shipping":
        executable = "SpearSim-Mac-Shipping"
    else:
        assert False

    executable_app = f"{executable}.app"
    executable_zip = f"{executable}.zip"
    executable_app_path = spear.path.realpath(output_dir, "Mac", executable_app)
    assert os.path.exists(executable_app_path)

    # files that need to be codesigned
    sign_files = [
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Apple", "MetalShaderConverter", "Mac", "libmetalirconverter.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "EOSSDK", "Mac", "libEOSSDK-Mac-Shipping.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac/libtbb.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac/libtbbmalloc.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "OpenColorIO", "Mac", "libOpenColorIO.2.3.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Plugins", "NNE", "NNERuntimeORT", "Binaries", "ThirdParty", "Onnxruntime", "Mac", "libonnxruntime.1.17.1.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "Engine", "Plugins", "NNE", "NNERuntimeORT", "Binaries", "ThirdParty", "Onnxruntime", "Mac", "libonnxruntime.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_atomic-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_chrono-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_filesystem-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_iostreams-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_program_options-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_python311-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_regex-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_system-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "UE", "SpearSim", "Binaries", "Mac", "libboost_thread-mt-a64.dylib"),
        spear.path.realpath(executable_app_path, "Contents", "MacOS", executable)]

    # Creating Distribution-Signed Code for Mac
    #     https://developer.apple.com/forums/thread/701514#701514021
    for sign_file in sign_files:

        spear.log("Signing file: ", sign_file)

        assert os.path.exists(sign_file)

        # sign
        cmd = ["codesign", "--force", "--timestamp", "--verbose", "--options", "runtime", "--entitlements", args.entitlements_file, "--sign", args.apple_developer_id, sign_file]
        spear.tools.run_cmd(cmd=cmd)

        # verify
        cmd = ["codesign", "--verify", "--deep", "--strict", "--verbose", sign_file]
        ps = spear.tools.popen_cmd(cmd=cmd, stderr=subprocess.PIPE, text=True) # need to use stderr instead of stdout
        valid_on_disk = None
        satisfies_designated_requirement = None
        for line in ps.stderr: # need to use stderr instead of stdout
            spear.log_no_prefix(line)
            if valid_on_disk is None and "valid on disk" in line:
                valid_on_disk = True
            if satisfies_designated_requirement is None and "satisfies its Designated Requirement" in line:
                satisfies_designated_requirement = True
        return_code = ps.wait()
        ps.stderr.close() # need to use stderr instead of stdout
        assert return_code == 0
        assert valid_on_disk
        assert satisfies_designated_requirement

    # Customizing the Notarization Workflow - create an archive (-c) in pkzip format (-k) and embed the parent directory name in the archive (-keepParent)
    # while preserving HFS metadata (--sequesterRsrc)
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    notarization_zip = spear.path.realpath(notarize_dir, executable_zip)
    cmd = ["ditto", "-c", "-k", "--sequesterRsrc", "--keepParent",
        executable_app_path,
        notarization_zip]
    spear.tools.run_cmd(cmd=cmd)

    # Customizing the Notarization Workflow - upload the archive for notarization
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    cmd = ["xcrun", "notarytool", "submit", notarization_zip, "--apple-id", args.apple_id, "--team-id", args.apple_team_id, "--password", args.apple_password, "--wait"]
    ps = spear.tools.popen_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)
    submission_id = None
    status = None
    for line in ps.stdout:
        spear.log_no_prefix(line)
        if submission_id is None and "  id: " in line:
            submission_id = line.split("  id: ")[1].strip()
        if "  status: " in line:
            status = line.split("  status: ")[1].strip()
    return_code = ps.wait()
    ps.stdout.close()
    assert submission_id is not None

    # Fetching the Notary Log
    #     https://developer.apple.com/forums/thread/705839
    log_file = spear.path.realpath(notarize_dir, "log.json")
    cmd = ["xcrun", "notarytool", "log", submission_id, "--apple-id", args.apple_id, "--team-id", args.apple_team_id, "--password", args.apple_password, log_file]
    spear.tools.run_cmd(cmd=cmd)
    spear.log(f"Log file associated with the notarization process has been successfully written to {log_file}.")
    spear.log("Printing the contents of this log file...")
    with open(log_file) as f:
        spear.log_no_prefix(json.dumps(json.load(f), indent=4))

    # validate return code and status from previous cmd, we do this after fetching the log so the log is printed even if notarization fails
    assert return_code == 0
    assert status == "Accepted"

    # Customizing the Notarization Workflow - staple the executable
    #     https://developer.apple.com/documentation/security/notarizing_macos_software_before_distribution/customizing_the_notarization_workflow
    cmd = ["xcrun", "stapler", "staple", executable_app_path]
    ps = spear.tools.popen_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)

    worked = None
    for line in ps.stdout:
        spear.log_no_prefix(line)
        if worked is None and "The staple and validate action worked!" in line:
            worked = True
    return_code = ps.wait()
    ps.stdout.close()
    assert return_code == 0
    assert worked

    # verify
    cmd = ["spctl", "--assess", "--type", "execute", "--verbose", executable_app_path]
    ps = spear.tools.popen_cmd(cmd=cmd, stderr=subprocess.PIPE, text=True) # need to use stderr instead of stdout

    accepted = None
    for line in ps.stderr: # need to use stderr instead of stdout
        spear.log_no_prefix(line)
        if accepted is None and ": accepted" in line:
            accepted = True
    return_code = ps.wait()
    ps.stderr.close() # need to use stderr instead of stdout
    assert return_code == 0
    assert accepted

    spear.log(f"{executable_app_path} has been successfully signed.")
    spear.log("Done.")
