#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Please refer to https://medium.com/@parthnaik92/codesign-and-notarize-unreal-engine-mac-builds-for-distribution-outside-of-the-mac-store-d2f6e444f3a7
# for pre-requisites and setup your system before trying to run this file.

import argparse
import os
import shutil
import subprocess
import time


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--macos_executable", required=True)
    parser.add_argument("--entitlements_file", required=True)
    parser.add_argument("--developer_id", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--apple_username", required=True)
    parser.add_argument("--apple_password", required=True)
    parser.add_argument("--wait_time_seconds", type=float, default=600.0)
    args = parser.parse_args()

    assert os.path.exists(args.macos_executable)
    radio_effect_units = [
        os.path.join(args.macos_executable, "Contents", "Resources", "RadioEffectUnit.component"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Build", "Mac", "RadioEffectUnit")]

    print(f"[SPEAR | codesign_mac_executable.py] Removing {radio_effect_units[0]}.")
    shutil.rmtree(radio_effect_units[0], ignore_errors=True)

    print(f"[SPEAR | codesign_mac_executable.py] Removing {radio_effect_units[1]}.")
    shutil.rmtree(radio_effect_units[1], ignore_errors=True)

    print(f"[SPEAR | codesign_mac_executable.py] Changing rpaths...")

    # change current working directory to add relative rpaths 
    cwd = os.getcwd()
    os.chdir(os.path.join(args.macos_executable, ".."))

    executable_name = os.path.basename(args.macos_executable)

    cmd = [
        "install_name_tool",
        "-add_rpath",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-add_rpath",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-add_rpath",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPxFoundation.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3Common.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libAPEX_Clothing.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPxPvdSDK.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libNvCloth.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3Cooking.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libAPEX_Legacy.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    cmd = [
        "install_name_tool",
        "-rpath",
        "/Volumes/Work/Perforce/UE4/Engine/Binaries/ThirdParty/PhysX3/Mac",
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac"),
        os.path.join(executable_name, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libApexFramework.dylib")]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # revert current working directory
    os.chdir(cwd)

    # files that need to be codesigned
    files = [
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3Common.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPxFoundation.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libAPEX_Clothing.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPxPvdSDK.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libNvCloth.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libPhysX3Cooking.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libAPEX_Legacy.dylib"),
        os.path.join(args.macos_executable, "Contents", "UE4", "Engine", "Binaries", "ThirdParty", "PhysX3", "Mac", "libApexFramework.dylib"),
        os.path.join(args.macos_executable, "Contents", "MacOS", os.path.splitext(executable_name)[0])]

    assert os.path.exists(args.entitlements_file)

    for file in files:
        cmd = ["sudo", "codesign", "-f", "-s", "-v", "--options", "runtime", "--timestamp", "--entitlements", args.entitlements_file, "--sign", f"Developer ID Application: {args.developer_id}", file]
        print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
        cmd_result = subprocess.run(cmd)
        assert cmd_result.returncode == 0

    os.makedirs(args.output_dir, exist_ok=True)

    # create a zip file for notarization
    temp_zip = os.path.join(args.output_dir, f"{os.path.splitext(os.path.basename(args.macos_executable))[0]}-temp.zip")
    cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", args.macos_executable, temp_zip]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    # send the zip file for notarization 
    cmd = ["xcrun", "altool", "--notarize-app", "--primary-bundle-id", "org.embodiedaifoundation.spear", "--username", args.apple_username, "--password", args.apple_password, "--file", temp_zip]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    ps = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    output = subprocess.check_output(["grep", "RequestUUID = "], stdin=ps.stdout, text=True)
    ps.wait()
    ps.stdout.close()
    request_uuid = " ".join(output[14:].split())
    print(f"[SPEAR | codesign_mac_executable.py] Request UUID: {request_uuid}")

    # check the status of notarization request
    cmd = [
        "xcrun", "altool", "--notarization-info", request_uuid, "--username", args.apple_username, "--password", args.apple_password]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    output = "             in progress"
    start_time = time.time()
    elapsed_time = time.time() - start_time
    while " ".join(output[13:].split()) == "in progress" and elapsed_time<args.wait_time_seconds:
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        output = subprocess.check_output(["grep", "     Status:"], stdin=ps.stdout, text=True)
        ps.wait()
        ps.stdout.close()
        time.sleep(20)
        elapsed_time = time.time() - start_time
        print(f"[SPEAR | codesign_mac_executable.py] Waiting to get more information on the notarization request...")
    
    assert elapsed_time <= args.wait_time_seconds, f"[SPEAR | codesign_mac_executable.py] ERROR: Exceeded provided wait time ({args.wait_time_seconds}s) for notarization. Please complete this procedure after notarization is complete."
    
    # staple the executable
    cmd = ["xcrun", "stapler", "staple", args.macos_executable]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print(f"[SPEAR | codesign_mac_executable.py] Removing the temporary zip file {temp_zip}") 
    os.remove(temp_zip)

    # zip the stapled executable for distribution
    notarized_zip = os.path.join(args.output_dir, f"{os.path.splitext(os.path.basename(args.macos_executable))[0]}-notarized.zip")
    cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", os.path.dirname(args.macos_executable), notarized_zip]
    print(f"[SPEAR | codesign_mac_executable.py] Executing: {' '.join(cmd)}")
    cmd_result = subprocess.run(cmd)
    assert cmd_result.returncode == 0

    print(f"[SPEAR | codesign_mac_executable.py] Done: {notarized_zip} is ready for distribution!")
