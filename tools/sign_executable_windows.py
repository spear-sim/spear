#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import shutil
import spear
import subprocess
import sys


assert sys.platform == "win32"

parser = argparse.ArgumentParser()
parser.add_argument("--build_config", required=True)
parser.add_argument("--code_sign_tool_dir", required=True)
parser.add_argument("--ssl_username", required=True)
parser.add_argument("--ssl_password", required=True)
parser.add_argument("--ssl_totp_secret", required=True)
parser.add_argument("--build_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
args = parser.parse_args()

assert os.path.exists(args.code_sign_dir)
assert os.path.exists(args.build_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    input_dir = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Win64-{args.build_config}-Unsigned"))
    output_dir = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Win64-{args.build_config}"))

    assert os.path.exists(input_dir)

    # make sure output_dir and staging_dir are empty
    shutil.rmtree(output_dir, ignore_errors=True)

    # create the temp directory
    spear.log("Creating directory if it does not already exist: ", staging_dir)
    os.makedirs(staging_dir, exist_ok=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(input_dir, output_dir)

    # files that need to be codesigned
    sign_files = [
        os.path.join("Windows", "SpearSim", "Binaries", "Win64", "Binaries", f"SpearSim-Win64-{args.build_config}.exe"),
        os.path.join("Windows", "SpearSim", "Binaries", "Win64", "Binaries", f"SpearSim-Win64-{args.build_config}-Cmd.exe"),
        os.path.join("Windows", "SpearSim.exe")]

    cwd = os.getcwd()
    os.chdir(args.code_sign_tool_dir)

    cmd = ["CodeSignTool.bat", "get_credential_ids", f"-username={args.ssl_username}", f"-password={args.ssl_password}"]
    spear.log("Executing: ", " ".join(cmd))
    ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

    has_credential_id = None
    crediential_id = None
    for line in ps.stdout:
        spear.log_no_prefix(line)
        if has_credential_id is None and "EVCS Credential ID(s):" in line:
            has_credential_id = True
        if "- " in line:
            crediential_id = line.split("- ")[1].strip()
    ps.wait()
    ps.stdout.close()
    assert has_credential_id
    assert crediential_id is not None

    for sign_file in sign_files:

        input_file = os.path.realpath(os.path.join(input_dir, sign_file))
        output_file = os.path.realpath(os.path.join(output_dir, sign_file))
        output_dir = os.path.dirname(input_file)
        os.path.exists(input_file)
        os.path.exists(output_dir)

        cmd = [
            "CodeSignTool.bat",
            "sign",
            f"-credential_id={crediential_id}",
            f"-username={args.ssl_username}",
            f"-password={args.ssl_password}",
            f"-totp_secret={args.ssl_totp_secret}",
            f"-input_file_path={input_file}",
            f"-output_dir_path={output_dir}"]
        spear.log("Executing: ", ' '.join(cmd))
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

        signed = None
        for line in ps.stdout:
            spear.log_no_prefix(line)
            if signed is None and "Code signed successfully: " in line:
                signed = True
        ps.wait()
        ps.stdout.close()
        assert signed

        # verify
        cmd = ["signtool", "verify", "/pa", f"{output_file}"]
        spear.log("Executing: ", ' '.join(cmd))
        ps = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True) # need to use stderr instead of stdout

        verified = None
        for line in ps.stderr: # need to use stderr instead of stdout
            spear.log_no_prefix(line)
            if success is None and "Successfully verified: " in line:
                verified = True
        ps.wait()
        ps.stderr.close() # need to use stderr instead of stdout
        assert verified

        spear.log(f"{output_file} has been successfully signed.")

    os.chdir(cwd)

    spear.log("Done.")
