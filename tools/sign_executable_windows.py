#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import shutil
import spear
import subprocess
import sys


assert sys.platform == "win32"

cxx_compiler = "cl"
cxx_compiler_path = shutil.which(cxx_compiler)
if cxx_compiler_path is None:
    spear.log("ERROR: Can't find the Visual Studio command-line tools. All SPEAR build steps must run in a terminal where the Visual Studio command-line tools are visible. Giving up...")
    assert False
if cxx_compiler_path.lower().endswith("hostx86\\x86\\cl.exe") or cxx_compiler_path.lower().endswith("hostx86\\x64\\cl.exe"):
    spear.log("ERROR: 32-bit terminal detected. All SPEAR build steps must run in a 64-bit terminal. Giving up...")
    spear.log("ERROR: Compiler path:", cxx_compiler_path)
    assert False

parser = argparse.ArgumentParser()
parser.add_argument("--build-config", required=True)
parser.add_argument("--code-sign-tool-dir", required=True)
parser.add_argument("--ssl-username", required=True)
parser.add_argument("--ssl-password", required=True)
parser.add_argument("--ssl-totp-secret", required=True)
parser.add_argument("--build-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "BUILD")))
args = parser.parse_args()

assert os.path.exists(args.code_sign_tool_dir)
assert os.path.exists(args.build_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    input_dir = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Win64-{args.build_config}-Unsigned"))
    output_dir = os.path.realpath(os.path.join(args.build_dir, f"SpearSim-Win64-{args.build_config}"))

    # TODO: Debug and Test builds only work when the engine is compiled from source, so I don't know the default name of the executable for these build configs
    if args.build_config == "DebugGame":
        executable_name = "SpearSim-Win64-DebugGame"
    elif args.build_config == "Development":
        executable_name = "SpearSim"
    elif args.build_config == "Shipping":
        executable_name = "SpearSim-Win64-Shipping"
    else:
        assert False

    assert os.path.exists(input_dir)

    # make sure output_dir is empty
    shutil.rmtree(output_dir, ignore_errors=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(input_dir, output_dir)

    # files that need to be codesigned
    sign_files = [
        os.path.join("Windows", "SpearSim", "Binaries", "Win64", f"{executable_name}.exe"),
        os.path.join("Windows", "SpearSim", "Binaries", "Win64", f"{executable_name}-Cmd.exe"),
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

        spear.log("Signing file: ", sign_file)

        input_file = os.path.realpath(os.path.join(input_dir, sign_file))
        output_file = os.path.realpath(os.path.join(output_dir, sign_file))
        output_dir_internal = os.path.dirname(output_file)
        assert os.path.exists(input_file)
        assert os.path.exists(output_dir_internal)

        cmd = [
            "CodeSignTool.bat",
            "sign",
            f"-credential_id={crediential_id}",
            f"-username={args.ssl_username}",
            f"-password={args.ssl_password}",
            f"-totp_secret={args.ssl_totp_secret}",
            f"-input_file_path={input_file}",
            f"-output_dir_path={output_dir_internal}"]
        spear.log("Executing: ", " ".join(cmd))
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
        cmd = ["signtool", "verify", "/a", "/pa", "/all", "/v", f"{output_file}"]
        spear.log("Executing: ", " ".join(cmd))
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

        verified = None
        for line in ps.stdout:
            spear.log_no_prefix(line)
            if verified is None and "Successfully verified: " in line:
                verified = True
        ps.wait()
        ps.stdout.close()
        assert verified

        spear.log(f"{output_file} has been successfully signed.")

    os.chdir(cwd)

    spear.log("Done.")
