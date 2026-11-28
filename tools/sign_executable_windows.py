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

parser = argparse.ArgumentParser()
parser.add_argument("--build-config", required=True)
parser.add_argument("--code-sign-tool-dir", required=True)
parser.add_argument("--ssl-username", required=True)
parser.add_argument("--ssl-password", required=True)
parser.add_argument("--ssl-totp-secret", required=True)
parser.add_argument("--build-dir", default=spear.path.realpath(os.path.dirname(__file__), "BUILD"))
args = parser.parse_args()

assert os.path.exists(args.code_sign_tool_dir)
assert os.path.exists(args.build_dir)
assert args.build_config in ["Debug", "DebugGame", "Development", "Shipping", "Test"]


if __name__ == "__main__":

    spear.tools.validate_build_environment()

    input_dir = spear.path.realpath(args.build_dir, f"SpearSim-Win64-{args.build_config}-Unsigned")
    output_dir = spear.path.realpath(args.build_dir, f"SpearSim-Win64-{args.build_config}")

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
    ps = spear.tools.popen_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)

    has_credential_id = None
    credential_id = None
    for line in ps.stdout:
        spear.log_no_prefix(line)
        if has_credential_id is None and "EVCS Credential ID(s):" in line:
            has_credential_id = True
        if "- " in line:
            credential_id = line.split("- ")[1].strip()
    return_code = ps.wait()
    ps.stdout.close()
    assert return_code == 0
    assert has_credential_id
    assert credential_id is not None

    for sign_file in sign_files:

        spear.log("Signing file: ", sign_file)

        input_file = spear.path.realpath(input_dir, sign_file)
        output_file = spear.path.realpath(output_dir, sign_file)
        output_dir_internal = os.path.dirname(output_file)
        assert os.path.exists(input_file)
        assert os.path.exists(output_dir_internal)

        cmd = [
            "CodeSignTool.bat",
            "sign",
            f"-credential_id={credential_id}",
            f"-username={args.ssl_username}",
            f"-password={args.ssl_password}",
            f"-totp_secret={args.ssl_totp_secret}",
            f"-input_file_path={input_file}",
            f"-output_dir_path={output_dir_internal}"]
        ps = spear.tools.popen_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)

        signed = None
        for line in ps.stdout:
            spear.log_no_prefix(line)
            if signed is None and "Code signed successfully: " in line:
                signed = True
        return_code = ps.wait()
        ps.stdout.close()
        assert return_code == 0
        assert signed

        # verify
        cmd = ["signtool", "verify", "/a", "/pa", "/all", "/v", f"{output_file}"]
        ps = spear.tools.popen_cmd(cmd=cmd, stdout=subprocess.PIPE, text=True)

        verified = None
        for line in ps.stdout:
            spear.log_no_prefix(line)
            if verified is None and "Successfully verified: " in line:
                verified = True
        return_code = ps.wait()
        ps.stdout.close()
        assert return_code == 0
        assert verified

        spear.log(f"{output_file} has been successfully signed.")

    os.chdir(cwd)

    spear.log("Done.")
