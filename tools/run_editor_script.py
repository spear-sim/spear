#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import subprocess
import sys


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--script", required=True)
    parser.add_argument("--unreal_engine_dir", required=True)
    parser.add_argument("--unreal_project_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
    args, unknown_args = parser.parse_known_args() # get remaining args to pass to inner script

    assert os.path.exists(args.unreal_engine_dir)

    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor-Cmd.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
    else:
        assert False

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uproject_name = os.path.split(unreal_project_dir)[1]
    uproject = os.path.realpath(os.path.join(unreal_project_dir, uproject_name + ".uproject"))

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " " + " ".join(unknown_args)

    # need shell=True to correctly handle the quotes in cmd
    cmd = unreal_editor_bin + " " + uproject + ' -run=pythonscript -script="' + args.script + unknown_arg_string + '"'
    spear.log(f"Executing: {cmd}")
    subprocess.run(cmd, shell=True, check=True)

    spear.log("Done.")
