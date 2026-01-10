#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import subprocess
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--level-sequence", required=True)
parser.add_argument("--map", required=True)
parser.add_argument("--mrq-config", required=True)
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to the editor

assert os.path.exists(args.unreal_engine_dir)


if __name__ == "__main__":

    if sys.platform == "win32":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor-Cmd.exe"))
    elif sys.platform == "darwin":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor"))
    elif sys.platform == "linux":
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor"))
    else:
        assert False

    unreal_project_dir = os.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(os.path.realpath(os.path.join(unreal_project_dir, "*.uproject")))
    assert len(uprojects) == 1
    uproject = uprojects[0]

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " ".join(unknown_args)

    # need shell=True to correctly handle the quotes in cmd
    cmd = f'"{unreal_editor_bin}" "{uproject}" {args.map} -game -levelsequence="{args.level_sequence}" -moviepipelineconfig="{args.mrq_config}"'
    spear.log("Executing: ", cmd)
    subprocess.run(cmd, shell=True, check=True)

    spear.log("Done.")
