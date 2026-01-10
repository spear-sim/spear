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
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--maps-file")
parser.add_argument("--script")
parser.add_argument("--render-offscreen", action="store_true")
parser.add_argument("--launch-mode", default="commandlet")
parser.add_argument("--unreal-project-dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim")))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script

assert args.script is not None or args.maps_file is not None
assert os.path.exists(args.unreal_engine_dir)

if args.script is None:
    assert len(unknown_args) == 0


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

    if args.maps_file is None:
        if args.script is None:
            assert False
        else:
            editor_script_args_string = f"{args.script} {unknown_arg_string}".strip()
    else:
        if args.script is None:
            editor_script_args_string = f"run_editor_script_for_each_map.py --maps_file {args.maps_file}".strip()
        else:
            editor_script_args_string = f"run_editor_script_for_each_map.py --maps_file {args.maps_file} --script {args.script} {unknown_arg_string}".strip()

    render_offscreen_arg_string = ""
    if args.render_offscreen:
        assert args.launch_mode == "full"
        render_offscreen_arg_string = "-renderoffscreen"

    if args.launch_mode == "commandlet":
        python_arg_string = "-run=pythonscript -script="
    elif args.launch_mode == "full":
        python_arg_string = "-executepythonscript="
    else:
        assert False

    # need shell=True to correctly handle the quotes in cmd
    cmd = f'"{unreal_editor_bin}" "{uproject}" {render_offscreen_arg_string} {python_arg_string}"{editor_script_args_string}"'
    spear.log("Executing: ", cmd)
    subprocess.run(cmd, shell=True, check=True)

    spear.log("Done.")
