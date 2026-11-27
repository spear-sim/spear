#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import glob
import os
import spear
import sys


parser = argparse.ArgumentParser()
parser.add_argument("--script", required=True)
parser.add_argument("--unreal-engine-dir", required=True)
parser.add_argument("--map")
parser.add_argument("--render-offscreen", action="store_true")
parser.add_argument("--launch-mode", default="commandlet")
parser.add_argument("--unreal-project-dir", default=spear.path.realpath(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script

assert os.path.exists(args.unreal_engine_dir)
assert os.path.exists(args.unreal_project_dir)
assert os.path.exists(args.script)


if __name__ == "__main__":

    if sys.platform == "win32":
        unreal_editor_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealEditor-Cmd.exe")
    elif sys.platform == "darwin":
        unreal_editor_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealEditor.app", "Contents", "MacOS", "UnrealEditor")
    elif sys.platform == "linux":
        unreal_editor_bin = spear.path.realpath(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealEditor")
    else:
        assert False

    unreal_project_dir = spear.path.realpath(args.unreal_project_dir)
    uprojects = glob.glob(spear.path.realpath(unreal_project_dir, "*.uproject"))
    assert len(uprojects) == 1
    uproject = uprojects[0]


    # unknown args are passed to the inner script (see python/spear/utils/tool_utils.py)
    editor_script_args = [
        spear.tools.normalize_unreal_cmd_path(path=args.script)] + \
        unknown_args
    editor_script_args_string = spear.tools.get_unreal_python_script_args_string(args=editor_script_args)

    map_args = []
    if args.map is not None:
        map_args = [args.map]

    render_offscreen_args = []
    if args.render_offscreen:
        assert args.launch_mode == "full"
        render_offscreen_args = ["-renderoffscreen"]

    if args.launch_mode == "commandlet":
        python_args = ["-run=pythonscript", f"-script={editor_script_args_string}"]
    elif args.launch_mode == "full":
        python_args = [f"-executepythonscript={editor_script_args_string}"]
    else:
        assert False

    cmd = [
        unreal_editor_bin,
        spear.tools.normalize_unreal_cmd_path(path=uproject)] + \
        map_args + \
        render_offscreen_args + \
        python_args
    spear.tools.run_unreal_cmd(cmd=cmd)

    spear.log("Done.")
