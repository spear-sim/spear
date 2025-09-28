#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import pandas as pd
import spear
import spear.utils.editor_utils
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--maps_file", required=True)
parser.add_argument("--script")
args, unknown_args = parser.parse_known_args() # get unknown args to pass to inner script

level_editor_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)


if __name__ == "__main__":

    unknown_arg_string = ""
    if len(unknown_args) > 0:
        unknown_arg_string = " ".join(unknown_args)

    maps = pd.read_csv(args.maps_file)["maps"].tolist()

    for m in maps:
        spear.log("Loading map: ", m)
        level_editor_subsystem.load_level(m)

        editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()
        actors = spear.utils.editor_utils.find_actors()
        spear.log(f"Loaded map: {editor_world_name} (contains {len(actors)} actors)")

        if args.script is not None:
            cmd = f"py {args.script} {unknown_arg_string}".strip()
            spear.log("Executing command: ", cmd)
            unreal.SystemLibrary.execute_console_command(unreal_editor_subsystem.get_editor_world(), cmd)

    spear.log("Done.")
