#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import json
import os
import spear
import unreal


parser = argparse.ArgumentParser()
parser.add_argument("--export-dir", required=True)
args = parser.parse_args()

unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)


def process_scene():

    editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()
    spear.log("Processing scene: " + editor_world_name)

    actors = spear.editor.find_actors()
    actors = { spear.editor.get_stable_name_for_actor(actor=actor, include_unreal_name=True): spear.editor.get_actor_desc(actor=actor) for actor in actors }

    unreal_metadata_dir = os.path.realpath(os.path.join(args.export_dir, "unreal_metadata"))
    actors_json_file = os.path.realpath(os.path.join(unreal_metadata_dir, "scene.json"))
    spear.log("Writing JSON file: " + actors_json_file)
    os.makedirs(unreal_metadata_dir, exist_ok=True)
    with open(actors_json_file, "w") as f:
        json.dump(actors, f, indent=4, sort_keys=True)

    spear.log("Done.")


if __name__ == "__main__":
    process_scene()
