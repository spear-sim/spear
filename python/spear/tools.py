#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import glob
import os
import pathlib

#
# Specifying multiple -cookdir arguments on the command-line doesn't work reliably, so we specify cook
# directories in DefaultGame.ini, and we provide this function to return the maps to cook. This is needed
# in several places, e.g., build_executable.py, build_paks.py, and run_uat.py.
#

def get_cook_maps():
    return [
        "apartment_0000",
        "debug_0000",
        "ThirdPersonMap",
        "VehicleExampleMap"]

#
# Get filesystem path from content path.
#

def get_filesystem_path_from_content_path(content_path, unreal_project_dir, unreal_engine_dir):

    content_path_tokens = pathlib.PurePosixPath(content_path).parts
    assert len(content_path_tokens) >= 2
    content_root = content_path_tokens[1]

    if content_root == "Game":
        filesystem_base_dir = os.path.join(unreal_project_dir, "Content")
    elif content_root == "Engine":
        filesystem_base_dir = os.path.join(unreal_engine_dir, "Engine", "Content")
    else:
        plugins_dirs = [
            os.path.join(unreal_project_dir, "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Plugins"),
            os.path.join(unreal_engine_dir, "Engine", "Experimental")]

        found_plugin = False
        for plugins_dir in plugins_dirs:
            plugin_dir = os.path.realpath(os.path.join(plugins_dir, content_root))
            if os.path.exists(plugin_dir):
                filesystem_base_dir = os.path.realpath(os.path.join(plugin_dir, "Content"))
                found_plugin = True
                break
        assert found_plugin

    if len(content_path_tokens) == 2:
        return filesystem_base_dir
    else:
        content_sub_path = os.path.join(*content_path_tokens[2:])
        filesystem_path = os.path.realpath(os.path.join(filesystem_base_dir, content_sub_path))
        if os.path.exists(filesystem_path) and os.path.isdir(filesystem_path):
            return filesystem_path
        else:
            content_file_tokens = content_path_tokens[-1].split(".")
            if len(content_file_tokens) == 1:
                filesystem_paths = glob.glob(os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], content_file_tokens[0] + ".*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            elif len(content_file_tokens) == 2:
                assert content_file_tokens[0] == content_file_tokens[1]
                filesystem_paths = glob.glob(os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:-1], content_file_tokens[0] + ".*")))
                if len(filesystem_paths) == 1:
                    return filesystem_paths[0]
                else:
                    return os.path.realpath(os.path.join(filesystem_base_dir, *content_path_tokens[2:]))
            else:
                assert False
