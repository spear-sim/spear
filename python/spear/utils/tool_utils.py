#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import sys

#
# Return a default set of maps to cook. This set is needed in several places, e.g., build_executable.py,
# build_paks.py, and run_uat.py. We would like to specify directories to always cook in a similar way, but
# specifying multiple -cookdir arguments on the command-line doesn't work reliably. So we specify cook
# directories in DefaultGame.ini.
#

def get_default_maps_to_cook():
    return [
        "apartment_0000",           # /Game/SPEAR/Scenes/apartment_0000/Maps/apartment_0000
        "debug_0000",               # /Game/SPEAR/Scenes/debug_0000/Maps/debug_0000
        "debug_0001",               # /Game/SPEAR/Scenes/debug_0001/Maps/debug_0001
        "Advanced_Lighting",        # /Game/StarterContent/Maps/Advanced_Lighting
        "Minimal_Default",          # /Game/StarterContent/Maps/Minimal_Default
        "StarterMap",               # /Game/StarterContent/Maps/StarterMap
        "ThirdPersonMap",           # /Game/ThirdPerson/Maps/ThirdPersonMap
        "VehicleExampleMap",        # /Game/VehicleTemplate/Maps/VehicleExampleMap
        "VehicleOffroadExampleMap"] # /Game/VehicleTemplate/Maps/VehicleOffroadExampleMap


#
# Helper functions for paths that handle symlinks robustly.
#

def path_exists(path):
    if os.path.exists(path) or os.path.islink(path):
        return True

    head, tail = os.path.split(path)
    if os.path.exists(head) or os.path.islink(head):
        if tail == "":
            return os.path.exists(head) or os.path.islink(head)
        else:
            return tail in os.listdir(head)
    else:
        return False

def remove_path(path):
    if not path_exists(path=path):
        return

    if os.path.islink(path):
        os.unlink(path)
    elif os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    else:
        # if we have a broken symlink, then try to use the command-line (we don't attempt to use
        # subprocess.run() because it returns an error when attempting to remove broken symlinks
        if sys.platform == "win32":
            rm_cmd = "del"
        elif sys.platform in ["darwin", "linux"]:
            rm_cmd = "rm"
        else:
            assert False
        cmd = f"{rm_cmd} {path}"
        cmd_result = os.system(cmd)
        assert cmd_result == 0
