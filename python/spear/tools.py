#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

#
# Specifying multiple -cookdir arguments on the command-line doesn't work reliably, so we specify cook
# directories in DefaultGame.ini, and we provide this function to return a default set of maps to cook. This
# set is needed in several places, e.g., build_executable.py, build_paks.py, and run_uat.py.
#

def get_cook_maps():
    return [
        "apartment_0000",
        "debug_0000",
        "debug_0001",
        "ThirdPersonMap",
        "VehicleExampleMap"]
