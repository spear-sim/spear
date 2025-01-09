#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os

# Specifying multiple -cookdir arguments on the command-line doesn't work reliably, so we specify cook
# directories in DefaultGame.ini.

def get_cook_maps():
    return [
        "apartment_0000",
        "debug_0000",
        "ThirdPersonMap",
        "VehicleExampleMap"]
