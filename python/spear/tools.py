#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os

# Plugins don't seem to get picked up by the Unreal build system when they're specified here and given as
# command-line arguments, so we specify them in DefaultGame.ini.
def get_cook_dirs():
    return [
        os.path.join("Content", "Characters"),
        os.path.join("Content", "Spear"),
        os.path.join("Content", "StarterContent"),
        os.path.join("Content", "ThirdPerson"),
        os.path.join("Content", "Vehicles"),
        os.path.join("Content", "VehicleTemplate")]

def get_cook_maps():
    return [
        "apartment_0000",
        "debug_0000"]
