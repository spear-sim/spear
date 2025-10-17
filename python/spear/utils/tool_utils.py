#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

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
