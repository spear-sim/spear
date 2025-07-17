#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import yacs.config

# ordered from low-level to high-level
default_config_files = [
    os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "config", "default_config.sp_core.yaml")),
    os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "config", "default_config.sp_services.yaml")),
    os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "config", "default_config.spear.yaml")) ]

# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the user_config_files input argument. This function is
# useful for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in user_config_files, all the default
# values required by the spear Python package and C++ plugins are loaded, and can be
# overridden by any of the files appearing in user_config_files.
def get_config(user_config_files=[]):

    # create a single CfgNode that will eventually contain data from all config files
    config = yacs.config.CfgNode(new_allowed=True)

    for c in default_config_files:
        config.merge_from_file(c)

    for c in user_config_files:
        config.set_new_allowed(True) # required to override an empty dict with a non-empty dict
        config.merge_from_file(c)

    config.freeze()

    return config
