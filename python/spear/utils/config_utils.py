#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import yacs.config

#
# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the user_config_files input argument. This function is
# useful for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in user_config_files, all the default
# values required by the spear Python package and C++ plugins are loaded, and can be
# overridden by any of the files appearing in user_config_files.
#

def get_config(user_config_files=None):
    if user_config_files is None:
        user_config_files = []

    # create a single CfgNode that will eventually contain data from all config files
    config = yacs.config.CfgNode(new_allowed=True)

    # ordered from low-level to high-level
    default_config_files = [
        spear.path.realpath(os.path.dirname(__file__), "..", "config", "default_config.sp_core.yaml"),
        spear.path.realpath(os.path.dirname(__file__), "..", "config", "default_config.sp_services.yaml"),
        spear.path.realpath(os.path.dirname(__file__), "..", "config", "default_config.spear.yaml") ]

    for c in default_config_files:
        config.merge_from_file(c)

    for c in user_config_files:
        assert os.path.exists(c)
        config.set_new_allowed(True) # required to override an empty dict with a non-empty dict
        config.merge_from_file(c)

    config.freeze()

    return config
