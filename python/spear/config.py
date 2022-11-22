from yacs.config import CfgNode

import os


SPEAR_ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

# ordered from low-level to high-level
DEFAULT_CONFIG_FILES = [
    os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "config", "default_config.openbot.yaml")),
    os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "config", "default_config.simulation_controller.yaml")),
    os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "config", "default_config.spear.yaml")) ]


# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the user_config_files input argument. This function is
# useful for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in user_config_files, all the default
# values required by the spear Python package and C++ plugins are loaded, and can be
# overridden by any of the files appearing in user_config_files.
def get_config(user_config_files):

    # create a single CfgNode that will eventually contain data from all config files
    config = CfgNode(new_allowed=True)

    for c in DEFAULT_CONFIG_FILES:
        config.merge_from_file(c)

    for c in user_config_files:
        config.merge_from_file(c)

    config.freeze()

    return config
