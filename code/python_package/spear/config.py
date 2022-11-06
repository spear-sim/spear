from yacs.config import CfgNode

import spear

# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the user_config_files input argument. This function is
# useful for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in user_config_files, all the default
# values required by the spear Python package are loaded into a top-level SPEAR namespace,
# and can be overridden by any of the files appearing in user_config_files.
def get_config(user_config_files):

    # create a single CfgNode that will eventually contain data from all config files
    config = CfgNode(new_allowed=True)

    # merge config files from low-level to high-level
    config.merge_from_file(spear.OPENBOT_DEFAULT_CONFIG_FILE)
    config.merge_from_file(spear.SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE)
    config.merge_from_file(spear.SPEAR_DEFAULT_CONFIG_FILE)

    for c in user_config_files:
        config.merge_from_file(c)

    config.freeze()

    return config
