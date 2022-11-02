from yacs.config import CfgNode

import interiorsim.constants as constants

# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the config_files input argument. This function is useful
# for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in config_files, all the default values
# required by the interiorsim Python module are loaded into a top-level interiorsim namespace, and
# can be overridden by any of the files appearing in config_files.
def get_config(config_files):

    # create a single CfgNode that will eventually contain data from all config files
    config = CfgNode(new_allowed=True)

    config.merge_from_file(constants.OPENBOT_DEFAULT_CONFIG_FILE)
    config.merge_from_file(constants.ROBOTSIM_DEFAULT_CONFIG_FILE)
    config.merge_from_file(constants.SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE)
    config.merge_from_file(constants.INTERIORSIM_DEFAULT_CONFIG_FILE)

    for c in config_files:
        config.merge_from_file(c)
    config.freeze()

    return config
