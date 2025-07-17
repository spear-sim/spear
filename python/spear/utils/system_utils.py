#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os

# perform configuration steps that should only be done once per system, as opposed to once per instance
def configure_system(config):

    # set environment variables
    if config.SPEAR.LAUNCH_MODE in ["editor", "game"]:
        for environment_var_name, environment_var_value in config.SPEAR.ENVIRONMENT_VARS.items():
            log("Setting environment variable: ", environment_var_name, " = ", environment_var_value)
            os.environ[environment_var_name] = environment_var_value
