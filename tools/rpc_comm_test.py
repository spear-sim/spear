#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import numpy as np
import os
import spear
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    
    sp_engine = spear.SpEngine(config)

    sp_engine.n_ticks(5)

    sp_engine.open_level("debug_0000")
    sp_engine.n_ticks(10)

    sp_engine.open_level("debug_0001")
    sp_engine.n_ticks(10)
    
    # close the sp_engineironment
    sp_engine.close()

    spear.log("Done.")
