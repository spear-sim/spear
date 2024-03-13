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

    time.sleep(5)

    # sp_engine.begin_tick()
    # sp_engine.rpc_client.call("game_world_service.open_level", "debug_0001", "debug_0001")
    # sp_engine.tick()
    # sp_engine.end_tick()

    # time.sleep(10)

    # close the sp_engineironment
    sp_engine.close()

    spear.log("Done.")
