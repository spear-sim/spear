
import argparse
import numpy as np
import os
from interiorsim import Env
from interiorsim.config import get_config
from interiorsim.constants import INTERIORSIM_ROOT_DIR
    

if __name__ == "__main__":

    # List of config files to be used
    config_files = []

    # Add default config files first and then user config files
    config_files.append(os.path.join(os.getcwd(), "user_config.yaml"))

    # Load configs
    config = get_config(config_files)
    print(config)

    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", nargs="+", default=[""], help="Array of map references. A number of s distinct runs will be executed in each map. This argument overwrites the LEVEL_ID argument and allows collecting data in multiple environments programatically", required=False)
    parser.add_argument("-c", "--connect", type=int, help="id of the connection port", required=True)
    args = parser.parse_args()

    config.defrost()
    config.SIMULATION_CONTROLLER.PORT = 30000 + args.connect
    config.freeze()

    if args.map == [""]: # Use the default LEVEL_ID argument from parameter file
        if config.SIMULATION_CONTROLLER.LEVEL_ID == "":
            mapNames = ["simpleMap"]
        else:
            mapNames = [config.SIMULATION_CONTROLLER.LEVEL_ID]

    else: # Overwrite the map value
        mapNames = args.map

    for mapName in mapNames: # For each map

        # Load the correct map:
        config.defrost()
        config.SIMULATION_CONTROLLER.LEVEL_ID = mapName
        config.freeze()

        # Create Env object:

        env = Env(config = config)

            
