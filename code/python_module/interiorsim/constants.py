import os

INTERIORSIM_ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

# ordered from low-level to high-level since that is the order they'll be loaded and merged in config.py
OPENBOT_DEFAULT_CONFIG_FILE = os.path.join(INTERIORSIM_ROOT_DIR, "..", "..", "unreal_plugins", "OpenBot", "default_config.yaml")
SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE = os.path.join(INTERIORSIM_ROOT_DIR, "..", "..", "unreal_plugins", "SimulationController", "default_config.yaml")
INTERIORSIM_DEFAULT_CONFIG_FILE = os.path.join(INTERIORSIM_ROOT_DIR, "default_config.yaml")
