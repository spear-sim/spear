import os

SPEAR_ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

# ordered from low-level to high-level since that is the order they'll be loaded and merged in config.py
OPENBOT_DEFAULT_CONFIG_FILE = os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "..", "..", "unreal_plugins", "OpenBot", "default_config.yaml"))
SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE = os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "..", "..", "unreal_plugins", "SimulationController", "default_config.yaml"))
SPEAR_DEFAULT_CONFIG_FILE = os.path.realpath(os.path.join(SPEAR_ROOT_DIR, "default_config.yaml"))
