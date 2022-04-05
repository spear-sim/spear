import os

PACKAGE_ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

PACKAGE_DEFAULT_CONFIG_FILE = os.path.join(PACKAGE_ROOT_DIR, "default_config.yaml")
SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE = os.path.join(PACKAGE_ROOT_DIR, "..", "..", "unreal_plugins", "SimulationController", "default_config.yaml")
