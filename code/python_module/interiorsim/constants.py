import os

INTERIORSIM_ROOT_DIR = os.path.dirname(os.path.realpath(__file__))

INTERIORSIM_DEFAULT_CONFIG_FILE = os.path.join(INTERIORSIM_ROOT_DIR, "default_config.yaml")
SIMULATION_CONTROLLER_DEFAULT_CONFIG_FILE = os.path.join(INTERIORSIM_ROOT_DIR, "..", "..", "unreal_plugins", "SimulationController", "default_config.yaml")
