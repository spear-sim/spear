import time
import cv2
import copy
import importlib

from vehicle_env_data import VehicleEnv_Data


class BehaviorPolicy:

    def __init__(self, config, map_names,
                 run_mode=None,
                 policy_path=None,
                 debug_mode=False,
                 num_iter=1000,
                 runs=5,
                 create_video=True,
                 dataset_path=None):
        self.env_config = {
            "config": None,
            "dataset_path": dataset_path,
            "num_steps": num_iter,
            "create_video": create_video,
            "debug_mode": debug_mode,
        }
        if run_mode == "Infer":
            self.env_config["policy_path"] = policy_path

        self.run_mode = run_mode
        self.debug_mode = debug_mode
        self.config = config
        self.map_names = map_names
        self.amount_of_runs = runs

        self.env = None

    def _load_map(self, map_name):
        # Load the correct map:
        self.config.defrost()
        self.config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_" + map_name
        if self.debug_mode:
            self.config.SIMULATION_CONTROLLER.OPENBOT_AGENT_CONTROLLER.PHYSICAL_OBSERVATION_MODE = "full-pose"
        self.config.freeze()

    def _load_env(self):
        self.env_config["config"] = copy.deepcopy(self.config)
        # Create Env object:
        module = importlib.import_module(
            f"vehicle_env_{self.run_mode.lower()}")
        env_class = getattr(module, f"VehicleEnv_{self.run_mode}")
        self.env = env_class(**self.env_config)
        print("==================================")
        print("Environment loaded")
        print("==================================")

    def _close_env(self):
        # Close the environment:
        self.env.close()
        time.sleep(3)

    def perform_run_in_all_maps(self):
        for map_name in self.map_names:  # For each map

            self._load_map(map_name)

            # creating one env here and resetting
            # instead of re-generating because loading one env takes a while
            self._load_env()
            run_no = 0
            while run_no < self.amount_of_runs:
                # Using while, because perform_run may be not successful
                # (collision etc)
                print("----------------------")
                print(f"run {run_no+1}/{self.amount_of_runs}")
                print("----------------------")
                success = self.env.perform_run(map_name, run_no)
                run_no += success

        if self.debug_mode:
            cv2.destroyAllWindows()

        self._close_env()
