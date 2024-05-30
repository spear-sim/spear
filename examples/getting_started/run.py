#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import spear
import time

common_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "common"))
import sys

sys.path.append(common_dir)

num_steps = 100

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(
        user_config_files=[
            os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml")),
            os.path.realpath(os.path.join(common_dir, "default_config.common.yaml"))])

    spear.configure_system(config)
    instance = spear.Instance(config)

    # spawn agent
    instance.engine_service.begin_tick()

    agent = instance.unreal_service.spawn_actor(
        class_name="/Game/MyVehiclePawn.MyVehiclePawn_C",
        location={"X": 0.0, "Y": 0.0, "Z": 100.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters={"Name": "Agent"}
    )
    spear.log("agent", agent)

    instance.engine_service.tick()
    instance.engine_service.end_tick()

    for i in range(num_steps):
        # set updated poses in SPEAR
        spear.log("frame=", i)
        instance.engine_service.begin_tick()

        # TODO update transform or apply keyboard actions

        instance.engine_service.tick()
        instance.engine_service.end_tick()

    instance.close()

    spear.log("Done.")
