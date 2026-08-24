#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import math
import numpy as np
import os
import shutil
import spear
import time


_ANIMATION_DURATION = 10.0


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        gameplay_statics.SetGamePaused(bPaused=False)

        # get control rig component
        character = game.unreal_service.find_actor_by_name(actor_name="Actor_metahuman_004_CR", uclass="AActor")
        control_rig_component = game.unreal_service.get_component_by_class(actor=character, uclass="UControlRigComponent")

    with instance.end_frame():
        pass

    spear.log(f"Animating character for {_ANIMATION_DURATION} seconds...")

    start_time = time.time()
    current_time = start_time

    while True:
        previous_time = current_time
        current_time = time.time()
        dt = current_time - previous_time
        elapsed_time = current_time - start_time

        t = elapsed_time
        s = 4.0*t
        cs = math.cos(s)
        ss = math.sin(s)

        v_1 = {"X": 0.25*cs, "Y": 0.25*ss}
        v_2 = {"X": -0.5*cs, "Y": -0.5*ss}

        with instance.begin_frame():
            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_C_jaw", Value=v_1)
            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_C_mouth", Value=v_1)

            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_L_eye", Value=v_2)
            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_L_nose", Value=v_2)
            control_rig_component.send_async.SetControlFloat(ControlName="CTRL_L_brow_raiseIn", Value=cs)

            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_R_eye", Value=v_1)
            control_rig_component.send_async.SetControlVector2D(ControlName="CTRL_R_nose", Value=v_2)
            control_rig_component.send_async.SetControlFloat(ControlName="CTRL_R_brow_raiseOut", Value=s)
            control_rig_component.send_async.SetControlFloat(ControlName="CTRL_R_eye_faceScrunch", Value=s)
            control_rig_component.send_async.SetControlFloat(ControlName="CTRL_R_eye_cheekRaise", Value=s)

        with instance.end_frame():
            pass

        if elapsed_time >= _ANIMATION_DURATION:
            break

    spear.log(f"Finished animation.")
    spear.log("Done.")
