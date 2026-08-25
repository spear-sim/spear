#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import cv2
import math
import numpy as np
import shutil
import spear
import time


_ANIMATION_DURATION_SECONDS = 10.0


if __name__ == "__main__":

    # create instance
    instance = spear.Instance()
    game = instance.get_game()

    with instance.begin_frame():
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")
        gameplay_statics.SetGamePaused(bPaused=False)

        # get control rig component
        character = game.unreal_service.find_actor_by_name(actor_name="Actor_metahuman_004_CR", uclass="AActor")
        control_rig_component = game.unreal_service.get_component_by_class(actor=character, uclass="UControlRigComponent")

        # print all available controls and their types. There is no dedicated UFUNCTION for querying a
        # control's type, so we look it up via the control rig's hierarchy instead.
        control_names = control_rig_component.GetElementNames(ElementType="Control")
        hierarchy = control_rig_component.GetControlRig().GetHierarchy()

        spear.log(f"Found {len(control_names)} controls:")
        for control_name in control_names:
            control = hierarchy.FindControl_ForBlueprintOnly(InKey={"Name": control_name, "Type": "Control"})
            control_type = control["settings"]["controlType"]
            spear.log_no_prefix("    ", f"{control_name} ({control_type})")

    with instance.end_frame():
        pass

    spear.log(f"Animating character for {_ANIMATION_DURATION_SECONDS} seconds...")

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

        if elapsed_time >= _ANIMATION_DURATION_SECONDS:
            break

    spear.log(f"Finished animation.")
    spear.log("Done.")
