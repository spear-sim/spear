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
        character = game.unreal_service.find_actor_by_name(actor_name="Zebra", uclass="AActor")
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

        body_pitch_degrees   = 10.0*(math.sin(2.0*t))
        neck_pitch_degrees   = 10.0*(math.sin(1.5*t))
        jaw_yaw_degrees      = 6.0*(1.0 - math.sin(5.0*t))
        eye_aim_target_x     = 20.0 + 6.0*(1.0 - math.sin(2.0*t))
        corner_l_position_y  = 30.0*(1.0 - math.sin(5.0*t))
        corner_l_position_z  = 30.0*(1.0 - math.sin(5.0*t))
        corner_r_position_y  = -30.0*(1.0 - math.sin(5.0*t))
        corner_r_position_z  = -30.0*(1.0 - math.sin(5.0*t))
        sneer_amount         = 6.0*(1.0 - math.sin(5.0*t))
        ear_l_pitch_degrees  = 10.0*(math.sin(5.0*t))
        ear_r_pitch_degrees  = 10.0*(math.sin(5.0*t + math.pi/2.0))
        brow_pitch_degrees   = 12.0*(math.sin(1.5*t))
        eye_look_yaw_degrees = 20.0*(math.sin(0.75*t))
        arm_l_pitch_degrees  = -20.0 + 20.0*(math.sin(2.0*t))
        arm_r_pitch_degrees  = -20.0 + 20.0*(math.sin(1.0*t))

        with instance.begin_frame():
            control_rig_component.send_async.SetControlRotator(ControlName="Body/Body", Value={"Pitch": body_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")

            control_rig_component.send_async.SetControlRotator(ControlName="Neck/Mid FK", Value={"Pitch": neck_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")

            control_rig_component.send_async.SetControlRotator(ControlName="Face/jaw", Value={"Pitch": 0.0, "Yaw": jaw_yaw_degrees, "Roll": 0.0}, Space="LocalSpace")
            control_rig_component.send_async.SetControlPosition(ControlName="Face/Eye Aim", Value={"X": eye_aim_target_x, "Y": 150.0, "Z": 0.0}, Space="LocalSpace")
            control_rig_component.send_async.SetControlPosition(ControlName="Face/Corner L", Value={"X": 0.0, "Y": corner_l_position_y, "Z": corner_l_position_z}, Space="LocalSpace")
            control_rig_component.send_async.SetControlPosition(ControlName="Face/Corner R", Value={"X": 0.0, "Y": corner_l_position_y, "Z": corner_l_position_z}, Space="LocalSpace")
            control_rig_component.send_async.SetControlFloat(ControlName="Face/Sneer Tp", Value=sneer_amount)
            control_rig_component.send_async.SetControlFloat(ControlName="Face/Sneer Bt", Value=sneer_amount)

            control_rig_component.send_async.SetControlRotator(ControlName="Ear L/Ear 01 L", Value={"Pitch": ear_l_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")
            control_rig_component.send_async.SetControlRotator(ControlName="Ear R/Ear 01 R", Value={"Pitch": ear_r_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")

            control_rig_component.send_async.SetControlRotator(ControlName="Face/Brow Main L", Value={"Pitch": brow_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")
            control_rig_component.send_async.SetControlRotator(ControlName="Face/Brow Main R", Value={"Pitch": brow_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")

            control_rig_component.send_async.SetControlRotator(ControlName="Arm L/FK 1", Value={"Pitch": arm_l_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")
            control_rig_component.send_async.SetControlRotator(ControlName="Arm R/FK 1", Value={"Pitch": arm_r_pitch_degrees, "Yaw": 0.0, "Roll": 0.0}, Space="LocalSpace")

        with instance.end_frame():
            pass

        if elapsed_time >= _ANIMATION_DURATION_SECONDS:
            break

    spear.log(f"Finished animation.")
    spear.log("Done.")
