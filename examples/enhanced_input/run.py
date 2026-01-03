#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear


if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()
    
    with instance.begin_frame():

        # get UEnhancedInputLocalPlayerSubsystem
        enhanced_input_subsystem = game.unreal_service.get_subsystem(subsystem_provider_class_name="ULocalPlayer", subsystem_uclass="UEnhancedInputLocalPlayerSubsystem")
        spear.log("enhanced_input_subsystem: ", enhanced_input_subsystem)
        pprint.pprint(enhanced_input_subsystem.get_properties())

        # create UInputAction
        input_action = game.unreal_service.load_object(uclass="UInputAction", name="/SpContent/Input/IA_InputAction.IA_InputAction")
        spear.log("input_action: ", input_action)
        pprint.pprint(input_action.get_properties())

        # create UInputModifierScalar
        modifier_scalar = game.unreal_service.new_object(uclass="UInputModifierScalar")
        spear.log("modifier_scalar: ", modifier_scalar)
        pprint.pprint(modifier_scalar.get_properties())

        # modify UInputModifierScalar
        modifier_scalar.Scalar = {"X": 4.0, "Y": 5.0, "Z": 6.0}
        spear.log("modifier_scalar: ", modifier_scalar)
        pprint.pprint(modifier_scalar.get_properties())

        # create UInputTriggerPressed
        trigger_pressed = game.unreal_service.new_object(uclass="UInputTriggerPressed")
        spear.log("trigger_pressed: ", trigger_pressed)
        pprint.pprint(trigger_pressed.get_properties())

        # modify UInputTriggerPressed
        trigger_pressed.ActuationThreshold = 0.75
        spear.log("trigger_pressed: ", trigger_pressed)
        pprint.pprint(trigger_pressed.get_properties())

        # call UEnhancedInputLocalPlayerSubsystem
        instance.enhanced_input_service.inject_input(
            enhanced_input_subsystem=enhanced_input_subsystem,
            input_action=input_action,
            input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 2.0, "Z": 3.0}},
            modifiers=[modifier_scalar],
            triggers=[trigger_pressed])

        # get actor
        actor = game.unreal_service.find_actor_by_name(actor_name="__SP_DEFAULT_PAWN__", uclass="AActor")
        spear.log("actor: ", actor)
        pprint.pprint(actor.get_properties())

        # inject input for a specific actor
        instance.enhanced_input_service.inject_input_for_actor(
            actor=actor,
            input_action_name="IA_InputAction",
            trigger_event="Completed",
            input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 2.0, "Z": 3.0}},
            input_action_instance={"TriggerEvent": "Completed", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 1.23, "ElapsedTriggeredTime": 4.56},
            modifiers=[modifier_scalar],
            triggers=[trigger_pressed])

        # inject debug key for a specific actor
        instance.enhanced_input_service.inject_debug_key_for_actor(
            actor=actor,
            chord={"Key": "BackSpace", "bShift": False, "bCtrl": False, "bAlt": False, "bCmd": False},
            key_event="IE_Pressed",
            input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 2.0, "Z": 3.0}})

    with instance.end_frame():
        pass

    # we can also use Unreal's legacy input system via instance.input_service

    with instance.begin_frame():

        instance.input_service.inject_action_for_actor(
            actor=actor,
            action_name="DebugActionMapping",
            key_event="IE_Pressed",
            key_name="Equals")

        instance.input_service.inject_axis_for_actor(
            actor=actor,
            axis_name="DebugAxisMapping",
            axis_value=1.0)

    with instance.end_frame():
        pass

    spear.log("Done.")
