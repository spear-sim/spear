#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import pprint
import spear


if __name__ == "__main__":

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)

    with instance.begin_frame():

        # get UEnhancedInputLocalPlayerSubsystem
        enhanced_input_subsystem = instance.unreal_service.get_subsystem_by_type(class_name="UEnhancedInputLocalPlayerSubsystem")
        spear.log("enhanced_input_subsystem: ", enhanced_input_subsystem)

        # create UInputAction
        input_action = instance.unreal_service.load_object(class_name="UInputAction", outer=0, name="/SpComponents/Input/IA_InputAction.IA_InputAction")
        spear.log("input_action: ", instance.unreal_service.get_object_properties_from_uobject(uobject=input_action))

        # create UInputModifierScalar
        modifier_scalar = instance.unreal_service.new_object(class_name="UInputModifierScalar")
        spear.log("modifier_scalar: ", instance.unreal_service.get_object_properties_from_uobject(uobject=modifier_scalar))

        # modify UInputModifierScalar
        modifier_scalar_value_property_desc = instance.unreal_service.find_property_by_name_on_uobject(uobject=modifier_scalar, property_name="Scalar")
        instance.unreal_service.set_property_value(property_desc=modifier_scalar_value_property_desc, property_value={"X": 4.0, "Y": 5.0, "Z": 6.0})
        spear.log("modifier_scalar: ", instance.unreal_service.get_object_properties_from_uobject(uobject=modifier_scalar))

        # create UInputTriggerPressed
        trigger_pressed = instance.unreal_service.new_object(class_name="UInputTriggerPressed")
        spear.log("trigger_pressed: ", instance.unreal_service.get_object_properties_from_uobject(uobject=trigger_pressed))

        # modify UInputTriggerPressed
        trigger_pressed_actuation_threshold_property_desc = instance.unreal_service.find_property_by_name_on_uobject(
            uobject=trigger_pressed, property_name="ActuationThreshold")
        instance.unreal_service.set_property_value(property_desc=trigger_pressed_actuation_threshold_property_desc, property_value=0.75)
        spear.log("trigger_pressed: ", instance.unreal_service.get_object_properties_from_uobject(uobject=trigger_pressed))

        # call UEnhancedInputLocalPlayerSubsystem
        instance.enhanced_input_service.inject_input(
            enhanced_input_subsystem=enhanced_input_subsystem,
            input_action=input_action,
            input_action_value={"ValueType": "Axis2D", "Value": {"X": 1.0, "Y": 2.0, "Z": 3.0}},
            modifiers=[modifier_scalar],
            triggers=[trigger_pressed])

        # get actor
        actor = instance.unreal_service.find_actor_by_name(class_name="AActor", actor_name="__SP_DEFAULT_PAWN__")
        spear.log("actor: ", actor)

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

    spear.log("Done.")
