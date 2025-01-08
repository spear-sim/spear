#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear

if __name__ == "__main__":

    # create instance
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)

    with instance.begin_frame():

        # find functions
        gameplay_statics_static_class = instance.unreal_service.get_static_class(class_name="UGameplayStatics")
        get_player_controller_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="GetPlayerController")
        set_game_paused_func = instance.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="SetGamePaused")

        player_controller_static_class = instance.unreal_service.get_static_class(class_name="APlayerController")
        possess_func = instance.unreal_service.find_function_by_name(uclass=player_controller_static_class, function_name="Possess")

        chaos_vehicle_movement_component_uclass = instance.unreal_service.get_static_class(class_name="UChaosVehicleMovementComponent")
        set_throttle_input_func = instance.unreal_service.find_function_by_name(uclass=chaos_vehicle_movement_component_uclass, function_name="SetThrottleInput")

        # get UGameplayStatics default object
        gameplay_statics_default_object = instance.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        # get player controller
        return_values = instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func, args={"PlayerIndex": 0})
        player_controller = spear.to_handle(return_values["ReturnValue"])

        #
        # initialize car
        #

        bp_car_uclass = instance.unreal_service.load_object(class_name="UObject", outer=0, name="/Game/VehicleTemplate/Blueprints/OffroadCar/OffroadCar_Pawn.OffroadCar_Pawn_C")
        car = instance.unreal_service.spawn_actor_from_uclass(
            uclass=bp_car_uclass,
            location={"X": -2500.0, "Y": -9330.0, "Z": 20.0},
            rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
            spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})
        chaos_vehicle_movement_component = instance.unreal_service.get_component_by_class(actor=car, uclass=chaos_vehicle_movement_component_uclass)

        # need player_controller current actor to inject input
        instance.unreal_service.call_function(
            uobject=player_controller,
            ufunction=possess_func,
            args={"InPawn": spear.to_ptr(car)})

    with instance.end_frame():
        pass

    # set throttle to 1.0
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 1.0})
    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive
    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # steer
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Steering",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive
    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # reset
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Reset",
            trigger_event="Triggered",
            input_action_value={},
            input_action_instance={})

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # set throttle to 1.0 and steer
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 1.0})
        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Steering",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": -1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive
    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # set throttle to 0.0
    with instance.begin_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 0.0})
    with instance.end_frame():
        instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # coast
    for _ in range(100):
        with instance.begin_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            instance.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # close the instance
    instance.close()

    spear.log("Done.")
