#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear


if __name__ == "__main__":

    # create game
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config)
    instance = spear.Instance(config)
    game = instance.get_game()

    with instance.begin_frame():

        # find functions
        gameplay_statics_static_class = game.unreal_service.get_static_class(class_name="UGameplayStatics")
        set_game_paused_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="SetGamePaused")

        # get UGameplayStatics default object
        gameplay_statics_default_object = game.unreal_service.get_default_object(uclass=gameplay_statics_static_class, create_if_needed=False)

        # find car
        car = game.unreal_service.find_actor_by_type(class_name="AWheeledVehiclePawn")

        #
        # Instead of finding the existing car in the scene, it is also possible to spawn a new car (e.g., the
        # default Unreal off-road car) as follows:
        #        
        # bp_car_uclass = game.unreal_service.load_object(class_name="UClass", outer=0, name="/Game/VehicleTemplate/Blueprints/OffroadCar/OffroadCar_Pawn.OffroadCar_Pawn_C")
        # car = game.unreal_service.spawn_actor_from_class(
        #     uclass=bp_car_uclass,
        #     location={"X": -2500.0, "Y": -9330.0, "Z": 20.0},
        #     rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        #     spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})
        #

        #
        # If we spawned a new car, then we would need to possess it as follows:
        #
        # get_player_controller_func = game.unreal_service.find_function_by_name(uclass=gameplay_statics_static_class, function_name="GetPlayerController")
        #
        # player_controller_static_class = game.unreal_service.get_static_class(class_name="APlayerController")
        # possess_func = game.unreal_service.find_function_by_name(uclass=player_controller_static_class, function_name="Possess")
        #
        # return_values = game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=get_player_controller_func, args={"PlayerIndex": 0})
        # player_controller = spear.to_handle(string=return_values["ReturnValue"])
        #
        # game.unreal_service.call_function(uobject=player_controller, ufunction=possess_func, args={"InPawn": spear.to_ptr(handle=car)})
        #

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, rather than by programmatically
        # injecting gamepad input via Unreal's Enhanced Input system, we would need to obtain handles to the
        # car's UChaosVehicleMovementComponent and its UFUNCTIONS as follows:
        #
        # chaos_vehicle_movement_component_static_class = game.unreal_service.get_static_class(class_name="UChaosVehicleMovementComponent")
        # set_steering_input_func = game.unreal_service.find_function_by_name(uclass=chaos_vehicle_movement_component_static_class, function_name="SetSteeringInput")
        # set_throttle_input_func = game.unreal_service.find_function_by_name(uclass=chaos_vehicle_movement_component_static_class, function_name="SetThrottleInput")
        #
        # chaos_vehicle_movement_component = game.unreal_service.get_component_by_class(actor=car, uclass=chaos_vehicle_movement_component_static_class)
        #

    with instance.end_frame():
        pass

    # set throttle to 1.0 (value will persist across frames until reset)
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # game.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 1.0})
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Throttle",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # set steering to 1.0 (value will persist across frames until reset)
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # game.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_steering_input_func, args={"Steering": 1.0})
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Steering",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # reset
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Reset",
            trigger_event="Triggered",
            input_action_value={},
            input_action_instance={})

    with instance.end_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # set throttle to 1.0 and steering to -1.0 (values will persist across frames until reset)
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # game.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 1.0})
        # game.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_steering_input_func, args={"Steering": 1.0})
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Throttle",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Steering",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": -1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # set throttle to 0.0 (value will persist across frames until reset)
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we would do so as follows:
        #
        # game.unreal_service.call_function(uobject=chaos_vehicle_movement_component, ufunction=set_throttle_input_func, args={"Throttle": 0.0})
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Throttle",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 0.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
        with instance.end_frame():
            game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": True})

    # unpause now that we're finished controlling the car
    with instance.begin_frame():
        game.unreal_service.call_function(uobject=gameplay_statics_default_object, ufunction=set_game_paused_func, args={"bPaused": False})
    with instance.end_frame():
        pass

    spear.log("Done.")
