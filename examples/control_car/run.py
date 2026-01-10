#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import os
import spear


if __name__ == "__main__":

    # create game
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])
    spear.configure_system(config=config)
    instance = spear.Instance(config=config)
    game = instance.get_game()

    with instance.begin_frame():

        # get UGameplayStatics
        gameplay_statics = game.get_unreal_object(uclass="UGameplayStatics")

        # get car
        car = game.unreal_service.find_actor_by_class(uclass="AWheeledVehiclePawn")

        #
        # Instead of finding the existing car in the scene, it is also possible to spawn a new car (e.g., the
        # default Unreal off-road car) as follows:
        #        
        # bp_car_uclass = game.unreal_service.load_class(class_name="AActor", name="/Game/VehicleTemplate/Blueprints/OffroadCar/OffroadCar_Pawn.OffroadCar_Pawn_C")
        # car = game.unreal_service.spawn_actor(
        #     uclass=bp_car_uclass,
        #     location={"X": -2500.0, "Y": -9330.0, "Z": 20.0},
        #     rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0},
        #     spawn_parameters={"Name": "Agent", "SpawnCollisionHandlingOverride": "AlwaysSpawn"})
        #

        #
        # If we spawned a new car, then we would need to possess it as follows:
        #
        # player_controller = gameplay_statics.GetPlayerController(PlayerIndex=0)
        # player_controller.Possess(InPawn=car)
        #

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, rather than by programmatically
        # injecting gamepad input via Unreal's Enhanced Input system, we would need to obtain a handle to the
        # car's UChaosVehicleMovementComponent as follows:
        #
        # chaos_vehicle_movement_component = game.unreal_service.get_component_by_class(actor=car, uclass="UChaosVehicleMovementComponent")
        #

    with instance.end_frame():
        pass

    # set throttle to 1.0 (value will persist across frames until reset)
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # chaos_vehicle_movement_component.SetThrottleInput(Throttle=1.0)
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Throttle",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        gameplay_statics.SetGamePaused(bPaused=True)

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # set steering to 1.0 (value will persist across frames until reset)
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # chaos_vehicle_movement_component.SetThrottleInput(Throttle=1.0)
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Steering",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 1.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        gameplay_statics.SetGamePaused(bPaused=True)

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # reset
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Reset",
            trigger_event="Triggered",
            input_action_value={},
            input_action_instance={})

    with instance.end_frame():
        gameplay_statics.SetGamePaused(bPaused=True)

    # set throttle to 1.0 and steering to -1.0 (values will persist across frames until reset)
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we could do so as follows:
        #
        # chaos_vehicle_movement_component.SetThrottleInput(Throttle=1.0)
        # chaos_vehicle_movement_component.SetSteeringInput(Steering=1.0)
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
        gameplay_statics.SetGamePaused(bPaused=True)

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # set throttle to 0.0 (value will persist across frames until reset)
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)

        #
        # If we wanted to control the car by calling its UFUNCTIONS directly, we would do so as follows:
        #
        # chaos_vehicle_movement_component.SetThrottleInput(Throttle=1.0)
        #

        instance.enhanced_input_service.inject_input_for_actor(
            actor=car,
            input_action_name="IA_Throttle",
            trigger_event="Triggered",
            input_action_value={"ValueType": "Axis1D", "Value": {"X": 0.0, "Y": 0.0, "Z": 0.0}},
            input_action_instance={"TriggerEvent": "Triggered", "LastTriggeredWorldTime": 0.0, "ElapsedProcessedTime": 0.01, "ElapsedTriggeredTime": 0.01})

    with instance.end_frame():
        gameplay_statics.SetGamePaused(bPaused=True)

    # drive for multiple frames
    for _ in range(100):
        with instance.begin_frame():
            gameplay_statics.SetGamePaused(bPaused=False)
        with instance.end_frame():
            gameplay_statics.SetGamePaused(bPaused=True)

    # unpause now that we're finished controlling the car
    with instance.begin_frame():
        gameplay_statics.SetGamePaused(bPaused=False)
    with instance.end_frame():
        pass

    instance.close()

    spear.log("Done.")
